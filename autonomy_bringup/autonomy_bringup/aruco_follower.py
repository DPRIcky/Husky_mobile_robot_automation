#!/usr/bin/env python3
"""
Follow an ArUco target using relative pose from the detector.

The detector publishes the target pose in the camera optical frame:
  x = right, y = down, z = forward

This node uses x for steering and z for distance control, then publishes a
TwistStamped command for the observing robot.
"""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


class ArucoFollower(Node):
    def __init__(self) -> None:
        super().__init__('aruco_follower')

        self.declare_parameter('target_pose_topic', '/a300_00000/aruco_detector/target_pose')
        self.declare_parameter('cmd_vel_topic', '/a300_00000/cmd_vel')
        self.declare_parameter('base_frame', 'a300_00000/base_link')
        self.declare_parameter('control_rate_hz', 15.0)
        self.declare_parameter('target_timeout_s', 2.0)
        self.declare_parameter('desired_standoff_m', 1.5)
        self.declare_parameter('distance_deadband_m', 0.08)
        # Hard minimum approach distance — stop unconditionally when the
        # camera-frame depth drops below this value, regardless of standoff
        # setting or controller output.  Prevents physical collision if the
        # robot overshoots or visual-servo gains cause it to approach too fast.
        # Default: slightly less than typical standoff so normal following
        # is unaffected; acts only as a collision safety net.
        self.declare_parameter('min_approach_dist_m', 0.6)
        self.declare_parameter('lateral_deadband_m', 0.10)
        self.declare_parameter('align_before_drive_m', 0.60)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_reverse_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('enable_lost_target_recovery', True)
        self.declare_parameter('lost_target_recovery_s', 3.0)
        self.declare_parameter('recovery_angular_speed', 0.20)
        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('tracker_status_topic', '')
        self.declare_parameter('predicted_path_topic', '')
        self.declare_parameter('prediction_horizon_s', 1.5)
        # When aruco_goal_manager is running it publishes nav_mode here.
        # "planned" or "idle" → follower goes completely silent so twist_mux
        # can fall through to the obstacle-aware path_follower.
        # "visual" (or empty topic) → normal direct ArUco following.
        self.declare_parameter('nav_mode_topic', '')
        # Scaling applied when tracker is in 'predicted' (KF-only) mode.
        # Reduces speed and relaxes deadbands so KF prediction errors do not
        # drive aggressive manoeuvres.
        self.declare_parameter('predicted_speed_scale', 0.60)
        self.declare_parameter('predicted_lateral_deadband_scale', 2.0)

        self._target_pose: PoseStamped | None = None
        self._last_target_time = None
        self._had_target = False
        # Recovery state
        self._last_lateral_error: float = 0.0
        self._in_recovery: bool = False
        self._recovery_start_time = None
        # KF predicted path — retained for external subscribers / diagnostics
        self._last_predicted_path: Path | None = None
        # Tracker status (updated from optional tracker_status_topic)
        self._tracker_status: str = 'measured'
        self._last_logged_tracker_status: str = ''
        # nav_mode from aruco_goal_manager:
        #   "visual"  → normal ArUco following (default when no manager running)
        #   "planned" → path_follower has command; this node goes completely silent
        #   "idle"    → nav timeout expired; this node goes completely silent
        self._nav_mode: str = 'visual'

        target_pose_topic = self.get_parameter('target_pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._base_frame = self.get_parameter('base_frame').value
        self._timeout_s = float(self.get_parameter('target_timeout_s').value)
        self._desired_standoff   = float(self.get_parameter('desired_standoff_m').value)
        self._distance_deadband  = float(self.get_parameter('distance_deadband_m').value)
        self._min_approach_dist  = float(self.get_parameter('min_approach_dist_m').value)
        self._lateral_deadband = float(self.get_parameter('lateral_deadband_m').value)
        self._align_before_drive = float(self.get_parameter('align_before_drive_m').value)
        self._linear_kp = float(self.get_parameter('linear_kp').value)
        self._angular_kp = float(self.get_parameter('angular_kp').value)
        self._max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self._max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self._max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self._enable_recovery = bool(self.get_parameter('enable_lost_target_recovery').value)
        self._recovery_window_s = float(self.get_parameter('lost_target_recovery_s').value)
        self._recovery_angular_speed = float(self.get_parameter('recovery_angular_speed').value)
        self._recovery_linear_speed = float(self.get_parameter('recovery_linear_speed').value)
        tracker_status_topic = str(self.get_parameter('tracker_status_topic').value)
        predicted_path_topic = str(self.get_parameter('predicted_path_topic').value)
        nav_mode_topic       = str(self.get_parameter('nav_mode_topic').value)
        self._prediction_horizon_s = float(self.get_parameter('prediction_horizon_s').value)
        self._predicted_speed_scale = float(self.get_parameter('predicted_speed_scale').value)
        self._predicted_deadband_scale = float(
            self.get_parameter('predicted_lateral_deadband_scale').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.create_subscription(PoseStamped, target_pose_topic, self._target_pose_cb, 10)
        if tracker_status_topic:
            self.create_subscription(String, tracker_status_topic, self._tracker_status_cb, 10)
        if predicted_path_topic:
            self.create_subscription(Path, predicted_path_topic, self._predicted_path_cb, 10)
        if nav_mode_topic:
            self.create_subscription(String, nav_mode_topic, self._nav_mode_cb, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'Aruco follower ready target={target_pose_topic} cmd_vel={cmd_vel_topic} '
            f'standoff={self._desired_standoff:.2f}m'
            + (f' tracker_status={tracker_status_topic}' if tracker_status_topic else '')
            + (f' nav_mode={nav_mode_topic}' if nav_mode_topic else '')
        )

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        self._target_pose = msg
        self._last_target_time = self.get_clock().now()
        if not self._had_target:
            self.get_logger().info('ArUco target acquired')
            self._had_target = True

    def _tracker_status_cb(self, msg: String) -> None:
        status = msg.data
        if status != self._last_logged_tracker_status:
            if status == 'predicted':
                self.get_logger().info(
                    'ArUco tracker: detection gap — following predicted pose'
                )
            elif status == 'lost':
                self.get_logger().warn(
                    'ArUco tracker: target fully lost — prediction expired'
                )
            elif status == 'measured':
                if self._last_logged_tracker_status in ('predicted', 'lost'):
                    self.get_logger().info('ArUco tracker: live measurements resumed')
            self._last_logged_tracker_status = status
        self._tracker_status = status

    def _predicted_path_cb(self, msg: Path) -> None:
        if msg.poses:
            self._last_predicted_path = msg

    def _nav_mode_cb(self, msg: String) -> None:
        """
        Receives nav_mode from aruco_goal_manager.
        "planned" / "idle" → go completely silent (path_follower has the bus).
        "visual"            → resume direct ArUco visual servo.
        """
        mode = msg.data
        if mode != self._nav_mode:
            if mode == 'planned':
                self.get_logger().info(
                    '[follower] nav_mode → planned: yielding cmd_vel to path_follower '
                    '(obstacle-aware navigation to frozen goal)'
                )
            elif mode == 'idle':
                self.get_logger().warn(
                    '[follower] nav_mode → idle: nav timeout expired — '
                    'going silent, waiting for ArUco reacquisition'
                )
            elif mode == 'visual':
                self.get_logger().info(
                    '[follower] nav_mode → visual: resuming direct ArUco visual servo'
                )
            self._nav_mode = mode

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _control_loop(self) -> None:
        now = self.get_clock().now()

        # When goal_manager has activated planned navigation, go completely silent.
        # Do NOT publish zeros — let the twist_mux teleop-timeout expire naturally
        # so path_follower's autonomous slot wins the bus.
        if self._nav_mode != 'visual':
            return

        # No target ever received.
        if self._target_pose is None or self._last_target_time is None:
            if self._had_target:
                self.get_logger().warn('ArUco target lost, stopping robot')
                self._had_target = False
            self._in_recovery = False
            self._recovery_start_time = None
            self._publish_stop()
            return

        age_s = (now - self._last_target_time).nanoseconds * 1e-9

        # --- Target is fresh: live or KF-predicted follow ---
        if age_s <= self._timeout_s:
            if self._in_recovery:
                self.get_logger().info('ArUco target reacquired — resuming normal follow')
                self._in_recovery = False
                self._recovery_start_time = None

            pose  = self._target_pose.pose
            depth = float(pose.position.z)   # camera-frame forward distance to marker

            # Hard minimum approach distance: stop unconditionally when too
            # close regardless of standoff setting or controller output.
            # This prevents physical collision when the robot overshoots,
            # or when planned-nav hands off to visual servo at close range.
            if depth < self._min_approach_dist:
                self._publish_stop()
                return

            lateral_error  = float(pose.position.x)
            distance_error = depth - self._desired_standoff

            # Remember lateral error for recovery mode.
            self._last_lateral_error = lateral_error

            # Scale aggressiveness based on tracker confidence:
            #   'measured'  — live detection, full speed and normal deadbands
            #   'predicted' — KF coasting, reduced speed and relaxed deadbands to
            #                 avoid over-correcting on prediction drift
            if self._tracker_status == 'predicted':
                lin_scale = self._predicted_speed_scale
                lat_db   = self._lateral_deadband  * self._predicted_deadband_scale
                dist_db  = self._distance_deadband * self._predicted_deadband_scale
            else:
                lin_scale = 1.0
                lat_db   = self._lateral_deadband
                dist_db  = self._distance_deadband

            angular_cmd = -self._angular_kp * lateral_error
            angular_cmd = _clamp(angular_cmd, -self._max_angular_speed, self._max_angular_speed)
            if abs(lateral_error) < lat_db:
                angular_cmd = 0.0

            linear_cmd = self._linear_kp * distance_error
            linear_cmd = _clamp(
                linear_cmd,
                -self._max_reverse_speed,
                self._max_linear_speed * lin_scale,
            )
            if abs(distance_error) < dist_db:
                linear_cmd = 0.0

            # If the marker is far off-centre, rotate first and reduce forward motion.
            if self._align_before_drive > 1e-6:
                alignment_scale = 1.0 - min(abs(lateral_error) / self._align_before_drive, 1.0)
                if linear_cmd > 0.0:
                    linear_cmd *= max(0.0, alignment_scale)

            self._publish_cmd(linear_cmd, angular_cmd)
            return

        # --- Target timed out ---

        # Recovery disabled: original stop-immediately behavior.
        if not self._enable_recovery:
            if self._had_target:
                self.get_logger().warn('ArUco target timed out, stopping robot')
                self._had_target = False
            self._publish_stop()
            return

        # Recovery enabled: enter or continue recovery mode.
        if not self._in_recovery:
            self._in_recovery = True
            self._recovery_start_time = now
            self.get_logger().warn(
                f'ArUco target lost — entering recovery (window={self._recovery_window_s:.1f}s), '
                f'last lateral_err={self._last_lateral_error:.3f}m '
                f'tracker_status={self._tracker_status}'
            )

        recovery_age_s = (now - self._recovery_start_time).nanoseconds * 1e-9
        if recovery_age_s <= self._recovery_window_s:
            # Always creep forward so the robot moves toward where the leader
            # was last seen rather than spinning in place.
            # Apply only a gentle angular nudge in the last-known direction —
            # do NOT gate forward motion on lateral error here, because that
            # last_lateral_error was measured while the robot was oscillating
            # and is not a reliable indicator of where the target actually is.
            if abs(self._last_lateral_error) > 1e-3:
                direction = -1.0 if self._last_lateral_error > 0.0 else 1.0
            else:
                direction = 0.0
            angular_recovery = direction * self._recovery_angular_speed * 0.5
            self._publish_cmd(self._recovery_linear_speed, angular_recovery)
            return

        # Recovery window expired: stop and reset.
        if self._had_target:
            self.get_logger().warn('ArUco target fully lost — recovery expired, stopping robot')
            self._had_target = False
        self._in_recovery = False
        self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
