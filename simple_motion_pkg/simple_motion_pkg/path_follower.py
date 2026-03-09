"""
Path-following node with obstacle-triggered replanning.

Normal operation: P-controller + lookahead on /planned_path.

On obstacle detection:
  - Stop immediately.
  - Wait map_update_delay_s for SLAM to update the map with the new obstacle.
  - Publish the current goal to /goal_pose so the planner rereplans around it.
  - Remain stopped until a NEW path arrives.
  - If still blocked after replan_retry_s, request another replan.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan

import tf2_ros


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ---- Parameters ----
        self.declare_parameter('path_topic',   '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/a300_00000/cmd_vel')
        self.declare_parameter('goal_topic',   '/goal_pose')
        self.declare_parameter('scan_topic',   '/a300_00000/sensors/lidar2d_0/scan')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_vel',  0.4)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('kp_linear',  0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('control_rate',   10.0)
        self.declare_parameter('obstacle_warn_dist',   0.5)   # slow down (m)
        self.declare_parameter('obstacle_stop_dist',   0.25)  # stop + replan (m)
        self.declare_parameter('obstacle_check_angle', 0.436) # ±25 deg cone (rad)
        self.declare_parameter('map_update_delay_s',   0.8)   # wait before first replan
        self.declare_parameter('replan_retry_s',       3.0)   # retry if still blocked
        self.declare_parameter('stuck_check_interval_s',  4.0)   # how often to check progress
        self.declare_parameter('stuck_dist_threshold_m',  0.15)  # min movement to not be stuck
        self.declare_parameter('off_path_dist_m',         1.5)   # replan if robot is this far from path

        path_topic    = self.get_parameter('path_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        goal_topic    = self.get_parameter('goal_topic').value
        scan_topic    = self.get_parameter('scan_topic').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.lookahead    = self.get_parameter('lookahead_distance').value
        self.max_v    = self.get_parameter('max_linear_vel').value
        self.max_w    = self.get_parameter('max_angular_vel').value
        self.kp_v     = self.get_parameter('kp_linear').value
        self.kp_w     = self.get_parameter('kp_angular').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        rate          = self.get_parameter('control_rate').value
        self.warn_dist      = self.get_parameter('obstacle_warn_dist').value
        self.stop_dist      = self.get_parameter('obstacle_stop_dist').value
        self.check_angle    = self.get_parameter('obstacle_check_angle').value
        self.map_upd_delay  = self.get_parameter('map_update_delay_s').value
        self.replan_retry   = self.get_parameter('replan_retry_s').value
        self.stuck_check_interval = self.get_parameter('stuck_check_interval_s').value
        self.stuck_dist_threshold = self.get_parameter('stuck_dist_threshold_m').value
        self.off_path_dist       = self.get_parameter('off_path_dist_m').value

        self.get_logger().info(
            f'PathFollower ready — scan={scan_topic}, '
            f'stop={self.stop_dist}m, warn={self.warn_dist}m')

        # ---- State ----
        self._path: list = []
        self._path_idx: int = 0
        self._active: bool = False
        self._current_goal = None          # (gx, gy) — stored for replanning
        self._latest_scan: LaserScan | None = None

        # Replanning state machine
        self._blocked_since = None         # when obstacle was first seen
        self._replan_requested: bool = False   # have we asked for a replan?
        self._last_replan_time = None      # when we last published a replan request

        # Stuck detection
        self._stuck_check_pose = None
        self._stuck_check_time = None

        # Cached yaw offset from laser frame → base_link (computed once from TF)
        self._laser_yaw_offset: float | None = None
        self._laser_frame: str | None = None

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- Sub / Pub ----
        self.create_subscription(Path,      path_topic, self._path_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self._cmd_pub  = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped,  goal_topic, 10)

        self._timer = self.create_timer(1.0 / rate, self._control_loop)

    # ------------------------------------------------------------------

    def _path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('Received path with < 2 poses, ignoring.')
            return
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_idx = 0
        self._active = True
        self._current_goal = self._path[-1]
        # New path received — clear blocked and stuck state, resume
        self._blocked_since = None
        self._replan_requested = False
        self._last_replan_time = None
        self._stuck_check_pose = None
        self._stuck_check_time = None
        self.get_logger().info(f'New path: {len(self._path)} waypoints — resuming.')

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg
        # Cache the laser frame name from the first message
        if self._laser_frame is None:
            self._laser_frame = msg.header.frame_id

    # ------------------------------------------------------------------

    def _get_laser_yaw_offset(self) -> float:
        """
        Return the yaw angle from the laser frame to base_link (cached).
        This corrects for the laser being mounted at any rotation on the robot.
        """
        if self._laser_yaw_offset is not None:
            return self._laser_yaw_offset
        if self._laser_frame is None:
            return 0.0
        try:
            t = self._tf_buffer.lookup_transform(
                self.base_frame, self._laser_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            q = t.transform.rotation
            siny  = 2.0 * (q.w * q.z + q.x * q.y)
            cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._laser_yaw_offset = math.atan2(siny, cosy)
            self.get_logger().info(
                f'Laser→base_link yaw offset: '
                f'{math.degrees(self._laser_yaw_offset):.1f}°')
            return self._laser_yaw_offset
        except Exception as e:
            self.get_logger().warn(
                f'Cannot get laser TF ({self._laser_frame}→{self.base_frame}): {e}',
                throttle_duration_sec=5.0)
            return 0.0

    def _scan_min_in_cone(self):
        """
        Return (forward_min, warn_triggered) using scan angles corrected
        for the laser's mounting rotation relative to base_link.
        """
        scan = self._latest_scan
        if scan is None:
            return float('inf'), False

        yaw_offset = self._get_laser_yaw_offset()
        forward_min = float('inf')

        for i, r in enumerate(scan.ranges):
            # Filter out invalid readings (including below sensor minimum range)
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            # Angle in laser frame → corrected to base_link frame
            angle_laser = scan.angle_min + i * scan.angle_increment
            angle_base  = self._normalize_angle(angle_laser + yaw_offset)
            if abs(angle_base) > self.check_angle:
                continue
            forward_min = min(forward_min, r)

        return forward_min, forward_min < self.warn_dist

    # ------------------------------------------------------------------

    def _control_loop(self):
        if not self._active or not self._path:
            return

        forward_min, in_warn_zone = self._scan_min_in_cone()
        in_danger = forward_min < self.stop_dist

        # ---- BLOCKED: obstacle in danger zone ----
        if in_danger:
            self._stop()
            now = self.get_clock().now()

            if self._blocked_since is None:
                # First detection — start timer, wait for SLAM to update map
                self._blocked_since = now
                self.get_logger().warn(
                    f'Obstacle at {forward_min:.2f}m — stopped, '
                    f'waiting {self.map_upd_delay:.1f}s for map update.')
                return

            blocked_sec = (now - self._blocked_since).nanoseconds * 1e-9

            # After map_update_delay, send first replan request
            if not self._replan_requested and blocked_sec >= self.map_upd_delay:
                self._publish_replan()
                self._replan_requested = True
                self._last_replan_time = now
                return

            # If still blocked after retry interval, request again
            if self._replan_requested and self._last_replan_time is not None:
                since_last = (now - self._last_replan_time).nanoseconds * 1e-9
                if since_last >= self.replan_retry:
                    self.get_logger().warn(
                        f'Still blocked after {since_last:.1f}s — retrying replan.')
                    self._publish_replan()
                    self._last_replan_time = now
            return

        # ---- CLEAR: obstacle gone or never there ----
        if self._blocked_since is not None:
            self.get_logger().info('Path clear — resuming without replan.')
            self._blocked_since = None
            self._replan_requested = False
            self._last_replan_time = None

        # ---- NORMAL PATH FOLLOWING ----
        pose = self._get_robot_pose()
        if pose is None:
            self._stop()
            return
        rx, ry, ryaw = pose

        # ---- STUCK DETECTION ----
        now = self.get_clock().now()
        if self._stuck_check_time is None:
            self._stuck_check_pose = (rx, ry)
            self._stuck_check_time = now
        else:
            elapsed = (now - self._stuck_check_time).nanoseconds * 1e-9
            if elapsed >= self.stuck_check_interval:
                moved = math.hypot(rx - self._stuck_check_pose[0],
                                   ry - self._stuck_check_pose[1])
                if moved < self.stuck_dist_threshold and self._current_goal:
                    self.get_logger().warn(
                        f'Stuck: moved only {moved:.2f}m in {elapsed:.1f}s — replanning.')
                    self._publish_replan()
                self._stuck_check_pose = (rx, ry)
                self._stuck_check_time = now

        # ---- OFF-PATH DETECTION ----
        # Find minimum distance from robot to any point on the current path
        min_path_dist = min(math.hypot(px - rx, py - ry) for px, py in self._path)
        if min_path_dist > self.off_path_dist and self._current_goal:
            self.get_logger().warn(
                f'Off-path: {min_path_dist:.2f}m from path (limit {self.off_path_dist}m) — replanning.')
            self._publish_replan()
            self._stop()
            return

        # Advance path index past points within lookahead
        while self._path_idx < len(self._path) - 1:
            px, py = self._path[self._path_idx]
            if math.hypot(px - rx, py - ry) > self.lookahead:
                break
            self._path_idx += 1

        tx, ty = self._path[self._path_idx]
        dist = math.hypot(tx - rx, ty - ry)

        gx, gy = self._path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tol:
            self._stop()
            self.get_logger().info('Goal reached!')
            self._active = False
            return

        desired_yaw = math.atan2(ty - ry, tx - rx)
        yaw_err = self._normalize_angle(desired_yaw - ryaw)

        v = min(self.kp_v * dist, self.max_v)
        w = max(-self.max_w, min(self.kp_w * yaw_err, self.max_w))

        if abs(yaw_err) > math.pi / 4:
            v *= 0.3

        # Reduce speed in warning zone
        if in_warn_zone:
            v *= 0.5

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x  = v
        cmd.twist.angular.z = w
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------

    def _publish_replan(self):
        if self._current_goal is None:
            return
        gx, gy = self._current_goal
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)
        self.get_logger().info(f'Replan requested → goal ({gx:.2f}, {gy:.2f})')

    def _get_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return (x, y, math.atan2(siny, cosy))
        except Exception as e:
            self.get_logger().warn(
                f'TF unavailable: {e}', throttle_duration_sec=5.0)
            return None

    def _stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _normalize_angle(a):
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
