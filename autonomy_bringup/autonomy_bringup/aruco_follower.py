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
import rclpy
from rclpy.node import Node


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


class ArucoFollower(Node):
    def __init__(self) -> None:
        super().__init__('aruco_follower')

        self.declare_parameter('target_pose_topic', '/a300_00000/aruco_detector/target_pose')
        self.declare_parameter('cmd_vel_topic', '/a300_00000/cmd_vel')
        self.declare_parameter('base_frame', 'a300_00000/base_link')
        self.declare_parameter('control_rate_hz', 15.0)
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('desired_standoff_m', 1.5)
        self.declare_parameter('distance_deadband_m', 0.08)
        self.declare_parameter('lateral_deadband_m', 0.03)
        self.declare_parameter('align_before_drive_m', 0.25)
        self.declare_parameter('linear_kp', 0.9)
        self.declare_parameter('angular_kp', 2.8)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_reverse_speed', 0.20)
        self.declare_parameter('max_angular_speed', 1.2)

        self._target_pose: PoseStamped | None = None
        self._last_target_time = None
        self._had_target = False

        target_pose_topic = self.get_parameter('target_pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._base_frame = self.get_parameter('base_frame').value
        self._timeout_s = float(self.get_parameter('target_timeout_s').value)
        self._desired_standoff = float(self.get_parameter('desired_standoff_m').value)
        self._distance_deadband = float(self.get_parameter('distance_deadband_m').value)
        self._lateral_deadband = float(self.get_parameter('lateral_deadband_m').value)
        self._align_before_drive = float(self.get_parameter('align_before_drive_m').value)
        self._linear_kp = float(self.get_parameter('linear_kp').value)
        self._angular_kp = float(self.get_parameter('angular_kp').value)
        self._max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self._max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self._max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.create_subscription(PoseStamped, target_pose_topic, self._target_pose_cb, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'Aruco follower ready target={target_pose_topic} cmd_vel={cmd_vel_topic} '
            f'standoff={self._desired_standoff:.2f}m'
        )

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        self._target_pose = msg
        self._last_target_time = self.get_clock().now()
        if not self._had_target:
            self.get_logger().info('ArUco target acquired')
            self._had_target = True

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
        if self._target_pose is None or self._last_target_time is None:
            if self._had_target:
                self.get_logger().warn('ArUco target lost, stopping robot')
                self._had_target = False
            self._publish_stop()
            return

        age_s = (self.get_clock().now() - self._last_target_time).nanoseconds * 1e-9
        if age_s > self._timeout_s:
            if self._had_target:
                self.get_logger().warn('ArUco target timed out, stopping robot')
                self._had_target = False
            self._publish_stop()
            return

        pose = self._target_pose.pose
        lateral_error = float(pose.position.x)
        distance_error = float(pose.position.z) - self._desired_standoff

        angular_cmd = -self._angular_kp * lateral_error
        angular_cmd = _clamp(angular_cmd, -self._max_angular_speed, self._max_angular_speed)
        if abs(lateral_error) < self._lateral_deadband:
            angular_cmd = 0.0

        linear_cmd = self._linear_kp * distance_error
        linear_cmd = _clamp(linear_cmd, -self._max_reverse_speed, self._max_linear_speed)
        if abs(distance_error) < self._distance_deadband:
            linear_cmd = 0.0

        # If the marker is far off-center, rotate first and reduce forward motion.
        if self._align_before_drive > 1e-6:
            alignment_scale = 1.0 - min(abs(lateral_error) / self._align_before_drive, 1.0)
            if linear_cmd > 0.0:
                linear_cmd *= max(0.0, alignment_scale)

        self._publish_cmd(linear_cmd, angular_cmd)


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
