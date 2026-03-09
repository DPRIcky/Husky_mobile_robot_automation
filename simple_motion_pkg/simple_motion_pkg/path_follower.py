"""
Minimal path-following node: subscribes to /planned_path, uses TF for current
pose, publishes cmd_vel with a simple P-controller + lookahead.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, TwistStamped

import tf2_ros


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ---- Parameters ----
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/a300_00000/cmd_vel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('lookahead_distance', 0.5)   # metres
        self.declare_parameter('max_linear_vel', 0.4)       # m/s
        self.declare_parameter('max_angular_vel', 1.0)      # rad/s
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('goal_tolerance', 0.25)       # metres
        self.declare_parameter('control_rate', 10.0)         # Hz

        path_topic = self.get_parameter('path_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.kp_v = self.get_parameter('kp_linear').value
        self.kp_w = self.get_parameter('kp_angular').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        rate = self.get_parameter('control_rate').value

        self.get_logger().info(
            f'PathFollower: path={path_topic}, cmd_vel={cmd_vel_topic}, '
            f'lookahead={self.lookahead}m, max_v={self.max_v}m/s')

        # ---- State ----
        self._path: list = []
        self._path_idx: int = 0
        self._active: bool = False

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- Sub / Pub ----
        self.create_subscription(Path, path_topic, self._path_cb, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        # ---- Control loop timer ----
        self._timer = self.create_timer(1.0 / rate, self._control_loop)

    # ------------------------------------------------------------------

    def _path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('Received path with < 2 poses, ignoring.')
            return
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_idx = 0
        self._active = True
        self.get_logger().info(f'New path received: {len(self._path)} waypoints')

    def _control_loop(self):
        if not self._active or not self._path:
            return

        pose = self._get_robot_pose()
        if pose is None:
            # No valid TF — stop the robot, don't drive blind
            self._stop()
            return
        rx, ry, ryaw = pose

        # Advance path index past points within lookahead
        while self._path_idx < len(self._path) - 1:
            px, py = self._path[self._path_idx]
            if math.hypot(px - rx, py - ry) > self.lookahead:
                break
            self._path_idx += 1

        tx, ty = self._path[self._path_idx]
        dist = math.hypot(tx - rx, ty - ry)

        # Check if we reached the final goal
        gx, gy = self._path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tol:
            self._stop()
            self.get_logger().info('Goal reached!')
            self._active = False
            return

        # Compute heading error
        desired_yaw = math.atan2(ty - ry, tx - rx)
        yaw_err = self._normalize_angle(desired_yaw - ryaw)

        # Proportional control with saturation
        v = min(self.kp_v * dist, self.max_v)
        w = max(-self.max_w, min(self.kp_w * yaw_err, self.max_w))

        # Slow down linear speed when heading error is large
        if abs(yaw_err) > math.pi / 4:
            v *= 0.3

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------

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
            yaw = math.atan2(siny, cosy)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(
                f'TF {self.global_frame}→{self.base_frame} unavailable: {e}',
                throttle_duration_sec=5.0)
            return None

    def _stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
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
