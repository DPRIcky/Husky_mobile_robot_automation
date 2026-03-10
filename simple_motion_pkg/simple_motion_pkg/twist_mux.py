"""
Twist multiplexer node.

Arbitrates between autonomous path-follower output and teleop (keyboard /
joystick) commands, publishing the winning command to the robot's cmd_vel.

Priority (highest wins):
  1. E-stop  (/estop, std_msgs/Bool)   — zeroes cmd_vel when True
  2. Teleop  (configurable topic)      — overrides autonomous while active
  3. Autonomous (/autonomous/cmd_vel)  — default when no teleop seen recently

Teleop timeout: if no teleop message arrives within `teleop_timeout_s` seconds
the mux falls back to autonomous.  Set to 0.0 to disable teleop.

Parameters (motion_params.yaml):
    mux_autonomous_topic   default: /autonomous/cmd_vel
    mux_teleop_topic       default: /a300_00000/joy_teleop/cmd_vel
    mux_output_topic       default: /a300_00000/cmd_vel
    mux_teleop_timeout_s   default: 0.5
    mux_rate_hz            default: 20.0
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


def _zero_twist(frame: str, stamp) -> TwistStamped:
    msg = TwistStamped()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame
    return msg


class TwistMux(Node):
    def __init__(self):
        super().__init__('twist_mux')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('mux_autonomous_topic', '/autonomous/cmd_vel')
        self.declare_parameter('mux_teleop_topic',
                               '/a300_00000/joy_teleop/cmd_vel')
        self.declare_parameter('mux_output_topic',     '/a300_00000/cmd_vel')
        self.declare_parameter('mux_teleop_timeout_s', 0.5)
        self.declare_parameter('mux_rate_hz',          20.0)
        self.declare_parameter('base_frame',           'base_link')

        auto_topic    = self.get_parameter('mux_autonomous_topic').value
        teleop_topic  = self.get_parameter('mux_teleop_topic').value
        output_topic  = self.get_parameter('mux_output_topic').value
        self._timeout = self.get_parameter('mux_teleop_timeout_s').value
        rate          = self.get_parameter('mux_rate_hz').value
        self._frame   = self.get_parameter('base_frame').value

        # ── State ─────────────────────────────────────────────────────────
        self._auto_cmd:     TwistStamped | None = None
        self._teleop_cmd:   TwistStamped | None = None
        self._estop:        bool                = False
        self._last_teleop_t                     = None

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(TwistStamped, auto_topic,
                                 self._auto_cb,   10)
        self.create_subscription(TwistStamped, teleop_topic,
                                 self._teleop_cb, 10)
        self.create_subscription(Bool, '/estop',
                                 self._estop_cb,  10)

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(TwistStamped, output_topic, 10)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._mux_loop)

        self.get_logger().info(
            f'TwistMux ready  auto={auto_topic}  teleop={teleop_topic}'
            f'  out={output_topic}  timeout={self._timeout}s')

    # ------------------------------------------------------------------

    def _auto_cb(self, msg: TwistStamped):
        self._auto_cmd = msg

    def _teleop_cb(self, msg: TwistStamped):
        self._teleop_cmd   = msg
        self._last_teleop_t = self.get_clock().now()

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().warn('E-STOP activated — zeroing cmd_vel.')
        elif not msg.data and self._estop:
            self.get_logger().info('E-STOP released.')
        self._estop = msg.data

    # ------------------------------------------------------------------

    def _mux_loop(self):
        stamp = self.get_clock().now().to_msg()

        # Priority 1: E-stop
        if self._estop:
            self._pub.publish(_zero_twist(self._frame, stamp))
            return

        # Priority 2: Teleop (only if recently active)
        teleop_active = False
        if self._timeout > 0.0 and self._last_teleop_t is not None:
            age = (self.get_clock().now() - self._last_teleop_t).nanoseconds * 1e-9
            teleop_active = age < self._timeout

        if teleop_active and self._teleop_cmd is not None:
            self._pub.publish(self._teleop_cmd)
            return

        # Priority 3: Autonomous
        if self._auto_cmd is not None:
            self._pub.publish(self._auto_cmd)


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
