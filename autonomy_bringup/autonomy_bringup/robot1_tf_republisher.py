#!/usr/bin/env python3
"""
robot1_tf_republisher — TF bridge for Robot 1 (a300_00000) in the dual-robot demo.

Problem
-------
Clearpath A300 robots publish non-namespaced TF frame names (base_link, odom,
chassis_link, …) to their own /a300_000X/tf topics.  RViz is remapped to
/a300_00001/tf (Robot 2's tree), so Robot 1's frames never reach RViz and it
remains invisible.

Solution
--------
Subscribe to /a300_00000/tf and /a300_00000/tf_static, prefix every frame ID
with "robot1/", and republish directly onto /a300_00001/tf and
/a300_00001/tf_static so that RViz sees both robots' frames in one tree.

Frame mapping examples
----------------------
  a300_00000 raw          → published into Robot 2 tree
  base_link               → robot1/base_link
  odom                    → robot1/odom
  chassis_link            → robot1/chassis_link
  lidar2d_0_link          → robot1/lidar2d_0_link
  camera_0_link           → robot1/camera_0_link

The RViz RobotModel display for Robot 1 is configured with TF Prefix "robot1"
so it resolves these prefixed frames automatically.
"""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

_STATIC_QOS = QoSProfile(
    depth=100,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

PREFIX = 'robot1/'


class Robot1TfRepublisher(Node):
    def __init__(self):
        super().__init__('robot1_tf_republisher')

        # Spawn-pose parameters — must match the Gazebo spawn args so that
        # map → robot1/odom places Robot 1 at the correct world position in RViz.
        # The EKF's odom frame originates at the robot's spawn point (identity
        # odom→base_link at t=0), so this transform carries the spawn offset.
        self.declare_parameter('robot1_initial_x',   0.0)
        self.declare_parameter('robot1_initial_y',   0.0)
        self.declare_parameter('robot1_initial_yaw', 0.0)

        x   = self.get_parameter('robot1_initial_x').value
        y   = self.get_parameter('robot1_initial_y').value
        yaw = self.get_parameter('robot1_initial_yaw').value

        # Dynamic TF — Robot 1 source
        self._sub_tf = self.create_subscription(
            TFMessage, '/a300_00000/tf', self._on_tf, 100)

        # Static TF — Robot 1 source (TRANSIENT_LOCAL so we receive latched msgs)
        self._sub_static = self.create_subscription(
            TFMessage, '/a300_00000/tf_static', self._on_static, _STATIC_QOS)

        # Publish into Robot 2's TF topic (RViz reads this via remapping)
        self._pub_tf = self.create_publisher(TFMessage, '/a300_00001/tf', 100)
        self._pub_static = self.create_publisher(
            TFMessage, '/a300_00001/tf_static', _STATIC_QOS)

        self.get_logger().info(
            f'robot1_tf_republisher: bridging /a300_00000/tf → /a300_00001/tf '
            f'with prefix "{PREFIX}", spawn=({x:.2f}, {y:.2f}, yaw={yaw:.3f} rad)'
        )

        # Anchor Robot 1's TF island into the RViz tree.
        #
        # After prefixing, Robot 1's chain is:
        #   robot1/odom → robot1/base_link → robot1/chassis_link → …
        # RViz fixed frame is "map".  Without a map→robot1/odom edge,
        # the whole chain floats and RViz reports "No transform from …".
        #
        # With enable_ekf=true the EKF resets its odom origin to the robot's
        # spawn position, so odom→base_link starts at identity.  We must
        # publish map→robot1/odom at the spawn pose (not identity) so that
        # Robot 1 appears at the correct world position in RViz.
        #
        # Published to /a300_00001/tf_static (TRANSIENT_LOCAL) so late
        # subscribers (e.g. RViz starting at t=16 s) receive it on join.
        self._publish_map_to_robot1_odom(x, y, yaw)

    def _publish_map_to_robot1_odom(self, x: float, y: float, yaw: float) -> None:
        """Publish static  map → robot1/odom  at the given spawn pose."""
        t = TransformStamped()
        # Timestamp 0 = "always valid" — standard for static TF.
        t.header.stamp.sec = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id = 'map'
        t.child_frame_id  = PREFIX + 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        # yaw-only rotation → quaternion: (0, 0, sin(yaw/2), cos(yaw/2))
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._pub_static.publish(TFMessage(transforms=[t]))
        self.get_logger().info(
            f'Published static TF: map → {PREFIX}odom  '
            f'x={x:.2f} y={y:.2f} yaw={yaw:.3f} rad'
        )

    # ------------------------------------------------------------------
    def _prefix(self, msg: TFMessage) -> TFMessage:
        out = TFMessage()
        for t in msg.transforms:
            t2 = copy.deepcopy(t)
            t2.header.frame_id = PREFIX + t.header.frame_id
            t2.child_frame_id  = PREFIX + t.child_frame_id
            out.transforms.append(t2)
        return out

    def _on_tf(self, msg: TFMessage) -> None:
        self._pub_tf.publish(self._prefix(msg))

    def _on_static(self, msg: TFMessage) -> None:
        self._pub_static.publish(self._prefix(msg))


def main(args=None):
    rclpy.init(args=args)
    node = Robot1TfRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
