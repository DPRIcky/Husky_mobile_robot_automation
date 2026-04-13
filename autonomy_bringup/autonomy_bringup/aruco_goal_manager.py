#!/usr/bin/env python3
"""
aruco_goal_manager — Bridge between ArUco tracker and obstacle-aware path planner.

Decouples tracker prediction timeout from navigation timeout so Robot 1 can
continue pursuing the last known ArUco goal long after the KF prediction has
expired.

State machine
-------------
VISUAL_FOLLOW  ArUco marker is "measured" (live detections).
               aruco_follower has command authority.
               nav_mode → "visual"

PLANNED_NAV    Marker left view ("predicted" or "lost") but nav_timeout_s not elapsed.
               Last odom position transformed to map frame and frozen as goal.
               Goal published to planner; path_follower drives the robot.
               aruco_follower goes silent (no cmd_vel published).
               nav_mode → "planned"

NAV_STALE      nav_timeout_s elapsed without ArUco reacquisition, or robot had
               no odom pose / TF when marker was lost.
               All motion stopped; waiting for next visual detection.
               nav_mode → "idle"

Transitions
-----------
  VISUAL  → PLANNED   tracker status changes from "measured" to "predicted"/"lost"
                       AND a valid odom pose + map TF are available.
  VISUAL  → IDLE      tracker loses marker but TF/pose unavailable (graceful degradation).
  PLANNED → VISUAL    tracker status returns to "measured".
  PLANNED → STALE     nav_timeout_s elapsed since entering PLANNED.
  STALE   → VISUAL    tracker status returns to "measured".

Command arbitration
-------------------
aruco_follower publishes to  /a300_00000/aruco_follower/cmd_vel  (high-priority "teleop" slot).
path_follower  publishes to  /a300_00000/aruco_autonomous/cmd_vel (low-priority "autonomous" slot).
twist_mux routes aruco_follower when it is active; after mux_teleop_timeout_s of silence
it automatically falls through to path_follower — no explicit locking needed.

When nav_mode = "planned" the aruco_follower publishes nothing, the mux timeout fires,
and path_follower takes the bus.  When ArUco is reacquired aruco_follower immediately
resumes and the mux re-elevates it (high-priority teleop always wins).

Topics published
----------------
  navigation_goal_topic     PoseStamped  (map frame) → trajectory_planner_pkg/planner_node
  nav_mode_topic            String       "visual" | "planned" | "idle"
  goal_marker_topic         Marker       green cylinder at frozen goal position (RViz)

Parameters
----------
  tracker_status_topic      String status from aruco_tracker        (default ~/aruco_tracker/status)
  tracked_pose_odom_topic   PoseStamped in odom from aruco_tracker  (default ~/aruco_tracker/tracked_pose_odom)
  navigation_goal_topic     Goal published to planner               (default ~/aruco_navigation_goal)
  nav_mode_topic            Mode signal for aruco_follower          (default ~/aruco_goal_manager/nav_mode)
  goal_marker_topic         RViz marker topic                       (default ~/aruco_goal_manager/goal_marker)
  map_frame                 Global frame for planner                (default map)
  odom_frame                Robot 1 odom frame                      (default odom)
  nav_timeout_s             Max time to pursue frozen goal          (default 30.0)
  goal_replan_s             How often to republish goal for replanning (default 10.0)
  update_rate_hz            State machine tick rate                 (default 5.0)
"""

from __future__ import annotations

import math
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


# ---------------------------------------------------------------------------
# Quaternion / transform helpers (inline — avoids cross-module dependencies)
# ---------------------------------------------------------------------------

def _rotate_by_quat(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate 3-D vector p by unit quaternion q = [qx, qy, qz, qw]."""
    qv = q[:3]
    t  = 2.0 * np.cross(qv, p)
    return p + q[3] * t + np.cross(qv, t)


def _qmul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two unit quaternions [qx, qy, qz, qw]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=float)


def _apply_transform(T, px: float, py: float, pz: float,
                     qx: float, qy: float, qz: float, qw: float
                     ) -> tuple[float, float, float, float, float, float, float]:
    """
    Apply a geometry_msgs TransformStamped T to a position + orientation.
    Returns (px', py', pz', qx', qy', qz', qw') in the target frame.
    """
    tr    = T.transform.translation
    ro    = T.transform.rotation
    q_tf  = np.array([ro.x,  ro.y,  ro.z,  ro.w],  dtype=float)
    t_tf  = np.array([tr.x,  tr.y,  tr.z],           dtype=float)
    p_src = np.array([px,    py,    pz],              dtype=float)
    p_dst = _rotate_by_quat(p_src, q_tf) + t_tf
    q_src = np.array([qx,    qy,    qz,    qw],       dtype=float)
    q_dst = _qmul(q_tf, q_src)
    return (float(p_dst[0]), float(p_dst[1]), float(p_dst[2]),
            float(q_dst[0]), float(q_dst[1]), float(q_dst[2]), float(q_dst[3]))


# ---------------------------------------------------------------------------
# Goal manager node
# ---------------------------------------------------------------------------

class ArucoGoalManager(Node):
    """
    State-machine node that converts the ArUco tracker's predicted odom pose
    into an obstacle-aware navigation goal for the trajectory planner.
    """

    _VISUAL  = 'visual'
    _PLANNED = 'planned'
    _IDLE    = 'idle'

    def __init__(self) -> None:
        super().__init__('aruco_goal_manager')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('tracker_status_topic',
                               '/a300_00000/aruco_tracker/status')
        self.declare_parameter('tracked_pose_odom_topic',
                               '/a300_00000/aruco_tracker/tracked_pose_odom')
        self.declare_parameter('navigation_goal_topic',
                               '/a300_00000/aruco_navigation_goal')
        self.declare_parameter('nav_mode_topic',
                               '/a300_00000/aruco_goal_manager/nav_mode')
        self.declare_parameter('goal_marker_topic',
                               '/a300_00000/aruco_goal_manager/goal_marker')
        self.declare_parameter('map_frame',      'map')
        self.declare_parameter('odom_frame',     'odom')
        self.declare_parameter('nav_timeout_s',  30.0)
        self.declare_parameter('goal_replan_s',  10.0)
        self.declare_parameter('update_rate_hz',  5.0)

        status_topic    = self.get_parameter('tracker_status_topic').value
        odom_pose_topic = self.get_parameter('tracked_pose_odom_topic').value
        goal_topic      = self.get_parameter('navigation_goal_topic').value
        nav_mode_topic  = self.get_parameter('nav_mode_topic').value
        marker_topic    = self.get_parameter('goal_marker_topic').value
        self._map_frame   = str(self.get_parameter('map_frame').value)
        self._odom_frame  = str(self.get_parameter('odom_frame').value)
        self._nav_timeout = float(self.get_parameter('nav_timeout_s').value)
        self._goal_replan = float(self.get_parameter('goal_replan_s').value)
        update_rate       = float(self.get_parameter('update_rate_hz').value)

        # ── State ────────────────────────────────────────────────────────
        self._state: str                          = self._VISUAL
        self._tracker_status: str                 = 'measured'
        self._last_odom_pose: PoseStamped | None  = None
        self._frozen_goal_map: PoseStamped | None = None
        self._nav_start_time: Time | None         = None
        self._last_goal_pub_time: Time | None     = None

        # Guard: only enter PLANNED after we have seen at least one genuine
        # "measured" detection since startup.  This prevents the goal_manager
        # from entering PLANNED mode at boot (before the follower even starts)
        # just because the tracker coasted through a brief gap during init.
        self._had_measured: bool = False

        # Reacquisition hysteresis: require tracker to be "measured" for this
        # many consecutive update ticks before switching PLANNED → VISUAL.
        # With 5 Hz update rate, 3 ticks = 600 ms.  Avoids VISUAL↔PLANNED
        # flapping with a 1 Hz Gazebo camera (tracker briefly "predicted" between frames).
        self._reacq_ticks_needed: int = 3
        self._reacq_ticks: int        = 0

        # ── TF ───────────────────────────────────────────────────────────
        # NOTE: the launch file remaps /tf → /a300_00000/tf so this buffer
        # sees only Robot 1's TF tree (map → odom → base_link from SLAM).
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_wait     = Duration(seconds=0.15)

        # ── Publishers ───────────────────────────────────────────────────
        self._pub_goal     = self.create_publisher(PoseStamped, goal_topic,     10)
        self._pub_nav_mode = self.create_publisher(String,      nav_mode_topic, 10)
        self._pub_marker   = self.create_publisher(Marker,      marker_topic,   10)

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(String,      status_topic,    self._status_cb,    10)
        self.create_subscription(PoseStamped, odom_pose_topic, self._odom_pose_cb, 10)

        self.create_timer(1.0 / update_rate, self._update)

        self.get_logger().info(
            f'aruco_goal_manager ready  '
            f'nav_timeout={self._nav_timeout:.1f}s  '
            f'goal_replan={self._goal_replan:.1f}s  '
            f'map={self._map_frame}  odom={self._odom_frame}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _status_cb(self, msg: String) -> None:
        self._tracker_status = msg.data

    def _odom_pose_cb(self, msg: PoseStamped) -> None:
        # Always keep the latest: used to freeze goal on VISUAL→PLANNED
        self._last_odom_pose = msg

    # ------------------------------------------------------------------
    # TF helper
    # ------------------------------------------------------------------

    def _odom_to_map(self, pose: PoseStamped) -> PoseStamped | None:
        """
        Transform a PoseStamped from odom frame to map frame.
        Returns None if the SLAM-provided map→odom TF is not yet available.
        """
        try:
            T = self._tf_buffer.lookup_transform(
                self._map_frame, self._odom_frame,
                Time(), timeout=self._tf_wait)
        except Exception as e:
            self.get_logger().warn(
                f'TF {self._odom_frame}→{self._map_frame} unavailable: {e}',
                throttle_duration_sec=5.0)
            return None

        px, py, pz, qx, qy, qz, qw = _apply_transform(
            T,
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        )

        result = PoseStamped()
        result.header.stamp    = self.get_clock().now().to_msg()
        result.header.frame_id = self._map_frame
        result.pose.position.x = px
        result.pose.position.y = py
        result.pose.position.z = pz
        result.pose.orientation.x = qx
        result.pose.orientation.y = qy
        result.pose.orientation.z = qz
        result.pose.orientation.w = qw
        return result

    # ------------------------------------------------------------------
    # Main update loop
    # ------------------------------------------------------------------

    def _update(self) -> None:  # noqa: C901
        now    = self.get_clock().now()
        status = self._tracker_status

        # ── State transitions ──────────────────────────────────────────
        if self._state == self._VISUAL:
            # Track whether we have ever had a genuine live detection.
            if status == 'measured':
                if not self._had_measured:
                    self._had_measured = True
                    self.get_logger().info('[goal_manager] First ArUco "measured" — PLANNED_NAV now armed')
                self._reacq_ticks = 0  # reset hysteresis counter while measured

            if status in ('predicted', 'lost'):
                if not self._had_measured:
                    # Never had a live sighting — refuse to enter planned mode.
                    # The follower will handle recovery on its own.
                    self.get_logger().warn(
                        f'[goal_manager] tracker={status} but never had "measured" '
                        'since startup — staying in VISUAL_FOLLOW',
                        throttle_duration_sec=5.0,
                    )
                elif self._last_odom_pose is None:
                    self.get_logger().warn(
                        f'[goal_manager] tracker={status} but no odom pose received yet; '
                        'staying in VISUAL_FOLLOW (aruco_follower keeps recovery)',
                        throttle_duration_sec=3.0,
                    )
                else:
                    goal_map = self._odom_to_map(self._last_odom_pose)
                    if goal_map is not None:
                        self._frozen_goal_map    = goal_map
                        self._nav_start_time     = now
                        self._last_goal_pub_time = None   # force immediate publish
                        self._reacq_ticks        = 0
                        self._state = self._PLANNED
                        self.get_logger().info(
                            f'[goal_manager] tracker={status} → PLANNED_NAV  '
                            f'frozen_goal=({goal_map.pose.position.x:.2f}, '
                            f'{goal_map.pose.position.y:.2f}) [{self._map_frame}]  '
                            f'nav_timeout={self._nav_timeout:.1f}s'
                        )
                    else:
                        # SLAM not yet initialised — degrade gracefully.
                        self.get_logger().warn(
                            f'[goal_manager] tracker={status} but odom→map TF unavailable; '
                            'staying in VISUAL_FOLLOW — no obstacle-aware nav yet',
                            throttle_duration_sec=3.0,
                        )

        elif self._state == self._PLANNED:
            if status == 'measured':
                # Require sustained "measured" for a few ticks before switching
                # back to visual — prevents VISUAL↔PLANNED oscillation with a
                # slow (1 Hz) camera where the tracker briefly dips to "predicted"
                # between detection frames.
                self._reacq_ticks += 1
                if self._reacq_ticks >= self._reacq_ticks_needed:
                    self._frozen_goal_map    = None
                    self._last_goal_pub_time = None
                    self._reacq_ticks        = 0
                    self._state = self._VISUAL
                    self.get_logger().info(
                        f'[goal_manager] ArUco stably reacquired ({self._reacq_ticks_needed} '
                        'ticks) → VISUAL_FOLLOW  (aruco_follower resumes visual servo)'
                    )
            else:
                self._reacq_ticks = 0  # reset on any non-measured tick
                # Check whether nav timeout has elapsed.
                elapsed = (now - self._nav_start_time).nanoseconds * 1e-9
                if elapsed >= self._nav_timeout:
                    self._frozen_goal_map    = None
                    self._last_goal_pub_time = None
                    self._state = self._IDLE
                    self.get_logger().warn(
                        f'[goal_manager] Nav timeout {self._nav_timeout:.1f}s elapsed '
                        'without ArUco reacquisition → IDLE  '
                        '(all motion stopped; waiting for visual reacquisition)'
                    )

        elif self._state == self._IDLE:
            if status == 'measured':
                self._reacq_ticks += 1
                if self._reacq_ticks >= self._reacq_ticks_needed:
                    self._had_measured = True
                    self._reacq_ticks  = 0
                    self._state = self._VISUAL
                    self.get_logger().info('[goal_manager] ArUco reacquired from IDLE → VISUAL_FOLLOW')
            else:
                self._reacq_ticks = 0

        # ── Publish nav_mode every tick ────────────────────────────────
        # aruco_follower subscribes here and goes completely silent when
        # mode is "planned" or "idle" so twist_mux falls through to path_follower.
        mode_msg = String()
        mode_msg.data = self._state   # 'visual' / 'planned' / 'idle'
        self._pub_nav_mode.publish(mode_msg)

        # ── Publish goal and RViz marker while in PLANNED state ────────
        if self._state == self._PLANNED and self._frozen_goal_map is not None:
            # Republish the frozen goal periodically so the planner can
            # replan after map updates (matches planner's replan_goal_repeat_s).
            needs_pub = (
                self._last_goal_pub_time is None
                or (now - self._last_goal_pub_time).nanoseconds * 1e-9 >= self._goal_replan
            )
            if needs_pub:
                self._frozen_goal_map.header.stamp = now.to_msg()
                self._pub_goal.publish(self._frozen_goal_map)
                if self._last_goal_pub_time is not None:
                    elapsed = (now - self._nav_start_time).nanoseconds * 1e-9
                    self.get_logger().debug(
                        f'[goal_manager] Republished frozen goal at t+{elapsed:.0f}s '
                        f'({self._frozen_goal_map.pose.position.x:.2f}, '
                        f'{self._frozen_goal_map.pose.position.y:.2f})'
                    )
                self._last_goal_pub_time = now

            # Always publish the marker so the cylinder stays visible in RViz.
            self._publish_goal_marker(self._frozen_goal_map, now)

    # ------------------------------------------------------------------
    # RViz marker
    # ------------------------------------------------------------------

    def _publish_goal_marker(self, goal: PoseStamped, now: Time) -> None:
        """Green cylinder at the frozen goal position — visible in the RViz map view."""
        m = Marker()
        m.header.stamp    = now.to_msg()
        m.header.frame_id = self._map_frame
        m.ns     = 'aruco_nav_goal'
        m.id     = 0
        m.type   = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x    = goal.pose.position.x
        m.pose.position.y    = goal.pose.position.y
        m.pose.position.z    = 0.5       # raise so it's visible above floor
        m.pose.orientation.w = 1.0
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 1.0
        m.color.r = 0.0
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 0.85
        m.lifetime.sec     = 0
        m.lifetime.nanosec = 600_000_000  # 600 ms auto-expire keeps marker fresh
        self._pub_marker.publish(m)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoGoalManager()
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
