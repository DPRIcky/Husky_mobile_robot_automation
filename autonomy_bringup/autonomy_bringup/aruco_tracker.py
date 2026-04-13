#!/usr/bin/env python3
"""
aruco_tracker — ego-motion-compensated Kalman tracker for ArUco target pose.

Why tracking in the camera frame is wrong
-----------------------------------------
A constant-velocity KF in the camera optical frame assumes that frame is
inertial.  It is not: when Robot 1 rotates, the camera rotates with it and a
stationary target appears to slide laterally even though it has not moved.
The KF misinterprets this as target velocity, predicts the target continuing
in that direction, and the follower keeps turning to track it → orbital spin.

Fix: track in the odom frame (Robot 1's fixed/inertial-ish frame)
-----------------------------------------------------------------
  Measurement path:
    camera-frame detected pose  →  TF  →  odom 2-D position  →  KF update

  Publish path (timer):
    KF predicted odom position  →  TF (using CURRENT Robot 1 pose)  →
    camera-frame tracked_pose / predicted_path

  Because odom is inertial, a stationary target stays at the same odom
  coordinates even as Robot 1 rotates.  The constant-velocity model there is
  physically correct.  Robot 1's ego-motion is accounted for automatically
  by the TF transforms.

State (odom 2-D):   [px, py, vx, vy]
Measurement (odom): [px, py]   (transformed from camera frame on each detection)

Outputs  (all in the camera optical frame so aruco_follower works unchanged)
--------
  tracked_pose      PoseStamped  — current predicted target pose in camera frame
  predicted_path    Path         — short-horizon rollout in camera frame
  status            String       — "measured" | "predicted" | "lost"

Parameters
----------
input_topic            Source PoseStamped (default: ~/aruco_detector/target_pose)
tracked_pose_topic     Output smoothed pose (default: ~/tracked_pose)
predicted_path_topic   Output predicted path (default: ~/predicted_path)
status_topic           Output status string  (default: ~/status)
odom_frame             Fixed frame for KF state (default: odom)
filter_rate_hz         Timer publish rate    (default: 20.0)
fresh_threshold_s      Age → status "measured"  (default: 2.0)
prediction_timeout_s   Age → status "lost"       (default: 4.0)
prediction_horizon_s   Path rollout horizon      (default: 1.5)
prediction_dt_s        Path rollout step size    (default: 0.1)
process_noise          Process noise σ²          (default: 0.5)
measurement_noise      Measurement noise σ²      (default: 0.1)
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
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker


# ---------------------------------------------------------------------------
# Quaternion rotation helper (avoids tf2_geometry_msgs dependency)
# ---------------------------------------------------------------------------

def _rotate_by_quat(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate 3-D vector p by unit quaternion q = [qx, qy, qz, qw]."""
    qv = q[:3]
    t = 2.0 * np.cross(qv, p)
    return p + q[3] * t + np.cross(qv, t)


def _transform_point(T, p: np.ndarray) -> np.ndarray:
    """Apply a geometry_msgs TransformStamped T to a numpy 3-D point."""
    tr = T.transform.translation
    ro = T.transform.rotation
    q = np.array([ro.x, ro.y, ro.z, ro.w], dtype=float)
    t = np.array([tr.x, tr.y, tr.z], dtype=float)
    return _rotate_by_quat(p, q) + t


# ---------------------------------------------------------------------------
# Tracker node
# ---------------------------------------------------------------------------

class ArucoTracker(Node):

    def __init__(self) -> None:
        super().__init__('aruco_tracker')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('input_topic',
                               '/a300_00000/aruco_detector/target_pose')
        self.declare_parameter('tracked_pose_topic',   '~/tracked_pose')
        self.declare_parameter('predicted_path_topic', '~/predicted_path')
        self.declare_parameter('status_topic',         '~/status')
        self.declare_parameter('odom_frame',           'odom')
        self.declare_parameter('filter_rate_hz',       20.0)
        self.declare_parameter('fresh_threshold_s',     2.0)
        self.declare_parameter('prediction_timeout_s',  4.0)
        self.declare_parameter('prediction_horizon_s',  1.5)
        self.declare_parameter('prediction_dt_s',       0.1)
        self.declare_parameter('process_noise',         0.5)
        self.declare_parameter('measurement_noise',     0.1)
        self.declare_parameter('target_marker_topic',       '~/target_marker')
        self.declare_parameter('predicted_path_odom_topic', '~/predicted_path_odom')
        # Odom-frame PoseStamped of the current KF estimate — consumed by goal_manager
        self.declare_parameter('tracked_pose_odom_topic',   '~/tracked_pose_odom')

        input_topic               = self.get_parameter('input_topic').value
        tracked_pose_topic        = self.get_parameter('tracked_pose_topic').value
        predicted_path_topic      = self.get_parameter('predicted_path_topic').value
        status_topic              = self.get_parameter('status_topic').value
        target_marker_topic       = self.get_parameter('target_marker_topic').value
        predicted_path_odom_topic = self.get_parameter('predicted_path_odom_topic').value
        tracked_pose_odom_topic   = self.get_parameter('tracked_pose_odom_topic').value
        self._odom_frame     = str(self.get_parameter('odom_frame').value)
        filter_rate_hz       = float(self.get_parameter('filter_rate_hz').value)
        self._fresh_thresh   = float(self.get_parameter('fresh_threshold_s').value)
        self._pred_timeout   = float(self.get_parameter('prediction_timeout_s').value)
        self._pred_horizon   = float(self.get_parameter('prediction_horizon_s').value)
        self._pred_dt        = float(self.get_parameter('prediction_dt_s').value)
        proc_noise           = float(self.get_parameter('process_noise').value)
        meas_noise           = float(self.get_parameter('measurement_noise').value)

        # ── Kalman matrices (odom 2-D state: [px, py, vx, vy]) ──────────
        # H: observe odom (px, py) directly
        self._H = np.array([[1., 0., 0., 0.],
                            [0., 1., 0., 0.]])
        self._R = meas_noise * np.eye(2)
        self._proc_noise = proc_noise

        # ── Filter state ─────────────────────────────────────────────────
        self._x: np.ndarray = np.zeros(4)       # [px, py, vx, vy] in odom
        self._P: np.ndarray = np.eye(4) * 10.0
        self._initialized        = False
        self._last_update_time: Time | None = None
        self._last_meas_time:   Time | None = None
        self._camera_frame: str  = ''   # frame_id of the detector output
        self._odom_z: float      = 0.0  # marker height in odom (approx. constant)

        # ── TF ───────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_wait     = Duration(seconds=0.05)

        # ── Pub / Sub ────────────────────────────────────────────────────
        self._pub_pose       = self.create_publisher(PoseStamped, tracked_pose_topic,        10)
        self._pub_path       = self.create_publisher(Path,        predicted_path_topic,      10)
        self._pub_stat       = self.create_publisher(String,      status_topic,              10)
        self._pub_marker     = self.create_publisher(Marker,      target_marker_topic,       10)
        self._pub_path_odom  = self.create_publisher(Path,        predicted_path_odom_topic, 10)
        # Odom-frame PoseStamped: position = KF predicted position, heading from velocity
        self._pub_pose_odom  = self.create_publisher(PoseStamped, tracked_pose_odom_topic,  10)

        self.create_subscription(PoseStamped, input_topic,
                                 self._measurement_cb, 10)
        self.create_timer(1.0 / filter_rate_hz, self._timer_cb)

        self.get_logger().info(
            f'aruco_tracker ready  input={input_topic}  '
            f'odom_frame={self._odom_frame}  '
            f'pred_timeout={self._pred_timeout:.1f}s  '
            f'horizon={self._pred_horizon:.1f}s'
        )

    # ------------------------------------------------------------------
    # Kalman helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_F(dt: float) -> np.ndarray:
        """Constant-velocity state transition (odom 2-D)."""
        return np.array([[1., 0., dt, 0.],
                         [0., 1., 0., dt],
                         [0., 0., 1., 0.],
                         [0., 0., 0., 1.]])

    def _make_Q(self, dt: float) -> np.ndarray:
        """Piecewise-constant-white-noise-acceleration process noise."""
        dt2, dt3, dt4 = dt ** 2, dt ** 3, dt ** 4
        block = np.array([[dt4 / 4., dt3 / 2.],
                          [dt3 / 2., dt2]])
        Q = np.zeros((4, 4))
        Q[0:2, 0:2] = block
        Q[2:4, 2:4] = block
        Q[0:2, 2:4] = block
        Q[2:4, 0:2] = block
        return self._proc_noise * Q

    def _kf_predict(self, x: np.ndarray, P: np.ndarray,
                    dt: float) -> tuple[np.ndarray, np.ndarray]:
        F = self._make_F(dt)
        Q = self._make_Q(dt)
        return F @ x, F @ P @ F.T + Q

    def _kf_update(self, x: np.ndarray, P: np.ndarray,
                   z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        H, R = self._H, self._R
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x_upd = x + K @ (z - H @ x)
        P_upd = (np.eye(4) - K @ H) @ P
        return x_upd, P_upd

    def _state_at_now(self, now: Time) -> tuple[np.ndarray, np.ndarray]:
        """Predict canonical odom state to `now` (read-only)."""
        dt = float(np.clip(
            (now - self._last_update_time).nanoseconds * 1e-9, 0.0, 5.0))
        if dt < 1e-6:
            return self._x.copy(), self._P.copy()
        return self._kf_predict(self._x, self._P, dt)

    # ------------------------------------------------------------------
    # TF helpers — return None on failure so callers can skip gracefully
    # ------------------------------------------------------------------

    def _lookup_tf(self, target: str, source: str):
        """Return latest TransformStamped or None."""
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=self._tf_wait)
        except Exception:
            return None

    def _cam_point_to_odom(self, cx: float, cz: float,
                           camera_frame: str) -> tuple[float, float, float] | None:
        """
        Transform a camera-frame observation (cx=lateral, cz=depth) to odom.
        Returns (ox, oy, oz) in odom or None if TF unavailable.
        We look up the LATEST transform rather than the message timestamp to
        avoid TF extrapolation errors; position error from this is < 1 cm
        at typical robot speeds over a 50 ms window.
        """
        T = self._lookup_tf(self._odom_frame, camera_frame)
        if T is None:
            return None
        p_cam = np.array([cx, 0.0, cz])   # y (down) is irrelevant for 2-D
        p_odom = _transform_point(T, p_cam)
        return float(p_odom[0]), float(p_odom[1]), float(p_odom[2])

    def _odom_point_to_cam(self, ox: float, oy: float, oz: float,
                           camera_frame: str) -> tuple[float, float, float] | None:
        """
        Transform an odom point back into the current camera frame.
        The CURRENT transform is used so that Robot 1's latest pose is
        accounted for — this is the ego-motion compensation step.
        Returns (cx, cy, cz) in camera optical frame or None.
        """
        T = self._lookup_tf(camera_frame, self._odom_frame)
        if T is None:
            return None
        p_odom = np.array([ox, oy, oz])
        p_cam = _transform_point(T, p_odom)
        return float(p_cam[0]), float(p_cam[1]), float(p_cam[2])

    # ------------------------------------------------------------------
    # Measurement callback
    # ------------------------------------------------------------------

    def _measurement_cb(self, msg: PoseStamped) -> None:
        cam_x = float(msg.pose.position.x)   # lateral in camera optical frame
        cam_z = float(msg.pose.position.z)   # depth in camera optical frame

        if not (np.isfinite(cam_x) and np.isfinite(cam_z)):
            return

        camera_frame = msg.header.frame_id
        if not camera_frame:
            return  # cannot transform without source frame

        self._camera_frame = camera_frame

        # Transform measurement from camera frame to odom
        odom_pos = self._cam_point_to_odom(cam_x, cam_z, camera_frame)
        if odom_pos is None:
            # TF not yet available (common during startup) — skip silently
            return

        ox, oy, oz = odom_pos
        self._odom_z = oz          # store marker height for output transforms
        z_meas = np.array([ox, oy])

        now = self.get_clock().now()

        if not self._initialized:
            self._x = np.array([ox, oy, 0.0, 0.0])
            self._P = np.eye(4) * 10.0
            self._initialized      = True
            self._last_update_time = now
            self._last_meas_time   = now
            self.get_logger().info(
                f'aruco_tracker: filter initialised  '
                f'odom=({ox:.2f}, {oy:.2f})  height={oz:.2f}m'
            )
            return

        # Predict to now → measurement update → store
        x_pred, P_pred = self._state_at_now(now)
        x_upd,  P_upd  = self._kf_update(x_pred, P_pred, z_meas)
        self._x = x_upd
        self._P = P_upd
        self._last_update_time = now
        self._last_meas_time   = now

    # ------------------------------------------------------------------
    # Timer callback — publish at filter_rate_hz
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        if not self._initialized or not self._camera_frame:
            return

        now = self.get_clock().now()
        meas_age_s = (now - self._last_meas_time).nanoseconds * 1e-9

        # ── Status ────────────────────────────────────────────────────
        if meas_age_s <= self._fresh_thresh:
            status = 'measured'
        elif meas_age_s <= self._pred_timeout:
            status = 'predicted'
        else:
            status = 'lost'

        stat_msg = String()
        stat_msg.data = status
        self._pub_stat.publish(stat_msg)

        if status == 'lost':
            return  # stop publishing pose/path once prediction has expired

        # ── Predict odom state to now ─────────────────────────────────
        x_now, P_now = self._state_at_now(now)
        px, py = float(x_now[0]), float(x_now[1])

        # ── Transform predicted odom position → current camera frame ──
        # This is the ego-motion compensation: the current Robot 1 pose
        # is baked into the TF lookup, so if Robot 1 has rotated since the
        # last measurement, the output correctly reflects where the target
        # appears in the camera right now.
        now_stamp = now.to_msg()
        cam_pos = self._odom_point_to_cam(px, py, self._odom_z, self._camera_frame)
        if cam_pos is None:
            return  # TF temporarily unavailable — skip this cycle

        cx, cy, cz = cam_pos

        # ── Publish tracked_pose (camera optical frame) ───────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = now_stamp
        pose_msg.header.frame_id = self._camera_frame
        pose_msg.pose.position.x = cx   # lateral (right +)
        pose_msg.pose.position.y = cy   # vertical (down +, not used by follower)
        pose_msg.pose.position.z = cz   # depth (forward +)
        pose_msg.pose.orientation.w = 1.0
        self._pub_pose.publish(pose_msg)

        # ── Publish predicted_path (camera optical frame) ─────────────
        # Roll the odom state forward, transform each waypoint to current
        # camera frame.  Since we use the same TF at each step the path
        # appears in the robot's current view.
        path_msg = Path()
        path_msg.header.stamp    = now_stamp
        path_msg.header.frame_id = self._camera_frame

        x_roll, P_roll = x_now.copy(), P_now.copy()
        t_horizon = 0.0
        while t_horizon <= self._pred_horizon + 1e-9:
            wp_cam = self._odom_point_to_cam(
                float(x_roll[0]), float(x_roll[1]), self._odom_z,
                self._camera_frame)
            if wp_cam is not None:
                wp = PoseStamped()
                wp.header = path_msg.header
                wp.pose.position.x = wp_cam[0]
                wp.pose.position.y = wp_cam[1]
                wp.pose.position.z = wp_cam[2]
                wp.pose.orientation.w = 1.0
                path_msg.poses.append(wp)
            x_roll, P_roll = self._kf_predict(x_roll, P_roll, self._pred_dt)
            t_horizon += self._pred_dt

        self._pub_path.publish(path_msg)

        # ── Debug: predicted position sphere (odom frame) ─────────────
        # Red = live measurement, orange = KF coasting (predicted)
        sphere = Marker()
        sphere.header.stamp    = now_stamp
        sphere.header.frame_id = self._odom_frame
        sphere.ns     = 'aruco_target'
        sphere.id     = 0
        sphere.type   = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = px
        sphere.pose.position.y = py
        sphere.pose.position.z = self._odom_z + 0.4   # raise above floor
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color.a = 1.0
        if status == 'measured':
            sphere.color.r = 1.0; sphere.color.g = 0.1; sphere.color.b = 0.1
        else:  # predicted
            sphere.color.r = 1.0; sphere.color.g = 0.55; sphere.color.b = 0.0
        sphere.lifetime.sec = 0
        sphere.lifetime.nanosec = 300_000_000   # 300 ms auto-expire
        self._pub_marker.publish(sphere)

        # ── Debug: predicted path in odom frame ───────────────────────
        odom_path = Path()
        odom_path.header.stamp    = now_stamp
        odom_path.header.frame_id = self._odom_frame
        x_roll, P_roll = x_now.copy(), P_now.copy()
        t_horizon = 0.0
        while t_horizon <= self._pred_horizon + 1e-9:
            wp = PoseStamped()
            wp.header = odom_path.header
            wp.pose.position.x = float(x_roll[0])
            wp.pose.position.y = float(x_roll[1])
            wp.pose.position.z = self._odom_z
            wp.pose.orientation.w = 1.0
            odom_path.poses.append(wp)
            x_roll, P_roll = self._kf_predict(x_roll, P_roll, self._pred_dt)
            t_horizon += self._pred_dt
        self._pub_path_odom.publish(odom_path)

        # ── tracked_pose_odom: KF position + velocity-derived heading ────
        # Consumed by aruco_goal_manager to freeze the target goal when
        # ArUco goes out of sight.  Heading approximated from KF velocity
        # so the planner receives a sensible goal orientation.
        odom_pose = PoseStamped()
        odom_pose.header.stamp    = now_stamp
        odom_pose.header.frame_id = self._odom_frame
        odom_pose.pose.position.x = px
        odom_pose.pose.position.y = py
        odom_pose.pose.position.z = self._odom_z
        vx_now = float(x_now[2])
        vy_now = float(x_now[3])
        speed  = math.hypot(vx_now, vy_now)
        if speed > 0.05:
            heading = math.atan2(vy_now, vx_now)
            odom_pose.pose.orientation.z = math.sin(heading / 2.0)
            odom_pose.pose.orientation.w = math.cos(heading / 2.0)
        else:
            odom_pose.pose.orientation.w = 1.0
        self._pub_pose_odom.publish(odom_pose)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoTracker()
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
