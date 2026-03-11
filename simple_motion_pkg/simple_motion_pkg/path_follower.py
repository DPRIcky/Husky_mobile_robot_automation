"""
Path-following node with selectable controller, velocity profiling,
obstacle-triggered replanning, stuck/off-path recovery, and compare mode.

Controller types (controller_type param):
    stanley      — Stanley geometric controller        (default)
    pid          — PID on yaw error
    pure_pursuit — Geometric Pure Pursuit arc curvature
    lqr          — Linear-Quadratic Regulator
    mpc          — Model Predictive Control (scipy.optimize)

Compare mode (controller_compare_mode: true):
    All 5 controllers run every tick. Only the active controller_type drives
    the robot. Metrics (CTE, heading error, speed) for every controller are
    logged every 2 s so you can compare tracking quality side-by-side.

Velocity profiler:
    Shared across all controllers. Computes desired speed from path curvature,
    goal proximity, and obstacle proximity before passing it to the controller.

Replanning (unchanged from Iter 8):
    - Obstacle in forward LiDAR cone  → stop → wait map_update_delay → replan
    - Off-path (> off_path_dist_m)    → replan (6 s cooldown)
    - Stuck (< stuck_dist in 4 s)     → replan
    - _waiting_for_replan flag keeps robot frozen until new path arrives

Twist mux:
    Publishes to /autonomous/cmd_vel.  Run twist_mux node to merge with
    teleop and forward to /a300_00000/cmd_vel.  If twist_mux is not running
    you can remap autonomous_cmd_vel_topic → /a300_00000/cmd_vel directly.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg  import Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg   import LaserScan
from std_msgs.msg      import Float64MultiArray

import tf2_ros

from .controllers import (
    StanleyController, PIDController, PurePursuitController,
    LQRController, MPCController, CONTROLLER_NAMES,
)
from .velocity_profiler import VelocityProfiler


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# ─────────────────────────────────────────────────────────────────────────────

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('path_topic',               '/planned_path')
        self.declare_parameter('autonomous_cmd_vel_topic', '/autonomous/cmd_vel')
        self.declare_parameter('cmd_vel_topic',            '/a300_00000/cmd_vel')
        self.declare_parameter('goal_topic',               '/goal_pose')
        self.declare_parameter('scan_topic',
                               '/a300_00000/sensors/lidar2d_0/scan')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('global_frame', 'map')

        # Controller selection
        self.declare_parameter('controller_type',         'stanley')
        self.declare_parameter('controller_compare_mode', False)

        # Common motion params
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_vel',     0.4)
        self.declare_parameter('max_angular_vel',    1.0)
        self.declare_parameter('kp_linear',          0.5)
        self.declare_parameter('goal_tolerance',     0.25)
        self.declare_parameter('control_rate',       10.0)

        # Obstacle detection
        self.declare_parameter('obstacle_warn_dist',   0.65)
        self.declare_parameter('obstacle_stop_dist',   0.40)
        self.declare_parameter('obstacle_check_angle', 0.698)
        self.declare_parameter('map_update_delay_s',   0.8)
        self.declare_parameter('replan_retry_s',       3.0)

        # Recovery
        self.declare_parameter('stuck_check_interval_s', 4.0)
        self.declare_parameter('stuck_dist_threshold_m', 0.15)
        self.declare_parameter('off_path_dist_m',        1.5)

        # Stanley
        self.declare_parameter('stanley_k',     1.0)
        self.declare_parameter('stanley_v_min', 0.10)

        # PID
        self.declare_parameter('pid_kp',          1.5)
        self.declare_parameter('pid_ki',          0.05)
        self.declare_parameter('pid_kd',          0.2)
        self.declare_parameter('pid_windup_limit', 2.0)

        # LQR
        self.declare_parameter('lqr_q_cte', 5.0)
        self.declare_parameter('lqr_q_he',  2.0)
        self.declare_parameter('lqr_r_w',   1.0)

        # MPC
        self.declare_parameter('mpc_horizon', 8)
        self.declare_parameter('mpc_q_cte',   5.0)
        self.declare_parameter('mpc_q_he',    2.0)
        self.declare_parameter('mpc_r_v',     0.5)
        self.declare_parameter('mpc_r_w',     0.2)

        # Velocity profiler
        self.declare_parameter('vp_curvature_gain',  3.0)
        self.declare_parameter('vp_goal_ramp_dist',  1.5)
        self.declare_parameter('vp_min_vel',         0.05)
        self.declare_parameter('vp_accel_limit',     0.5)

        # ── Resolve parameter values ───────────────────────────────────────
        path_topic         = self.get_parameter('path_topic').value
        auto_cmd_topic     = self.get_parameter('autonomous_cmd_vel_topic').value
        cmd_vel_topic      = self.get_parameter('cmd_vel_topic').value
        goal_topic         = self.get_parameter('goal_topic').value
        scan_topic         = self.get_parameter('scan_topic').value
        self.base_frame    = self.get_parameter('base_frame').value
        self.global_frame  = self.get_parameter('global_frame').value

        self.controller_type    = self.get_parameter('controller_type').value
        self.compare_mode       = self.get_parameter('controller_compare_mode').value

        self.lookahead  = self.get_parameter('lookahead_distance').value
        self.max_v      = self.get_parameter('max_linear_vel').value
        self.max_w      = self.get_parameter('max_angular_vel').value
        self.goal_tol   = self.get_parameter('goal_tolerance').value
        rate            = self.get_parameter('control_rate').value

        self.warn_dist     = self.get_parameter('obstacle_warn_dist').value
        self.stop_dist     = self.get_parameter('obstacle_stop_dist').value
        self.check_angle   = self.get_parameter('obstacle_check_angle').value
        self.map_upd_delay = self.get_parameter('map_update_delay_s').value
        self.replan_retry  = self.get_parameter('replan_retry_s').value

        self.stuck_check_interval  = self.get_parameter('stuck_check_interval_s').value
        self.stuck_dist_threshold  = self.get_parameter('stuck_dist_threshold_m').value
        self.off_path_dist         = self.get_parameter('off_path_dist_m').value

        # ── Build shared params dict for all controllers ───────────────────
        ctrl_params = {k: self.get_parameter(k).value for k in [
            'lookahead_distance', 'max_linear_vel', 'max_angular_vel',
            'kp_linear', 'goal_tolerance', 'control_rate',
            'stanley_k', 'stanley_v_min',
            'pid_kp', 'pid_ki', 'pid_kd', 'pid_windup_limit',
            'lqr_q_cte', 'lqr_q_he', 'lqr_r_w',
            'mpc_horizon', 'mpc_q_cte', 'mpc_q_he', 'mpc_r_v', 'mpc_r_w',
        ]}

        # ── Instantiate all controllers ────────────────────────────────────
        self._controllers = {
            'stanley':      StanleyController(ctrl_params),
            'pid':          PIDController(ctrl_params),
            'pure_pursuit': PurePursuitController(ctrl_params),
            'lqr':          LQRController(ctrl_params),
            'mpc':          MPCController(ctrl_params),
        }
        if self.controller_type not in self._controllers:
            self.get_logger().warn(
                f'Unknown controller_type "{self.controller_type}", '
                f'falling back to "stanley".')
            self.controller_type = 'stanley'

        # ── Velocity profiler ──────────────────────────────────────────────
        vp_params = {k: self.get_parameter(k).value for k in [
            'max_linear_vel', 'vp_min_vel', 'vp_curvature_gain',
            'vp_goal_ramp_dist', 'vp_accel_limit',
        ]}
        self._profiler = VelocityProfiler(vp_params)

        self.get_logger().info(
            f'PathFollower  controller={self.controller_type}'
            f'  compare_mode={self.compare_mode}')

        # ── Path follower state ────────────────────────────────────────────
        self._path:     list = []
        self._path_idx: int  = 0
        self._active:   bool = False
        self._current_goal   = None

        # Obstacle / replan state machine
        self._blocked_since      = None
        self._replan_requested   = False
        self._last_replan_time   = None
        self._waiting_for_replan = False

        # Stuck detection
        self._stuck_check_pose = None
        self._stuck_check_time = None

        # LiDAR
        self._latest_scan:  LaserScan | None = None
        self._laser_frame:  str | None       = None
        self._laser_yaw_offset: float | None = None

        # Timing
        self._last_control_t = None

        # Compare-mode metrics log timer
        self._compare_metrics: dict = {}
        self._last_metrics_log = None

        # ── TF ────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Pub / Sub ─────────────────────────────────────────────────────
        self.create_subscription(Path,      path_topic, self._path_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        # Publish to /autonomous/cmd_vel (twist_mux forwards to robot)
        # Fall back to direct cmd_vel if autonomous topic == cmd_vel topic
        self._cmd_pub  = self.create_publisher(TwistStamped, auto_cmd_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped,  goal_topic, 10)
        self._diag_pub = self.create_publisher(
            Float64MultiArray, '/controller_diagnostics', 10)

        # Actual trajectory trace
        self._traj_pub         = self.create_publisher(Path, '/actual_trajectory', 10)
        self._actual_traj      = Path()
        self._actual_traj.header.frame_id = self.global_frame
        self._traj_max_poses   = 2000   # cap to avoid unbounded memory

        self._timer = self.create_timer(1.0 / rate, self._control_loop)

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('Received path with < 2 poses, ignoring.')
            return

        self._path = [(p.pose.position.x, p.pose.position.y)
                      for p in msg.poses]

        # Start from the closest waypoint to current robot pose (avoids backtracking)
        pose = self._get_robot_pose()
        if pose is not None:
            rx, ry, _ = pose
            dists = [math.hypot(px - rx, py - ry)
                     for px, py in self._path[:-1]]
            self._path_idx = dists.index(min(dists))
        else:
            self._path_idx = 0

        # Clear actual trajectory trace when the goal destination changes
        # (keep it across replans to the same goal so the detour is visible)
        new_goal = self._path[-1]
        if (self._current_goal is None or
                math.hypot(new_goal[0] - self._current_goal[0],
                           new_goal[1] - self._current_goal[1]) > 0.5):
            self._actual_traj.poses.clear()

        self._active       = True
        self._current_goal = self._path[-1]

        # Reset replan / recovery state
        # NOTE: _blocked_since intentionally NOT cleared — scan clears it.
        # NOTE: _last_replan_time intentionally NOT cleared — cooldown persists.
        self._waiting_for_replan = False
        self._replan_requested   = False
        self._stuck_check_pose   = None
        self._stuck_check_time   = None

        # Reset all controllers (PID integral, MPC warm-start, etc.)
        for ctrl in self._controllers.values():
            ctrl.reset()
        self._profiler.reset()

        self.get_logger().info(
            f'New path: {len(self._path)} wp  '
            f'start=wp{self._path_idx}  ctrl={self.controller_type}')

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg
        if self._laser_frame is None:
            self._laser_frame = msg.header.frame_id

    # ──────────────────────────────────────────────────────────────────────────
    # Laser utilities
    # ──────────────────────────────────────────────────────────────────────────

    def _get_laser_yaw_offset(self) -> float:
        if self._laser_yaw_offset is not None:
            return self._laser_yaw_offset
        if self._laser_frame is None:
            return 0.0
        try:
            t = self._tf_buffer.lookup_transform(
                self.base_frame, self._laser_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._laser_yaw_offset = math.atan2(siny, cosy)
            self.get_logger().info(
                f'Laser→base_link yaw offset: '
                f'{math.degrees(self._laser_yaw_offset):.1f}°')
        except Exception as e:
            self.get_logger().warn(
                f'Cannot get laser TF: {e}', throttle_duration_sec=5.0)
            return 0.0
        return self._laser_yaw_offset

    def _scan_min_in_cone(self):
        """Return (forward_min_m, in_warn_zone)."""
        scan = self._latest_scan
        if scan is None:
            return float('inf'), False
        yaw_off     = self._get_laser_yaw_offset()
        forward_min = float('inf')
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            a = _normalize_angle(scan.angle_min + i * scan.angle_increment + yaw_off)
            if abs(a) > self.check_angle:
                continue
            forward_min = min(forward_min, r)
        return forward_min, forward_min < self.warn_dist

    # ──────────────────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._active or not self._path:
            return

        # ── dt ───────────────────────────────────────────────────────────
        now = self.get_clock().now()
        if self._last_control_t is None:
            dt = 1.0 / 10.0
        else:
            dt = (now - self._last_control_t).nanoseconds * 1e-9
            dt = max(0.001, min(dt, 0.5))   # clamp: never > 0.5 s
        self._last_control_t = now

        # ── Obstacle detection ───────────────────────────────────────────
        forward_min, in_warn_zone = self._scan_min_in_cone()
        in_danger = forward_min < self.stop_dist

        if in_danger:
            self._stop()
            if self._blocked_since is None:
                self._blocked_since = now
                self.get_logger().warn(
                    f'Obstacle at {forward_min:.2f}m — stopped, '
                    f'waiting {self.map_upd_delay:.1f}s for map update.')
                return
            blocked_sec = (now - self._blocked_since).nanoseconds * 1e-9
            if not self._replan_requested and blocked_sec >= self.map_upd_delay:
                self._publish_replan()
                self._replan_requested = True
                return
            if self._replan_requested and self._last_replan_time is not None:
                since_last = (now - self._last_replan_time).nanoseconds * 1e-9
                if since_last >= self.replan_retry:
                    self.get_logger().warn(
                        f'Still blocked after {since_last:.1f}s — retrying replan.')
                    self._publish_replan()
            return

        # ── Obstacle cleared ─────────────────────────────────────────────
        if self._blocked_since is not None:
            self.get_logger().info('Path clear — resuming.')
            self._blocked_since    = None
            self._replan_requested = False

        # ── Waiting for replanned path ───────────────────────────────────
        if self._waiting_for_replan:
            self._stop()
            return

        # ── Robot pose ───────────────────────────────────────────────────
        pose = self._get_robot_pose()
        if pose is None:
            self._stop()
            return
        rx, ry, ryaw = pose

        # ── Record actual trajectory ─────────────────────────────────────
        ps = PoseStamped()
        ps.header.stamp    = now.to_msg()
        ps.header.frame_id = self.global_frame
        ps.pose.position.x = rx
        ps.pose.position.y = ry
        ps.pose.orientation.z = math.sin(ryaw / 2.0)
        ps.pose.orientation.w = math.cos(ryaw / 2.0)
        self._actual_traj.poses.append(ps)
        if len(self._actual_traj.poses) > self._traj_max_poses:
            self._actual_traj.poses.pop(0)
        self._actual_traj.header.stamp = now.to_msg()
        self._traj_pub.publish(self._actual_traj)

        # ── Heading alignment ────────────────────────────────────────────
        # If the robot's heading is badly misaligned with the path (> 60°),
        # spin in place before engaging any controller.  This handles the
        # common case where the goal is behind the robot and the first path
        # segment points ~180° opposite to the robot's current heading.
        # All controllers reduce v to zero when |he| > 90° (via cos factor),
        # but MPC can output w≈0 near he=±π due to gradient discontinuity.
        _ALIGN_THRESH = math.radians(60)
        if self._path_idx < len(self._path) - 1:
            px0, py0 = self._path[self._path_idx]
            px1, py1 = self._path[self._path_idx + 1]
            path_head_now = math.atan2(py1 - py0, px1 - px0)
            he_now = _normalize_angle(path_head_now - ryaw)
            if abs(he_now) > _ALIGN_THRESH:
                spin_cmd = TwistStamped()
                spin_cmd.header.stamp    = now.to_msg()
                spin_cmd.header.frame_id = self.base_frame
                spin_cmd.twist.angular.z = float(
                    math.copysign(self.max_w, he_now))
                self._cmd_pub.publish(spin_cmd)
                self._stuck_check_time = None   # reset — spinning is intentional
                return

        # ── Stuck detection ──────────────────────────────────────────────
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
                        f'Stuck: moved {moved:.2f}m in {elapsed:.1f}s — replanning.')
                    self._publish_replan()
                self._stuck_check_pose = (rx, ry)
                self._stuck_check_time = now

        # ── Off-path detection ───────────────────────────────────────────
        min_path_dist = min(math.hypot(px - rx, py - ry)
                            for px, py in self._path)
        replan_cooldown = self.replan_retry * 2.0
        since_last_replan = (
            (now - self._last_replan_time).nanoseconds * 1e-9
            if self._last_replan_time is not None else float('inf')
        )
        if (min_path_dist > self.off_path_dist
                and self._current_goal
                and since_last_replan > replan_cooldown):
            self.get_logger().warn(
                f'Off-path: {min_path_dist:.2f}m — replanning.')
            self._publish_replan()

        if self._waiting_for_replan:
            return

        # ── Goal reached? ────────────────────────────────────────────────
        gx, gy = self._path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tol:
            self._stop()
            self.get_logger().info('Goal reached!')
            self._active = False
            return

        # ── Velocity profiler ────────────────────────────────────────────
        dist_to_goal     = math.hypot(gx - rx, gy - ry)
        obstacle_factor  = 0.5 if in_warn_zone else 1.0
        v_desired = self._profiler.compute(
            self._path, self._path_idx, dist_to_goal, obstacle_factor, dt)

        # ── Controller dispatch ──────────────────────────────────────────
        if self.compare_mode:
            v_cmd, w_cmd = self._run_compare(rx, ry, ryaw, v_desired, dt, now)
        else:
            ctrl = self._controllers[self.controller_type]
            v_cmd, w_cmd, _, _, new_idx = ctrl.compute(
                rx, ry, ryaw, self._path, self._path_idx, v_desired, dt)
            self._path_idx = new_idx

        # ── Publish ──────────────────────────────────────────────────────
        cmd = TwistStamped()
        cmd.header.stamp    = now.to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x  = float(max(0.0, min(v_cmd, self.max_v)))
        cmd.twist.angular.z = float(max(-self.max_w, min(w_cmd, self.max_w)))
        self._cmd_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────────────────────
    # Compare mode
    # ──────────────────────────────────────────────────────────────────────────

    def _run_compare(self, rx, ry, ryaw, v_desired, dt, now):
        """
        Call all 5 controllers once.  Only active controller_type drives.
        Publishes diagnostics and logs a comparison table every 2 s.
        Returns (v_cmd, w_cmd) for the active controller.
        """
        results = {}
        active_new_idx = self._path_idx

        for name, ctrl in self._controllers.items():
            try:
                v, w, cte, he, new_idx = ctrl.compute(
                    rx, ry, ryaw, self._path, self._path_idx, v_desired, dt)
            except Exception as e:
                self.get_logger().warn(
                    f'Controller {name} error: {e}', throttle_duration_sec=5.0)
                v, w, cte, he, new_idx = 0.0, 0.0, 0.0, 0.0, self._path_idx
            results[name] = (v, w, cte, he)
            if name == self.controller_type:
                active_new_idx = new_idx

        self._path_idx = active_new_idx

        # Publish diagnostics as flat float array:
        # [stanley_v, stanley_w, stanley_cte, stanley_he,
        #  pid_v, pid_w, pid_cte, pid_he, ... × 5]
        diag = Float64MultiArray()
        for name in CONTROLLER_NAMES:
            v, w, cte, he = results.get(name, (0.0, 0.0, 0.0, 0.0))
            diag.data.extend([v, w, cte, he])
        self._diag_pub.publish(diag)

        # Log comparison table every 2 s
        log_interval = 2.0
        if (self._last_metrics_log is None or
                (now - self._last_metrics_log).nanoseconds * 1e-9 >= log_interval):
            self._last_metrics_log = now
            self.get_logger().info('── Controller compare ────────────────')
            header = f'  {"Controller":14s}  {"v(m/s)":>7s}  {"ω(r/s)":>7s}'
            header += f'  {"CTE(m)":>8s}  {"HE(°)":>7s}'
            self.get_logger().info(header)
            for name in CONTROLLER_NAMES:
                v, w, cte, he = results.get(name, (0.0, 0.0, 0.0, 0.0))
                marker = ' ◄' if name == self.controller_type else ''
                self.get_logger().info(
                    f'  {name:14s}  {v:7.3f}  {w:7.3f}'
                    f'  {cte:8.3f}  {math.degrees(he):7.1f}{marker}')

        active_v, active_w, _, _ = results[self.controller_type]
        return active_v, active_w

    # ──────────────────────────────────────────────────────────────────────────
    # Replanning helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_replan(self):
        if self._current_goal is None:
            return
        gx, gy = self._current_goal
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)
        self._waiting_for_replan = True
        self._last_replan_time   = self.get_clock().now()
        self.get_logger().info(f'Replan → goal ({gx:.2f}, {gy:.2f})')

    # ──────────────────────────────────────────────────────────────────────────
    # TF / motion helpers
    # ──────────────────────────────────────────────────────────────────────────

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
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        self._cmd_pub.publish(cmd)


# ─────────────────────────────────────────────────────────────────────────────

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
