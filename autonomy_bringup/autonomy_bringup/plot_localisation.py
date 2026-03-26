#!/usr/bin/env python3
"""
Real-time ground-truth vs SLAM state-estimation comparison plot.

Two signals are compared:
  Ground truth   — /a300_00000/platform/odom  (raw Gazebo wheel odometry;
                   in simulation this is the physics-engine "true" pose)
  State estimate — TF map → base_link          (SLAM-corrected pose used by
                   path_follower for planning and control)

Reference path (/planned_path) and actual trajectory (/actual_trajectory)
are overlaid on the 2-D panel so you can see tracking quality alongside
localisation quality.

Six panels
  1  2-D X-Y trajectory     reference path · actual trace · raw odom · SLAM
  2  X position vs time     ground truth vs SLAM
  3  Y position vs time     ground truth vs SLAM
  4  Heading (yaw) vs time  ground truth vs SLAM
  5  Localisation error     ‖GT − SLAM‖ (m) vs time
  6  Live statistics        current / mean / RMSE / max error

Run standalone:
  python3 autonomy_bringup/plot_localisation.py

Or via the launch file:
  ros2 launch autonomy_bringup autonomy.launch.py launch_plot:=true
"""

import math
import sys
import threading
from collections import deque

import numpy as np
import matplotlib
for _backend in ('TkAgg', 'Qt5Agg', 'Qt6Agg', 'GTK3Agg', 'WebAgg'):
    try:
        matplotlib.use(_backend)
        import matplotlib.pyplot as plt
        plt.figure()        # test that the backend actually opens a window
        plt.close()
        break
    except Exception:
        continue
else:
    import matplotlib.pyplot as plt  # last resort — may be non-interactive
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import rclpy.duration

from nav_msgs.msg import Odometry, Path
import tf2_ros


_MAX_PTS = 2000   # rolling-buffer depth (≈ 200 s at 10 Hz)
_ANIM_INTERVAL_MS = 200   # redraw interval (5 fps keeps CPU low)


# ──────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────────────────────────────────────

def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _interpolate_to_common_times(t_a, v_a, t_b, v_b):
    """
    Linear-interpolate both signals onto a shared time grid so errors can be
    computed sample-by-sample.  Returns (t_common, va_interp, vb_interp).
    """
    if len(t_a) < 2 or len(t_b) < 2:
        return np.array([]), np.array([]), np.array([])
    t_lo = max(t_a[0],  t_b[0])
    t_hi = min(t_a[-1], t_b[-1])
    if t_hi <= t_lo:
        return np.array([]), np.array([]), np.array([])
    t_com = np.linspace(t_lo, t_hi, min(len(t_a), len(t_b)))
    va = np.interp(t_com, t_a, v_a)
    vb = np.interp(t_com, t_b, v_b)
    return t_com, va, vb


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 node
# ──────────────────────────────────────────────────────────────────────────────

class LocalisationPlotter(Node):
    """
    Collects data from ROS topics/TF into thread-safe deques.
    The matplotlib animation reads from these deques on the main thread.
    """

    def __init__(self):
        super().__init__('localisation_plotter')

        self.declare_parameter('odom_topic',
                               '/a300_00000/platform/odom')
        self.declare_parameter('map_frame',  'map')
        self.declare_parameter('base_frame', 'base_link')

        odom_topic  = self.get_parameter('odom_topic').value
        self._map_f = self.get_parameter('map_frame').value
        self._base_f = self.get_parameter('base_frame').value

        self._lock = threading.Lock()

        # Ground truth (raw wheel odometry from Gazebo physics)
        self._gt_t:   deque = deque(maxlen=_MAX_PTS)
        self._gt_x:   deque = deque(maxlen=_MAX_PTS)
        self._gt_y:   deque = deque(maxlen=_MAX_PTS)
        self._gt_yaw: deque = deque(maxlen=_MAX_PTS)
        self._gt_x0:  float | None = None  # initial position (for display offset)
        self._gt_y0:  float | None = None

        # SLAM state estimate (map → base_link TF)
        self._sl_t:   deque = deque(maxlen=_MAX_PTS)
        self._sl_x:   deque = deque(maxlen=_MAX_PTS)
        self._sl_y:   deque = deque(maxlen=_MAX_PTS)
        self._sl_yaw: deque = deque(maxlen=_MAX_PTS)

        # Reference path & actual trajectory (map frame)
        self._ref_x: list = []
        self._ref_y: list = []
        self._act_x: list = []
        self._act_y: list = []

        self._t0: float | None = None   # wall-clock session start

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # Subscribers
        self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(
            Path, '/planned_path',      self._path_cb, 10)
        self.create_subscription(
            Path, '/actual_trajectory', self._actual_cb, 10)

        # 10 Hz TF poll for SLAM estimate
        self.create_timer(0.1, self._tf_poll)

        self.get_logger().info(
            f'Localisation plotter  odom={odom_topic}  '
            f'TF: {self._map_f} → {self._base_f}')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _elapsed(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    # ── callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        t   = self._elapsed()
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            if self._gt_x0 is None:
                self._gt_x0 = x
                self._gt_y0 = y
            self._gt_t.append(t)
            self._gt_x.append(x)
            self._gt_y.append(y)
            self._gt_yaw.append(yaw)

    def _tf_poll(self):
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_f, self._base_f, Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            return
        t   = self._elapsed()
        x   = tf.transform.translation.x
        y   = tf.transform.translation.y
        yaw = _yaw_from_quat(tf.transform.rotation)
        with self._lock:
            self._sl_t.append(t)
            self._sl_x.append(x)
            self._sl_y.append(y)
            self._sl_yaw.append(yaw)

    def _path_cb(self, msg: Path):
        rx = [p.pose.position.x for p in msg.poses]
        ry = [p.pose.position.y for p in msg.poses]
        with self._lock:
            self._ref_x = rx
            self._ref_y = ry

    def _actual_cb(self, msg: Path):
        ax = [p.pose.position.x for p in msg.poses]
        ay = [p.pose.position.y for p in msg.poses]
        with self._lock:
            self._act_x = ax
            self._act_y = ay

    # ── thread-safe snapshot for animation ───────────────────────────────────

    def snapshot(self) -> dict:
        with self._lock:
            return dict(
                gt_t   = np.array(self._gt_t),
                gt_x   = np.array(self._gt_x),
                gt_y   = np.array(self._gt_y),
                gt_yaw = np.array(self._gt_yaw),
                gt_x0  = self._gt_x0 or 0.0,
                gt_y0  = self._gt_y0 or 0.0,
                sl_t   = np.array(self._sl_t),
                sl_x   = np.array(self._sl_x),
                sl_y   = np.array(self._sl_y),
                sl_yaw = np.array(self._sl_yaw),
                ref_x  = list(self._ref_x),
                ref_y  = list(self._ref_y),
                act_x  = list(self._act_x),
                act_y  = list(self._act_y),
            )


# ──────────────────────────────────────────────────────────────────────────────
# Matplotlib layout
# ──────────────────────────────────────────────────────────────────────────────

def build_figure():
    fig = plt.figure(figsize=(16, 9))
    fig.patch.set_facecolor('#1a1a2e')

    gs = gridspec.GridSpec(
        2, 3,
        figure=fig,
        hspace=0.42, wspace=0.38,
        left=0.06, right=0.97, top=0.92, bottom=0.07,
    )

    axes = {
        'traj': fig.add_subplot(gs[0, 0]),
        'x':    fig.add_subplot(gs[0, 1]),
        'y':    fig.add_subplot(gs[0, 2]),
        'yaw':  fig.add_subplot(gs[1, 0]),
        'err':  fig.add_subplot(gs[1, 1]),
        'stat': fig.add_subplot(gs[1, 2]),
    }

    _DARK_BG  = '#16213e'
    _GRID_CLR = '#2a2a5a'

    for ax in axes.values():
        ax.set_facecolor(_DARK_BG)
        ax.tick_params(colors='#cccccc', labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')

    fig.suptitle(
        'Ground Truth vs SLAM State Estimation  —  Live',
        color='white', fontsize=13, fontweight='bold', y=0.97,
    )

    # ── Panel 1: 2-D trajectory ───────────────────────────────────────────
    ax = axes['traj']
    ax.set_title('2-D Trajectory', color='white', fontsize=9)
    ax.set_xlabel('X (m)', color='#aaaacc', fontsize=8)
    ax.set_ylabel('Y (m)', color='#aaaacc', fontsize=8)
    ax.grid(True, color=_GRID_CLR, linewidth=0.5)
    ax.set_aspect('equal', adjustable='datalim')

    lines = {}
    lines['ref'],  = ax.plot([], [], '--', color='#00cc88', lw=1.2,
                              label='Ref path',   alpha=0.7)
    lines['act'],  = ax.plot([], [], '-',  color='#ffdd44', lw=1.5,
                              label='Actual traj', alpha=0.85)
    lines['gt2d'], = ax.plot([], [], '-',  color='#55aaff', lw=1.5,
                              label='GT odom',    alpha=0.9)
    lines['sl2d'], = ax.plot([], [], '-',  color='#ff6655', lw=1.8,
                              label='SLAM est.',  alpha=0.9)
    lines['gt_dot'], = ax.plot([], [], 'o', color='#55aaff', ms=6)
    lines['sl_dot'], = ax.plot([], [], 'o', color='#ff6655', ms=6)
    ax.legend(loc='upper right', fontsize=7, facecolor='#1a1a2e',
              labelcolor='white', framealpha=0.7)

    # ── Panel 2: X vs time ────────────────────────────────────────────────
    ax = axes['x']
    ax.set_title('X Position vs Time', color='white', fontsize=9)
    ax.set_xlabel('t (s)', color='#aaaacc', fontsize=8)
    ax.set_ylabel('X (m)', color='#aaaacc', fontsize=8)
    ax.grid(True, color=_GRID_CLR, linewidth=0.5)
    lines['gt_x'], = ax.plot([], [], '-', color='#55aaff', lw=1.5, label='GT odom')
    lines['sl_x'], = ax.plot([], [], '-', color='#ff6655', lw=1.5, label='SLAM')
    ax.legend(loc='upper right', fontsize=7, facecolor='#1a1a2e',
              labelcolor='white', framealpha=0.7)

    # ── Panel 3: Y vs time ────────────────────────────────────────────────
    ax = axes['y']
    ax.set_title('Y Position vs Time', color='white', fontsize=9)
    ax.set_xlabel('t (s)', color='#aaaacc', fontsize=8)
    ax.set_ylabel('Y (m)', color='#aaaacc', fontsize=8)
    ax.grid(True, color=_GRID_CLR, linewidth=0.5)
    lines['gt_y'], = ax.plot([], [], '-', color='#55aaff', lw=1.5, label='GT odom')
    lines['sl_y'], = ax.plot([], [], '-', color='#ff6655', lw=1.5, label='SLAM')
    ax.legend(loc='upper right', fontsize=7, facecolor='#1a1a2e',
              labelcolor='white', framealpha=0.7)

    # ── Panel 4: Heading vs time ──────────────────────────────────────────
    ax = axes['yaw']
    ax.set_title('Heading (Yaw) vs Time', color='white', fontsize=9)
    ax.set_xlabel('t (s)', color='#aaaacc', fontsize=8)
    ax.set_ylabel('Yaw (deg)', color='#aaaacc', fontsize=8)
    ax.grid(True, color=_GRID_CLR, linewidth=0.5)
    lines['gt_yaw'], = ax.plot([], [], '-', color='#55aaff', lw=1.5, label='GT odom')
    lines['sl_yaw'], = ax.plot([], [], '-', color='#ff6655', lw=1.5, label='SLAM')
    ax.legend(loc='upper right', fontsize=7, facecolor='#1a1a2e',
              labelcolor='white', framealpha=0.7)

    # ── Panel 5: Localisation error ───────────────────────────────────────
    ax = axes['err']
    ax.set_title('Localisation Error  ‖GT − SLAM‖', color='white', fontsize=9)
    ax.set_xlabel('t (s)', color='#aaaacc', fontsize=8)
    ax.set_ylabel('Error (m)', color='#aaaacc', fontsize=8)
    ax.grid(True, color=_GRID_CLR, linewidth=0.5)
    lines['err'], = ax.plot([], [], '-', color='#ff9944', lw=1.8)
    lines['err_mean'], = ax.plot([], [], '--', color='#ffccaa', lw=1.2, label='Mean')
    ax.legend(loc='upper right', fontsize=7, facecolor='#1a1a2e',
              labelcolor='white', framealpha=0.7)

    # ── Panel 6: Statistics ───────────────────────────────────────────────
    ax = axes['stat']
    ax.set_facecolor(_DARK_BG)
    ax.axis('off')
    ax.set_title('Live Statistics', color='white', fontsize=9, fontweight='bold')
    lines['stat_txt'] = ax.text(
        0.05, 0.92, 'Waiting for data…',
        transform=ax.transAxes,
        color='#ddddee', fontsize=9, family='monospace',
        verticalalignment='top',
    )

    return fig, axes, lines


# ──────────────────────────────────────────────────────────────────────────────
# Animation update
# ──────────────────────────────────────────────────────────────────────────────

def _autoscale(ax, xs, ys, margin=0.5):
    if len(xs) == 0:
        return
    xlo, xhi = min(xs) - margin, max(xs) + margin
    ylo, yhi = min(ys) - margin, max(ys) + margin
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)


def _autoscale_time(ax, ts, *vals, margin_y=0.1):
    if len(ts) == 0:
        return
    ax.set_xlim(0.0, max(ts) + 1.0)
    all_v = np.concatenate([v for v in vals if len(v) > 0])
    if len(all_v) == 0:
        return
    lo, hi = float(all_v.min()), float(all_v.max())
    span = max(hi - lo, 0.01)
    ax.set_ylim(lo - span * margin_y, hi + span * margin_y)


def make_updater(node: LocalisationPlotter, axes: dict, lines: dict):
    """Return the FuncAnimation callback closure."""

    def update(_frame):
        d = node.snapshot()

        gt_t, gt_x, gt_y    = d['gt_t'], d['gt_x'], d['gt_y']
        gt_yaw               = d['gt_yaw']
        gt_x0, gt_y0         = d['gt_x0'], d['gt_y0']
        sl_t, sl_x, sl_y    = d['sl_t'], d['sl_x'], d['sl_y']
        sl_yaw               = d['sl_yaw']
        ref_x, ref_y         = d['ref_x'], d['ref_y']
        act_x, act_y         = d['act_x'], d['act_y']

        if len(gt_t) == 0 and len(sl_t) == 0:
            return

        # Ground truth is in the odom frame; SLAM is in the map frame.
        # In Gazebo both frames start at the robot spawn (≈ same origin).
        # We align gt to SLAM by offsetting with the initial SLAM position
        # minus the initial gt position so the two trajectories share a
        # common start point on the 2-D panel.
        if len(sl_x) > 0 and len(gt_x) > 0:
            sl_x0 = float(sl_x[0])
            sl_y0 = float(sl_y[0])
            dx_off = sl_x0 - float(gt_x0)
            dy_off = sl_y0 - float(gt_y0)
            gt_x_map = gt_x + dx_off
            gt_y_map = gt_y + dy_off
        else:
            gt_x_map = gt_x
            gt_y_map = gt_y

        # ── 2-D trajectory ────────────────────────────────────────────────
        if ref_x:
            lines['ref'].set_data(ref_x, ref_y)
        if act_x:
            lines['act'].set_data(act_x, act_y)
        if len(gt_x_map) > 0:
            lines['gt2d'].set_data(gt_x_map, gt_y_map)
            lines['gt_dot'].set_data([gt_x_map[-1]], [gt_y_map[-1]])
        if len(sl_x) > 0:
            lines['sl2d'].set_data(sl_x, sl_y)
            lines['sl_dot'].set_data([sl_x[-1]], [sl_y[-1]])

        all_x = list(ref_x) + list(act_x) + list(gt_x_map) + list(sl_x)
        all_y = list(ref_y) + list(act_y) + list(gt_y_map) + list(sl_y)
        _autoscale(axes['traj'], all_x, all_y, margin=0.8)

        # ── X vs time ─────────────────────────────────────────────────────
        if len(gt_t) > 0:
            lines['gt_x'].set_data(gt_t, gt_x_map)
        if len(sl_t) > 0:
            lines['sl_x'].set_data(sl_t, sl_x)
        _autoscale_time(axes['x'], np.concatenate([gt_t, sl_t]) if len(gt_t) > 0 else sl_t,
                        gt_x_map, sl_x)

        # ── Y vs time ─────────────────────────────────────────────────────
        if len(gt_t) > 0:
            lines['gt_y'].set_data(gt_t, gt_y_map)
        if len(sl_t) > 0:
            lines['sl_y'].set_data(sl_t, sl_y)
        _autoscale_time(axes['y'], np.concatenate([gt_t, sl_t]) if len(gt_t) > 0 else sl_t,
                        gt_y_map, sl_y)

        # ── Heading vs time ───────────────────────────────────────────────
        if len(gt_t) > 0:
            lines['gt_yaw'].set_data(gt_t, np.degrees(gt_yaw))
        if len(sl_t) > 0:
            lines['sl_yaw'].set_data(sl_t, np.degrees(sl_yaw))
        _autoscale_time(axes['yaw'],
                        np.concatenate([gt_t, sl_t]) if len(gt_t) > 0 else sl_t,
                        np.degrees(gt_yaw), np.degrees(sl_yaw))

        # ── Error vs time ─────────────────────────────────────────────────
        t_com, gx_i, sx_i = _interpolate_to_common_times(
            gt_t, gt_x_map, sl_t, sl_x)
        if len(t_com) > 0:
            _, gy_i, sy_i = _interpolate_to_common_times(
                gt_t, gt_y_map, sl_t, sl_y)
            err = np.sqrt((gx_i - sx_i)**2 + (gy_i - sy_i)**2)
            err_mean_line = np.full_like(err, err.mean())

            lines['err'].set_data(t_com, err)
            lines['err_mean'].set_data(t_com, err_mean_line)
            _autoscale_time(axes['err'], t_com, err, margin_y=0.15)

            # ── Statistics ────────────────────────────────────────────────
            rmse    = float(np.sqrt(np.mean(err**2)))
            mean_e  = float(err.mean())
            max_e   = float(err.max())
            cur_e   = float(err[-1])

            # Heading error
            t_yaw, gyaw_i, syaw_i = _interpolate_to_common_times(
                gt_t, gt_yaw, sl_t, sl_yaw)
            if len(t_yaw) > 0:
                yaw_err_deg = np.degrees(
                    np.abs(np.arctan2(
                        np.sin(gyaw_i - syaw_i),
                        np.cos(gyaw_i - syaw_i))))
                yaw_rmse = float(np.sqrt(np.mean(yaw_err_deg**2)))
            else:
                yaw_rmse = 0.0

            n_pts = len(err)
            elapsed = float(t_com[-1]) if len(t_com) > 0 else 0.0

            stat_str = (
                f'Duration   : {elapsed:6.1f} s\n'
                f'Samples    : {n_pts}\n'
                f'\n'
                f'Pos. error (GT vs SLAM)\n'
                f'  Current  : {cur_e:6.4f} m\n'
                f'  Mean     : {mean_e:6.4f} m\n'
                f'  RMSE     : {rmse:6.4f} m\n'
                f'  Max      : {max_e:6.4f} m\n'
                f'\n'
                f'Heading RMSE: {yaw_rmse:5.2f} °\n'
                f'\n'
                f'GT  pos : ({float(gt_x_map[-1]):.2f}, {float(gt_y_map[-1]):.2f}) m\n'
                f'SLAM pos: ({float(sl_x[-1]):.2f}, {float(sl_y[-1]):.2f}) m'
            )
            lines['stat_txt'].set_text(stat_str)

    return update


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init(args=sys.argv)
    node = LocalisationPlotter()

    # Spin ROS 2 in a background thread so matplotlib can run on main thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    fig, axes, lines = build_figure()
    updater = make_updater(node, axes, lines)

    anim = FuncAnimation(
        fig, updater,
        interval=_ANIM_INTERVAL_MS,
        cache_frame_data=False,
        blit=False,
    )

    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
