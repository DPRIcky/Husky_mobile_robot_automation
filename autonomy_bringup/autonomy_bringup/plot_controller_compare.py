#!/usr/bin/env python3
"""
Live plotter for controller compare mode.

Reads `/controller_diagnostics` from `simple_motion_pkg/path_follower`, where
each message contains, for every controller in order:

    [v_cmd, w_cmd, cte, heading_error] × 5

The plotter shows:
  1. Reference path vs actual trajectory (active controller only)
  2. Cross-track error history for all controllers
  3. Heading error history for all controllers
  4. Linear velocity command history for all controllers
  5. Angular velocity command history for all controllers

This is a live diagnostic comparison from the same robot state at each control
tick. It is not a full independent closed-loop rollout for all five
controllers simultaneously.
"""

from __future__ import annotations

import math
import sys
import threading
from collections import deque

import matplotlib
for _backend in ('TkAgg', 'Qt5Agg', 'Qt6Agg', 'GTK3Agg', 'WebAgg'):
    try:
        matplotlib.use(_backend)
        import matplotlib.pyplot as plt
        plt.figure()
        plt.close()
        break
    except Exception:
        continue
else:
    import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray


_CONTROLLERS = ('stanley', 'pid', 'pure_pursuit', 'lqr', 'mpc')
_COLORS = {
    'stanley': '#00c853',
    'pid': '#ff7043',
    'pure_pursuit': '#29b6f6',
    'lqr': '#ffee58',
    'mpc': '#ab47bc',
}
_MAX_PTS = 2500
_ANIM_INTERVAL_MS = 200


class ControllerComparePlotter(Node):
    def __init__(self):
        super().__init__('controller_compare_plotter')

        self.declare_parameter('diag_topic', '/controller_diagnostics')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('actual_trajectory_topic', '/actual_trajectory')
        self.declare_parameter('active_controller', 'lqr')

        diag_topic = str(self.get_parameter('diag_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        actual_topic = str(self.get_parameter('actual_trajectory_topic').value)
        self._active_controller = str(self.get_parameter('active_controller').value)

        self._lock = threading.Lock()
        self._t0 = None
        self._last_diag_time = None

        self._time = deque(maxlen=_MAX_PTS)
        self._metrics = {
            name: {
                'v': deque(maxlen=_MAX_PTS),
                'w': deque(maxlen=_MAX_PTS),
                'cte': deque(maxlen=_MAX_PTS),
                'he': deque(maxlen=_MAX_PTS),
            }
            for name in _CONTROLLERS
        }

        self._ref_x = []
        self._ref_y = []
        self._act_x = []
        self._act_y = []

        self.create_subscription(Float64MultiArray, diag_topic, self._diag_cb, 10)
        self.create_subscription(Path, path_topic, self._path_cb, 10)
        self.create_subscription(Path, actual_topic, self._actual_cb, 10)

        self.get_logger().info(
            f'Controller compare plotter  diag={diag_topic}  '
            f'path={path_topic}  actual={actual_topic}  '
            f'active={self._active_controller}'
        )

    def _elapsed(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    def _diag_cb(self, msg: Float64MultiArray):
        expected = 4 * len(_CONTROLLERS)
        if len(msg.data) < expected:
            self.get_logger().warn(
                f'Expected at least {expected} controller-diagnostic floats, '
                f'got {len(msg.data)}',
                throttle_duration_sec=5.0,
            )
            return

        t = self._elapsed()
        with self._lock:
            self._time.append(t)
            self._last_diag_time = t
            for i, name in enumerate(_CONTROLLERS):
                base = 4 * i
                self._metrics[name]['v'].append(float(msg.data[base + 0]))
                self._metrics[name]['w'].append(float(msg.data[base + 1]))
                self._metrics[name]['cte'].append(float(msg.data[base + 2]))
                self._metrics[name]['he'].append(float(msg.data[base + 3]))

    def _path_cb(self, msg: Path):
        with self._lock:
            self._ref_x = [p.pose.position.x for p in msg.poses]
            self._ref_y = [p.pose.position.y for p in msg.poses]

    def _actual_cb(self, msg: Path):
        with self._lock:
            self._act_x = [p.pose.position.x for p in msg.poses]
            self._act_y = [p.pose.position.y for p in msg.poses]

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'time': np.array(self._time, dtype=float),
                'last_diag_time': self._last_diag_time,
                'ref_x': np.array(self._ref_x, dtype=float),
                'ref_y': np.array(self._ref_y, dtype=float),
                'act_x': np.array(self._act_x, dtype=float),
                'act_y': np.array(self._act_y, dtype=float),
                'metrics': {
                    name: {
                        key: np.array(values, dtype=float)
                        for key, values in metric.items()
                    }
                    for name, metric in self._metrics.items()
                },
            }


def _configure_axis(ax, title: str, ylabel: str):
    ax.set_title(title, fontsize=11, pad=8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)


def _build_figure(active_controller: str):
    fig = plt.figure(figsize=(16, 9), constrained_layout=True)
    fig.canvas.manager.set_window_title('Controller Comparison')
    gs = gridspec.GridSpec(2, 3, figure=fig)

    axes = {
        'traj': fig.add_subplot(gs[0, :2]),
        'cte': fig.add_subplot(gs[0, 2]),
        'he': fig.add_subplot(gs[1, 0]),
        'v': fig.add_subplot(gs[1, 1]),
        'w': fig.add_subplot(gs[1, 2]),
    }

    axes['traj'].set_title(
        f'Path Tracking (actual path = active controller: {active_controller})',
        fontsize=11,
        pad=8,
    )
    axes['traj'].set_xlabel('X (m)')
    axes['traj'].set_ylabel('Y (m)')
    axes['traj'].grid(True, alpha=0.25)
    axes['traj'].set_aspect('equal', adjustable='datalim')

    _configure_axis(axes['cte'], 'Cross-Track Error', 'CTE (m)')
    _configure_axis(axes['he'], 'Heading Error', 'Heading Error (deg)')
    _configure_axis(axes['v'], 'Linear Velocity Commands', 'v_cmd (m/s)')
    _configure_axis(axes['w'], 'Angular Velocity Commands', 'w_cmd (rad/s)')

    lines = {
        'traj_ref': axes['traj'].plot(
            [], [], '--', color='#1f3a5f', lw=2.4, alpha=0.95,
            label='Reference path', zorder=2
        )[0],
        'traj_act': axes['traj'].plot(
            [], [], '-', color='#ffd54f', lw=2.2,
            label='Actual trajectory', zorder=3
        )[0],
        'traj_dot': axes['traj'].plot(
            [], [], 'o', color='#ffca28', ms=6, zorder=4
        )[0],
    }

    for key, ylabel in (('cte', 'cte'), ('he', 'he'), ('v', 'v'), ('w', 'w')):
        for name in _CONTROLLERS:
            label = name + (' *' if name == active_controller else '')
            lines[f'{key}_{name}'] = axes[key].plot(
                [], [], color=_COLORS[name], lw=1.8, label=label
            )[0]
        axes[key].legend(loc='best', fontsize=8)

    axes['traj'].legend(loc='best', fontsize=8)
    return fig, axes, lines


def _update(_, plotter: ControllerComparePlotter, axes: dict, lines: dict, active_controller: str):
    snap = plotter.snapshot()
    t = snap['time']
    metrics = snap['metrics']

    lines['traj_ref'].set_data(snap['ref_x'], snap['ref_y'])
    lines['traj_act'].set_data(snap['act_x'], snap['act_y'])
    if len(snap['act_x']) > 0:
        lines['traj_dot'].set_data([snap['act_x'][-1]], [snap['act_y'][-1]])
    else:
        lines['traj_dot'].set_data([], [])

    for name in _CONTROLLERS:
        lines[f'cte_{name}'].set_data(t, metrics[name]['cte'])
        lines[f'he_{name}'].set_data(t, np.degrees(metrics[name]['he']))
        lines[f'v_{name}'].set_data(t, metrics[name]['v'])
        lines[f'w_{name}'].set_data(t, metrics[name]['w'])

    for key in ('traj', 'cte', 'he', 'v', 'w'):
        axes[key].relim()
        axes[key].autoscale_view()

    return list(lines.values())


def main(args=None):
    rclpy.init(args=args)
    node = ControllerComparePlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes, lines = _build_figure(node._active_controller)
    _anim = FuncAnimation(
        fig,
        _update,
        fargs=(node, axes, lines, node._active_controller),
        interval=_ANIM_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main(sys.argv)
