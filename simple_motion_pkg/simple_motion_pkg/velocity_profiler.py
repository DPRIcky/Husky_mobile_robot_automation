"""
Velocity profiler — computes desired linear speed before the controller applies
its own yaw-error scaling.

Three independent factors are multiplied together:

  1. Curvature factor  — slow down for sharp path curvature ahead
  2. Goal-ramp factor  — decelerate as the robot approaches the goal
  3. Obstacle factor   — passed in from scan (1.0 = clear, 0.5 = warn, 0.0 = stop)

An acceleration ramp prevents jerky speed-up; deceleration is instantaneous
so safety stops are not delayed.

Parameters (all in motion_params.yaml under vp_* prefix):
    vp_curvature_gain   — divisor sensitivity to curvature (larger → slower on curves)
    vp_goal_ramp_dist   — start decelerating within this distance of goal (m)
    vp_min_vel          — floor speed while active (m/s)
    vp_accel_limit      — max speed increase per second (m/s²)
"""

import math


class VelocityProfiler:
    def __init__(self, params: dict):
        self.v_max      = params.get('max_linear_vel',     0.4)
        self.v_min      = params.get('vp_min_vel',         0.05)
        self.k_curv     = params.get('vp_curvature_gain',  3.0)
        self.goal_ramp  = params.get('vp_goal_ramp_dist',  1.5)
        self.a_max      = params.get('vp_accel_limit',     0.5)
        self._v_prev    = 0.0

    def reset(self):
        self._v_prev = 0.0

    # ------------------------------------------------------------------

    def compute(self, path: list, path_idx: int,
                dist_to_goal: float, obstacle_factor: float,
                dt: float) -> float:
        """
        Returns the profiled desired speed (m/s).

        path           — current path waypoints
        path_idx       — current waypoint index (curvature look-ahead starts here)
        dist_to_goal   — Euclidean distance from robot to goal (m)
        obstacle_factor — 1.0 (clear) | 0.5 (warn zone) | 0.0 (stop)
        dt             — seconds since last call
        """
        # ── 1. Curvature-based speed ──────────────────────────────────────
        curvature = self._menger_curvature(path, path_idx, window=6)
        v_curv = self.v_max / (1.0 + self.k_curv * abs(curvature))

        # ── 2. Goal proximity ramp-down ───────────────────────────────────
        if dist_to_goal < self.goal_ramp:
            ratio  = dist_to_goal / max(self.goal_ramp, 1e-3)
            v_goal = self.v_min + (self.v_max - self.v_min) * ratio
        else:
            v_goal = self.v_max

        # ── 3. Obstacle factor ────────────────────────────────────────────
        v_desired = min(v_curv, v_goal) * obstacle_factor
        v_desired = max(self.v_min, v_desired)

        # ── 4. Acceleration limit (ramp up; instant ramp down for safety) ─
        v_ramp  = self._v_prev + self.a_max * max(dt, 0.001)
        v_final = min(v_desired, v_ramp)
        self._v_prev = v_final

        return v_final

    # ------------------------------------------------------------------

    @staticmethod
    def _menger_curvature(path: list, idx: int, window: int = 6) -> float:
        """
        Estimate path curvature at idx using three points spaced `window` apart
        via the Menger curvature formula:  κ = 2·area / (|ab|·|bc|·|ac|)
        """
        n = len(path)
        if idx >= n - 2:
            return 0.0

        # Near the goal on short replanned paths, shrinking the window keeps
        # the three-point curvature sample valid instead of collapsing mid/end
        # onto the same waypoint and falsely reporting zero curvature.
        end = min(idx + window, n - 1)
        if end - idx < 2:
            return 0.0

        mid = idx + max(1, (end - idx) // 2)
        if mid >= end:
            mid = end - 1

        ax, ay = path[idx]
        bx, by = path[mid]
        cx, cy = path[end]

        # Twice the triangle area (cross product magnitude)
        area2 = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))
        dab   = math.hypot(bx - ax, by - ay)
        dbc   = math.hypot(cx - bx, cy - by)
        dac   = math.hypot(cx - ax, cy - ay)
        denom = dab * dbc * dac

        return area2 / denom if denom > 1e-6 else 0.0
