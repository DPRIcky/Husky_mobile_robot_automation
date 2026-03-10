"""
PID path-following controller.

Error signal: yaw error to the lookahead waypoint.
Adds integral (removes steady-state drift) and derivative (damps overshoot)
terms on top of a pure proportional yaw controller.

State:  _integral, _prev_error  (reset on new path)
"""

import math
from .utils import normalize_angle, advance_lookahead, signed_cte


class PIDController:
    def __init__(self, params: dict):
        self.kp        = params.get('pid_kp',           1.5)
        self.ki        = params.get('pid_ki',           0.05)
        self.kd        = params.get('pid_kd',           0.2)
        self.kp_v      = params.get('kp_linear',        0.5)
        self.max_v     = params.get('max_linear_vel',   0.4)
        self.max_w     = params.get('max_angular_vel',  1.0)
        self.lookahead = params.get('lookahead_distance', 0.5)
        self.windup    = params.get('pid_windup_limit',  2.0)  # rad·s

        self._integral  = 0.0
        self._prev_error = 0.0
        self._has_prev  = False

    def reset(self):
        self._integral  = 0.0
        self._prev_error = 0.0
        self._has_prev  = False

    def compute(self, rx: float, ry: float, ryaw: float,
                path: list, path_idx: int, v: float, dt: float):
        """
        Returns (v_cmd, w_cmd, cte, heading_error, new_path_idx).
        """
        new_idx = advance_lookahead(rx, ry, path, path_idx, self.lookahead)
        tx, ty  = path[new_idx]

        dist        = math.hypot(tx - rx, ty - ry)
        desired_yaw = math.atan2(ty - ry, tx - rx)
        error       = normalize_angle(desired_yaw - ryaw)

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral  = max(-self.windup, min(self.windup, self._integral))

        # Derivative (zero on first call to avoid spike)
        if self._has_prev:
            derivative = (error - self._prev_error) / max(dt, 1e-3)
        else:
            derivative = 0.0
            self._has_prev = True
        self._prev_error = error

        w = self.kp * error + self.ki * self._integral + self.kd * derivative
        w = max(-self.max_w, min(self.max_w, w))

        # Speed: scale with v_desired, reduce for large yaw errors
        yaw_factor = max(0.1, 1.0 - abs(error) / math.pi)
        v_cmd = v * yaw_factor

        cte = signed_cte(rx, ry, path, new_idx)

        return v_cmd, w, cte, error, new_idx
