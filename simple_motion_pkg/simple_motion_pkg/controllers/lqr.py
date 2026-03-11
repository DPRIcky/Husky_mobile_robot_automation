"""
LQR (Linear-Quadratic Regulator) path-following controller.

State:   x = [cte, he]
Input:   u = [ω]

Sign conventions:
    cte = nearest_on_path CTE → positive = robot to the LEFT of path
    he  = path_heading − ryaw → positive = path points LEFT of robot heading
    ω   > 0 → counterclockwise (left) turn

Linearised unicycle (correct signs):
    ĊTE = −v · he      (robot pointing left of path → he < 0 → cte increases)
    ḣe  = −ω           (left turn increases ryaw → he = path_heading−ryaw decreases)

Continuous:  A = [[0, −v], [0, 0]]   B = [[0], [−1]]
Discretised: Ad = I + A·dt            Bd = B·dt

Gain K is computed by solving the Discrete Algebraic Riccati Equation (DARE)
and is recomputed whenever speed or dt changes significantly.

Tuning:
    lqr_q_cte   — penalty on cross-track error   (larger → tighter tracking)
    lqr_q_he    — penalty on heading error        (larger → faster heading correction)
    lqr_r_w     — penalty on angular velocity     (larger → smoother turns)
"""

import math
import numpy as np
from scipy.linalg import solve_discrete_are

from .utils import normalize_angle, nearest_on_path


class LQRController:
    def __init__(self, params: dict):
        self.max_v     = params.get('max_linear_vel',   0.4)
        self.max_w     = params.get('max_angular_vel',  1.0)
        self.lookahead = params.get('lookahead_distance', 0.5)

        q_cte = params.get('lqr_q_cte', 5.0)
        q_he  = params.get('lqr_q_he',  2.0)
        r_w   = params.get('lqr_r_w',   1.0)
        self.Q = np.diag([q_cte, q_he])
        self.R = np.array([[r_w]])

        self._K        = None
        self._last_v   = None
        self._last_dt  = None

    def reset(self):
        self._K       = None
        self._last_v  = None
        self._last_dt = None

    # ------------------------------------------------------------------

    def _update_gain(self, v: float, dt: float):
        """Recompute K only when v or dt changes meaningfully."""
        if (self._K is not None
                and self._last_v  is not None
                and abs(v  - self._last_v)  < 0.02
                and abs(dt - self._last_dt) < 0.001):
            return

        v_eff = max(abs(v), 0.05)           # prevent singular A at v≈0
        A  = np.array([[0.0, -v_eff], [0.0, 0.0]])  # ĊTE = -v·he
        B  = np.array([[0.0], [-1.0]])               # ḣe  = -ω
        Ad = np.eye(2) + A * dt
        Bd = B * dt

        try:
            P       = solve_discrete_are(Ad, Bd, self.Q, self.R)
            self._K = np.linalg.inv(self.R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
        except Exception:
            # Fallback: pure heading-error gain
            self._K = np.array([[0.0, 2.0]])

        self._last_v  = v
        self._last_dt = dt

    # ------------------------------------------------------------------

    def compute(self, rx: float, ry: float, ryaw: float,
                path: list, path_idx: int, v: float, dt: float):
        """
        Returns (v_cmd, w_cmd, cte, heading_error, new_path_idx).
        """
        self._update_gain(v, dt)

        best_idx, cte, path_heading = nearest_on_path(rx, ry, path, path_idx)
        he = normalize_angle(path_heading - ryaw)

        state = np.array([cte, he])
        w     = float(-(self._K @ state)[0])
        w     = max(-self.max_w, min(self.max_w, w))

        # Speed: maintain desired speed but slow down for large heading errors
        v_cmd = v * max(0.0, math.cos(he))

        return v_cmd, w, cte, he, best_idx
