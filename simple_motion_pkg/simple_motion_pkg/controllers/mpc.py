"""
MPC (Model Predictive Control) path-following controller.

Prediction model:  unicycle
    x     += v · cos(θ) · dt
    y     += v · sin(θ) · dt
    θ     += ω · dt

Optimisation horizon: N steps
Decision variables:   [v₀, ω₀, v₁, ω₁, …, v_{N-1}, ω_{N-1}]

Cost per step:
    Q_cte · cte²  +  Q_he · he²  +  R_v · v²  +  R_w · ω²

Solved with scipy.optimize.minimize (SLSQP), warm-started from the
previous solution shifted one step forward.

Tuning:
    mpc_horizon   — prediction steps N      (more → smoother, slower)
    mpc_q_cte     — weight on cross-track error
    mpc_q_he      — weight on heading error
    mpc_r_v       — weight on linear speed (regularises solution)
    mpc_r_w       — weight on angular velocity (penalises sharp turns)
"""

import math
import numpy as np
from scipy.optimize import minimize

from .utils import normalize_angle, nearest_on_path, advance_lookahead


class MPCController:
    def __init__(self, params: dict):
        self.N       = int(params.get('mpc_horizon',     8))
        self.max_v   = params.get('max_linear_vel',      0.4)
        self.max_w   = params.get('max_angular_vel',     1.0)
        self.Q_cte   = params.get('mpc_q_cte',           5.0)
        self.Q_he    = params.get('mpc_q_he',            2.0)
        self.R_v     = params.get('mpc_r_v',             0.5)
        self.R_w     = params.get('mpc_r_w',             0.2)
        self.lookahead = params.get('lookahead_distance', 0.5)
        self._dt_nom = 1.0 / params.get('control_rate',  10.0)
        self._warm   = None   # warm-start control sequence

    def reset(self):
        self._warm = None

    # ------------------------------------------------------------------

    def compute(self, rx: float, ry: float, ryaw: float,
                path: list, path_idx: int, v_desired: float, dt: float):
        """
        Returns (v_cmd, w_cmd, cte, heading_error, new_path_idx).
        """
        N      = self.N
        n_path = len(path)
        dt_use = dt if dt > 0 else self._dt_nom

        # Warm start: shift previous sequence 1 step, duplicate last
        if self._warm is None:
            u0 = np.array([v_desired, 0.0] * N, dtype=float)
        else:
            u0 = np.roll(self._warm, -2)
            u0[-2] = self._warm[-2]
            u0[-1] = self._warm[-1]

        # Bounds: v ∈ [0, max_v], ω ∈ [-max_w, max_w]
        bounds = [(0.0, self.max_v), (-self.max_w, self.max_w)] * N

        x0 = (rx, ry, ryaw)

        result = minimize(
            self._cost,
            u0,
            args=(x0, path, path_idx, n_path, dt_use),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 40, 'ftol': 1e-4},
        )

        u_opt      = result.x if result.success else u0
        self._warm = u_opt.copy()

        v_cmd = float(np.clip(u_opt[0], 0.0,      self.max_v))
        w_cmd = float(np.clip(u_opt[1], -self.max_w, self.max_w))

        # Metrics at current state
        best_idx, cte, path_heading = nearest_on_path(rx, ry, path, path_idx)
        he = normalize_angle(path_heading - ryaw)

        # Advance lookahead index
        new_idx = advance_lookahead(rx, ry, path, path_idx, self.lookahead)

        return v_cmd, w_cmd, cte, he, new_idx

    # ------------------------------------------------------------------

    def _cost(self, u_flat, x0, path, start_idx, n_path, dt):
        x, y, theta = x0
        cost = 0.0

        for i in range(self.N):
            v = u_flat[2 * i]
            w = u_flat[2 * i + 1]

            # Unicycle step
            x     += v * math.cos(theta) * dt
            y     += v * math.sin(theta) * dt
            theta += w * dt

            # Reference point along path
            ref_idx = min(start_idx + i, n_path - 1)
            rx_ref, ry_ref = path[ref_idx]

            # Cross-track error (Euclidean; MPC minimises absolute deviation)
            cte = math.hypot(x - rx_ref, y - ry_ref)

            # Heading error vs path tangent
            if ref_idx < n_path - 1:
                dxp = path[ref_idx + 1][0] - path[ref_idx][0]
                dyp = path[ref_idx + 1][1] - path[ref_idx][1]
                path_h = math.atan2(dyp, dxp)
            else:
                path_h = theta
            he = abs(normalize_angle(path_h - theta))

            cost += (self.Q_cte * cte ** 2
                     + self.Q_he * he  ** 2
                     + self.R_v  * v   ** 2
                     + self.R_w  * w   ** 2)

        return cost
