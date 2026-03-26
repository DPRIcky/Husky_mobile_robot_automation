"""
MPC (Model Predictive Control) path-following controller.

Prediction model:  unicycle
    x     += v · cos(θ) · dt
    y     += v · sin(θ) · dt
    θ     += ω · dt

Optimisation horizon: N steps
Decision variables:   [v₀, ω₀, v₁, ω₁, …, v_{N-1}, ω_{N-1}]

Cost per step:
    Q_cte · cte²  +  Q_he · he²  +  R_v · (v − v_ref)²  +  R_w · ω²
Terminal step (i == N-1) has 3× heavier Q_cte / Q_he.

Reference assignment (distance-based):
    At each horizon step i the reference waypoint is the one at cumulative
    travel distance sum(v_k · dt, k=0..i) along the path from the current
    nearest waypoint.  This ensures the reference does NOT jump ahead faster
    than the robot actually moves, regardless of waypoint density.

Solved with scipy.optimize.minimize (SLSQP), warm-started from the
previous solution shifted one step forward.

Tuning:
    mpc_horizon   — prediction steps N      (more → smoother, slower)
    mpc_q_cte     — weight on cross-track error
    mpc_q_he      — weight on heading error
    mpc_r_v       — weight on (v − v_desired)²  (speed-tracking regulariser)
    mpc_r_w       — weight on angular velocity (penalises sharp turns)
"""

import math
import numpy as np
from scipy.optimize import minimize

from .utils import normalize_angle, nearest_on_path, advance_lookahead


class MPCController:
    def __init__(self, params: dict):
        self.N         = int(params.get('mpc_horizon',     8))
        self.max_v     = params.get('max_linear_vel',      0.4)
        self.max_w     = params.get('max_angular_vel',     1.0)
        self.Q_cte     = params.get('mpc_q_cte',           5.0)
        self.Q_he      = params.get('mpc_q_he',            2.0)
        self.R_v       = params.get('mpc_r_v',             0.5)
        self.R_w       = params.get('mpc_r_w',             0.2)
        self.lookahead = params.get('lookahead_distance',  0.5)
        self._dt_nom   = 1.0 / params.get('control_rate', 10.0)
        self._warm     = None   # warm-start control sequence

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

        # Find current nearest waypoint (more reliable than raw path_idx)
        best_idx, cte_now, path_heading_now = nearest_on_path(
            rx, ry, path, path_idx)

        # Pre-compute cumulative path distances from best_idx.
        # Covers up to N steps at max speed plus safety margin.
        max_reach = self.max_v * dt_use * N
        n_ahead   = min(N + 30, n_path - best_idx)
        cum_d     = [0.0]
        for k in range(best_idx, best_idx + n_ahead - 1):
            dx = path[k + 1][0] - path[k][0]
            dy = path[k + 1][1] - path[k][1]
            cum_d.append(cum_d[-1] + math.hypot(dx, dy))

        # Warm start: shift previous sequence one step, repeat last step.
        # If the previous solve fell into a near-zero-speed spin, bias the
        # next seed back toward v_desired so SLSQP does not keep restarting
        # from the same stalled local minimum.
        if self._warm is None:
            u0 = np.array([v_desired, 0.0] * N, dtype=float)
        else:
            u0 = np.roll(self._warm, -2)
            u0[-2] = self._warm[-2]
            u0[-1] = self._warm[-1]
            speed_seed = np.clip(u0[::2], 0.0, self.max_v)
            stale_thresh = max(0.05, 0.5 * min(v_desired, self.max_v))
            speed_seed[speed_seed < stale_thresh] = min(v_desired, self.max_v)
            u0[::2] = speed_seed

        # Bounds: v ∈ [0, max_v], ω ∈ [-max_w, max_w]
        bounds = [(0.0, self.max_v), (-self.max_w, self.max_w)] * N

        x0 = (rx, ry, ryaw)

        result = minimize(
            self._cost,
            u0,
            args=(x0, path, best_idx, n_path, dt_use, v_desired, cum_d),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-5},
        )

        u_opt      = result.x if result.success else u0
        self._warm = u_opt.copy()

        v_cmd = float(np.clip(u_opt[0], 0.0,        self.max_v))
        w_cmd = float(np.clip(u_opt[1], -self.max_w, self.max_w))

        # Advance lookahead index for path_idx bookkeeping
        new_idx = advance_lookahead(rx, ry, path, best_idx, self.lookahead)

        return v_cmd, w_cmd, cte_now, normalize_angle(path_heading_now - ryaw), new_idx

    # ------------------------------------------------------------------

    def _cost(self, u_flat, x0, path, start_idx, n_path, dt, v_ref, cum_d):
        x, y, theta = x0
        cost          = 0.0
        dist_traveled = 0.0
        n_cum         = len(cum_d)

        for i in range(self.N):
            v = u_flat[2 * i]
            w = u_flat[2 * i + 1]

            # Unicycle step
            x     += v * math.cos(theta) * dt
            y     += v * math.sin(theta) * dt
            theta += w * dt

            dist_traveled += v * dt

            # Distance-based reference: find the waypoint at dist_traveled
            # along the path from start_idx (last k where cum_d[k] <= dist)
            ref_offset = 0
            for k in range(n_cum):
                if cum_d[k] <= dist_traveled:
                    ref_offset = k
                else:
                    break
            ref_idx = min(start_idx + ref_offset, n_path - 1)

            rx_ref, ry_ref = path[ref_idx]

            # Path tangent at reference point
            if ref_idx < n_path - 1:
                dxp    = path[ref_idx + 1][0] - path[ref_idx][0]
                dyp    = path[ref_idx + 1][1] - path[ref_idx][1]
                path_h = math.atan2(dyp, dxp)
                mag    = math.hypot(dxp, dyp) + 1e-9
                tx_n   = dxp / mag
                ty_n   = dyp / mag
            else:
                path_h = theta
                tx_n   = math.cos(theta)
                ty_n   = math.sin(theta)

            # Signed cross-track error: positive = robot to LEFT of path
            ex  = x - rx_ref
            ey  = y - ry_ref
            cte = tx_n * ey - ty_n * ex

            # Signed heading error: positive = path heading left of robot
            he = normalize_angle(path_h - theta)

            # Terminal step: triple the tracking cost
            w_term = 3.0 if i == self.N - 1 else 1.0

            cost += (w_term * self.Q_cte * cte ** 2
                     + w_term * self.Q_he  * he  ** 2
                     + self.R_v * (v - v_ref) ** 2
                     + self.R_w * w ** 2)

        return cost
