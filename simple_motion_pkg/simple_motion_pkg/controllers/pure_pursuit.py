"""
Pure Pursuit geometric path-following controller.

Finds the lookahead point at distance L along the path, transforms it to the
robot body frame, and computes the arc curvature needed to reach it:

    κ = 2·y_L / L²       (y_L = lateral offset to lookahead in robot frame)
    ω = v × κ

This gives exact arc following rather than the proportional-yaw approximation
used by the P-controller.
"""

import math
from .utils import normalize_angle, advance_lookahead, signed_cte


class PurePursuitController:
    def __init__(self, params: dict):
        self.L     = params.get('lookahead_distance', 0.5)
        self.max_v = params.get('max_linear_vel',     0.4)
        self.max_w = params.get('max_angular_vel',    1.0)

    def reset(self):
        pass  # no internal state

    def compute(self, rx: float, ry: float, ryaw: float,
                path: list, path_idx: int, v: float, dt: float):
        """
        Returns (v_cmd, w_cmd, cte, heading_error, new_path_idx).
        """
        new_idx = advance_lookahead(rx, ry, path, path_idx, self.L)
        tx, ty  = path[new_idx]

        dx = tx - rx
        dy = ty - ry

        # Transform lookahead point to robot body frame
        cos_h   = math.cos(ryaw)
        sin_h   = math.sin(ryaw)
        # y_robot is the lateral component (positive = lookahead is to the left)
        y_robot = -dx * sin_h + dy * cos_h

        dist  = math.hypot(dx, dy)
        L_eff = max(dist, self.L)   # never divide by < L

        # Arc curvature → angular velocity
        kappa = 2.0 * y_robot / (L_eff ** 2)
        w     = v * kappa
        w     = max(-self.max_w, min(self.max_w, w))

        v_cmd = v  # pure pursuit does not alter speed; profiler handles that

        # Metrics
        desired_yaw = math.atan2(dy, dx)
        he  = normalize_angle(desired_yaw - ryaw)
        cte = signed_cte(rx, ry, path, new_idx)

        return v_cmd, w, cte, he, new_idx
