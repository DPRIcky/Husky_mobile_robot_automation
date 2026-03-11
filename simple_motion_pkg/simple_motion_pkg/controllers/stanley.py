"""
Stanley path-following controller.

Reference: Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge"

Sign conventions (matching nearest_on_path in utils.py):
    cte > 0  →  robot is to the LEFT  of the path tangent
    he  > 0  →  path_heading > ryaw   →  path points LEFT of robot heading
    w   > 0  →  counterclockwise (left) turn

Control law:
    heading_error  = normalise(path_tangent_heading − robot_heading)
    cross_track_error = signed perpendicular distance (positive = robot LEFT of path)
    ω = heading_error − atan2(k_e × cte, max(v, v_min))

    The minus sign on the CTE term is essential: if robot is to the LEFT (cte > 0),
    we need to steer RIGHT (negative ω contribution), hence subtract.
    v = v_desired × max(0, cos(heading_error))   — slow down for big misalignments
"""

import math
from .utils import normalize_angle, nearest_on_path


class StanleyController:
    def __init__(self, params: dict):
        self.k_e    = params.get('stanley_k',      1.0)   # cross-track gain
        self.v_min  = params.get('stanley_v_min',  0.10)  # min speed for denominator
        self.max_w  = params.get('max_angular_vel', 1.0)
        self.max_v  = params.get('max_linear_vel',  0.4)

    def reset(self):
        pass  # no internal state

    def compute(self, rx: float, ry: float, ryaw: float,
                path: list, path_idx: int, v: float, dt: float):
        """
        Returns (v_cmd, w_cmd, cte, heading_error, new_path_idx).
        """
        best_idx, cte, path_heading = nearest_on_path(rx, ry, path, path_idx)

        heading_error = normalize_angle(path_heading - ryaw)

        # Stanley steering law
        # Subtract CTE term: cte > 0 = robot to LEFT → need right turn (negative ω)
        v_eff = max(abs(v), self.v_min)
        w = heading_error - math.atan2(self.k_e * cte, v_eff)
        w = max(-self.max_w, min(self.max_w, w))

        # Speed: reduce proportionally to heading misalignment
        v_cmd = v * max(0.0, math.cos(heading_error))

        return v_cmd, w, cte, heading_error, best_idx
