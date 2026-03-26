"""Shared geometric utilities used by all path-following controllers."""

import math


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def advance_lookahead(rx: float, ry: float, path: list,
                      idx: int, lookahead: float) -> int:
    """
    Advance idx to a waypoint that is both ahead of the robot on the path and
    at least `lookahead` metres away.

    Sparse paths can leave a waypoint behind the robot while still farther than
    `lookahead`; skipping those prevents controllers from targeting waypoints
    the robot has already passed.
    """
    n = len(path)
    while idx < n - 1:
        px, py = path[idx]
        nx, ny = path[idx + 1]
        sx = nx - px
        sy = ny - py

        # If the robot has already progressed past this waypoint along the
        # current path segment, keep moving the target forward.
        if sx * sx + sy * sy > 1e-9:
            progress = (rx - px) * sx + (ry - py) * sy
            if progress > 0.0:
                idx += 1
                continue

        if math.hypot(px - rx, py - ry) >= lookahead:
            break
        idx += 1
    return idx


def nearest_on_path(rx: float, ry: float, path: list,
                    path_idx: int, window: int = 25):
    """
    Find the nearest path point to the robot within a search window.

    Returns (idx, signed_cte, path_heading) where:
      - idx          nearest waypoint index
      - signed_cte   lateral error; positive = robot is to the LEFT of path
                     (2-D cross product: tangent × robot_offset > 0 ↔ LEFT)
      - path_heading heading of path tangent at nearest point (radians)
    """
    n = len(path)
    search_start = max(0, path_idx - 2)
    search_end   = min(n - 1, path_idx + window)

    best_idx  = path_idx
    best_dist = float('inf')
    for i in range(search_start, search_end + 1):
        d = math.hypot(path[i][0] - rx, path[i][1] - ry)
        if d < best_dist:
            best_dist = d
            best_idx  = i

    # Tangent direction at best_idx
    if best_idx < n - 1:
        dx = path[best_idx + 1][0] - path[best_idx][0]
        dy = path[best_idx + 1][1] - path[best_idx][1]
    else:
        dx = path[best_idx][0] - path[best_idx - 1][0]
        dy = path[best_idx][1] - path[best_idx - 1][1]
    path_heading = math.atan2(dy, dx)

    # Signed cross-track error via 2-D cross product
    # Tangent unit vector
    mag   = math.hypot(dx, dy) + 1e-9
    tx_n  = dx / mag
    ty_n  = dy / mag
    # Vector from nearest point to robot
    ex = rx - path[best_idx][0]
    ey = ry - path[best_idx][1]
    # cte = tx_n × (ex, ey): positive → robot is to the left of path direction
    cte = tx_n * ey - ty_n * ex

    return best_idx, cte, path_heading


def signed_cte(rx: float, ry: float, path: list, path_idx: int) -> float:
    """Convenience: return only the signed CTE from nearest_on_path."""
    _, cte, _ = nearest_on_path(rx, ry, path, path_idx)
    return cte
