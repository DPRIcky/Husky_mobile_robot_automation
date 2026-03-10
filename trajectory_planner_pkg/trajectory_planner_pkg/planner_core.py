"""
Grid-based planning algorithms: A*, Hybrid-A*, RRT*.
All operate on a 2D numpy occupancy grid (0=free, 1=obstacle).
"""

import heapq
import math
import random
import numpy as np
from typing import List, Tuple, Optional


# ---------------------------------------------------------------------------
# A* (grid-based, 8-connected)
# ---------------------------------------------------------------------------

def astar(grid: np.ndarray,
          start: Tuple[int, int],
          goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """Return list of (row, col) from start to goal, or None if no path."""
    rows, cols = grid.shape
    if grid[start[0], start[1]] != 0 or grid[goal[0], goal[1]] != 0:
        return None

    # 8-connected neighbours
    neighbours = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (-1, 1), (1, -1), (1, 1)]
    SQRT2 = math.sqrt(2.0)

    open_set: list = []
    heapq.heappush(open_set, (0.0, start))
    came_from: dict = {}
    g_score = {start: 0.0}

    def heuristic(a, b):
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return SQRT2 * min(dr, dc) + abs(dr - dc)  # octile

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dr, dc in neighbours:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                step = SQRT2 if (dr != 0 and dc != 0) else 1.0
                tentative_g = g_score[current] + step
                neighbour = (nr, nc)
                if tentative_g < g_score.get(neighbour, float('inf')):
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g
                    f = tentative_g + heuristic(neighbour, goal)
                    heapq.heappush(open_set, (f, neighbour))
    return None


# ---------------------------------------------------------------------------
# Hybrid-A*  (x, y, theta) with Reeds-Shepp-like discrete headings
# ---------------------------------------------------------------------------

def hybrid_astar(grid: np.ndarray,
                 start: Tuple[int, int, float],
                 goal: Tuple[int, int, float],
                 num_headings: int = 72,
                 step_size: float = 1.0,
                 steer_angles: list = None) -> Optional[List[Tuple[float, float, float]]]:
    """start/goal = (row, col, theta_rad). Returns list of (row, col, theta)."""
    if steer_angles is None:
        steer_angles = [-0.5, 0.0, 0.5]

    rows, cols = grid.shape
    sr, sc, stheta = start
    gr, gc, gtheta = goal
    if grid[int(sr), int(sc)] != 0 or grid[int(gr), int(gc)] != 0:
        return None

    def _disc(theta):
        return int(round(theta / (2.0 * math.pi) * num_headings)) % num_headings

    open_set: list = []
    sh = _disc(stheta)
    start_key = (int(sr), int(sc), sh)
    heapq.heappush(open_set, (0.0, start_key, (float(sr), float(sc), stheta)))
    came_from: dict = {}
    g_score = {start_key: 0.0}

    goal_tol_cells = 2.0
    goal_tol_heading = 1

    def heuristic(r, c):
        return math.hypot(r - gr, c - gc)

    while open_set:
        _, cur_key, cur_state = heapq.heappop(open_set)
        cr, cc, ctheta = cur_state

        if (math.hypot(cr - gr, cc - gc) < goal_tol_cells and
                abs(_disc(ctheta) - _disc(gtheta)) <= goal_tol_heading):
            path = [(cr, cc, ctheta)]
            k = cur_key
            while k in came_from:
                k, st = came_from[k]
                path.append(st)
            path.reverse()
            return path

        for steer in steer_angles:
            new_theta = ctheta + steer
            nr = cr + step_size * math.cos(new_theta)
            nc = cc + step_size * math.sin(new_theta)
            ri, ci = int(round(nr)), int(round(nc))
            if 0 <= ri < rows and 0 <= ci < cols and grid[ri, ci] == 0:
                nh = _disc(new_theta)
                nkey = (ri, ci, nh)
                tentative_g = g_score[cur_key] + step_size
                if tentative_g < g_score.get(nkey, float('inf')):
                    g_score[nkey] = tentative_g
                    came_from[nkey] = (cur_key, (nr, nc, new_theta))
                    f = tentative_g + heuristic(nr, nc)
                    heapq.heappush(open_set, (f, nkey, (nr, nc, new_theta)))
    return None


# ---------------------------------------------------------------------------
# RRT*
# ---------------------------------------------------------------------------

class _RRTNode:
    __slots__ = ('r', 'c', 'cost', 'parent')
    def __init__(self, r, c, cost=0.0, parent=None):
        self.r = r
        self.c = c
        self.cost = cost
        self.parent = parent

def rrt_star(grid: np.ndarray,
             start: Tuple[int, int],
             goal: Tuple[int, int],
             max_iter: int = 5000,
             step_size: float = 5.0,
             goal_sample_rate: float = 0.1,
             search_radius: float = 10.0) -> Optional[List[Tuple[int, int]]]:
    """Return list of (row, col) from start to goal, or None."""
    rows, cols = grid.shape
    if grid[start[0], start[1]] != 0 or grid[goal[0], goal[1]] != 0:
        return None

    rng = random.Random(42)
    root = _RRTNode(start[0], start[1])
    nodes: List[_RRTNode] = [root]

    def _dist(a, b):
        return math.hypot(a.r - b.r, a.c - b.c)

    def _collision_free(r0, c0, r1, c1):
        dist = math.hypot(r1 - r0, c1 - c0)
        steps = max(int(dist), 1)
        for i in range(steps + 1):
            t = i / steps
            ri = int(round(r0 + t * (r1 - r0)))
            ci = int(round(c0 + t * (c1 - c0)))
            if ri < 0 or ri >= rows or ci < 0 or ci >= cols:
                return False
            if grid[ri, ci] != 0:
                return False
        return True

    best_goal_node = None

    for _ in range(max_iter):
        # sample
        if rng.random() < goal_sample_rate:
            sr, sc = goal
        else:
            sr = rng.randint(0, rows - 1)
            sc = rng.randint(0, cols - 1)
        sample = _RRTNode(sr, sc)

        # nearest
        nearest = min(nodes, key=lambda n: _dist(n, sample))
        d = _dist(nearest, sample)
        if d < 1e-6:
            continue
        if d > step_size:
            ratio = step_size / d
            sr = int(round(nearest.r + ratio * (sr - nearest.r)))
            sc = int(round(nearest.c + ratio * (sc - nearest.c)))
        if sr < 0 or sr >= rows or sc < 0 or sc >= cols:
            continue
        if grid[sr, sc] != 0:
            continue
        if not _collision_free(nearest.r, nearest.c, sr, sc):
            continue

        new_node = _RRTNode(sr, sc, nearest.cost + math.hypot(sr - nearest.r, sc - nearest.c), nearest)

        # rewire (RRT*)
        for n in nodes:
            d2 = math.hypot(n.r - sr, n.c - sc)
            if d2 < search_radius and _collision_free(n.r, n.c, sr, sc):
                new_cost = n.cost + d2
                if new_cost < new_node.cost:
                    new_node.cost = new_cost
                    new_node.parent = n
        nodes.append(new_node)

        # rewire neighbours
        for n in nodes[:-1]:
            d2 = math.hypot(n.r - sr, n.c - sc)
            if d2 < search_radius and _collision_free(sr, sc, n.r, n.c):
                new_cost = new_node.cost + d2
                if new_cost < n.cost:
                    n.cost = new_cost
                    n.parent = new_node

        # check goal
        if math.hypot(sr - goal[0], sc - goal[1]) < step_size:
            if _collision_free(sr, sc, goal[0], goal[1]):
                goal_cost = new_node.cost + math.hypot(sr - goal[0], sc - goal[1])
                if best_goal_node is None or goal_cost < best_goal_node.cost:
                    best_goal_node = _RRTNode(goal[0], goal[1], goal_cost, new_node)

    if best_goal_node is None:
        return None

    path = []
    node = best_goal_node
    while node is not None:
        path.append((node.r, node.c))
        node = node.parent
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Post-processing utilities
# ---------------------------------------------------------------------------

def prune_collinear(path: List[Tuple[int, int]], tol: float = 0.5) -> List[Tuple[int, int]]:
    """Remove intermediate collinear points."""
    if len(path) <= 2:
        return list(path)
    pruned = [path[0]]
    for i in range(1, len(path) - 1):
        r0, c0 = pruned[-1]
        r1, c1 = path[i]
        r2, c2 = path[i + 1]
        # cross product magnitude
        cross = abs((r1 - r0) * (c2 - c0) - (c1 - c0) * (r2 - r0))
        if cross > tol:
            pruned.append(path[i])
    pruned.append(path[-1])
    return pruned


def shortcut_smooth(path: List[Tuple[int, int]],
                    grid: np.ndarray,
                    iterations: int = 50) -> List[Tuple[int, int]]:
    """Try random shortcuts to shorten path while staying collision-free."""
    if len(path) <= 2:
        return list(path)
    rows, cols = grid.shape
    rng = random.Random(123)
    result = list(path)
    for _ in range(iterations):
        if len(result) <= 2:
            break
        i = rng.randint(0, len(result) - 2)
        j = rng.randint(i + 1, len(result) - 1)
        if j - i <= 1:
            continue
        r0, c0 = result[i]
        r1, c1 = result[j]
        dist = math.hypot(r1 - r0, c1 - c0)
        steps = max(int(dist), 1)
        ok = True
        for s in range(steps + 1):
            t = s / steps
            ri = int(round(r0 + t * (r1 - r0)))
            ci = int(round(c0 + t * (c1 - c0)))
            if ri < 0 or ri >= rows or ci < 0 or ci >= cols or grid[ri, ci] != 0:
                ok = False
                break
        if ok:
            result = result[:i + 1] + result[j:]
    return result
