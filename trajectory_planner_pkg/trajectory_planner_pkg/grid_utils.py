"""Utilities for converting OccupancyGrid ↔ numpy and world↔grid coords."""

import math
import numpy as np
from nav_msgs.msg import OccupancyGrid


def occupancy_grid_to_numpy(msg: OccupancyGrid,
                            occupied_threshold: int = 65,
                            allow_unknown: bool = True) -> np.ndarray:
    """Convert OccupancyGrid to binary numpy array (0=free, 1=obstacle).

    Cells with value >= occupied_threshold → obstacle.
    Unknown cells (-1): free if allow_unknown else obstacle.
    """
    raw = np.array(msg.data, dtype=np.int8).reshape(
        (msg.info.height, msg.info.width))
    grid = np.zeros_like(raw, dtype=np.uint8)
    grid[raw >= occupied_threshold] = 1
    if not allow_unknown:
        grid[raw < 0] = 1
    return grid


def inflate_grid(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Inflate obstacles by radius_cells using a circular kernel."""
    if radius_cells <= 0:
        return grid.copy()
    from scipy.ndimage import binary_dilation
    # Build circular structuring element
    size = 2 * radius_cells + 1
    y, x = np.ogrid[-radius_cells:radius_cells + 1,
                     -radius_cells:radius_cells + 1]
    kernel = (x * x + y * y) <= radius_cells * radius_cells
    inflated = binary_dilation(grid.astype(bool), structure=kernel).astype(np.uint8)
    return inflated


def world_to_grid(wx: float, wy: float, origin_x: float, origin_y: float,
                  resolution: float) -> tuple:
    """World (x, y) → grid (row, col)."""
    col = int(math.floor((wx - origin_x) / resolution))
    row = int(math.floor((wy - origin_y) / resolution))
    return (row, col)


def grid_to_world(row: int, col: int, origin_x: float, origin_y: float,
                  resolution: float) -> tuple:
    """Grid (row, col) → world (x, y) at cell centre."""
    wx = origin_x + (col + 0.5) * resolution
    wy = origin_y + (row + 0.5) * resolution
    return (wx, wy)
