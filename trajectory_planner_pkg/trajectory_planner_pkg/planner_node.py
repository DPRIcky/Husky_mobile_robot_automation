"""
ROS 2 planner node — subscribes to OccupancyGrid and goal pose, publishes
nav_msgs/Path using A* (default), Hybrid-A*, or RRT*.

Compare mode (compare_mode: true):
  Runs ALL THREE planners on every goal and publishes each on a separate topic
  so you can see them side-by-side in RViz.  Paths are shown side-by-side for visual comparison — robot does NOT move.
  Set compare_mode: false to drive with a single planner.

  Topics published in compare mode:
    /planned_path_astar         — A* result  (green in RViz)
    /planned_path_hybrid_astar  — Hybrid-A*  (cyan in RViz)
    /planned_path_rrtstar       — RRT*        (orange in RViz)
    /planned_path               — NOT published (robot stays still)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from trajectory_planner_pkg.grid_utils import (
    occupancy_grid_to_numpy, inflate_grid, world_to_grid, grid_to_world,
)
from trajectory_planner_pkg.planner_core import (
    astar, hybrid_astar, rrt_star, prune_collinear, shortcut_smooth,
)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # ---- Parameters ----
        self.declare_parameter('planner_type', 'astar')
        self.declare_parameter('map_topic', '/a300_00000/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('inflation_radius_m', 0.40)
        self.declare_parameter('occupied_threshold', 85)
        self.declare_parameter('allow_unknown', True)
        # Hybrid-A* params
        self.declare_parameter('hybrid_num_headings', 72)
        self.declare_parameter('hybrid_step_size', 1.0)
        # RRT* params
        self.declare_parameter('rrt_max_iter', 8000)
        self.declare_parameter('rrt_step_size', 5.0)
        self.declare_parameter('rrt_goal_sample_rate', 0.10)
        self.declare_parameter('rrt_search_radius', 10.0)
        # Compare mode
        self.declare_parameter('compare_mode', False)

        self.planner_type = self.get_parameter('planner_type').value
        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.inflation_radius_m = self.get_parameter('inflation_radius_m').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.allow_unknown = self.get_parameter('allow_unknown').value
        self.hybrid_num_headings  = self.get_parameter('hybrid_num_headings').value
        self.hybrid_step_size     = self.get_parameter('hybrid_step_size').value
        self.rrt_max_iter         = self.get_parameter('rrt_max_iter').value
        self.rrt_step_size        = self.get_parameter('rrt_step_size').value
        self.rrt_goal_sample_rate = self.get_parameter('rrt_goal_sample_rate').value
        self.rrt_search_radius    = self.get_parameter('rrt_search_radius').value
        self.compare_mode         = self.get_parameter('compare_mode').value

        self.get_logger().info(
            f'Planner starting — type={self.planner_type}, '
            f'map={map_topic}, goal={goal_topic}, path={path_topic}')

        # ---- State ----
        self._grid: np.ndarray | None = None
        self._grid_inflated: np.ndarray | None = None
        self._map_info = None  # OccupancyGrid.info

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- Subscribers ----
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)

        # ---- Publishers ----
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, '/planner_debug', 10)
        # Per-algorithm publishers (always created; only used in compare_mode)
        self._path_pub_astar  = self.create_publisher(Path, '/planned_path_astar', 10)
        self._path_pub_hybrid = self.create_publisher(Path, '/planned_path_hybrid_astar', 10)
        self._path_pub_rrt    = self.create_publisher(Path, '/planned_path_rrtstar', 10)

        mode_str = 'COMPARE (all 3 planners)' if self.compare_mode else self.planner_type
        self.get_logger().info(f'Waiting for map on {map_topic} … mode={mode_str}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        self._map_info = msg.info
        self._grid = occupancy_grid_to_numpy(
            msg, self.occupied_threshold, self.allow_unknown)
        radius_cells = max(
            int(math.ceil(self.inflation_radius_m / msg.info.resolution)), 0)
        self._grid_inflated = inflate_grid(self._grid, radius_cells)
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'res={msg.info.resolution:.3f}m, '
            f'inflation={radius_cells} cells')

    def _goal_cb(self, msg: PoseStamped):
        if self._grid_inflated is None or self._map_info is None:
            self.get_logger().warn('Goal received but no map available yet.')
            return

        # Get start pose from TF
        start_xy = self._get_robot_xy()
        if start_xy is None:
            self.get_logger().error(
                f'Cannot look up TF {self.global_frame} → {self.base_frame}. '
                'Is localization running?')
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        info = self._map_info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution

        start_rc = world_to_grid(start_xy[0], start_xy[1], ox, oy, res)
        goal_rc = world_to_grid(goal_x, goal_y, ox, oy, res)

        rows, cols = self._grid_inflated.shape
        if not (0 <= start_rc[0] < rows and 0 <= start_rc[1] < cols):
            self.get_logger().error('Start pose is outside the map bounds.')
            return
        if not (0 <= goal_rc[0] < rows and 0 <= goal_rc[1] < cols):
            self.get_logger().error('Goal pose is outside the map bounds.')
            return

        if self._grid_inflated[start_rc[0], start_rc[1]] != 0:
            self.get_logger().warn(
                'Start cell is inside inflated obstacle — trying un-inflated grid.')
            grid = self._grid
        else:
            grid = self._grid_inflated

        # Snap goal to nearest free cell if it falls inside the inflated zone
        if grid[goal_rc[0], goal_rc[1]] != 0:
            snapped = self._nearest_free_cell(grid, goal_rc)
            if snapped is None:
                self.get_logger().error(
                    'Goal is inside obstacle and no free cell found nearby.')
                return
            self.get_logger().warn(
                f'Goal cell {goal_rc} is inside inflated obstacle — '
                f'snapped to {snapped}')
            goal_rc = snapped

        if self.compare_mode:
            self._plan_compare(grid, start_rc, goal_rc, start_xy, goal_x, goal_y, ox, oy, res)
        else:
            self.get_logger().info(
                f'Planning {self.planner_type}: '
                f'start=({start_xy[0]:.2f},{start_xy[1]:.2f}) → '
                f'goal=({goal_x:.2f},{goal_y:.2f})')
            path_cells = self._run_planner(grid, start_rc, goal_rc, goal_x, goal_y)
            if path_cells is None:
                self.get_logger().warn('Planner found NO path.')
                self._publish_debug_markers([], start_rc, goal_rc)
                return
            path_cells = prune_collinear(path_cells)
            path_cells = shortcut_smooth(path_cells, grid, iterations=80)
            self.get_logger().info(f'Path found: {len(path_cells)} waypoints')
            self._path_pub.publish(
                self._cells_to_path_msg(path_cells, ox, oy, res))
            self._publish_debug_markers(path_cells, start_rc, goal_rc)

    # ------------------------------------------------------------------
    # Compare mode — run all 3, publish each, select best for robot
    # ------------------------------------------------------------------

    def _plan_compare(self, grid, start_rc, goal_rc, start_xy, goal_x, goal_y, ox, oy, res):
        self.get_logger().info(
            f'COMPARE MODE — running A*, Hybrid-A*, RRT* '
            f'start=({start_xy[0]:.2f},{start_xy[1]:.2f}) → '
            f'goal=({goal_x:.2f},{goal_y:.2f})')

        start_yaw = self._get_robot_yaw() or 0.0
        goal_yaw  = math.atan2(goal_rc[0] - start_rc[0], goal_rc[1] - start_rc[1])

        results = {}  # name → (path_cells, elapsed_s, length_m)

        # ── A* ──────────────────────────────────────────────────────────
        t0 = time.monotonic()
        p = astar(grid, start_rc, goal_rc)
        t_astar = time.monotonic() - t0
        if p:
            p = shortcut_smooth(prune_collinear(p), grid, iterations=80)
            results['astar'] = (p, t_astar, _path_length(p, res))
            self._path_pub_astar.publish(self._cells_to_path_msg(p, ox, oy, res))
        else:
            self._path_pub_astar.publish(Path())   # clear old display
        self.get_logger().info(
            f'  A*:          {"OK  " if p else "FAIL"} '
            f'{len(p) if p else 0:3d} pts  '
            f'{results["astar"][2]:.2f}m  {t_astar*1000:.0f}ms' if p else
            f'  A*:          FAIL  {t_astar*1000:.0f}ms')

        # ── Hybrid-A* ───────────────────────────────────────────────────
        t0 = time.monotonic()
        p = hybrid_astar(
            grid,
            (start_rc[0], start_rc[1], start_yaw),
            (goal_rc[0],  goal_rc[1],  goal_yaw),
            num_headings=self.hybrid_num_headings,
            step_size=self.hybrid_step_size,
        )
        t_hybrid = time.monotonic() - t0
        if p:
            p = [(int(round(r)), int(round(c))) for r, c, _ in p]
            p = shortcut_smooth(prune_collinear(p), grid, iterations=80)
            results['hybrid_astar'] = (p, t_hybrid, _path_length(p, res))
            self._path_pub_hybrid.publish(self._cells_to_path_msg(p, ox, oy, res))
        else:
            self._path_pub_hybrid.publish(Path())
        self.get_logger().info(
            f'  Hybrid-A*:   {"OK  " if p else "FAIL"} '
            f'{len(p) if p else 0:3d} pts  '
            f'{results["hybrid_astar"][2]:.2f}m  {t_hybrid*1000:.0f}ms' if p else
            f'  Hybrid-A*:   FAIL  {t_hybrid*1000:.0f}ms')

        # ── RRT* ────────────────────────────────────────────────────────
        t0 = time.monotonic()
        p = rrt_star(
            grid, start_rc, goal_rc,
            max_iter=self.rrt_max_iter,
            step_size=self.rrt_step_size,
            goal_sample_rate=self.rrt_goal_sample_rate,
            search_radius=self.rrt_search_radius,
        )
        t_rrt = time.monotonic() - t0
        if p:
            p = shortcut_smooth(prune_collinear(p), grid, iterations=80)
            results['rrtstar'] = (p, t_rrt, _path_length(p, res))
            self._path_pub_rrt.publish(self._cells_to_path_msg(p, ox, oy, res))
        else:
            self._path_pub_rrt.publish(Path())
        self.get_logger().info(
            f'  RRT*:        {"OK  " if p else "FAIL"} '
            f'{len(p) if p else 0:3d} pts  '
            f'{results["rrtstar"][2]:.2f}m  {t_rrt*1000:.0f}ms' if p else
            f'  RRT*:        FAIL  {t_rrt*1000:.0f}ms')

        if not results:
            self.get_logger().warn('All planners failed — no path to display.')
            self._publish_debug_markers([], start_rc, goal_rc)
            return

        # In compare mode the robot stays still — paths are for visual comparison only.
        # Log a summary table so results are easy to compare in the terminal.
        self.get_logger().info('  ── Summary ──────────────────────────────────')
        for name, (_, elapsed, length) in sorted(results.items(), key=lambda x: x[1][2]):
            self.get_logger().info(
                f'    {name:20s}  {length:.2f}m  {elapsed*1000:.0f}ms')
        self.get_logger().info(
            '  (compare_mode=true: robot does not move — set compare_mode: false to drive)')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _cells_to_path_msg(self, path_cells, ox, oy, res) -> 'Path':
        """Convert list of (row,col) grid cells to a nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.global_frame
        for r, c in path_cells:
            wx, wy = grid_to_world(r, c, ox, oy, res)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        # Orient each pose toward the next waypoint
        for i in range(len(path_msg.poses) - 1):
            p0 = path_msg.poses[i].pose.position
            p1 = path_msg.poses[i + 1].pose.position
            yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
            path_msg.poses[i].pose.orientation = _yaw_to_quaternion(yaw)
        if len(path_msg.poses) >= 2:
            path_msg.poses[-1].pose.orientation = path_msg.poses[-2].pose.orientation
        return path_msg

    # ------------------------------------------------------------------
    # Planner dispatch (single-planner mode)
    # ------------------------------------------------------------------

    def _run_planner(self, grid, start_rc, goal_rc, goal_x, goal_y):
        if self.planner_type == 'hybrid_astar':
            start_yaw = self._get_robot_yaw() or 0.0
            goal_yaw = math.atan2(goal_rc[0] - start_rc[0],
                                  goal_rc[1] - start_rc[1])
            result = hybrid_astar(
                grid,
                (start_rc[0], start_rc[1], start_yaw),
                (goal_rc[0], goal_rc[1], goal_yaw),
                num_headings=self.hybrid_num_headings,
                step_size=self.hybrid_step_size,
            )
            if result is not None:
                return [(int(round(r)), int(round(c))) for r, c, _ in result]
            return None

        elif self.planner_type == 'rrt_star':
            return rrt_star(
                grid, start_rc, goal_rc,
                max_iter=self.rrt_max_iter,
                step_size=self.rrt_step_size,
                goal_sample_rate=self.rrt_goal_sample_rate,
                search_radius=self.rrt_search_radius,
            )

        else:  # 'astar' default
            return astar(grid, start_rc, goal_rc)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _nearest_free_cell(grid, rc, search_radius: int = 20):
        """BFS outward from rc to find the nearest free (==0) cell."""
        from collections import deque
        rows, cols = grid.shape
        r0, c0 = rc
        visited = {(r0, c0)}
        queue = deque([(r0, c0)])
        while queue:
            r, c = queue.popleft()
            if grid[r, c] == 0:
                return (r, c)
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nr, nc = r + dr, c + dc
                if (0 <= nr < rows and 0 <= nc < cols
                        and (nr, nc) not in visited
                        and abs(nr - r0) <= search_radius
                        and abs(nc - c0) <= search_radius):
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        return None

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_robot_xy(self):
        # Try map→base_link first
        try:
            t = self._tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e1:
            self.get_logger().warn(
                f'TF {self.global_frame}→{self.base_frame} failed: {e1}')
        # Fallback: odom→base_link (if map frame not yet available)
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', self.base_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            self.get_logger().warn(
                'Using odom→base_link as fallback (map frame not ready)')
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e2:
            self.get_logger().warn(f'TF odom→{self.base_frame} also failed: {e2}')
        # Last resort: use map origin
        if self._map_info is not None:
            ox = self._map_info.origin.position.x
            oy = self._map_info.origin.position.y
            w = self._map_info.width * self._map_info.resolution
            h = self._map_info.height * self._map_info.resolution
            cx, cy = ox + w / 2.0, oy + h / 2.0
            self.get_logger().warn(
                f'Using map center ({cx:.1f}, {cy:.1f}) as start — TF unavailable')
            return (cx, cy)
        return None

    def _get_robot_yaw(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny, cosy)
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------

    def _publish_debug_markers(self, path_cells, start_rc, goal_rc):
        ma = MarkerArray()

        # Start marker
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'planner'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        info = self._map_info
        sx, sy = grid_to_world(start_rc[0], start_rc[1],
                               info.origin.position.x, info.origin.position.y,
                               info.resolution)
        m.pose.position.x = sx
        m.pose.position.y = sy
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.g = 1.0
        m.color.a = 1.0
        ma.markers.append(m)

        # Goal marker
        m2 = Marker()
        m2.header = m.header
        m2.ns = 'planner'
        m2.id = 1
        m2.type = Marker.SPHERE
        m2.action = Marker.ADD
        gx, gy = grid_to_world(goal_rc[0], goal_rc[1],
                                info.origin.position.x, info.origin.position.y,
                                info.resolution)
        m2.pose.position.x = gx
        m2.pose.position.y = gy
        m2.pose.position.z = 0.1
        m2.pose.orientation.w = 1.0
        m2.scale.x = m2.scale.y = m2.scale.z = 0.3
        m2.color.r = 1.0
        m2.color.a = 1.0
        ma.markers.append(m2)

        self._debug_pub.publish(ma)


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _path_length(path_cells, resolution: float) -> float:
    """Total Euclidean length of a cell-coordinate path converted to metres."""
    total = 0.0
    for i in range(len(path_cells) - 1):
        dr = path_cells[i+1][0] - path_cells[i][0]
        dc = path_cells[i+1][1] - path_cells[i][1]
        total += math.hypot(dr, dc) * resolution
    return total


def _yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
