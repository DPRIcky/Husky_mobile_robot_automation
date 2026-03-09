"""
ROS 2 planner node — subscribes to OccupancyGrid and goal pose, publishes
nav_msgs/Path using A* (default), Hybrid-A*, or RRT*.
"""

import math
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
        self.declare_parameter('inflation_radius_m', 0.35)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown', True)

        self.planner_type = self.get_parameter('planner_type').value
        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.inflation_radius_m = self.get_parameter('inflation_radius_m').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.allow_unknown = self.get_parameter('allow_unknown').value

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

        self.get_logger().info('Waiting for map on %s …' % map_topic)

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

        self.get_logger().info(
            f'Planning {self.planner_type}: '
            f'start=({start_xy[0]:.2f},{start_xy[1]:.2f}) → '
            f'goal=({goal_x:.2f},{goal_y:.2f})')

        path_cells = self._run_planner(grid, start_rc, goal_rc, goal_x, goal_y)

        if path_cells is None:
            self.get_logger().warn('Planner found NO path.')
            self._publish_debug_markers([], start_rc, goal_rc)
            return

        # Post-process
        path_cells = prune_collinear(path_cells)
        path_cells = shortcut_smooth(path_cells, grid, iterations=80)

        self.get_logger().info(f'Path found: {len(path_cells)} waypoints')

        # Convert to world & publish
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

        # Set orientations along path tangent for nicer display
        for i in range(len(path_msg.poses) - 1):
            p0 = path_msg.poses[i].pose.position
            p1 = path_msg.poses[i + 1].pose.position
            yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
            q = _yaw_to_quaternion(yaw)
            path_msg.poses[i].pose.orientation = q
        if len(path_msg.poses) >= 2:
            path_msg.poses[-1].pose.orientation = path_msg.poses[-2].pose.orientation

        self._path_pub.publish(path_msg)
        self._publish_debug_markers(path_cells, start_rc, goal_rc)

    # ------------------------------------------------------------------
    # Planner dispatch
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
            )
            if result is not None:
                return [(int(round(r)), int(round(c))) for r, c, _ in result]
            return None

        elif self.planner_type == 'rrt_star':
            return rrt_star(grid, start_rc, goal_rc,
                            max_iter=8000, step_size=5.0)

        else:  # 'astar' default
            return astar(grid, start_rc, goal_rc)

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
