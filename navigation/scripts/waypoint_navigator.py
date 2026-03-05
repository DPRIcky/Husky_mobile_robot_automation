#!/usr/bin/env python3
"""
Waypoint Navigator Node

Autonomous waypoint navigation using Nav2 for the Clearpath A300 robot.
Supports patrol mode, single-run mode, and interactive waypoint management.

Author: DPRicky
Date: 2026-03-05
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
import yaml
import math
from enum import Enum


class NavigationMode(Enum):
    """Navigation modes"""
    SINGLE = "single"  # Visit waypoints once
    LOOP = "loop"  # Continuously loop through waypoints
    PATROL = "patrol"  # Go forward then backward (patrol)


class WaypointNavigator(Node):
    """
    Waypoint Navigator Node
    
    Manages autonomous navigation through a sequence of waypoints using Nav2.
    """
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('mode', 'single')  # single, loop, patrol
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_follow_waypoints', True)  # Use FollowWaypoints action
        self.declare_parameter('wait_duration', 2.0)  # Seconds to wait at each waypoint
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').value
        mode_str = self.get_parameter('mode').value
        self.mode = NavigationMode(mode_str)
        self.frame_id = self.get_parameter('frame_id').value
        self.use_follow_waypoints = self.get_parameter('use_follow_waypoints').value
        self.wait_duration = self.get_parameter('wait_duration').value
        
        # State variables
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        self.patrol_forward = True  # For patrol mode
        self.loop_count = 0
        
        # Action clients
        if self.use_follow_waypoints:
            self.follow_waypoints_client = ActionClient(
                self, 
                FollowWaypoints, 
                'follow_waypoints'
            )
            self.get_logger().info('Using FollowWaypoints action')
        else:
            self.navigate_to_pose_client = ActionClient(
                self,
                NavigateToPose,
                'navigate_to_pose'
            )
            self.get_logger().info('Using NavigateToPose action')
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'waypoint_status', 10)
        
        # Load waypoints
        if self.waypoints_file:
            self.load_waypoints_from_file(self.waypoints_file)
        else:
            self.get_logger().warn('No waypoints file specified. Use add_waypoint service or load_waypoints service.')
        
        # Timer for status updates
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'Waypoint Navigator initialized in {self.mode.value} mode')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
    def load_waypoints_from_file(self, filename):
        """Load waypoints from YAML file"""
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                
            if 'waypoints' not in data:
                self.get_logger().error(f'No waypoints found in {filename}')
                return
                
            self.waypoints = []
            for wp_data in data['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = float(wp_data['x'])
                pose.pose.position.y = float(wp_data['y'])
                pose.pose.position.z = float(wp_data.get('z', 0.0))
                
                # Convert yaw to quaternion if provided
                if 'yaw' in wp_data:
                    yaw = float(wp_data['yaw'])
                    quat = self.yaw_to_quaternion(yaw)
                    pose.pose.orientation = quat
                elif 'quat' in wp_data:
                    pose.pose.orientation.x = float(wp_data['quat'].get('x', 0.0))
                    pose.pose.orientation.y = float(wp_data['quat'].get('y', 0.0))
                    pose.pose.orientation.z = float(wp_data['quat'].get('z', 0.0))
                    pose.pose.orientation.w = float(wp_data['quat'].get('w', 1.0))
                else:
                    pose.pose.orientation.w = 1.0
                
                self.waypoints.append(pose)
                
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
        
    def add_waypoint(self, x, y, yaw=0.0, z=0.0):
        """Add a waypoint programmatically"""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        self.waypoints.append(pose)
        self.get_logger().info(f'Added waypoint: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        
    def start_navigation(self):
        """Start navigating through waypoints"""
        if not self.waypoints:
            self.get_logger().error('No waypoints to navigate!')
            return False
            
        if self.is_navigating:
            self.get_logger().warn('Already navigating!')
            return False
            
        self.is_navigating = True
        self.current_waypoint_index = 0
        self.loop_count = 0
        
        if self.use_follow_waypoints:
            self.send_follow_waypoints_goal()
        else:
            self.send_next_waypoint()
            
        return True
        
    def send_follow_waypoints_goal(self):
        """Send all waypoints as FollowWaypoints goal"""
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        self.follow_waypoints_client.wait_for_server()
        
        goal_msg = FollowWaypoints.Goal()
        
        # Update timestamps
        for wp in self.waypoints:
            wp.header.stamp = self.get_clock().now().to_msg()
            
        goal_msg.poses = self.waypoints
        
        self.get_logger().info(f'Sending {len(self.waypoints)} waypoints to FollowWaypoints action')
        
        self._send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_waypoints_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.follow_waypoints_goal_response_callback)
        
    def follow_waypoints_goal_response_callback(self, future):
        """Handle FollowWaypoints goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected')
            self.is_navigating = False
            return
            
        self.get_logger().info('FollowWaypoints goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.follow_waypoints_result_callback)
        
    def follow_waypoints_feedback_callback(self, feedback_msg):
        """Handle FollowWaypoints feedback"""
        feedback = feedback_msg.feedback
        self.current_waypoint_index = feedback.current_waypoint
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}'
        )
        
    def follow_waypoints_result_callback(self, future):
        """Handle FollowWaypoints result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Successfully completed all waypoints!')
            self.handle_waypoints_completed()
        else:
            self.get_logger().error(f'FollowWaypoints failed with status: {status}')
            self.is_navigating = False
            
    def send_next_waypoint(self):
        """Send next waypoint using NavigateToPose"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.handle_waypoints_completed()
            return
            
        self.get_logger().info('Waiting for NavigateToPose action server...')
        self.navigate_to_pose_client.wait_for_server()
        
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})'
        )
        
        self._send_goal_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.navigate_goal_response_callback)
        
    def navigate_goal_response_callback(self, future):
        """Handle NavigateToPose goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal rejected')
            self.is_navigating = False
            return
            
        self.get_logger().info('NavigateToPose goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigate_result_callback)
        
    def navigate_feedback_callback(self, feedback_msg):
        """Handle NavigateToPose feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        # self.get_logger().info(f'Distance remaining: {distance:.2f}m', throttle_duration_sec=2.0)
        
    def navigate_result_callback(self, future):
        """Handle NavigateToPose result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}'
            )
            self.current_waypoint_index += 1
            
            # Wait before next waypoint
            if self.wait_duration > 0:
                self.get_logger().info(f'Waiting {self.wait_duration}s at waypoint...')
                self.create_timer(
                    self.wait_duration,
                    self.send_next_waypoint,
                    oneshot=True
                )
            else:
                self.send_next_waypoint()
        else:
            self.get_logger().error(f'NavigateToPose failed with status: {status}')
            self.is_navigating = False
            
    def handle_waypoints_completed(self):
        """Handle completion of all waypoints based on mode"""
        self.loop_count += 1
        self.get_logger().info(f'Completed loop {self.loop_count}')
        
        if self.mode == NavigationMode.SINGLE:
            self.get_logger().info('Navigation completed (single mode)')
            self.is_navigating = False
            
        elif self.mode == NavigationMode.LOOP:
            self.get_logger().info('Restarting loop...')
            self.current_waypoint_index = 0
            if self.use_follow_waypoints:
                self.send_follow_waypoints_goal()
            else:
                self.send_next_waypoint()
                
        elif self.mode == NavigationMode.PATROL:
            # Reverse waypoint order for patrol
            self.get_logger().info('Reversing patrol direction...')
            self.waypoints.reverse()
            self.current_waypoint_index = 0
            if self.use_follow_waypoints:
                self.send_follow_waypoints_goal()
            else:
                self.send_next_waypoint()
                
    def publish_status(self):
        """Publish current navigation status"""
        if self.is_navigating:
            status = f'Navigating: {self.current_waypoint_index}/{len(self.waypoints)}, Loop: {self.loop_count}'
        else:
            status = f'Idle: {len(self.waypoints)} waypoints loaded'
            
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
    def stop_navigation(self):
        """Stop current navigation"""
        if self.use_follow_waypoints:
            # Cancel FollowWaypoints goal
            if hasattr(self, '_send_goal_future'):
                self.follow_waypoints_client.cancel_goal_async(self._send_goal_future)
        else:
            # Cancel NavigateToPose goal
            if hasattr(self, '_send_goal_future'):
                self.navigate_to_pose_client.cancel_goal_async(self._send_goal_future)
                
        self.is_navigating = False
        self.get_logger().info('Navigation stopped')


def main(args=None):
    rclpy.init(args=args)
    
    navigator = WaypointNavigator()
    
    # Auto-start if waypoints are loaded
    if navigator.waypoints:
        navigator.get_logger().info('Auto-starting navigation in 3 seconds...')
        navigator.create_timer(3.0, navigator.start_navigation, oneshot=True)
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down...')
        navigator.stop_navigation()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
