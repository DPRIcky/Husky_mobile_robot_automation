#!/usr/bin/env python3
"""
Performance evaluation script for state estimation
Compares estimated state with ground truth (if available)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from collections import deque
from transforms3d.euler import quat2euler
import time


class StateEvaluatorNode(Node):
    """Evaluate state estimation performance"""
    
    def __init__(self):
        super().__init__('state_evaluator')
        
        # Buffers for comparison
        self.max_points = 1000
        self.estimated_poses = deque(maxlen=self.max_points)
        self.ground_truth_poses = deque(maxlen=self.max_points)
        self.timestamps = deque(maxlen=self.max_points)
        
        # Error statistics
        self.position_errors = []
        self.orientation_errors = []
        
        # Subscribe to estimated state
        self.estimate_sub = self.create_subscription(
            Odometry,
            '/state_estimate/odom',
            self.estimate_callback,
            10
        )
        
        # Subscribe to ground truth (if available)
        self.ground_truth_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',  # Change this to your ground truth topic
            self.ground_truth_callback,
            10
        )
        
        # Timer for periodic statistics
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('State evaluator initialized')
        self.get_logger().info('Comparing /state_estimate/odom with /ground_truth/odom')
        
    def estimate_callback(self, msg):
        """Store estimated state"""
        timestamp = self.get_clock().now().nanoseconds
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        # transforms3d uses (w, x, y, z) order
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        self.estimated_poses.append((timestamp, x, y, theta))
        
    def ground_truth_callback(self, msg):
        """Store ground truth state"""
        timestamp = self.get_clock().now().nanoseconds
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        # transforms3d uses (w, x, y, z) order
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        self.ground_truth_poses.append((timestamp, x, y, theta))
        
        # Find closest estimated pose in time
        if len(self.estimated_poses) > 0:
            estimated_array = np.array([(t, x, y, th) for t, x, y, th in self.estimated_poses])
            time_diffs = np.abs(estimated_array[:, 0] - timestamp)
            closest_idx = np.argmin(time_diffs)
            
            if time_diffs[closest_idx] < 1e8:  # Within 100ms
                est_x = estimated_array[closest_idx, 1]
                est_y = estimated_array[closest_idx, 2]
                est_theta = estimated_array[closest_idx, 3]
                
                # Calculate errors
                position_error = np.sqrt((x - est_x)**2 + (y - est_y)**2)
                orientation_error = self._angle_diff(theta, est_theta)
                
                self.position_errors.append(position_error)
                self.orientation_errors.append(abs(orientation_error))
                
    def print_statistics(self):
        """Print error statistics"""
        if len(self.position_errors) < 10:
            self.get_logger().info('Not enough data yet for statistics...')
            return
        
        # Position statistics
        pos_errors = np.array(self.position_errors)
        pos_mean = np.mean(pos_errors)
        pos_std = np.std(pos_errors)
        pos_max = np.max(pos_errors)
        pos_rmse = np.sqrt(np.mean(pos_errors**2))
        
        # Orientation statistics
        orient_errors = np.array(self.orientation_errors)
        orient_mean = np.mean(orient_errors)
        orient_std = np.std(orient_errors)
        orient_max = np.max(orient_errors)
        orient_rmse = np.sqrt(np.mean(orient_errors**2))
        
        # Print report
        self.get_logger().info('=' * 60)
        self.get_logger().info('STATE ESTIMATION PERFORMANCE REPORT')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Sample size: {len(self.position_errors)} comparisons')
        self.get_logger().info('')
        self.get_logger().info('POSITION ERROR (meters):')
        self.get_logger().info(f'  Mean:     {pos_mean:.4f} m')
        self.get_logger().info(f'  Std Dev:  {pos_std:.4f} m')
        self.get_logger().info(f'  RMSE:     {pos_rmse:.4f} m')
        self.get_logger().info(f'  Max:      {pos_max:.4f} m')
        self.get_logger().info('')
        self.get_logger().info('ORIENTATION ERROR (radians):')
        self.get_logger().info(f'  Mean:     {orient_mean:.4f} rad ({np.rad2deg(orient_mean):.2f}°)')
        self.get_logger().info(f'  Std Dev:  {orient_std:.4f} rad ({np.rad2deg(orient_std):.2f}°)')
        self.get_logger().info(f'  RMSE:     {orient_rmse:.4f} rad ({np.rad2deg(orient_rmse):.2f}°)')
        self.get_logger().info(f'  Max:      {orient_max:.4f} rad ({np.rad2deg(orient_max):.2f}°)')
        self.get_logger().info('=' * 60)
        
        # Performance rating
        if pos_rmse < 0.1 and orient_rmse < 0.05:
            rating = "EXCELLENT"
        elif pos_rmse < 0.3 and orient_rmse < 0.1:
            rating = "GOOD"
        elif pos_rmse < 0.5 and orient_rmse < 0.2:
            rating = "ACCEPTABLE"
        else:
            rating = "NEEDS IMPROVEMENT"
        
        self.get_logger().info(f'Overall Rating: {rating}')
        self.get_logger().info('')
        
    @staticmethod
    def _angle_diff(a, b):
        """Calculate shortest angle difference"""
        diff = a - b
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    
    node = StateEvaluatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        if len(node.position_errors) > 0:
            node.get_logger().info('\n\nFINAL STATISTICS:')
            node.print_statistics()
            
            # Save to file
            with open('state_estimation_results.txt', 'w') as f:
                f.write('State Estimation Performance Report\n')
                f.write('=' * 60 + '\n')
                f.write(f'Total samples: {len(node.position_errors)}\n\n')
                
                pos_errors = np.array(node.position_errors)
                orient_errors = np.array(node.orientation_errors)
                
                f.write('Position Errors (m):\n')
                f.write(f'  Mean: {np.mean(pos_errors):.4f}\n')
                f.write(f'  Std:  {np.std(pos_errors):.4f}\n')
                f.write(f'  RMSE: {np.sqrt(np.mean(pos_errors**2)):.4f}\n')
                f.write(f'  Max:  {np.max(pos_errors):.4f}\n\n')
                
                f.write('Orientation Errors (rad):\n')
                f.write(f'  Mean: {np.mean(orient_errors):.4f}\n')
                f.write(f'  Std:  {np.std(orient_errors):.4f}\n')
                f.write(f'  RMSE: {np.sqrt(np.mean(orient_errors**2)):.4f}\n')
                f.write(f'  Max:  {np.max(orient_errors):.4f}\n')
                
            node.get_logger().info('Results saved to state_estimation_results.txt')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
