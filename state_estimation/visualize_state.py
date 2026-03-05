#!/usr/bin/env python3
"""
Visualization tool for state estimation
Subscribes to state estimate and creates plots
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
from transforms3d.euler import quat2euler


class StateVisualizerNode(Node):
    """Visualize state estimation in real-time"""
    
    def __init__(self):
        super().__init__('state_visualizer')
        
        # Data buffers
        self.max_points = 500
        self.time_buffer = deque(maxlen=self.max_points)
        self.x_buffer = deque(maxlen=self.max_points)
        self.y_buffer = deque(maxlen=self.max_points)
        self.theta_buffer = deque(maxlen=self.max_points)
        self.vx_buffer = deque(maxlen=self.max_points)
        self.vy_buffer = deque(maxlen=self.max_points)
        self.omega_buffer = deque(maxlen=self.max_points)
        
        self.x_cov_buffer = deque(maxlen=self.max_points)
        self.y_cov_buffer = deque(maxlen=self.max_points)
        self.theta_cov_buffer = deque(maxlen=self.max_points)
        
        self.start_time = None
        
        # Subscribe to state estimate
        self.odom_sub = self.create_subscription(
            Odometry,
            '/state_estimate/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('State visualizer initialized')
        
        # Set up the plot
        self.setup_plots()
        
    def odom_callback(self, msg):
        """Process odometry messages"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        # transforms3d uses (w, x, y, z) order
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        # Extract twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Extract covariances
        pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        x_cov = np.sqrt(pose_cov[0, 0])
        y_cov = np.sqrt(pose_cov[1, 1])
        theta_cov = np.sqrt(pose_cov[5, 5])
        
        # Store data
        self.time_buffer.append(current_time)
        self.x_buffer.append(x)
        self.y_buffer.append(y)
        self.theta_buffer.append(theta)
        self.vx_buffer.append(vx)
        self.vy_buffer.append(vy)
        self.omega_buffer.append(omega)
        self.x_cov_buffer.append(x_cov)
        self.y_cov_buffer.append(y_cov)
        self.theta_cov_buffer.append(theta_cov)
        
    def setup_plots(self):
        """Set up matplotlib plots"""
        self.fig, self.axes = plt.subplots(3, 2, figsize=(15, 10))
        self.fig.suptitle('State Estimation Visualization', fontsize=16)
        
        # Position trajectory (2D)
        self.ax_traj = self.axes[0, 0]
        self.ax_traj.set_title('Position Trajectory')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True)
        self.line_traj, = self.ax_traj.plot([], [], 'b-', linewidth=2)
        self.robot_marker, = self.ax_traj.plot([], [], 'ro', markersize=10)
        
        # Position over time
        self.ax_pos = self.axes[0, 1]
        self.ax_pos.set_title('Position vs Time')
        self.ax_pos.set_xlabel('Time (s)')
        self.ax_pos.set_ylabel('Position (m)')
        self.ax_pos.grid(True)
        self.line_x, = self.ax_pos.plot([], [], 'r-', label='X', linewidth=2)
        self.line_y, = self.ax_pos.plot([], [], 'b-', label='Y', linewidth=2)
        self.ax_pos.legend()
        
        # Orientation over time
        self.ax_theta = self.axes[1, 0]
        self.ax_theta.set_title('Orientation vs Time')
        self.ax_theta.set_xlabel('Time (s)')
        self.ax_theta.set_ylabel('Theta (rad)')
        self.ax_theta.grid(True)
        self.line_theta, = self.ax_theta.plot([], [], 'g-', linewidth=2)
        
        # Velocity over time
        self.ax_vel = self.axes[1, 1]
        self.ax_vel.set_title('Velocity vs Time')
        self.ax_vel.set_xlabel('Time (s)')
        self.ax_vel.set_ylabel('Velocity (m/s)')
        self.ax_vel.grid(True)
        self.line_vx, = self.ax_vel.plot([], [], 'r-', label='Vx', linewidth=2)
        self.line_vy, = self.ax_vel.plot([], [], 'b-', label='Vy', linewidth=2)
        self.ax_vel.legend()
        
        # Angular velocity over time
        self.ax_omega = self.axes[2, 0]
        self.ax_omega.set_title('Angular Velocity vs Time')
        self.ax_omega.set_xlabel('Time (s)')
        self.ax_omega.set_ylabel('Omega (rad/s)')
        self.ax_omega.grid(True)
        self.line_omega, = self.ax_omega.plot([], [], 'purple', linewidth=2)
        
        # Uncertainty over time
        self.ax_cov = self.axes[2, 1]
        self.ax_cov.set_title('Position Uncertainty (Std Dev)')
        self.ax_cov.set_xlabel('Time (s)')
        self.ax_cov.set_ylabel('Std Dev (m)')
        self.ax_cov.grid(True)
        self.line_x_cov, = self.ax_cov.plot([], [], 'r-', label='X', linewidth=2)
        self.line_y_cov, = self.ax_cov.plot([], [], 'b-', label='Y', linewidth=2)
        self.ax_cov.legend()
        
        plt.tight_layout()
        
    def update_plots(self, frame):
        """Update plot data"""
        if len(self.time_buffer) < 2:
            return
        
        time_data = np.array(self.time_buffer)
        x_data = np.array(self.x_buffer)
        y_data = np.array(self.y_buffer)
        theta_data = np.array(self.theta_buffer)
        vx_data = np.array(self.vx_buffer)
        vy_data = np.array(self.vy_buffer)
        omega_data = np.array(self.omega_buffer)
        x_cov_data = np.array(self.x_cov_buffer)
        y_cov_data = np.array(self.y_cov_buffer)
        
        # Update trajectory
        self.line_traj.set_data(x_data, y_data)
        if len(x_data) > 0:
            self.robot_marker.set_data([x_data[-1]], [y_data[-1]])
            
            # Update axis limits for trajectory
            x_min, x_max = np.min(x_data), np.max(x_data)
            y_min, y_max = np.min(y_data), np.max(y_data)
            margin = 1.0
            self.ax_traj.set_xlim(x_min - margin, x_max + margin)
            self.ax_traj.set_ylim(y_min - margin, y_max + margin)
        
        # Update position
        self.line_x.set_data(time_data, x_data)
        self.line_y.set_data(time_data, y_data)
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()
        
        # Update orientation
        self.line_theta.set_data(time_data, theta_data)
        self.ax_theta.relim()
        self.ax_theta.autoscale_view()
        
        # Update velocity
        self.line_vx.set_data(time_data, vx_data)
        self.line_vy.set_data(time_data, vy_data)
        self.ax_vel.relim()
        self.ax_vel.autoscale_view()
        
        # Update angular velocity
        self.line_omega.set_data(time_data, omega_data)
        self.ax_omega.relim()
        self.ax_omega.autoscale_view()
        
        # Update uncertainty
        self.line_x_cov.set_data(time_data, x_cov_data)
        self.line_y_cov.set_data(time_data, y_cov_data)
        self.ax_cov.relim()
        self.ax_cov.autoscale_view()
        
        return (self.line_traj, self.robot_marker, self.line_x, self.line_y,
                self.line_theta, self.line_vx, self.line_vy, self.line_omega,
                self.line_x_cov, self.line_y_cov)
        
    def run(self):
        """Run the visualizer with animation"""
        ani = FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    node = StateVisualizerNode()
    
    # Run visualization in a separate thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
