#!/usr/bin/env python3
"""
Filter Comparison Visualizer
Compares EKF, UKF, and PF performance in real-time
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
from transforms3d.euler import quat2euler


class FilterComparatorNode(Node):
    """Visualize and compare multiple state estimation filters"""
    
    def __init__(self):
        super().__init__('filter_comparator')
        
        # Data buffers for each filter
        self.max_points = 500
        
        # EKF data
        self.ekf_time = deque(maxlen=self.max_points)
        self.ekf_x = deque(maxlen=self.max_points)
        self.ekf_y = deque(maxlen=self.max_points)
        self.ekf_theta = deque(maxlen=self.max_points)
        self.ekf_cov = deque(maxlen=self.max_points)
        
        # UKF data
        self.ukf_time = deque(maxlen=self.max_points)
        self.ukf_x = deque(maxlen=self.max_points)
        self.ukf_y = deque(maxlen=self.max_points)
        self.ukf_theta = deque(maxlen=self.max_points)
        self.ukf_cov = deque(maxlen=self.max_points)
        
        # PF data
        self.pf_time = deque(maxlen=self.max_points)
        self.pf_x = deque(maxlen=self.max_points)
        self.pf_y = deque(maxlen=self.max_points)
        self.pf_theta = deque(maxlen=self.max_points)
        self.pf_cov = deque(maxlen=self.max_points)
        
        # Ground truth (if available from mock sensors or simulation)
        self.gt_time = deque(maxlen=self.max_points)
        self.gt_x = deque(maxlen=self.max_points)
        self.gt_y = deque(maxlen=self.max_points)
        
        self.start_time = None
        
        # Subscribers for each filter
        self.ekf_sub = self.create_subscription(
            Odometry,
            '/state_estimate/odom',
            lambda msg: self.filter_callback(msg, 'ekf'),
            10
        )
        
        self.ukf_sub = self.create_subscription(
            Odometry,
            '/state_estimate_ukf/odom',
            lambda msg: self.filter_callback(msg, 'ukf'),
            10
        )
        
        self.pf_sub = self.create_subscription(
            Odometry,
            '/state_estimate_pf/odom',
            lambda msg: self.filter_callback(msg, 'pf'),
            10
        )
        
        # Subscribe to ground truth odometry (from simulation or mock)
        self.gt_sub = self.create_subscription(
            Odometry,
            '/a300_00000/platform/odom',
            self.gt_callback,
            10
        )
        
        self.get_logger().info('Filter comparator initialized')
        self.setup_plots()
        
    def filter_callback(self, msg, filter_type):
        """Process odometry messages from different filters"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        # Extract covariance
        pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        pos_uncertainty = np.sqrt(pose_cov[0, 0] + pose_cov[1, 1])
        
        # Store data based on filter type
        if filter_type == 'ekf':
            self.ekf_time.append(current_time)
            self.ekf_x.append(x)
            self.ekf_y.append(y)
            self.ekf_theta.append(theta)
            self.ekf_cov.append(pos_uncertainty)
        elif filter_type == 'ukf':
            self.ukf_time.append(current_time)
            self.ukf_x.append(x)
            self.ukf_y.append(y)
            self.ukf_theta.append(theta)
            self.ukf_cov.append(pos_uncertainty)
        elif filter_type == 'pf':
            self.pf_time.append(current_time)
            self.pf_x.append(x)
            self.pf_y.append(y)
            self.pf_theta.append(theta)
            self.pf_cov.append(pos_uncertainty)
    
    def gt_callback(self, msg):
        """Process ground truth odometry"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.gt_time.append(current_time)
        self.gt_x.append(x)
        self.gt_y.append(y)
        
    def setup_plots(self):
        """Set up matplotlib plots"""
        self.fig = plt.figure(figsize=(18, 10))
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        self.fig.suptitle('State Estimation Filter Comparison: EKF vs UKF vs PF', fontsize=16, fontweight='bold')
        
        # Plot 1: Trajectory Comparison (Main plot)
        self.ax_traj = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax_traj.set_title('Position Trajectory Comparison', fontsize=12, fontweight='bold')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True, alpha=0.3)
        
        self.line_gt, = self.ax_traj.plot([], [], 'k--', linewidth=2, label='Ground Truth', alpha=0.7)
        self.line_ekf, = self.ax_traj.plot([], [], 'r-', linewidth=2, label='EKF')
        self.line_ukf, = self.ax_traj.plot([], [], 'b-', linewidth=2, label='UKF')
        self.line_pf, = self.ax_traj.plot([], [], 'g-', linewidth=2, label='PF')
        
        self.marker_ekf, = self.ax_traj.plot([], [], 'ro', markersize=8)
        self.marker_ukf, = self.ax_traj.plot([], [], 'bo', markersize=8)
        self.marker_pf, = self.ax_traj.plot([], [], 'go', markersize=8)
        
        self.ax_traj.legend(loc='upper right')
        
        # Plot 2: Position X Error
        self.ax_x_error = self.fig.add_subplot(gs[0, 2])
        self.ax_x_error.set_title('X Position Error', fontsize=10)
        self.ax_x_error.set_xlabel('Time (s)')
        self.ax_x_error.set_ylabel('Error (m)')
        self.ax_x_error.grid(True, alpha=0.3)
        
        self.line_x_err_ekf, = self.ax_x_error.plot([], [], 'r-', linewidth=1.5, label='EKF')
        self.line_x_err_ukf, = self.ax_x_error.plot([], [], 'b-', linewidth=1.5, label='UKF')
        self.line_x_err_pf, = self.ax_x_error.plot([], [], 'g-', linewidth=1.5, label='PF')
        self.ax_x_error.legend(fontsize=8)
        
        # Plot 3: Position Y Error
        self.ax_y_error = self.fig.add_subplot(gs[1, 2])
        self.ax_y_error.set_title('Y Position Error', fontsize=10)
        self.ax_y_error.set_xlabel('Time (s)')
        self.ax_y_error.set_ylabel('Error (m)')
        self.ax_y_error.grid(True, alpha=0.3)
        
        self.line_y_err_ekf, = self.ax_y_error.plot([], [], 'r-', linewidth=1.5, label='EKF')
        self.line_y_err_ukf, = self.ax_y_error.plot([], [], 'b-', linewidth=1.5, label='UKF')
        self.line_y_err_pf, = self.ax_y_error.plot([], [], 'g-', linewidth=1.5, label='PF')
        self.ax_y_error.legend(fontsize=8)
        
        # Plot 4: Uncertainty Comparison
        self.ax_uncertainty = self.fig.add_subplot(gs[2, 0])
        self.ax_uncertainty.set_title('Position Uncertainty', fontsize=10)
        self.ax_uncertainty.set_xlabel('Time (s)')
        self.ax_uncertainty.set_ylabel('Std Dev (m)')
        self.ax_uncertainty.grid(True, alpha=0.3)
        
        self.line_cov_ekf, = self.ax_uncertainty.plot([], [], 'r-', linewidth=2, label='EKF')
        self.line_cov_ukf, = self.ax_uncertainty.plot([], [], 'b-', linewidth=2, label='UKF')
        self.line_cov_pf, = self.ax_uncertainty.plot([], [], 'g-', linewidth=2, label='PF')
        self.ax_uncertainty.legend(fontsize=8)
        
        # Plot 5: RMSE Comparison (Bar chart)
        self.ax_rmse = self.fig.add_subplot(gs[2, 1])
        self.ax_rmse.set_title('Root Mean Square Error', fontsize=10)
        self.ax_rmse.set_ylabel('RMSE (m)')
        self.ax_rmse.grid(True, alpha=0.3, axis='y')
        
        # Plot 6: Statistics Summary
        self.ax_stats = self.fig.add_subplot(gs[2, 2])
        self.ax_stats.axis('off')
        self.ax_stats.set_title('Performance Statistics', fontsize=10, fontweight='bold')
        
        self.stats_text = self.ax_stats.text(0.05, 0.95, '', transform=self.ax_stats.transAxes,
                                             verticalalignment='top', fontsize=9, family='monospace')
        
    def update_plots(self, frame):
        """Update plot data"""
        if len(self.gt_x) < 2:
            return
        
        # Update trajectory
        if len(self.gt_x) > 0:
            self.line_gt.set_data(np.array(self.gt_x), np.array(self.gt_y))
            
        if len(self.ekf_x) > 0:
            self.line_ekf.set_data(np.array(self.ekf_x), np.array(self.ekf_y))
            self.marker_ekf.set_data([self.ekf_x[-1]], [self.ekf_y[-1]])
            
        if len(self.ukf_x) > 0:
            self.line_ukf.set_data(np.array(self.ukf_x), np.array(self.ukf_y))
            self.marker_ukf.set_data([self.ukf_x[-1]], [self.ukf_y[-1]])
            
        if len(self.pf_x) > 0:
            self.line_pf.set_data(np.array(self.pf_x), np.array(self.pf_y))
            self.marker_pf.set_data([self.pf_x[-1]], [self.pf_y[-1]])
        
        # Update trajectory limits
        all_x = list(self.gt_x) + list(self.ekf_x) + list(self.ukf_x) + list(self.pf_x)
        all_y = list(self.gt_y) + list(self.ekf_y) + list(self.ukf_y) + list(self.pf_y)
        
        if len(all_x) > 0:
            margin = 1.0
            self.ax_traj.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.ax_traj.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        # Compute and plot errors
        ekf_x_err, ukf_x_err, pf_x_err = [], [], []
        ekf_y_err, ukf_y_err, pf_y_err = [], [], []
        ekf_times, ukf_times, pf_times = [], [], []
        
        gt_x_array = np.array(self.gt_x)
        gt_y_array = np.array(self.gt_y)
        gt_time_array = np.array(self.gt_time)
        
        # Compute EKF errors
        for i, t in enumerate(self.ekf_time):
            idx = np.argmin(np.abs(gt_time_array - t))
            if np.abs(gt_time_array[idx] - t) < 0.1:  # Within 100ms
                ekf_x_err.append(self.ekf_x[i] - gt_x_array[idx])
                ekf_y_err.append(self.ekf_y[i] - gt_y_array[idx])
                ekf_times.append(t)
        
        # Compute UKF errors
        for i, t in enumerate(self.ukf_time):
            idx = np.argmin(np.abs(gt_time_array - t))
            if np.abs(gt_time_array[idx] - t) < 0.1:
                ukf_x_err.append(self.ukf_x[i] - gt_x_array[idx])
                ukf_y_err.append(self.ukf_y[i] - gt_y_array[idx])
                ukf_times.append(t)
        
        # Compute PF errors
        for i, t in enumerate(self.pf_time):
            idx = np.argmin(np.abs(gt_time_array - t))
            if np.abs(gt_time_array[idx] - t) < 0.1:
                pf_x_err.append(self.pf_x[i] - gt_x_array[idx])
                pf_y_err.append(self.pf_y[i] - gt_y_array[idx])
                pf_times.append(t)
        
        # Update error plots
        if len(ekf_x_err) > 0:
            self.line_x_err_ekf.set_data(ekf_times, ekf_x_err)
            self.line_y_err_ekf.set_data(ekf_times, ekf_y_err)
        
        if len(ukf_x_err) > 0:
            self.line_x_err_ukf.set_data(ukf_times, ukf_x_err)
            self.line_y_err_ukf.set_data(ukf_times, ukf_y_err)
        
        if len(pf_x_err) > 0:
            self.line_x_err_pf.set_data(pf_times, pf_x_err)
            self.line_y_err_pf.set_data(pf_times, pf_y_err)
        
        # Auto-scale error plots
        for ax in [self.ax_x_error, self.ax_y_error]:
            ax.relim()
            ax.autoscale_view()
        
        # Update uncertainty plot
        if len(self.ekf_cov) > 0:
            self.line_cov_ekf.set_data(self.ekf_time, self.ekf_cov)
        if len(self.ukf_cov) > 0:
            self.line_cov_ukf.set_data(self.ukf_time, self.ukf_cov)
        if len(self.pf_cov) > 0:
            self.line_cov_pf.set_data(self.pf_time, self.pf_cov)
        
        self.ax_uncertainty.relim()
        self.ax_uncertainty.autoscale_view()
        
        # Compute RMSE
        ekf_rmse = np.sqrt(np.mean(np.array(ekf_x_err)**2 + np.array(ekf_y_err)**2)) if len(ekf_x_err) > 0 else 0
        ukf_rmse = np.sqrt(np.mean(np.array(ukf_x_err)**2 + np.array(ukf_y_err)**2)) if len(ukf_x_err) > 0 else 0
        pf_rmse = np.sqrt(np.mean(np.array(pf_x_err)**2 + np.array(pf_y_err)**2)) if len(pf_x_err) > 0 else 0
        
        # Update RMSE bar chart
        self.ax_rmse.clear()
        self.ax_rmse.set_title('Root Mean Square Error', fontsize=10)
        self.ax_rmse.set_ylabel('RMSE (m)')
        self.ax_rmse.grid(True, alpha=0.3, axis='y')
        
        filters = ['EKF', 'UKF', 'PF']
        rmse_values = [ekf_rmse, ukf_rmse, pf_rmse]
        colors = ['red', 'blue', 'green']
        
        bars = self.ax_rmse.bar(filters, rmse_values, color=colors, alpha=0.7)
        for bar in bars:
            height = bar.get_height()
            self.ax_rmse.text(bar.get_x() + bar.get_width()/2., height,
                             f'{height:.4f}m', ha='center', va='bottom', fontsize=9)
        
        # Update statistics text
        stats_str = f"Performance Statistics\n"
        stats_str += f"{'='*35}\n\n"
        
        if len(ekf_x_err) > 10:
            stats_str += f"EKF:\n"
            stats_str += f"  RMSE:      {ekf_rmse:.4f} m\n"
            stats_str += f"  Std Dev:   {np.mean(self.ekf_cov) if len(self.ekf_cov) > 0 else 0:.4f} m\n"
            stats_str += f"  Samples:   {len(ekf_x_err)}\n\n"
        
        if len(ukf_x_err) > 10:
            stats_str += f"UKF:\n"
            stats_str += f"  RMSE:      {ukf_rmse:.4f} m\n"
            stats_str += f"  Std Dev:   {np.mean(self.ukf_cov) if len(self.ukf_cov) > 0 else 0:.4f} m\n"
            stats_str += f"  Samples:   {len(ukf_x_err)}\n\n"
        
        if len(pf_x_err) > 10:
            stats_str += f"PF:\n"
            stats_str += f"  RMSE:      {pf_rmse:.4f} m\n"
            stats_str += f"  Std Dev:   {np.mean(self.pf_cov) if len(self.pf_cov) > 0 else 0:.4f} m\n"
            stats_str += f"  Samples:   {len(pf_x_err)}\n\n"
        
        if ekf_rmse > 0 and ukf_rmse > 0 and pf_rmse > 0:
            best_filter = filters[np.argmin(rmse_values)]
            stats_str += f"Best Filter: {best_filter}\n"
        
        self.stats_text.set_text(stats_str)
        
    def run(self):
        """Run the visualizer with animation"""
        ani = FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    node = FilterComparatorNode()
    
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
