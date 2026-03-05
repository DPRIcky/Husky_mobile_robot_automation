#!/usr/bin/env python3
"""
State Estimation Node using Unscented Kalman Filter (UKF)
Handles nonlinearities better than EKF using unscented transform

State Vector: [x, y, theta, vx, vy, omega, ax, ay]
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from transforms3d.euler import euler2quat, quat2euler
import math


class UnscentedKalmanFilter:
    """Unscented Kalman Filter for nonlinear state estimation"""
    
    def __init__(self, state_dim=8, dt=0.02):
        self.state_dim = state_dim
        self.dt = dt
        
        # State vector: [x, y, theta, vx, vy, omega, ax, ay]
        self.state = np.zeros(state_dim)
        
        # State covariance matrix
        self.P = np.eye(state_dim) * 1.0
        
        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.01
        self.Q[6:8, 6:8] = np.eye(2) * 0.1  # Higher noise for accelerations
        
        # UKF parameters (alpha, beta, kappa)
        self.alpha = 1e-3  # Spread of sigma points
        self.beta = 2.0    # Optimal for Gaussian distributions
        self.kappa = 0.0   # Secondary scaling parameter
        
        # Compute lambda and weights
        self.lambda_ = self.alpha**2 * (self.state_dim + self.kappa) - self.state_dim
        
        # Weights for sigma points
        self.Wm = np.zeros(2 * self.state_dim + 1)  # Weights for means
        self.Wc = np.zeros(2 * self.state_dim + 1)  # Weights for covariances
        
        self.Wm[0] = self.lambda_ / (self.state_dim + self.lambda_)
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)
        
        for i in range(1, 2 * self.state_dim + 1):
            self.Wm[i] = 1 / (2 * (self.state_dim + self.lambda_))
            self.Wc[i] = self.Wm[i]
    
    def _generate_sigma_points(self, state, P):
        """Generate sigma points around the mean state"""
        sigma_points = np.zeros((2 * self.state_dim + 1, self.state_dim))
        sigma_points[0] = state
        
        # Matrix square root using Cholesky decomposition
        try:
            L = np.linalg.cholesky((self.state_dim + self.lambda_) * P)
        except np.linalg.LinAlgError:
            # If Cholesky fails, use eigenvalue decomposition
            eigenvalues, eigenvectors = np.linalg.eigh(P)
            eigenvalues = np.maximum(eigenvalues, 1e-10)  # Ensure positive
            L = eigenvectors @ np.diag(np.sqrt(eigenvalues * (self.state_dim + self.lambda_)))
        
        for i in range(self.state_dim):
            sigma_points[i + 1] = state + L[:, i]
            sigma_points[i + 1 + self.state_dim] = state - L[:, i]
        
        return sigma_points
    
    def _motion_model(self, state):
        """Apply motion model to a state vector"""
        dt = self.dt
        x, y, theta, vx, vy, omega, ax, ay = state
        
        # Motion model (constant acceleration)
        new_state = np.zeros(self.state_dim)
        new_state[0] = x + vx * dt + 0.5 * ax * dt**2
        new_state[1] = y + vy * dt + 0.5 * ay * dt**2
        new_state[2] = self._normalize_angle(theta + omega * dt)
        new_state[3] = vx + ax * dt
        new_state[4] = vy + ay * dt
        new_state[5] = omega
        new_state[6] = ax
        new_state[7] = ay
        
        return new_state
    
    def predict(self):
        """UKF Prediction step using unscented transform"""
        # Generate sigma points
        sigma_points = self._generate_sigma_points(self.state, self.P)
        
        # Propagate sigma points through motion model
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(2 * self.state_dim + 1):
            sigma_points_pred[i] = self._motion_model(sigma_points[i])
        
        # Compute predicted mean
        self.state = np.zeros(self.state_dim)
        for i in range(2 * self.state_dim + 1):
            self.state += self.Wm[i] * sigma_points_pred[i]
        
        # Normalize angle in state
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Compute predicted covariance
        self.P = self.Q.copy()
        for i in range(2 * self.state_dim + 1):
            diff = sigma_points_pred[i] - self.state
            diff[2] = self._normalize_angle(diff[2])  # Normalize angle difference
            self.P += self.Wc[i] * np.outer(diff, diff)
    
    def update_imu(self, linear_accel, angular_vel, orientation):
        """Update state with IMU measurements using unscented transform"""
        # Measurement dimension
        meas_dim = 4  # theta, omega, ax, ay
        
        # Generate sigma points
        sigma_points = self._generate_sigma_points(self.state, self.P)
        
        # Propagate through measurement model
        Z = np.zeros((2 * self.state_dim + 1, meas_dim))
        for i in range(2 * self.state_dim + 1):
            # Measurement model: h(x) = [theta, omega, ax, ay]
            Z[i] = np.array([
                sigma_points[i, 2],  # theta
                sigma_points[i, 5],  # omega
                sigma_points[i, 6],  # ax
                sigma_points[i, 7]   # ay
            ])
        
        # Compute predicted measurement mean
        z_pred = np.zeros(meas_dim)
        for i in range(2 * self.state_dim + 1):
            z_pred += self.Wm[i] * Z[i]
        
        # Actual measurement
        z = np.array([orientation, angular_vel, linear_accel[0], linear_accel[1]])
        
        # Measurement noise covariance
        R = np.diag([0.01, 0.01, 0.1, 0.1])
        
        # Innovation covariance
        Pzz = R.copy()
        for i in range(2 * self.state_dim + 1):
            diff = Z[i] - z_pred
            diff[0] = self._normalize_angle(diff[0])  # Normalize angle
            Pzz += self.Wc[i] * np.outer(diff, diff)
        
        # Cross-covariance
        Pxz = np.zeros((self.state_dim, meas_dim))
        for i in range(2 * self.state_dim + 1):
            state_diff = sigma_points[i] - self.state
            state_diff[2] = self._normalize_angle(state_diff[2])
            meas_diff = Z[i] - z_pred
            meas_diff[0] = self._normalize_angle(meas_diff[0])
            Pxz += self.Wc[i] * np.outer(state_diff, meas_diff)
        
        # Kalman gain
        K = Pxz @ np.linalg.inv(Pzz)
        
        # Update state
        innovation = z - z_pred
        innovation[0] = self._normalize_angle(innovation[0])
        self.state = self.state + K @ innovation
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Update covariance
        self.P = self.P - K @ Pzz @ K.T
    
    def update_odometry(self, x, y, theta, vx, vy, omega):
        """Update state with odometry measurements using unscented transform"""
        # Measurement dimension
        meas_dim = 6
        
        # Generate sigma points
        sigma_points = self._generate_sigma_points(self.state, self.P)
        
        # Propagate through measurement model
        Z = np.zeros((2 * self.state_dim + 1, meas_dim))
        for i in range(2 * self.state_dim + 1):
            Z[i] = sigma_points[i, :6]  # [x, y, theta, vx, vy, omega]
        
        # Compute predicted measurement mean
        z_pred = np.zeros(meas_dim)
        for i in range(2 * self.state_dim + 1):
            z_pred += self.Wm[i] * Z[i]
        
        # Actual measurement
        z = np.array([x, y, theta, vx, vy, omega])
        
        # Measurement noise covariance
        R = np.diag([0.1, 0.1, 0.05, 0.1, 0.1, 0.05])
        
        # Innovation covariance
        Pzz = R.copy()
        for i in range(2 * self.state_dim + 1):
            diff = Z[i] - z_pred
            diff[2] = self._normalize_angle(diff[2])
            Pzz += self.Wc[i] * np.outer(diff, diff)
        
        # Cross-covariance
        Pxz = np.zeros((self.state_dim, meas_dim))
        for i in range(2 * self.state_dim + 1):
            state_diff = sigma_points[i] - self.state
            state_diff[2] = self._normalize_angle(state_diff[2])
            meas_diff = Z[i] - z_pred
            meas_diff[2] = self._normalize_angle(meas_diff[2])
            Pxz += self.Wc[i] * np.outer(state_diff, meas_diff)
        
        # Kalman gain
        K = Pxz @ np.linalg.inv(Pzz)
        
        # Update state
        innovation = z - z_pred
        innovation[2] = self._normalize_angle(innovation[2])
        self.state = self.state + K @ innovation
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Update covariance
        self.P = self.P - K @ Pzz @ K.T
    
    def get_state(self):
        """Return current state estimate"""
        return self.state.copy()
    
    def get_covariance(self):
        """Return current covariance matrix"""
        return self.P.copy()
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


class SensorFusionUKFNode(Node):
    """ROS2 Node for UKF-based sensor fusion"""
    
    def __init__(self):
        super().__init__('sensor_fusion_ukf')
        
        # Declare parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('use_imu', True)
        self.declare_parameter('use_odometry', True)
        self.declare_parameter('robot_namespace', 'a300_00000')
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_odometry = self.get_parameter('use_odometry').value
        robot_ns = self.get_parameter('robot_namespace').value
        
        self.get_logger().info(f'Starting UKF sensor fusion node at {self.update_rate} Hz')
        
        # Initialize UKF
        dt = 1.0 / self.update_rate
        self.ukf = UnscentedKalmanFilter(state_dim=8, dt=dt)
        
        # QoS profile for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                f'/{robot_ns}/sensors/imu_0/data',
                self.imu_callback,
                sensor_qos
            )
            
        if self.use_odometry:
            self.odom_sub = self.create_subscription(
                Odometry,
                f'/{robot_ns}/platform/odom',
                self.odom_callback,
                10
            )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/state_estimate_ukf/pose',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/state_estimate_ukf/odom',
            10
        )
        
        # Timer for prediction step
        self.timer = self.create_timer(dt, self.timer_callback)
        
        self.get_logger().info('UKF sensor fusion node initialized')
        
    def imu_callback(self, msg):
        """Process IMU data"""
        orientation_q = msg.orientation
        (roll, pitch, yaw) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        omega = msg.angular_velocity.z
        linear_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y]
        
        self.ukf.update_imu(linear_accel, omega, yaw)
        
    def odom_callback(self, msg):
        """Process odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        self.ukf.update_odometry(x, y, theta, vx, vy, omega)
        
    def timer_callback(self):
        """Periodic prediction and publishing"""
        self.ukf.predict()
        state = self.ukf.get_state()
        cov = self.ukf.get_covariance()
        self.publish_state(state, cov)
        
    def publish_state(self, state, covariance):
        """Publish estimated state"""
        current_time = self.get_clock().now().to_msg()
        x, y, theta, vx, vy, omega, ax, ay = state
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        q = euler2quat(0, 0, theta)
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = covariance[0:3, 0:3]
        odom_msg.pose.covariance = pose_cov.flatten().tolist()
        
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionUKFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
