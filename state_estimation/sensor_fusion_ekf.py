#!/usr/bin/env python3
"""
State Estimation Node using Extended Kalman Filter (EKF)
Fuses IMU, Lidar, GPS, and Odometry data for robust state estimation

State Vector: [x, y, theta, vx, vy, omega, ax, ay]
- x, y: Position in 2D
- theta: Orientation (yaw)
- vx, vy: Linear velocities
- omega: Angular velocity
- ax, ay: Linear accelerations
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from transforms3d.euler import euler2quat, quat2euler
import math


class ExtendedKalmanFilter:
    """Extended Kalman Filter for sensor fusion"""
    
    def __init__(self, state_dim=8, dt=0.02):
        self.state_dim = state_dim
        self.dt = dt
        
        # State vector: [x, y, theta, vx, vy, omega, ax, ay]
        self.state = np.zeros(state_dim)
        
        # State covariance matrix
        self.P = np.eye(state_dim) * 1.0
        
        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.01
        self.Q[6:8, 6:8] = np.eye(2) * 0.1  
        
        # Measurement noise covariance
        self.R = np.eye(state_dim)
        
    def predict(self):
        """Prediction step using motion model"""
        dt = self.dt
        
        # Extract state components
        x, y, theta, vx, vy, omega, ax, ay = self.state
        
        # Motion model
        # x_k+1 = x_k + vx*dt + 0.5*ax*dt^2
        # y_k+1 = y_k + vy*dt + 0.5*ay*dt^2
        # theta_k+1 = theta_k + omega*dt
        # vx_k+1 = vx_k + ax*dt
        # vy_k+1 = vy_k + ay*dt
        # omega, ax, ay remain constant
        
        # Predict new state
        self.state[0] += vx * dt + 0.5 * ax * dt**2  # x
        self.state[1] += vy * dt + 0.5 * ay * dt**2  # y
        self.state[2] += omega * dt  # theta
        self.state[3] += ax * dt  # vx
        self.state[4] += ay * dt  # vy
        # omega, ax, ay remain the same
        
        # Normalize theta to [-pi, pi]
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Jacobian of motion model
        F = np.eye(self.state_dim)
        F[0, 3] = dt
        F[0, 6] = 0.5 * dt**2
        F[1, 4] = dt
        F[1, 7] = 0.5 * dt**2
        F[2, 5] = dt
        F[3, 6] = dt
        F[4, 7] = dt
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_imu(self, linear_accel, angular_vel, orientation):
        """Update state with IMU measurements"""

        # Measurement vector: [theta, omega, ax, ay]
        z = np.array([orientation, angular_vel, linear_accel[0], linear_accel[1]])
        
        # Measurement model: H maps state to measurement
        H = np.zeros((4, self.state_dim))
        H[0, 2] = 1.0  # theta
        H[1, 5] = 1.0  # omega
        H[2, 6] = 1.0  # ax
        H[3, 7] = 1.0  # ay
        
        # Measurement noise covariance
        R = np.diag([0.01, 0.01, 0.1, 0.1])  # theta, omega, ax, ay
        
        # Innovation
        h = H @ self.state  # Predicted measurement
        y = z - h
        y[0] = self._normalize_angle(y[0])  # Normalize angle difference
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Update covariance
        I_KH = np.eye(self.state_dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
    def update_odometry(self, x, y, theta, vx, vy, omega):
        """Update state with odometry measurements"""
        # Measurement vector: [x, y, theta, vx, vy, omega]
        z = np.array([x, y, theta, vx, vy, omega])
        
        # Measurement model
        H = np.zeros((6, self.state_dim))
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y
        H[2, 2] = 1.0  # theta
        H[3, 3] = 1.0  # vx
        H[4, 4] = 1.0  # vy
        H[5, 5] = 1.0  # omega
        
        # Measurement noise covariance
        R = np.diag([0.1, 0.1, 0.05, 0.1, 0.1, 0.05])
        
        # Innovation
        h = H @ self.state
        y = z - h
        y[2] = self._normalize_angle(y[2])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Update covariance
        I_KH = np.eye(self.state_dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
    def update_gps(self, x, y):
        """Update state with GPS measurements"""
        # Measurement vector: [x, y]
        z = np.array([x, y])
        
        # Measurement model
        H = np.zeros((2, self.state_dim))
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y
        
        # Measurement noise covariance
        R = np.diag([1.0, 1.0])
        
        # Innovation
        h = H @ self.state
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        I_KH = np.eye(self.state_dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def update_lidar(self, dx, dy, dtheta):
        """Update state with LIDAR scan matching measurements"""
        # Measurement vector: [dx, dy, dtheta] - relative displacement from scan matching
        # Convert to absolute position correction
        current_x = self.state[0]
        current_y = self.state[1]
        current_theta = self.state[2]
        
        # Transform relative displacement to global frame
        cos_theta = np.cos(current_theta)
        sin_theta = np.sin(current_theta)
        
        dx_global = cos_theta * dx - sin_theta * dy
        dy_global = sin_theta * dx + cos_theta * dy
        
        # Expected position based on scan matching
        x_meas = current_x + dx_global
        y_meas = current_y + dy_global
        theta_meas = current_theta + dtheta
        
        z = np.array([x_meas, y_meas, theta_meas])
        
        # Measurement model
        H = np.zeros((3, self.state_dim))
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y
        H[2, 2] = 1.0  # theta
        
        # Measurement noise covariance (LIDAR scan matching is fairly accurate)
        R = np.diag([0.05, 0.05, 0.02])
        
        # Innovation
        h = H @ self.state
        y = z - h
        y[2] = self._normalize_angle(y[2])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Update covariance
        I_KH = np.eye(self.state_dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
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


class SensorFusionNode(Node):
    """ROS2 Node for sensor fusion and state estimation"""
    
    def __init__(self):
        super().__init__('sensor_fusion_ekf')
        
        # Declare parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('use_imu', True)
        self.declare_parameter('use_odometry', True)
        self.declare_parameter('use_gps', True)
        self.declare_parameter('use_lidar', False)
        self.declare_parameter('robot_namespace', 'a300_00000')
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_odometry = self.get_parameter('use_odometry').value
        self.use_gps = self.get_parameter('use_gps').value
        self.use_lidar = self.get_parameter('use_lidar').value
        robot_ns = self.get_parameter('robot_namespace').value
        
        self.get_logger().info(f'Starting sensor fusion node with update rate: {self.update_rate} Hz')
        
        # Initialize EKF
        dt = 1.0 / self.update_rate
        self.ekf = ExtendedKalmanFilter(state_dim=8, dt=dt)
        
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
            self.get_logger().info('Subscribed to IMU data')
            
        if self.use_odometry:
            self.odom_sub = self.create_subscription(
                Odometry,
                f'/{robot_ns}/platform/odom',
                self.odom_callback,
                10
            )
            self.get_logger().info('Subscribed to Odometry data')
            
        if self.use_gps:
            self.gps_sub = self.create_subscription(
                NavSatFix,
                f'/{robot_ns}/sensors/gps_0/fix',
                self.gps_callback,
                sensor_qos
            )
            self.get_logger().info('Subscribed to GPS data')
            
        if self.use_lidar:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                f'/{robot_ns}/sensors/lidar2d_0/scan',
                self.lidar_callback,
                sensor_qos
            )
            self.get_logger().info('Subscribed to Lidar data')
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/state_estimate/pose',
            10
        )
        
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/state_estimate/twist',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/state_estimate/odom',
            10
        )
        
        # Timer for prediction step
        self.timer = self.create_timer(dt, self.timer_callback)
        
        # Tracking variables
        self.last_imu_time = None
        self.last_odom_time = None
        self.gps_origin = None  # For GPS to local coordinate conversion
        self.prev_scan = None  # For LIDAR scan matching
        self.prev_scan_pose = None  # Pose when previous scan was taken
        
        self.get_logger().info('Sensor fusion node initialized')
        
    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation (roll, pitch, yaw)
        orientation_q = msg.orientation
        # transforms3d uses (w, x, y, z) order
        (roll, pitch, yaw) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        # Extract angular velocity
        omega = msg.angular_velocity.z
        
        # Extract linear acceleration
        linear_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y]
        
        # Update EKF with IMU data
        self.ekf.update_imu(linear_accel, omega, yaw)
        
        self.last_imu_time = self.get_clock().now()
        
    def odom_callback(self, msg):
        """Process odometry data"""
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        # transforms3d uses (w, x, y, z) order
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Update EKF with odometry data
        self.ekf.update_odometry(x, y, theta, vx, vy, omega)
        
        self.last_odom_time = self.get_clock().now()
        
    def gps_callback(self, msg):
        """Process GPS data"""
        if msg.status.status < 0:  # No GPS
            return
            
        # Initialize GPS origin
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude)
            self.get_logger().info(f'GPS origin set to: {self.gps_origin}')
            return
        
        # Convert GPS to local coordinates (relative to origin)
        lat, lon = msg.latitude, msg.longitude
        lat0, lon0 = self.gps_origin
        
        # Approximate conversion
        meters_per_deg_lat = 111132.92
        meters_per_deg_lon = 111132.92 * math.cos(math.radians(lat0))
        
        x = (lon - lon0) * meters_per_deg_lon
        y = (lat - lat0) * meters_per_deg_lat
        
        # Update EKF with GPS data
        self.ekf.update_gps(x, y)
        
    def lidar_callback(self, msg):
        """Process Lidar data using scan matching"""
        # Convert LaserScan to Cartesian points
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates (in sensor frame)
        current_scan = np.column_stack((
            valid_ranges * np.cos(valid_angles),
            valid_ranges * np.sin(valid_angles)
        ))
        
        # Skip if scan is too sparse
        if len(current_scan) < 10:
            return
        
        # Initialize previous scan if not set
        if self.prev_scan is None:
            self.prev_scan = current_scan
            self.prev_scan_pose = self.ekf.get_state()[:3].copy()  # [x, y, theta]
            return
        
        # Perform simple ICP-based scan matching
        dx, dy, dtheta = self.simple_scan_matching(self.prev_scan, current_scan)
        
        # Update EKF if displacement is significant and reasonable
        displacement_magnitude = np.sqrt(dx**2 + dy**2)
        if displacement_magnitude > 0.001 and displacement_magnitude < 1.0:  # Between 1mm and 1m
            self.ekf.update_lidar(dx, dy, dtheta)
        
        # Update previous scan (downsample to reduce memory)
        if len(current_scan) > 100:
            indices = np.linspace(0, len(current_scan)-1, 100, dtype=int)
            self.prev_scan = current_scan[indices]
        else:
            self.prev_scan = current_scan
        self.prev_scan_pose = self.ekf.get_state()[:3].copy()
    
    def simple_scan_matching(self, prev_scan, current_scan):
        """Simple scan matching using iterative closest point (ICP) approach"""
        # Simplified ICP for 2D lidar scans
        # Returns relative displacement (dx, dy, dtheta)
        
        max_iterations = 10
        convergence_threshold = 0.001
        
        dx, dy, dtheta = 0.0, 0.0, 0.0
        
        for iteration in range(max_iterations):
            # Transform current scan by current estimate
            cos_theta = np.cos(dtheta)
            sin_theta = np.sin(dtheta)
            
            transformed_scan = np.column_stack((
                cos_theta * current_scan[:, 0] - sin_theta * current_scan[:, 1] + dx,
                sin_theta * current_scan[:, 0] + cos_theta * current_scan[:, 1] + dy
            ))
            
            # Find closest points in previous scan
            total_dx = 0.0
            total_dy = 0.0
            total_dtheta = 0.0
            count = 0
            
            for point in transformed_scan[::5]:  # Sample every 5th point for speed
                # Find closest point in previous scan
                distances = np.sum((prev_scan - point)**2, axis=1)
                closest_idx = np.argmin(distances)
                
                if distances[closest_idx] < 0.25:  # Within 50cm
                    closest_point = prev_scan[closest_idx]
                    total_dx += closest_point[0] - point[0]
                    total_dy += closest_point[1] - point[1]
                    count += 1
            
            if count == 0:
                break
            
            # Average displacement
            avg_dx = total_dx / count
            avg_dy = total_dy / count
            
            # Update estimate
            dx += avg_dx
            dy += avg_dy
            
            # Check convergence
            if abs(avg_dx) < convergence_threshold and abs(avg_dy) < convergence_threshold:
                break
        
        # Estimate rotation from point correspondences
        if np.sqrt(dx**2 + dy**2) > 0.01:
            # Small angle approximation
            dtheta = np.arctan2(dy, dx) * 0.1
        
        return dx, dy, dtheta
        
    def timer_callback(self):
        """Periodic prediction and publishing"""
        # Prediction step
        self.ekf.predict()
        
        # Get current state estimate
        state = self.ekf.get_state()
        cov = self.ekf.get_covariance()
        
        # Publish state estimate
        self.publish_state(state, cov)
        
    def publish_state(self, state, covariance):
        """Publish estimated state"""
        current_time = self.get_clock().now().to_msg()
        
        # Extract state components
        x, y, theta, vx, vy, omega, ax, ay = state
        
        # Publish Pose with Covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'odom'
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        # euler2quat returns (w, x, y, z)
        q = euler2quat(0, 0, theta)
        pose_msg.pose.pose.orientation.w = q[0]
        pose_msg.pose.pose.orientation.x = q[1]
        pose_msg.pose.pose.orientation.y = q[2]
        pose_msg.pose.pose.orientation.z = q[3]
        
        # Extract relevant covariance (6x6 for pose)
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = covariance[0:3, 0:3]  # x, y, theta
        pose_msg.pose.covariance = pose_cov.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
        
        # Publish Twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time
        twist_msg.header.frame_id = 'base_link'
        
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = omega
        
        self.twist_pub.publish(twist_msg)
        
        # Publish Odometry (combined pose and twist)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose = pose_msg.pose.pose
        odom_msg.pose.covariance = pose_msg.pose.covariance
        
        odom_msg.twist.twist = twist_msg.twist
        # Twist covariance
        twist_cov = np.zeros((6, 6))
        twist_cov[0:3, 0:3] = covariance[3:6, 3:6]  # vx, vy, omega
        odom_msg.twist.covariance = twist_cov.flatten().tolist()
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = SensorFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
