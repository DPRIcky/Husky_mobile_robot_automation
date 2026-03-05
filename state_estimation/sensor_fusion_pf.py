#!/usr/bin/env python3
"""
State Estimation Node using Particle Filter (PF)
Handles multi-modal and highly nonlinear distributions

State Vector: [x, y, theta, vx, vy, omega, ax, ay]
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat, quat2euler
import math


class ParticleFilter:
    """Particle Filter for nonlinear/non-Gaussian state estimation"""
    
    def __init__(self, state_dim=8, num_particles=500, dt=0.02):
        self.state_dim = state_dim
        self.num_particles = num_particles
        self.dt = dt
        
        # Initialize particles uniformly around origin
        self.particles = np.random.randn(num_particles, state_dim) * 0.5
        
        # Initialize weights uniformly
        self.weights = np.ones(num_particles) / num_particles
        
        # Process noise standard deviations
        self.process_noise_std = np.array([0.05, 0.05, 0.02, 0.05, 0.05, 0.02, 0.1, 0.1])
        
        # Effective sample size threshold for resampling
        self.resample_threshold = num_particles / 2.0
        
    def predict(self):
        """Prediction step: propagate particles through motion model"""
        dt = self.dt
        
        for i in range(self.num_particles):
            x, y, theta, vx, vy, omega, ax, ay = self.particles[i]
            
            # Motion model with process noise
            noise = np.random.randn(self.state_dim) * self.process_noise_std
            
            self.particles[i, 0] = x + vx * dt + 0.5 * ax * dt**2 + noise[0]
            self.particles[i, 1] = y + vy * dt + 0.5 * ay * dt**2 + noise[1]
            self.particles[i, 2] = self._normalize_angle(theta + omega * dt + noise[2])
            self.particles[i, 3] = vx + ax * dt + noise[3]
            self.particles[i, 4] = vy + ay * dt + noise[4]
            self.particles[i, 5] = omega + noise[5]
            self.particles[i, 6] = ax + noise[6]
            self.particles[i, 7] = ay + noise[7]
    
    def update_imu(self, linear_accel, angular_vel, orientation):
        """Update particle weights based on IMU measurements"""
        # Measurement: [theta, omega, ax, ay]
        z = np.array([orientation, angular_vel, linear_accel[0], linear_accel[1]])
        
        # Measurement noise standard deviations
        meas_noise_std = np.array([0.1, 0.1, 0.3, 0.3])
        
        # Update weights based on measurement likelihood
        for i in range(self.num_particles):
            # Extract relevant states from particle
            h = np.array([
                self.particles[i, 2],  # theta
                self.particles[i, 5],  # omega
                self.particles[i, 6],  # ax
                self.particles[i, 7]   # ay
            ])
            
            # Compute innovation
            innovation = z - h
            innovation[0] = self._normalize_angle(innovation[0])
            
            # Gaussian likelihood
            likelihood = 1.0
            for j in range(4):
                likelihood *= np.exp(-0.5 * (innovation[j] / meas_noise_std[j])**2)
                likelihood /= (meas_noise_std[j] * np.sqrt(2 * np.pi))
            
            self.weights[i] *= likelihood
        
        # Normalize weights
        weight_sum = np.sum(self.weights)
        if weight_sum > 1e-10:
            self.weights /= weight_sum
        else:
            # Reset weights if all are near zero
            self.weights = np.ones(self.num_particles) / self.num_particles
        
        # Check if resampling is needed
        self._resample_if_needed()
    
    def update_odometry(self, x, y, theta, vx, vy, omega):
        """Update particle weights based on odometry measurements"""
        # Measurement: [x, y, theta, vx, vy, omega]
        z = np.array([x, y, theta, vx, vy, omega])
        
        # Measurement noise standard deviations
        meas_noise_std = np.array([0.3, 0.3, 0.2, 0.3, 0.3, 0.2])
        
        # Update weights based on measurement likelihood
        for i in range(self.num_particles):
            # Extract relevant states from particle
            h = self.particles[i, :6]  # [x, y, theta, vx, vy, omega]
            
            # Compute innovation
            innovation = z - h
            innovation[2] = self._normalize_angle(innovation[2])
            
            # Gaussian likelihood
            likelihood = 1.0
            for j in range(6):
                likelihood *= np.exp(-0.5 * (innovation[j] / meas_noise_std[j])**2)
                likelihood /= (meas_noise_std[j] * np.sqrt(2 * np.pi))
            
            self.weights[i] *= likelihood
        
        # Normalize weights
        weight_sum = np.sum(self.weights)
        if weight_sum > 1e-10:
            self.weights /= weight_sum
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles
        
        # Check if resampling is needed
        self._resample_if_needed()
    
    def _resample_if_needed(self):
        """Resample particles if effective sample size is too low"""
        # Compute effective sample size
        neff = 1.0 / np.sum(self.weights**2)
        
        if neff < self.resample_threshold:
            self._resample()
    
    def _resample(self):
        """Systematic resampling of particles"""
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0  # Ensure last value is exactly 1
        
        # Systematic resampling
        positions = (np.arange(self.num_particles) + np.random.random()) / self.num_particles
        
        indices = np.zeros(self.num_particles, dtype=int)
        i, j = 0, 0
        while i < self.num_particles:
            if positions[i] < cumsum[j]:
                indices[i] = j
                i += 1
            else:
                j += 1
        
        # Resample particles
        self.particles = self.particles[indices]
        
        # Reset weights to uniform
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        # Add small noise to avoid particle depletion
        noise = np.random.randn(self.num_particles, self.state_dim) * self.process_noise_std * 0.1
        self.particles += noise
        
        # Normalize angles
        self.particles[:, 2] = np.array([self._normalize_angle(theta) for theta in self.particles[:, 2]])
    
    def get_state(self):
        """Return weighted mean of particles"""
        state = np.average(self.particles, weights=self.weights, axis=0)
        state[2] = self._circular_mean(self.particles[:, 2], self.weights)
        return state
    
    def get_covariance(self):
        """Return weighted covariance of particles"""
        mean = self.get_state()
        
        # Compute weighted covariance
        diff = self.particles - mean
        diff[:, 2] = np.array([self._normalize_angle(angle) for angle in diff[:, 2]])
        
        cov = np.zeros((self.state_dim, self.state_dim))
        for i in range(self.num_particles):
            cov += self.weights[i] * np.outer(diff[i], diff[i])
        
        return cov
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    @staticmethod
    def _circular_mean(angles, weights):
        """Compute circular mean of angles"""
        sin_sum = np.sum(weights * np.sin(angles))
        cos_sum = np.sum(weights * np.cos(angles))
        return np.arctan2(sin_sum, cos_sum)


class SensorFusionPFNode(Node):
    """ROS2 Node for Particle Filter-based sensor fusion"""
    
    def __init__(self):
        super().__init__('sensor_fusion_pf')
        
        # Declare parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('num_particles', 500)
        self.declare_parameter('use_imu', True)
        self.declare_parameter('use_odometry', True)
        self.declare_parameter('robot_namespace', 'a300_00000')
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        num_particles = self.get_parameter('num_particles').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_odometry = self.get_parameter('use_odometry').value
        robot_ns = self.get_parameter('robot_namespace').value
        
        self.get_logger().info(f'Starting PF sensor fusion node at {self.update_rate} Hz with {num_particles} particles')
        
        # Initialize Particle Filter
        dt = 1.0 / self.update_rate
        self.pf = ParticleFilter(state_dim=8, num_particles=num_particles, dt=dt)
        
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
            '/state_estimate_pf/pose',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/state_estimate_pf/odom',
            10
        )
        
        # Timer for prediction step
        self.timer = self.create_timer(dt, self.timer_callback)
        
        self.get_logger().info('PF sensor fusion node initialized')
        
    def imu_callback(self, msg):
        """Process IMU data"""
        orientation_q = msg.orientation
        (roll, pitch, yaw) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        omega = msg.angular_velocity.z
        linear_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y]
        
        self.pf.update_imu(linear_accel, omega, yaw)
        
    def odom_callback(self, msg):
        """Process odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        (_, _, theta) = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        self.pf.update_odometry(x, y, theta, vx, vy, omega)
        
    def timer_callback(self):
        """Periodic prediction and publishing"""
        self.pf.predict()
        state = self.pf.get_state()
        cov = self.pf.get_covariance()
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
    node = SensorFusionPFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
