#!/usr/bin/env python3
"""
Simple test script to verify state estimation setup
Publishes mock sensor data to test the filter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import math


class MockSensorPublisher(Node):
    """Publishes mock sensor data for testing"""
    
    def __init__(self):
        super().__init__('mock_sensor_publisher')
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/a300_00000/sensors/imu_0/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/a300_00000/platform/odom', 10)
        
        # Simulation parameters
        self.t = 0.0
        self.dt = 0.02  # 50 Hz
        
        # Simulated robot state (circular motion)
        self.radius = 5.0  # Circle radius
        self.angular_vel = 0.5  # rad/s
        self.linear_vel = self.radius * self.angular_vel
        
        # Timer for publishing
        self.timer = self.create_timer(self.dt, self.publish_sensors)
        
        self.get_logger().info('Mock sensor publisher started - simulating circular motion')
        
    def publish_sensors(self):
        """Publish mock IMU and odometry data"""
        self.t += self.dt
        
        # Simulate circular motion
        x = self.radius * math.cos(self.angular_vel * self.t)
        y = self.radius * math.sin(self.angular_vel * self.t)
        theta = self.angular_vel * self.t + math.pi / 2
        
        vx = -self.linear_vel * math.sin(self.angular_vel * self.t)
        vy = self.linear_vel * math.cos(self.angular_vel * self.t)
        
        # Centripetal acceleration
        ax = -self.angular_vel**2 * self.radius * math.cos(self.angular_vel * self.t)
        ay = -self.angular_vel**2 * self.radius * math.sin(self.angular_vel * self.t)
        
        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Orientation (as quaternion)
        qz = math.sin(theta / 2)
        qw = math.cos(theta / 2)
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        # Angular velocity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = self.angular_vel
        
        # Linear acceleration (with noise)
        noise_scale = 0.1
        imu_msg.linear_acceleration.x = ax + np.random.normal(0, noise_scale)
        imu_msg.linear_acceleration.y = ay + np.random.normal(0, noise_scale)
        imu_msg.linear_acceleration.z = 0.0
        
        self.imu_pub.publish(imu_msg)
        
        # Publish Odometry data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position (with noise)
        noise_pos = 0.05
        odom_msg.pose.pose.position.x = x + np.random.normal(0, noise_pos)
        odom_msg.pose.pose.position.y = y + np.random.normal(0, noise_pos)
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        # Velocity (with noise)
        noise_vel = 0.05
        odom_msg.twist.twist.linear.x = vx + np.random.normal(0, noise_vel)
        odom_msg.twist.twist.linear.y = vy + np.random.normal(0, noise_vel)
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = MockSensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
