#!/usr/bin/env python3
"""
Launch file for sensor fusion state estimation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(os.path.dirname(current_dir), 'config')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='a300_00000',
        description='Robot namespace'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_dir, 'sensor_fusion.yaml'),
        description='Path to sensor fusion configuration file'
    )
    
    # Sensor fusion node
    sensor_fusion_node = Node(
        package='state_estimation',  # You may need to adjust this
        executable='sensor_fusion_ekf.py',
        name='sensor_fusion_ekf',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        config_file_arg,
        sensor_fusion_node,
    ])
