#!/usr/bin/env python3
"""Launch the ArUco follower for a300_00000."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('target_pose_topic',
                              default_value='/a300_00000/aruco_detector/target_pose'),
        DeclareLaunchArgument('cmd_vel_topic',
                              default_value='/a300_00000/cmd_vel'),
        DeclareLaunchArgument('desired_standoff_m', default_value='1.5'),
        DeclareLaunchArgument('target_timeout_s', default_value='0.5'),
    ]

    follower = Node(
        package='autonomy_bringup',
        executable='aruco_follower',
        name='aruco_follower',
        namespace='a300_00000',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_pose_topic': LaunchConfiguration('target_pose_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'desired_standoff_m': LaunchConfiguration('desired_standoff_m'),
            'target_timeout_s': LaunchConfiguration('target_timeout_s'),
        }],
    )

    ld = LaunchDescription(args)
    ld.add_action(follower)
    return ld
