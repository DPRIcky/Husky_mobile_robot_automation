#!/usr/bin/env python3
"""Launch the workspace-local ArUco detector against a robot camera stream."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('observer_namespace', default_value='a300_00000'),
        DeclareLaunchArgument('image_topic',
                              default_value='/a300_00000/sensors/camera_0/color/image'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/a300_00000/sensors/camera_0/color/camera_info'),
        DeclareLaunchArgument('marker_length_m', default_value='0.4'),
        DeclareLaunchArgument('dictionary', default_value='DICT_4X4_50'),
    ]

    detector = Node(
        package='autonomy_bringup',
        executable='aruco_detector',
        name='aruco_detector',
        namespace=LaunchConfiguration('observer_namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'marker_length_m': LaunchConfiguration('marker_length_m'),
            'dictionary': LaunchConfiguration('dictionary'),
        }],
    )

    ld = LaunchDescription(args)
    ld.add_action(detector)
    return ld
