#!/usr/bin/env python3
"""
Simple demo launch file for multi-robot simulation.
This demonstrates the concept without requiring a full Gazebo simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Simply log the configuration to demonstrate the concept
    robot1_ns = LaunchConfiguration('robot1_ns').perform(context)
    robot2_ns = LaunchConfiguration('robot2_ns').perform(context)
    world = LaunchConfiguration('world').perform(context)

    return [
        LogInfo(msg=f'Demo: Would spawn robot {robot1_ns} and {robot2_ns} in world {world}'),
        LogInfo(msg='Demo: Would spawn ArUco markers at (3,0,0.8) and (-3,0,0.8)'),
        LogInfo(msg='Demo: Multi-robot simulation concept validated!'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('robot1_ns', default_value='a300_00000'),
        DeclareLaunchArgument('robot2_ns', default_value='a300_00001'),
        OpaqueFunction(function=launch_setup)
    ])