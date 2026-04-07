#!/usr/bin/env python3
"""
Launch file for multi-robot simulation with ArUco markers.
Spawns two robots in the warehouse world with ArUco markers for tracking.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Simulation arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation time',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            description='Gazebo world name',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start RViz',
        )
    )

    # Robot 1 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot1_namespace',
            default_value='a300_00000',
            description='Namespace for robot 1',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot1_x',
            default_value='0.0',
            description='X position of robot 1',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot1_y',
            default_value='0.0',
            description='Y position of robot 1',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot1_z',
            default_value='0.15',
            description='Z position of robot 1',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot1_yaw',
            default_value='0.0',
            description='Yaw orientation of robot 1',
        )
    )

    # Robot 2 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot2_namespace',
            default_value='a300_00001',
            description='Namespace for robot 2',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot2_x',
            default_value='5.0',
            description='X position of robot 2',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot2_y',
            default_value='0.0',
            description='Y position of robot 2',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot2_z',
            default_value='0.15',
            description='Z position of robot 2',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot2_yaw',
            default_value='3.14159',  # 180 degrees - facing opposite direction
            description='Yaw orientation of robot 2',
        )
    )

    # ArUco marker arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_aruco_markers',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn ArUco markers for tracking',
        )
    )

    # Get package directories
    pkg_clearpath_gz = FindPackageShare('clearpath_gz')
    pkg_autonomy_bringup = FindPackageShare('autonomy_bringup')

    # Paths
    gz_sim_launch = PathJoinSubstitution([
        pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'
    ])

    robot_spawn_launch = PathJoinSubstitution([
        pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'
    ])

    # Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # Robot 1 spawn
    robot1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', '/home/prajjwal/clearpath'),
            ('world', LaunchConfiguration('world')),
            ('rviz', 'false'),  # We'll handle RViz separately
            ('x', LaunchConfiguration('robot1_x')),
            ('y', LaunchConfiguration('robot1_y')),
            ('z', LaunchConfiguration('robot1_z')),
            ('yaw', LaunchConfiguration('robot1_yaw')),
            # Override the namespace from robot.yaml
            ('namespace', LaunchConfiguration('robot1_namespace'))
        ]
    )

    # Robot 2 spawn
    robot2_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', '/home/prajjwal/clearpath'),
            ('world', LaunchConfiguration('world')),
            ('rviz', 'false'),  # We'll handle RViz separately
            ('x', LaunchConfiguration('robot2_x')),
            ('y', LaunchConfiguration('robot2_y')),
            ('z', LaunchConfiguration('robot2_z')),
            ('yaw', LaunchConfiguration('robot2_yaw')),
            # Override the namespace from robot.yaml
            ('namespace', LaunchConfiguration('robot2_namespace'))
        ]
    )

    # ArUco marker spawner (delayed to ensure Gazebo is ready)
    aruco_spawner = TimerAction(
        period=8.0,  # Wait 8 seconds for Gazebo to be ready (increased from 5.0 for reliability)
        actions=[
            LogInfo(msg='Spawning ArUco markers for multi-robot tracking...'),
            ExecuteProcess(
                cmd=[
                    'python3',
                    os.path.join(
                        os.environ.get('HOME', '/home/prajjwal'),
                        'clearpath',
                        'install',
                        'autonomy_bringup',
                        'share',
                        'autonomy_bringup',
                        'scripts',
                        'spawn_multi_aruco_markers.py'
                    ),
                    '--world',
                    LaunchConfiguration('world')
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('spawn_aruco_markers'))
            )
        ]
    )

    # RViz (optional)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_clearpath_gz, 'launch', 'view_robot.launch.py'
        ]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('robot1_namespace')),  # Show robot 1 by default
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Create launch description
    ld = LaunchDescription(declared_arguments)

    # Add actions
    ld.add_action(gz_sim)
    ld.add_action(LogInfo(msg='Starting Gazebo simulation...'))

    # Create a delayed action group for spawning robots and markers
    delayed_spawn_actions = TimerAction(
        period=3.0,  # Wait for Gazebo to be fully ready
        actions=[
            LogInfo(msg='Gazebo started, spawning robots...'),
            LogInfo(msg='About to spawn robot 1...'),
            robot1_spawn,
            LogInfo(msg='About to spawn robot 2...'),
            robot2_spawn,
            LogInfo(msg='Finished spawning both robots'),
        ]
    )

    ld.add_action(delayed_spawn_actions)
    ld.add_action(aruco_spawner)
    ld.add_action(rviz)

    return ld