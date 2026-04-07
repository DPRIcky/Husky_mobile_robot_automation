#!/usr/bin/env python3
"""
Working multi-robot launch file that properly sequences robot and ArUco marker spawning.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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

    # === SEQUENTIAL SPAWNING APPROACH ===
    # Each step waits for the previous to complete before starting

    # Step 1: Start Gazebo
    start_gazebo = LogInfo(msg='=== STEP 1: Starting Gazebo simulation ===')

    # Step 2: Wait a bit for Gazebo to fully initialize, then spawn robot 1
    wait_for_gazebo = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='=== STEP 2: Gazebo ready, spawning robot 1 ==='),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_spawn_launch]),
                launch_arguments=[
                    ('use_sim_time', LaunchConfiguration('use_sim_time')),
                    ('setup_path', '/home/prajjwal/clearpath'),
                    ('world', LaunchConfiguration('world')),
                    ('rviz', 'false'),
                    ('x', LaunchConfiguration('robot1_x')),
                    ('y', LaunchConfiguration('robot1_y')),
                    ('z', LaunchConfiguration('robot1_z')),
                    ('yaw', LaunchConfiguration('robot1_yaw')),
                    ('namespace', LaunchConfiguration('robot1_namespace'))
                ]
            )
        ]
    )

    # Step 3: Wait a bit more, then spawn robot 2
    wait_for_robot1 = TimerAction(
        period=6.0,  # 3s for Gazebo + 3s for robot 1 to settle
        actions=[
            LogInfo(msg='=== STEP 3: Spawning robot 2 ==='),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_spawn_launch]),
                launch_arguments=[
                    ('use_sim_time', LaunchConfiguration('use_sim_time')),
                    ('setup_path', '/home/prajjwal/clearpath'),
                    ('world', LaunchConfiguration('world')),
                    ('rviz', 'false'),
                    ('x', LaunchConfiguration('robot2_x')),
                    ('y', LaunchConfiguration('robot2_y')),
                    ('z', LaunchConfiguration('robot2_z')),
                    ('yaw', LaunchConfiguration('robot2_yaw')),
                    ('namespace', LaunchConfiguration('robot2_namespace'))
                ]
            )
        ]
    )

    # Step 4: Wait for both robots to be ready, then spawn ArUco markers
    wait_for_robots = TimerAction(
        period=9.0,  # 3s for Gazebo + 3s for robot 1 + 3s for robot 2
        actions=[
            LogInfo(msg='=== STEP 4: Both robots ready, spawning ArUco markers ==='),
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

    # Step 5: Optional RViz
    rviz_step = TimerAction(
        period=12.0,  # Start RViz last
        actions=[
            LogInfo(msg='=== STEP 5: Starting RViz (optional) ==='),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    pkg_clearpath_gz, 'launch', 'view_robot.launch.py'
                ]),
                launch_arguments=[
                    ('namespace', LaunchConfiguration('robot1_namespace')),  # Show robot 1 by default
                    ('use_sim_time', LaunchConfiguration('use_sim_time'))
                ],
                condition=IfCondition(LaunchConfiguration('rviz'))
            )
        ]
    )

    # Create launch description and add all actions in sequence
    ld = LaunchDescription(declared_arguments)
    ld.add_action(start_gazebo)
    ld.add_action(wait_for_gazebo)
    ld.add_action(wait_for_robot1)
    ld.add_action(wait_for_robots)
    ld.add_action(rviz_step)

    return ld