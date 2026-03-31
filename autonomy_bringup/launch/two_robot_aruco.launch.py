#!/usr/bin/env python3
"""
Canonical launch file: 2 × Clearpath A300 robots + ArUco marker on Robot 2's rear.

Robot 1  namespace=a300_00000  pose=(0, 0, 0.15)  — no marker
Robot 2  namespace=a300_00001  pose=(5, 0, 0.15)  — ArUco marker on rear bumper

Design notes
------------
* Robot 1 uses the standard Clearpath robot_spawn.launch.py (generate=true).
  Generators run first (~10 s), then platform-service + sensors-service + Gazebo
  spawn all fire together inside group_action_spawn_robot via event handlers.

* Robot 2 bypasses robot_spawn.launch.py to avoid two Clearpath bugs:
    1. TimerAction + RegisterEventHandler scoping ("launch configuration
       'generate' does not exist")
    2. generate=false is broken: Python bool('false') is True, so the direct
       branch never executes.

  Instead, ALL of Robot 2's pieces (platform-service, sensors-service, and the
  ros_gz_sim create spawn) start together inside a single TimerAction at t=8 s.
  This mirrors Robot 1 exactly: create subscribes to robot_description before
  robot_state_publisher publishes it, so there is no QoS / timing gap.

  The 8 s delay is enough for Gazebo to be ready; Robot 1's generators still
  need ~10 s to finish so Robot 1 and Robot 2 spawn at about the same time.

Usage
-----
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch autonomy_bringup two_robot_aruco.launch.py

Optional args
-------------
  robot1_x / robot1_y / robot1_yaw  — Robot 1 start pose (default 0 0 0)
  robot2_x / robot2_y / robot2_yaw  — Robot 2 start pose (default 5 0 0)
  world                              — Gazebo world name  (default warehouse)
  use_sim_time                       — true/false         (default true)

Expected timeline
-----------------
  t=0 s   Gazebo starts, Robot 1 generators start
  t=8 s   Robot 2 platform-service, sensors-service, and Gazebo spawn all fire
  t=~10s  Robot 1 generators finish → Robot 1 spawned in Gazebo
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CLEARPATH_PATH = '/home/prajjwal/clearpath'
ROBOT2_PATH    = '/home/prajjwal/clearpath/robot2'


def generate_launch_description():

    # ------------------------------------------------------------------ Args
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false'],
                              description='Use simulation time'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Gazebo world name'),
        DeclareLaunchArgument('robot1_x',   default_value='0.0'),
        DeclareLaunchArgument('robot1_y',   default_value='0.0'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot2_x',   default_value='5.0'),
        DeclareLaunchArgument('robot2_y',   default_value='0.0'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0'),
        DeclareLaunchArgument('launch_aruco_detector', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_aruco_follower', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('detector_image_topic',
                              default_value='/a300_00000/sensors/camera_0/color/image'),
        DeclareLaunchArgument('detector_camera_info_topic',
                              default_value='/a300_00000/sensors/camera_0/color/camera_info'),
        DeclareLaunchArgument('detector_marker_length_m', default_value='0.4'),
        DeclareLaunchArgument('follower_cmd_vel_topic', default_value='/a300_00000/cmd_vel'),
        DeclareLaunchArgument('follower_desired_standoff_m', default_value='1.5'),
    ]

    pkg_clearpath_gz   = FindPackageShare('clearpath_gz')
    gz_sim_launch      = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    # ------------------------------------------------------------ Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[('world', LaunchConfiguration('world'))]
    )

    # ------------------------------------------------ Robot 1 (no marker)
    # Included DIRECTLY — no TimerAction wrapper.
    # Clearpath generators run alongside Gazebo; they finish ~10 s later and
    # then robot_spawn fires automatically via the event-handler chain.
    robot1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path',   CLEARPATH_PATH),
            ('world',        LaunchConfiguration('world')),
            ('rviz',         'false'),
            ('x',            LaunchConfiguration('robot1_x')),
            ('y',            LaunchConfiguration('robot1_y')),
            ('z',            '0.15'),
            ('yaw',          LaunchConfiguration('robot1_yaw')),
        ]
    )

    # ------------------------------------------ Robot 2 (rear ArUco marker)
    # All three pieces start together at t=8 s so that ros_gz_sim create
    # subscribes to robot_description before robot_state_publisher publishes
    # (avoids the timing gap that breaks transient_local delivery).
    robot2_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ROBOT2_PATH + '/platform/launch/platform-service.launch.py']
        )
    )

    robot2_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ROBOT2_PATH + '/sensors/launch/sensors-service.launch.py']
        )
    )

    robot2_gz_create = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='a300_00001',
        arguments=[
            '-name',  'a300_00001/robot',
            '-topic', 'robot_description',
            '-x',     LaunchConfiguration('robot2_x'),
            '-y',     LaunchConfiguration('robot2_y'),
            '-z',     '0.15',
            '-Y',     LaunchConfiguration('robot2_yaw'),
        ],
        output='screen',
    )

    robot2_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=8s: Starting Robot 2 (a300_00001) …'),
            robot2_platform,
            robot2_sensors,
            robot2_gz_create,
        ]
    )

    aruco_detector = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] Starting ArUco detector on Robot 1 camera …'),
            Node(
                package='autonomy_bringup',
                executable='aruco_detector',
                name='aruco_detector',
                namespace='a300_00000',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'image_topic': LaunchConfiguration('detector_image_topic'),
                    'camera_info_topic': LaunchConfiguration('detector_camera_info_topic'),
                    'marker_length_m': LaunchConfiguration('detector_marker_length_m'),
                    'dictionary': 'DICT_4X4_50',
                    'target_ids': [0],
                }],
                condition=IfCondition(LaunchConfiguration('launch_aruco_detector')),
            ),
        ]
    )

    aruco_follower = TimerAction(
        period=13.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] Starting ArUco follower on a300_00000 …'),
            Node(
                package='autonomy_bringup',
                executable='aruco_follower',
                name='aruco_follower',
                namespace='a300_00000',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'target_pose_topic': '/a300_00000/aruco_detector/target_pose',
                    'cmd_vel_topic': LaunchConfiguration('follower_cmd_vel_topic'),
                    'desired_standoff_m': LaunchConfiguration('follower_desired_standoff_m'),
                }],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),
        ]
    )

    # --------------------------------------------------------- Assemble
    ld = LaunchDescription(args)
    ld.add_action(LogInfo(msg='[two_robot_aruco] Starting Gazebo …'))
    ld.add_action(gz_sim)

    # Robot 1 — generators start in parallel with Gazebo (standard Clearpath)
    ld.add_action(LogInfo(msg='[two_robot_aruco] Starting Robot 1 generators (a300_00000) …'))
    ld.add_action(robot1_spawn)

    # Robot 2 — everything deferred to t=8s so Gazebo is ready
    ld.add_action(LogInfo(msg='[two_robot_aruco] Robot 2 (a300_00001) will start at t=8s …'))
    ld.add_action(robot2_launch)
    ld.add_action(aruco_detector)
    ld.add_action(aruco_follower)

    return ld
