"""
Bringup launch: starts the trajectory planner, path follower, twist_mux, and RViz.

Prerequisites (run in separate terminals BEFORE this launch):
  1. Gazebo:     ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
  2. SLAM/Map:   ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

This launch file starts:
  - trajectory_planner (A* by default)
  - path_follower      (Stanley/PID/PurePursuit/LQR/MPC controller toward waypoints)
  - twist_mux          (arbitrates between autonomous and teleop cmd_vel)
  - rviz2              (minimal config with map, path, TF, goal tool)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_share = get_package_share_directory('trajectory_planner_pkg')
    motion_share = get_package_share_directory('simple_motion_pkg')
    bringup_share = get_package_share_directory('autonomy_bringup')

    planner_params = os.path.join(planner_share, 'config', 'planner_params.yaml')
    motion_params = os.path.join(motion_share, 'config', 'motion_params.yaml')
    rviz_config = os.path.join(bringup_share, 'config', 'autonomy.rviz')

    return LaunchDescription([
        # ---- Arguments ----
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('planner_params', default_value=planner_params),
        DeclareLaunchArgument('motion_params', default_value=motion_params),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('launch_motion', default_value='true'),

        LogInfo(msg='=== Autonomy Bringup: starting planner + motion + RViz ==='),

        # ---- Trajectory planner ----
        Node(
            package='trajectory_planner_pkg',
            executable='planner_node',
            name='trajectory_planner',
            output='screen',
            parameters=[
                LaunchConfiguration('planner_params'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('/tf', '/a300_00000/tf'),
                ('/tf_static', '/a300_00000/tf_static'),
            ],
        ),

        # ---- Path follower (optional) ----
        Node(
            package='simple_motion_pkg',
            executable='path_follower',
            name='path_follower',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_motion')),
            parameters=[
                LaunchConfiguration('motion_params'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('/tf', '/a300_00000/tf'),
                ('/tf_static', '/a300_00000/tf_static'),
            ],
        ),

        # ---- Twist multiplexer ----
        Node(
            package='simple_motion_pkg',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_motion')),
            parameters=[
                LaunchConfiguration('motion_params'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ---- RViz (optional) ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', '/a300_00000/tf'),
                ('/tf_static', '/a300_00000/tf_static'),
            ],
        ),
    ])
