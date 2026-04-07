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
  - optional live controller-comparison plot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planner_share = get_package_share_directory('trajectory_planner_pkg')
    motion_share = get_package_share_directory('simple_motion_pkg')
    bringup_share = get_package_share_directory('autonomy_bringup')

    planner_params = os.path.join(planner_share, 'config', 'planner_params.yaml')
    motion_params = os.path.join(motion_share, 'config', 'motion_params.yaml')
    rviz_config = os.path.join(bringup_share, 'config', 'autonomy.rviz')

    return LaunchDescription([
        # ---- Arguments ----
        DeclareLaunchArgument('use_sim_time',    default_value='true'),
        DeclareLaunchArgument('planner_params',  default_value=planner_params),
        DeclareLaunchArgument('motion_params',   default_value=motion_params),
        DeclareLaunchArgument('launch_rviz',     default_value='true'),
        DeclareLaunchArgument('launch_motion',   default_value='true'),
        DeclareLaunchArgument('launch_plot',     default_value='false',
            description='Launch real-time GT vs SLAM localisation plot'),
        DeclareLaunchArgument('controller_type', default_value='lqr',
            description='Active controller used by path_follower'),
        DeclareLaunchArgument('controller_compare_mode', default_value='false',
            description='Run all 5 controllers each tick and publish /controller_diagnostics'),
        DeclareLaunchArgument('launch_controller_plot', default_value='false',
            description='Launch the live controller-comparison plotter'),

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
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'controller_type': LaunchConfiguration('controller_type'),
                    'controller_compare_mode': LaunchConfiguration('controller_compare_mode'),
                },
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

        # ---- Live controller comparison plot (optional) ----
        Node(
            package='autonomy_bringup',
            executable='plot_controller_compare',
            name='controller_compare_plotter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_controller_plot')),
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'active_controller': LaunchConfiguration('controller_type'),
                }
            ],
        ),

        # ---- Ground-truth vs SLAM localisation plot (optional) ----
        Node(
            package='autonomy_bringup',
            executable='plot_localisation',
            name='localisation_plotter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_plot')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', '/a300_00000/tf'),
                ('/tf_static', '/a300_00000/tf_static'),
            ],
        ),
    ])
