#!/usr/bin/env python3
"""
Navigation Launch File for Clearpath A300

Launches Nav2 stack with sensor fusion and waypoint navigation.
Integrates all 4 sensors: IMU, GPS, LiDAR, Depth Camera

Author: DPRicky
Date: 2026-03-05
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_gps = LaunchConfiguration('use_gps')
    map_file = LaunchConfiguration('map_file')
    
    # Paths
    nav_config_dir = '/home/prajjwal/clearpath/navigation/config'
    nav2_params_file = os.path.join(nav_config_dir, 'nav2_params.yaml')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='a300_00000',
        description='Top-level namespace'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_use_gps_cmd = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Whether to use GPS for global localization'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to map yaml file (if using pre-built map)'
    )
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'map': map_file,
            'use_composition': 'False',  # Set to True for better performance
            'use_respawn': 'False',
        }.items()
    )
    
    # GPS navsat transform node (optional)
    gps_localization_config = os.path.join(nav_config_dir, 'localization_gps.yaml')
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        namespace=namespace,
        output='screen',
        parameters=[gps_localization_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', 'sensors/imu_0/data'),
            ('gps/fix', 'sensors/gps_0/fix'),
            ('odometry/filtered', 'odometry/local'),
            ('odometry/gps', 'odometry/gps'),
        ],
        condition=IfCondition(use_gps)
    )
    
    # Map server (if map file provided)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        parameters=[
            configured_params,
            {'yaml_filename': map_file}
        ],
        condition=IfCondition(LaunchConfiguration('map_file'))
    )
    
    # Lifecycle manager for map server
    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']}
        ],
        condition=IfCondition(LaunchConfiguration('map_file'))
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_gps_cmd)
    ld.add_action(declare_map_file_cmd)
    
    # Add nodes
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(navsat_transform_node)
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)
    
    return ld
