"""Launch the path follower node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('simple_motion_pkg')
    default_params = os.path.join(pkg_share, 'config', 'motion_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='simple_motion_pkg',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('/tf', '/a300_00000/tf'),
                ('/tf_static', '/a300_00000/tf_static'),
            ],
        ),
    ])
