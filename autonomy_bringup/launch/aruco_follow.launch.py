#!/usr/bin/env python3
"""Launch the ArUco Kalman tracker + follower for a300_00000.

The tracker runs a linear Kalman filter (KF) in the odom frame (not camera
frame) so that Robot 1 ego-motion is automatically compensated.  The follower
subscribes to the tracker's output (tracked_pose + predicted_path) rather than
the raw detector, preventing stale-camera-frame orbital spin.

Follow modes (driven by tracker_status):
  measured  — live ArUco detections, full speed
  predicted — KF coasting through detection gap, reduced speed + relaxed deadbands
  lost      — KF prediction expired; follower enters timed recovery, then stops
"""

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
        DeclareLaunchArgument('target_timeout_s',   default_value='2.0'),
        DeclareLaunchArgument('odom_frame',         default_value='odom'),
    ]

    # Static TF: camera_0_link → camera_0_color_optical_frame
    # The Clearpath D435 URDF does not generate optical-frame links by default
    # (use_nominal_extrinsics=false), so this transform is missing from the tree.
    # Rotation rpy=(-π/2, 0, -π/2) → quat (x=-0.5, y=0.5, z=-0.5, w=0.5),
    # matching the standard RealSense d435 color_optical_joint in
    # clearpath_sensors_description/urdf/intel/d435.urdf.xacro.
    # Published to /a300_00000/tf_static so the tracker's TF buffer sees it.
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_0_optical_tf',
        namespace='a300_00000',
        output='screen',
        arguments=[
            '--frame-id', 'camera_0_link',
            '--child-frame-id', 'camera_0_color_optical_frame',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
        ],
        remappings=[('/tf_static', '/a300_00000/tf_static')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    tracker = Node(
        package='autonomy_bringup',
        executable='aruco_tracker',
        name='aruco_tracker',
        namespace='a300_00000',
        output='screen',
        parameters=[{
            'use_sim_time':         LaunchConfiguration('use_sim_time'),
            'input_topic':          LaunchConfiguration('target_pose_topic'),
            'odom_frame':           LaunchConfiguration('odom_frame'),
            'fresh_threshold_s':    2.0,
            'prediction_timeout_s': 4.0,
            'prediction_horizon_s': 1.5,
        }],
        # tf2_ros uses absolute topic names /tf and /tf_static regardless of
        # node namespace.  Remap to the namespaced topics where Robot 1's TF
        # tree is actually published.
        remappings=[
            ('/tf',        '/a300_00000/tf'),
            ('/tf_static', '/a300_00000/tf_static'),
        ],
    )

    follower = Node(
        package='autonomy_bringup',
        executable='aruco_follower',
        name='aruco_follower',
        namespace='a300_00000',
        output='screen',
        parameters=[{
            'use_sim_time':                    LaunchConfiguration('use_sim_time'),
            'target_pose_topic':               '/a300_00000/aruco_tracker/tracked_pose',
            'tracker_status_topic':            '/a300_00000/aruco_tracker/status',
            'predicted_path_topic':            '/a300_00000/aruco_tracker/predicted_path',
            'prediction_horizon_s':            1.5,
            'cmd_vel_topic':                   LaunchConfiguration('cmd_vel_topic'),
            'desired_standoff_m':              LaunchConfiguration('desired_standoff_m'),
            'target_timeout_s':                LaunchConfiguration('target_timeout_s'),
            # Recovery: 3 s window after tracker goes 'lost'; creep forward + hold heading
            # Gains tuned for ~1 Hz Gazebo camera: low angular_kp avoids
            # overshoot/oscillation; wide deadband absorbs detection jitter;
            # wide align_before_drive keeps forward motion during correction.
            'angular_kp':                      1.0,
            'max_angular_speed':               0.5,
            'lateral_deadband_m':              0.10,
            'align_before_drive_m':            0.60,
            'max_linear_speed':                0.35,
            'lost_target_recovery_s':          3.0,
            'recovery_linear_speed':           0.12,
            'recovery_angular_speed':          0.20,
            # KF-predicted mode: scale back speed and relax deadbands
            'predicted_speed_scale':           0.60,
            'predicted_lateral_deadband_scale': 2.0,
        }],
    )

    ld = LaunchDescription(args)
    ld.add_action(camera_optical_tf)
    ld.add_action(tracker)
    ld.add_action(follower)
    return ld
