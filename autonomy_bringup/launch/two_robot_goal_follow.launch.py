#!/usr/bin/env python3
"""
Integrated two-robot demo: RViz goal-pose autonomy + ArUco following.

Role assignment
---------------
  Robot 2  a300_00001  LEADER    — receives /goal_pose, runs custom planner +
                                   path follower + obstacle avoidance, drives
  Robot 1  a300_00000  FOLLOWER  — Robot 1 camera detects Robot 2's rear ArUco
                                   marker; ArUco follower drives Robot 1 to
                                   trail Robot 2 at a standoff distance

TF frame notes
--------------
  Clearpath A300 uses NON-NAMESPACED TF frame names regardless of the robot
  namespace.  Both robots publish:
    base_link, odom, chassis_link, lidar2d_0_link, camera_0_link, …
  BUT each robot publishes its TF to its own namespaced topic:
    Robot 1 → /a300_00000/tf     Robot 2 → /a300_00001/tf
  Each planning node is therefore remapped to the correct /tf topic so it
  operates in the right robot's frame without frame-name conflicts.

  SLAM for Robot 2 (run via this launch) publishes:
    map  →  odom  →  base_link  on  /a300_00001/tf

  Because the planner and path_follower are remapped to /a300_00001/tf they
  correctly look up  map → base_link  for Robot 2.
  base_frame: "base_link"  (default in yaml) — correct for both robots.

Physical layout
---------------
  Robot 2  rear bumper  → ArUco marker ID 0  (existing, in robot2/robot.urdf.xacro)
  Robot 1  front camera → detects Robot 2's marker when trailing behind it

Usage (single command)
----------------------
  ros2 launch autonomy_bringup two_robot_goal_follow.launch.py

Workflow
--------
  1. Wait ~20 s for both robots to spawn and SLAM to initialise.
  2. In RViz: click "2D Goal Pose" to drive Robot 2.
  3. Robot 1 follows Robot 2 automatically once the ArUco marker is visible.

Optional args
-------------
  robot1_x / robot1_y / robot1_yaw   Robot 1 start pose (default 3 0 3.14)
  robot2_x / robot2_y / robot2_yaw   Robot 2 start pose (default 0 0 0)
  world                               Gazebo world (default warehouse)
  use_sim_time                        (default true)
  launch_rviz                         (default true)
  launch_slam                         (default true) — set false if SLAM already running
  launch_aruco_follower               (default true)
  controller_type                     (default lqr)
  controller_compare_mode             (default false)
  launch_controller_plot              (default false)
  follower_desired_standoff_m         (default 1.5)
  follower_target_timeout_s           (default 2.0)

Timeline
--------
  t= 0 s   Gazebo starts; Robot 1 generators start (standard Clearpath)
  t= 8 s   Robot 2 spawns (direct, no generators — preserves ArUco URDF)
  t=10 s   Robot 1 generators finish → Robot 1 spawns
  t=12 s   SLAM for Robot 2 + ArUco detector (Robot 1 camera)
  t=16 s   ArUco follower + Autonomy stack (planner/path_follower/twist_mux) + RViz
           (4 s gap gives SLAM time to produce first map→odom transform)
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
                              choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='warehouse'),
        # Robot 1 (follower) starts in front of / near Robot 2 so Robot 2 can
        # navigate toward it first; Robot 1 then trails behind.
        DeclareLaunchArgument('robot1_x',   default_value='3.0'),
        DeclareLaunchArgument('robot1_y',   default_value='0.0'),
        DeclareLaunchArgument('robot1_yaw', default_value='3.14159'),
        DeclareLaunchArgument('robot2_x',   default_value='0.0'),
        DeclareLaunchArgument('robot2_y',   default_value='0.0'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_slam', default_value='true',
                              choices=['true', 'false'],
                              description='Launch SLAM for Robot 2 (leader). '
                                          'Set false if SLAM is already running.'),
        DeclareLaunchArgument('launch_aruco_follower', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controller_type', default_value='lqr'),
        DeclareLaunchArgument('controller_compare_mode', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_controller_plot', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('follower_desired_standoff_m', default_value='1.5'),
        DeclareLaunchArgument('follower_target_timeout_s',   default_value='2.0'),
    ]

    # -------------------------------------------------------- Package paths
    pkg_clearpath_gz   = FindPackageShare('clearpath_gz')
    gz_sim_launch      = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    bringup_share = FindPackageShare('autonomy_bringup')
    planner_share = FindPackageShare('trajectory_planner_pkg')
    motion_share  = FindPackageShare('simple_motion_pkg')

    planner_params = PathJoinSubstitution([planner_share, 'config', 'planner_params.yaml'])
    motion_params  = PathJoinSubstitution([motion_share,  'config', 'motion_params.yaml'])
    rviz_config    = PathJoinSubstitution([bringup_share, 'config', 'dual_robot.rviz'])

    # ------------------------------------------------------------ Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[('world', LaunchConfiguration('world'))]
    )

    # -------------------------------- Robot 1 (FOLLOWER — standard Clearpath spawn)
    # Generators run alongside Gazebo (~10 s); robot_spawn fires when done.
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

    # ------------------------------ Robot 2 (LEADER — has rear ArUco marker)
    # Bypasses robot_spawn to preserve the custom URDF with the ArUco marker
    # (same pattern as two_robot_aruco.launch.py).
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
            LogInfo(msg='[goal_follow] t=8s: Starting Robot 2 (a300_00001 — LEADER) …'),
            robot2_platform,
            robot2_sensors,
            robot2_gz_create,
        ]
    )

    # ----------------------------------------- Robot 1 TF bridge (for RViz)
    # Reads /a300_00000/tf, prefixes all frames with "robot1/", republishes
    # into /a300_00001/tf so that RViz (remapped to /a300_00001/tf) can see
    # both robots in the same window without frame-name collisions.
    robot1_tf_bridge = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[goal_follow] t=12s: Starting Robot 1 TF republisher …'),
            Node(
                package='autonomy_bringup',
                executable='robot1_tf_republisher',
                name='robot1_tf_republisher',
                output='screen',
                parameters=[{
                    # Must match the Gazebo spawn pose so map→robot1/odom
                    # places Robot 1 at the correct world position in RViz.
                    'robot1_initial_x':   LaunchConfiguration('robot1_x'),
                    'robot1_initial_y':   LaunchConfiguration('robot1_y'),
                    'robot1_initial_yaw': LaunchConfiguration('robot1_yaw'),
                }],
            ),
        ]
    )

    # ------------------------------------------------- SLAM for Robot 2 (leader)
    # setup_path=ROBOT2_PATH → reads robot2/robot.yaml (namespace a300_00001)
    # → subscribes to /a300_00001/sensors/lidar2d_0/scan
    # → publishes /a300_00001/map and TF (map→odom→base_link) on /a300_00001/tf
    slam = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[goal_follow] t=12s: Starting SLAM for Robot 2 (a300_00001) …'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('clearpath_nav2_demos'),
                        'launch', 'slam.launch.py',
                    ])
                ]),
                launch_arguments=[
                    ('use_sim_time', LaunchConfiguration('use_sim_time')),
                    ('setup_path',   ROBOT2_PATH),
                ],
                condition=IfCondition(LaunchConfiguration('launch_slam')),
            ),
        ]
    )

    # ---------------------------------- ArUco detector (Robot 1 camera → Robot 2 marker)
    aruco_detector = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[goal_follow] t=12s: Starting ArUco detector on Robot 1 camera …'),
            Node(
                package='autonomy_bringup',
                executable='aruco_detector',
                name='aruco_detector',
                namespace='a300_00000',
                output='screen',
                parameters=[{
                    'use_sim_time':      LaunchConfiguration('use_sim_time'),
                    'image_topic':       '/a300_00000/sensors/camera_0/color/image',
                    'camera_info_topic': '/a300_00000/sensors/camera_0/color/camera_info',
                    'marker_length_m':   0.4,
                    'dictionary':        'DICT_4X4_50',
                    'target_ids':        [0],
                }],
            ),
        ]
    )

    # ------------------------------- ArUco Kalman tracker (smooths + predicts pose)
    # Sits between the raw detector and the follower.  During gaps in detection it
    # continues to publish a forward-rolled predicted pose so the follower doesn't
    # stop unnecessarily.  Only stops publishing when status = "lost".
    aruco_tracker = TimerAction(
        period=13.0,
        actions=[
            LogInfo(msg='[goal_follow] t=13s: Starting ArUco Kalman tracker + camera optical TF …'),
            # Static TF: camera_0_link → camera_0_color_optical_frame
            # Required because the Clearpath D435 URDF skips optical frames by default.
            Node(
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
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),
            Node(
                package='autonomy_bringup',
                executable='aruco_tracker',
                name='aruco_tracker',
                namespace='a300_00000',
                output='screen',
                parameters=[{
                    'use_sim_time':          LaunchConfiguration('use_sim_time'),
                    'input_topic':           '/a300_00000/aruco_detector/target_pose',
                    'fresh_threshold_s':     2.0,
                    'prediction_timeout_s':  4.0,
                    'prediction_horizon_s':  1.5,
                }],
                # tf2_ros uses absolute /tf and /tf_static regardless of namespace.
                # Remap to the namespaced topics where Robot 1's TF tree lives.
                remappings=[
                    ('/tf',        '/a300_00000/tf'),
                    ('/tf_static', '/a300_00000/tf_static'),
                ],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),
        ]
    )

    # ------------------------------- ArUco follower (drives Robot 1 toward Robot 2)
    # Now consumes the tracker's smoothed/predicted pose instead of raw detector output.
    # During detection gaps the tracker keeps feeding poses, so the follower doesn't stop.
    # Recovery only triggers once the tracker declares "lost" and stops publishing.
    aruco_follower = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg='[goal_follow] t=16s: Starting ArUco follower for Robot 1 …'),
            Node(
                package='autonomy_bringup',
                executable='aruco_follower',
                name='aruco_follower',
                namespace='a300_00000',
                output='screen',
                parameters=[{
                    'use_sim_time':                     LaunchConfiguration('use_sim_time'),
                    'target_pose_topic':                '/a300_00000/aruco_tracker/tracked_pose',
                    'tracker_status_topic':             '/a300_00000/aruco_tracker/status',
                    'predicted_path_topic':             '/a300_00000/aruco_tracker/predicted_path',
                    'prediction_horizon_s':             1.5,
                    'cmd_vel_topic':                    '/a300_00000/cmd_vel',
                    'desired_standoff_m':               LaunchConfiguration('follower_desired_standoff_m'),
                    'target_timeout_s':                 LaunchConfiguration('follower_target_timeout_s'),
                    # Gains tuned for ~1 Hz Gazebo camera
                    'angular_kp':                       1.0,
                    'max_angular_speed':                0.5,
                    'lateral_deadband_m':               0.10,
                    'align_before_drive_m':             0.60,
                    'max_linear_speed':                 0.35,
                    # Recovery: creep forward + gentle scan, no spinning
                    'lost_target_recovery_s':           3.0,
                    'recovery_linear_speed':            0.12,
                    'recovery_angular_speed':           0.20,
                    # KF-predicted mode: scale back speed and relax deadbands
                    'predicted_speed_scale':            0.60,
                    'predicted_lateral_deadband_scale': 2.0,
                }],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),
        ]
    )

    # ------------------------- Autonomy stack for Robot 2 (planner + controller)
    #
    # IMPORTANT: Clearpath A300 TF frames are NON-NAMESPACED (base_link, odom, …)
    # regardless of robot namespace.  We DO NOT override base_frame — the default
    # "base_link" from planner_params.yaml / motion_params.yaml is correct.
    #
    # Isolation from Robot 1 is achieved by remapping /tf → /a300_00001/tf so
    # each node sees only Robot 2's TF tree.
    #
    # Topics overridden to Robot 2's namespace:
    #   map_topic, scan_topic, autonomous_cmd_vel_topic, mux_output_topic, etc.
    #
    # Delay to t=16s gives SLAM ~4 s after it starts (t=12s) to produce the
    # first  map→odom  transform before the planner needs it.
    autonomy_stack = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg='[goal_follow] t=16s: Starting autonomy stack for Robot 2 (LEADER) …'),

            # ── Trajectory planner ──────────────────────────────────────────
            # Subscribes: /a300_00001/map (SLAM), /goal_pose (RViz goal tool)
            # Publishes:  /planned_path
            # TF lookup:  map → base_link  (via /a300_00001/tf remap)
            Node(
                package='trajectory_planner_pkg',
                executable='planner_node',
                name='trajectory_planner',
                output='screen',
                parameters=[
                    planner_params,
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'map_topic':    '/a300_00001/map',
                        # base_frame stays "base_link" (default) — Clearpath frames
                        # are non-namespaced; TF remap below provides correct tree.
                        # goal_topic and path_topic stay at defaults (/goal_pose, /planned_path)
                    },
                ],
                remappings=[
                    # Use Robot 2's TF tree so  map → odom → base_link  resolves correctly
                    ('/tf',        '/a300_00001/tf'),
                    ('/tf_static', '/a300_00001/tf_static'),
                ],
            ),

            # ── Path follower ───────────────────────────────────────────────
            # Subscribes: /planned_path, /a300_00001/sensors/lidar2d_0/scan
            # Publishes:  /a300_00001/autonomous/cmd_vel
            # TF lookup:  map → base_link  (via /a300_00001/tf remap)
            Node(
                package='simple_motion_pkg',
                executable='path_follower',
                name='path_follower',
                output='screen',
                parameters=[
                    motion_params,
                    {
                        'use_sim_time':             LaunchConfiguration('use_sim_time'),
                        'controller_type':          LaunchConfiguration('controller_type'),
                        'controller_compare_mode':  LaunchConfiguration('controller_compare_mode'),
                        # Robot 2's LiDAR for obstacle detection
                        'scan_topic':               '/a300_00001/sensors/lidar2d_0/scan',
                        # Publish to a Robot-2-namespaced intermediate topic
                        # so it doesn't collide with any Robot 1 cmd_vel
                        'autonomous_cmd_vel_topic': '/a300_00001/autonomous/cmd_vel',
                        # base_frame stays "base_link" — see note above
                    },
                ],
                remappings=[
                    ('/tf',        '/a300_00001/tf'),
                    ('/tf_static', '/a300_00001/tf_static'),
                ],
            ),

            # ── Twist multiplexer ───────────────────────────────────────────
            # Subscribes: /a300_00001/autonomous/cmd_vel
            # Publishes:  /a300_00001/cmd_vel  → Robot 2 platform controller
            Node(
                package='simple_motion_pkg',
                executable='twist_mux',
                name='twist_mux',
                output='screen',
                parameters=[
                    motion_params,
                    {
                        'use_sim_time':         LaunchConfiguration('use_sim_time'),
                        'mux_autonomous_topic': '/a300_00001/autonomous/cmd_vel',
                        'mux_teleop_topic':     '/a300_00001/joy_teleop/cmd_vel',
                        'mux_output_topic':     '/a300_00001/cmd_vel',
                        # base_frame stays "base_link" — see note above
                    },
                ],
            ),
        ]
    )

    # ------------------------------------------------- Live compare plot
    compare_plot = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='autonomy_bringup',
                executable='plot_controller_compare',
                name='controller_compare_plotter',
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_controller_plot')),
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'active_controller': LaunchConfiguration('controller_type'),
                }],
            ),
        ]
    )

    # ----------------------------------------------------------------- RViz
    # Remapped to Robot 2's TF (leader) — has the map frame from SLAM.
    # Robot 1 frames arrive via robot1_tf_republisher with "robot1/" prefix
    # (robot1/base_link, robot1/odom, …) so they coexist in the same TF tree
    # without frame-name collisions.  The dual_robot.rviz config shows both.
    rviz = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[
                    ('/tf',        '/a300_00001/tf'),
                    ('/tf_static', '/a300_00001/tf_static'),
                ],
            ),
        ]
    )

    # -------------------------------------------------------------- Assemble
    ld = LaunchDescription(args)

    ld.add_action(LogInfo(msg=''))
    ld.add_action(LogInfo(msg='=== Two-Robot Goal-Follow Demo ==='))
    ld.add_action(LogInfo(msg='  Robot 2 (a300_00001) = LEADER  — use RViz "2D Goal Pose" to send it a destination'))
    ld.add_action(LogInfo(msg='  Robot 1 (a300_00000) = FOLLOWER — trails Robot 2 via ArUco detection'))
    ld.add_action(LogInfo(msg='  Wait ~20 s after launch before clicking a goal pose (SLAM warm-up).'))
    ld.add_action(LogInfo(msg=''))

    ld.add_action(gz_sim)

    ld.add_action(LogInfo(msg='[goal_follow] Robot 1 generators starting (a300_00000) …'))
    ld.add_action(robot1_spawn)

    ld.add_action(LogInfo(msg='[goal_follow] Robot 2 (a300_00001) will start at t=8s …'))
    ld.add_action(robot2_launch)

    ld.add_action(robot1_tf_bridge)
    ld.add_action(slam)
    ld.add_action(aruco_detector)
    ld.add_action(aruco_tracker)
    ld.add_action(aruco_follower)
    ld.add_action(autonomy_stack)
    ld.add_action(compare_plot)
    ld.add_action(rviz)

    return ld
