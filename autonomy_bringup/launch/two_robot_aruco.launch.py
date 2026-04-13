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

Obstacle-aware ArUco following pipeline for Robot 1
----------------------------------------------------
  When ArUco is visible (tracker_status = "measured"):
    aruco_follower → /a300_00000/aruco_follower/cmd_vel
    twist_mux (teleop slot, high priority) → /a300_00000/cmd_vel

  When ArUco is out of view (tracker_status = "predicted" / "lost"):
    aruco_goal_manager freezes last odom pose → transforms to map frame
                       → publishes /a300_00000/aruco_navigation_goal
    trajectory_planner_pkg/planner_node plans obstacle-aware path on
                       Robot 1's live SLAM map → /a300_00000/aruco_planned_path
    simple_motion_pkg/path_follower follows planned path
                       → /a300_00000/aruco_autonomous/cmd_vel
    aruco_follower goes completely silent (no cmd_vel output)
    twist_mux (teleop slot times out, autonomous slot wins) → /a300_00000/cmd_vel

  When ArUco is reacquired:
    aruco_goal_manager → nav_mode = "visual"
    aruco_follower resumes publishing → twist_mux teleop slot wins immediately

  Navigation timeout (nav_timeout_s = 30 s) is decoupled from tracker
  prediction timeout (prediction_timeout_s = 10 s) so the robot keeps
  navigating long after the KF stops predicting.

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
  launch_slam                        — launch SLAM for Robot 1 (default true)
  launch_aruco_detector              — (default true)
  launch_aruco_follower              — launches tracker + follower + nav stack (default true)
  launch_rviz                        — launch RViz ArUco debug view (default true)
  follower_desired_standoff_m        — (default 1.5)
  nav_timeout_s                      — seconds before giving up after ArUco lost (default 30.0)

Expected timeline
-----------------
  t= 0 s   Gazebo starts, Robot 1 generators start
  t= 8 s   Robot 2 platform-service, sensors-service, and Gazebo spawn all fire
  t=~10s   Robot 1 generators finish → Robot 1 spawned in Gazebo
  t=12 s   SLAM for Robot 1 + ArUco detector on Robot 1 camera
  t=13 s   ArUco Kalman tracker + camera optical frame TF
  t=15 s   ArUco goal manager (SLAM map→odom TF warm-up window)
  t=16 s   ArUco follower + planner + path_follower + twist_mux + RViz
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
        DeclareLaunchArgument('launch_slam', default_value='true',
                              choices=['true', 'false'],
                              description='Launch SLAM for Robot 1 (provides live map for '
                                          'obstacle-aware planning). Set false if already running.'),
        DeclareLaunchArgument('launch_aruco_detector', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_aruco_follower', default_value='true',
                              choices=['true', 'false'],
                              description='Launch full ArUco following stack '
                                          '(tracker + follower + goal_manager + planner + path_follower + mux)'),
        DeclareLaunchArgument('detector_image_topic',
                              default_value='/a300_00000/sensors/camera_0/color/image'),
        DeclareLaunchArgument('detector_camera_info_topic',
                              default_value='/a300_00000/sensors/camera_0/color/camera_info'),
        DeclareLaunchArgument('detector_marker_length_m', default_value='0.4'),
        DeclareLaunchArgument('follower_desired_standoff_m', default_value='1.5'),
        DeclareLaunchArgument('nav_timeout_s', default_value='30.0',
                              description='Seconds to navigate toward frozen goal after ArUco lost'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Launch RViz with ArUco debug view'),
    ]

    # -------------------------------------------------------- Package paths
    pkg_autonomy_bringup = FindPackageShare('autonomy_bringup')
    pkg_clearpath_gz     = FindPackageShare('clearpath_gz')
    planner_share        = FindPackageShare('trajectory_planner_pkg')
    motion_share         = FindPackageShare('simple_motion_pkg')

    gz_sim_launch        = PathJoinSubstitution([pkg_clearpath_gz,  'launch', 'gz_sim.launch.py'])
    robot_spawn_launch   = PathJoinSubstitution([pkg_clearpath_gz,  'launch', 'robot_spawn.launch.py'])
    planner_params       = PathJoinSubstitution([planner_share, 'config', 'planner_params.yaml'])
    motion_params        = PathJoinSubstitution([motion_share,  'config', 'motion_params.yaml'])
    rviz_config          = PathJoinSubstitution([pkg_autonomy_bringup, 'config', 'aruco_debug.rviz'])

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

    # ------------------------------------------------- SLAM for Robot 1
    # setup_path=CLEARPATH_PATH → reads robot.yaml (namespace: a300_00000)
    # → subscribes to /a300_00000/sensors/lidar2d_0/scan
    # → publishes /a300_00000/map and TF (map→odom→base_link) on /a300_00000/tf
    # Required for obstacle-aware path planning on Robot 1.
    slam_robot1 = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=12s: Starting SLAM for Robot 1 (a300_00000) …'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('clearpath_nav2_demos'),
                        'launch', 'slam.launch.py',
                    ])
                ]),
                launch_arguments=[
                    ('use_sim_time', LaunchConfiguration('use_sim_time')),
                    ('setup_path',   CLEARPATH_PATH),
                ],
                condition=IfCondition(LaunchConfiguration('launch_slam')),
            ),
        ]
    )

    # ---------------------------------------------- ArUco detector
    aruco_detector = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=12s: Starting ArUco detector on Robot 1 camera …'),
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

    # ------------------------------------------------ ArUco Kalman tracker
    # Linear KF in odom frame: smooths raw detector, publishes:
    #   tracked_pose          — smoothed camera-frame pose (consumed by aruco_follower)
    #   status                — "measured" / "predicted" / "lost"
    #   tracked_pose_odom     — odom-frame PoseStamped (consumed by aruco_goal_manager)
    #   target_marker         — RViz sphere (red=live, orange=KF coasting)
    #   predicted_path_odom   — RViz kinematic rollout (debug only)
    #
    # prediction_timeout_s = 10 s (longer than default 4 s) because the nav stack
    # will handle recovery — the tracker can coast longer before declaring "lost".
    aruco_tracker = TimerAction(
        period=13.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=13s: Starting ArUco Kalman tracker + camera optical TF …'),
            # Static TF: camera_0_link → camera_0_color_optical_frame
            # The Clearpath D435 URDF skips optical frames (use_nominal_extrinsics=false).
            # Without this the tracker TF lookup from camera_0_color_optical_frame
            # to odom always fails → tracker never initialises → follower stays stopped.
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
                    'use_sim_time':         LaunchConfiguration('use_sim_time'),
                    'input_topic':          '/a300_00000/aruco_detector/target_pose',
                    'odom_frame':           'odom',
                    'fresh_threshold_s':    2.0,
                    # Longer timeout: nav stack handles recovery, tracker can coast further
                    'prediction_timeout_s': 10.0,
                    'prediction_horizon_s': 1.5,
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

    # ------------------------------------------- ArUco goal manager
    # State machine: VISUAL_FOLLOW / PLANNED_NAV / NAV_STALE.
    # Converts tracker's odom pose to map-frame goal for the planner.
    # Publishes nav_mode ("visual" / "planned" / "idle") for aruco_follower arbitration.
    # Started 3 s after SLAM to allow the initial map→odom TF to arrive.
    aruco_goal_manager = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=15s: Starting ArUco goal manager …'),
            Node(
                package='autonomy_bringup',
                executable='aruco_goal_manager',
                name='aruco_goal_manager',
                namespace='a300_00000',
                output='screen',
                parameters=[{
                    'use_sim_time':            LaunchConfiguration('use_sim_time'),
                    'tracker_status_topic':    '/a300_00000/aruco_tracker/status',
                    'tracked_pose_odom_topic': '/a300_00000/aruco_tracker/tracked_pose_odom',
                    'navigation_goal_topic':   '/a300_00000/aruco_navigation_goal',
                    'nav_mode_topic':          '/a300_00000/aruco_goal_manager/nav_mode',
                    'goal_marker_topic':       '/a300_00000/aruco_goal_manager/goal_marker',
                    'map_frame':               'map',
                    'odom_frame':              'odom',
                    'nav_timeout_s':           LaunchConfiguration('nav_timeout_s'),
                    'goal_replan_s':           10.0,
                    'update_rate_hz':          5.0,
                }],
                # Needs Robot 1's TF to resolve odom→map (from SLAM)
                remappings=[
                    ('/tf',        '/a300_00000/tf'),
                    ('/tf_static', '/a300_00000/tf_static'),
                ],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),
        ]
    )

    # ----------------------------------------- Full following stack at t=16s
    # Delay gives SLAM 4 s after start (t=12s) to publish the first map→odom
    # transform before the planner needs it.
    aruco_following_stack = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=16s: Starting ArUco following stack …'),

            # ── ArUco visual follower ─────────────────────────────────────────
            # Publishes to /a300_00000/aruco_follower/cmd_vel (twist_mux teleop slot).
            # Goes completely silent when nav_mode = "planned" / "idle" so
            # the mux's teleop-timeout fires and path_follower takes the bus.
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
                    'nav_mode_topic':                   '/a300_00000/aruco_goal_manager/nav_mode',
                    # Publish to the twist_mux teleop slot (NOT directly to /cmd_vel)
                    'cmd_vel_topic':                    '/a300_00000/aruco_follower/cmd_vel',
                    'prediction_horizon_s':             1.5,
                    'desired_standoff_m':               LaunchConfiguration('follower_desired_standoff_m'),
                    # Hard collision guard — stop when marker depth < this value
                    'min_approach_dist_m':              1.2,
                    'target_timeout_s':                 2.0,
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

            # ── Obstacle-aware trajectory planner for Robot 1 ─────────────────
            # Subscribes: /a300_00000/aruco_navigation_goal (from goal_manager)
            #             /a300_00000/map (from SLAM)
            # Publishes:  /a300_00000/aruco_planned_path
            # TF lookup:  map → base_link  (via /a300_00000/tf remap)
            Node(
                package='trajectory_planner_pkg',
                executable='planner_node',
                name='aruco_planner',
                output='screen',
                parameters=[
                    planner_params,
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'map_topic':    '/a300_00000/map',
                        'goal_topic':   '/a300_00000/aruco_navigation_goal',
                        'path_topic':   '/a300_00000/aruco_planned_path',
                        # base_frame stays "base_link" — Clearpath frames are non-namespaced;
                        # TF remap below provides the correct Robot 1 tree.
                        # global_frame stays "map" — SLAM publishes map→odom on /a300_00000/tf.
                        # A* is fast enough for the short paths in ArUco recovery
                    # (robot is never far from the predicted goal).
                    # Trade optimality for sub-100ms planning latency.
                    'planner_type': 'astar',
                    'compare_mode': False,
                    },
                ],
                remappings=[
                    ('/tf',        '/a300_00000/tf'),
                    ('/tf_static', '/a300_00000/tf_static'),
                ],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),

            # ── Path follower for Robot 1 ─────────────────────────────────────
            # Subscribes: /a300_00000/aruco_planned_path (from planner above)
            #             /a300_00000/sensors/lidar2d_0/scan (live obstacle detection)
            # Publishes:  /a300_00000/aruco_autonomous/cmd_vel (twist_mux autonomous slot)
            # TF lookup:  map → base_link  (via /a300_00000/tf remap)
            Node(
                package='simple_motion_pkg',
                executable='path_follower',
                name='aruco_path_follower',
                output='screen',
                parameters=[
                    motion_params,
                    {
                        'use_sim_time':             LaunchConfiguration('use_sim_time'),
                        'path_topic':               '/a300_00000/aruco_planned_path',
                        'scan_topic':               '/a300_00000/sensors/lidar2d_0/scan',
                        'autonomous_cmd_vel_topic': '/a300_00000/aruco_autonomous/cmd_vel',
                        # Stop planned-nav when within desired_standoff of the frozen
                        # goal (= leader's last position).  Visual servo then takes over
                        # at the right distance and fine-tunes.  Must match
                        # follower_desired_standoff_m so planned nav never drives closer
                        # than the standoff the visual servo is configured to hold.
                        'goal_tolerance':           1.5,
                        # base_frame stays "base_link" — non-namespaced on Clearpath
                        # global_frame stays "map"
                    },
                ],
                remappings=[
                    ('/tf',        '/a300_00000/tf'),
                    ('/tf_static', '/a300_00000/tf_static'),
                ],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),

            # ── Twist multiplexer for Robot 1 ─────────────────────────────────
            # Priority (highest to lowest):
            #   teleop   slot → /a300_00000/aruco_follower/cmd_vel  (aruco visual servo)
            #   autonomous slot → /a300_00000/aruco_autonomous/cmd_vel  (planned nav)
            # Output: /a300_00000/cmd_vel → platform controller
            #
            # When aruco_follower goes silent (nav_mode = "planned"), the 1.5 s
            # teleop timeout fires and autonomous slot wins automatically.
            # When ArUco is reacquired aruco_follower resumes immediately and
            # the teleop slot re-asserts (no explicit locking needed).
            Node(
                package='simple_motion_pkg',
                executable='twist_mux',
                name='aruco_twist_mux',
                output='screen',
                parameters=[
                    motion_params,
                    {
                        'use_sim_time':         LaunchConfiguration('use_sim_time'),
                        'mux_autonomous_topic': '/a300_00000/aruco_autonomous/cmd_vel',
                        'mux_teleop_topic':     '/a300_00000/aruco_follower/cmd_vel',
                        'mux_output_topic':     '/a300_00000/cmd_vel',
                        # 1.5 s silence → fall through to planned navigation
                        'mux_teleop_timeout_s': 1.5,
                        # base_frame stays "base_link"
                    },
                ],
                condition=IfCondition(LaunchConfiguration('launch_aruco_follower')),
            ),
        ]
    )

    # --------------------------------------------------------- RViz (optional)
    # Remapped to /a300_00000/tf — fixed frame "map" resolves Robot 1's SLAM tree.
    # Shows: Robot 1 model, LiDAR, SLAM map, ArUco target sphere (red/orange),
    #        KF path (magenta), obstacle-aware planned path (green), nav goal (cylinder).
    rviz = TimerAction(
        period=17.0,
        actions=[
            LogInfo(msg='[two_robot_aruco] t=17s: Starting RViz ArUco debug view …'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                remappings=[
                    ('/tf',        '/a300_00000/tf'),
                    ('/tf_static', '/a300_00000/tf_static'),
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
                output='screen',
            ),
        ]
    )

    # --------------------------------------------------------- Assemble
    ld = LaunchDescription(args)
    ld.add_action(LogInfo(msg=''))
    ld.add_action(LogInfo(msg='=== Two-Robot ArUco Following Demo ==='))
    ld.add_action(LogInfo(msg='  Robot 2 (a300_00001) = LEADER   — ArUco marker on rear'))
    ld.add_action(LogInfo(msg='  Robot 1 (a300_00000) = FOLLOWER — detects marker, plans obstacle-aware path'))
    ld.add_action(LogInfo(msg='  Drive Robot 2 with teleop after ~16 s.'))
    ld.add_action(LogInfo(msg=''))

    ld.add_action(LogInfo(msg='[two_robot_aruco] Starting Gazebo …'))
    ld.add_action(gz_sim)

    # Robot 1 — generators start in parallel with Gazebo (standard Clearpath)
    ld.add_action(LogInfo(msg='[two_robot_aruco] Starting Robot 1 generators (a300_00000) …'))
    ld.add_action(robot1_spawn)

    # Robot 2 — everything deferred to t=8s so Gazebo is ready
    ld.add_action(LogInfo(msg='[two_robot_aruco] Robot 2 (a300_00001) will start at t=8s …'))
    ld.add_action(robot2_launch)

    ld.add_action(slam_robot1)
    ld.add_action(aruco_detector)
    ld.add_action(aruco_tracker)
    ld.add_action(aruco_goal_manager)
    ld.add_action(aruco_following_stack)
    ld.add_action(rviz)

    return ld
