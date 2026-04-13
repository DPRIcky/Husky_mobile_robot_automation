# Multi-Robot Simulation Setup

Two Clearpath A300 robots in Gazebo: Robot 2 (leader) navigates autonomously while Robot 1 (follower) trails using ArUco marker detection with obstacle-aware recovery planning.

---

## Directory Layout

```
clearpath/
├── robot.yaml              # Robot 1 config  (namespace: a300_00000)
├── robot.urdf.xacro        # Robot 1 URDF    (no marker)
├── platform/launch/        # Robot 1 generated platform launch files
├── sensors/launch/         # Robot 1 generated sensor launch files
│
└── robot2/                 # Robot 2 setup (same workspace)
    ├── robot.yaml              # Robot 2 config  (namespace: a300_00001)
    ├── robot.urdf.xacro        # Robot 2 URDF    (ArUco marker on rear — do NOT overwrite)
    ├── platform/launch/        # Robot 2 generated platform launch files
    └── sensors/launch/         # Robot 2 generated sensor launch files
```

> **Important:** Robot 2's `robot.urdf.xacro` has a custom ArUco marker link embedded. If `robot2/robot.yaml` changes, re-run the four Clearpath generators manually and then re-add the marker before building. The canonical launch file uses `generate=false` for Robot 2 to prevent the generator from overwriting the custom URDF.

---

## Launch

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py
```

After ~16 seconds, drive Robot 2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=true -p frame_id:=a300_00001/base_link \
    -r cmd_vel:=/a300_00001/cmd_vel
```

Robot 1 follows automatically once the ArUco marker is in view.

---

## Startup Timeline

| Time | Event |
|------|-------|
| t=0 s | Gazebo starts; Robot 1 Clearpath generators begin |
| t=8 s | Robot 2 platform-service, sensors-service, and Gazebo spawn all start |
| t~10 s | Robot 1 generators finish → Robot 1 spawned in Gazebo |
| t=12 s | SLAM for Robot 1 (`/a300_00000/map`, `map→odom` TF) |
| t=12 s | ArUco detector on Robot 1 camera |
| t=13 s | ArUco Kalman tracker + camera optical frame static TF |
| t=15 s | ArUco goal manager (3 s after SLAM for TF warm-up) |
| t=16 s | Full following stack (visual follower + planner + path follower + twist mux) |
| t=17 s | RViz ArUco debug view |

---

## Full Following Pipeline

### ArUco Visual Following (marker visible)

```
Camera → ArUco Detector → ArUco Tracker (KF) → ArUco Follower
                                                     │
                                    /a300_00000/aruco_follower/cmd_vel
                                                     │
                                                 Twist Mux  ──► /a300_00000/cmd_vel
```

The visual follower uses a proportional controller on camera-frame lateral error (steering) and depth error (forward speed). It publishes to the twist_mux **teleop slot** which has highest priority.

### Obstacle-Aware Recovery (marker out of view)

```
ArUco Tracker (status="predicted"/"lost")
       │
       ▼
ArUco Goal Manager  ──── freezes last odom pose ────► odom→map TF ──► frozen goal (map frame)
       │                                                                      │
       │  nav_mode="planned"                                                  ▼
       │                                                         Trajectory Planner (A*)
       ▼                                                                      │
ArUco Follower goes completely silent                                         ▼
(no cmd_vel published)                                                  Path Follower
                                                                              │
                                               /a300_00000/aruco_autonomous/cmd_vel
                                                                              │
                                                                          Twist Mux
                                                              (1.5s teleop timeout fires)
                                                                              │
                                                                    /a300_00000/cmd_vel
```

### Reacquisition (marker visible again)

```
ArUco Tracker (status="measured" × 3 consecutive ticks)
       │
       ▼
ArUco Goal Manager  →  nav_mode="visual"
       │
       ▼
ArUco Follower resumes → Twist Mux teleop slot wins immediately
```

---

## State Machine (aruco_goal_manager)

| State | nav_mode | Condition to enter | Motion authority |
|-------|----------|--------------------|-----------------|
| `VISUAL_FOLLOW` | `"visual"` | Startup, or ArUco stably reacquired (3 consecutive "measured" ticks) | ArUco visual follower |
| `PLANNED_NAV` | `"planned"` | Tracker becomes "predicted" or "lost"; valid odom pose + map TF available | Path follower via A* |
| `NAV_STALE` | `"idle"` | `nav_timeout_s` elapsed without reacquisition | All motion stopped |

**Startup guard:** Goal manager refuses to enter `PLANNED_NAV` until at least one genuine `"measured"` detection has been received. This prevents the robot from attempting planned navigation before the visual follower has even started.

**Reacquisition hysteresis:** Three consecutive `"measured"` ticks (600 ms at 5 Hz update rate) required before `PLANNED_NAV → VISUAL_FOLLOW`. This avoids oscillation caused by the 1 Hz Gazebo camera where the tracker briefly dips to `"predicted"` between detection frames.

---

## Command Arbitration (twist_mux)

| Slot | Topic | Priority | Timeout |
|------|-------|----------|---------|
| Teleop | `/a300_00000/aruco_follower/cmd_vel` | High | 1.5 s |
| Autonomous | `/a300_00000/aruco_autonomous/cmd_vel` | Low | — |

When `nav_mode = "planned"`, the ArUco follower publishes nothing. After 1.5 s of silence the mux teleop slot times out and the autonomous slot (path follower) takes command. No explicit lock needed — priority is handled purely by timeout.

---

## Key Topics

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/a300_00000/aruco_detector/target_pose` | `PoseStamped` | aruco_detector | aruco_tracker |
| `/a300_00000/aruco_tracker/tracked_pose` | `PoseStamped` | aruco_tracker | aruco_follower |
| `/a300_00000/aruco_tracker/tracked_pose_odom` | `PoseStamped` | aruco_tracker | aruco_goal_manager |
| `/a300_00000/aruco_tracker/status` | `String` | aruco_tracker | aruco_follower, aruco_goal_manager |
| `/a300_00000/aruco_tracker/target_marker` | `Marker` | aruco_tracker | RViz |
| `/a300_00000/aruco_tracker/predicted_path_odom` | `Path` | aruco_tracker | RViz |
| `/a300_00000/aruco_goal_manager/nav_mode` | `String` | aruco_goal_manager | aruco_follower |
| `/a300_00000/aruco_navigation_goal` | `PoseStamped` | aruco_goal_manager | aruco_planner |
| `/a300_00000/aruco_goal_manager/goal_marker` | `Marker` | aruco_goal_manager | RViz |
| `/a300_00000/aruco_planned_path` | `Path` | aruco_planner | aruco_path_follower, RViz |
| `/a300_00000/aruco_follower/cmd_vel` | `TwistStamped` | aruco_follower | aruco_twist_mux (teleop slot) |
| `/a300_00000/aruco_autonomous/cmd_vel` | `TwistStamped` | aruco_path_follower | aruco_twist_mux (autonomous slot) |
| `/a300_00000/cmd_vel` | `TwistStamped` | aruco_twist_mux | platform_velocity_controller |
| `/a300_00000/map` | `OccupancyGrid` | slam_toolbox | aruco_planner |

---

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo clock |
| `world` | `warehouse` | Gazebo world name |
| `robot1_x/y/yaw` | `0 0 0` | Robot 1 start pose |
| `robot2_x/y/yaw` | `5 0 0` | Robot 2 start pose |
| `launch_slam` | `true` | SLAM for Robot 1; set `false` if running separately |
| `launch_aruco_detector` | `true` | ArUco OpenCV detector |
| `launch_aruco_follower` | `true` | Full following stack (tracker + follower + goal manager + planner + path follower + mux) |
| `launch_rviz` | `true` | RViz ArUco debug view |
| `follower_desired_standoff_m` | `1.5` | Visual servo standoff distance (m) |
| `nav_timeout_s` | `30.0` | Max time to pursue frozen goal after ArUco loss |
| `detector_marker_length_m` | `0.4` | ArUco marker side length (m) — must match simulation |

---

## Collision Protection

Two independent mechanisms prevent Robot 1 from colliding with Robot 2:

1. **Hard minimum approach distance** (`min_approach_dist_m = 1.2 m`): Visual servo stops unconditionally when the camera-frame marker depth falls below this value. Active regardless of standoff setting or controller state.

2. **Planned-nav goal tolerance** (`goal_tolerance = 1.5 m`): Path follower stops at the same radius as the visual servo standoff. Planned navigation never drives Robot 1 closer than intended.

Both parameters can be adjusted in the launch file.

---

## RViz Displays (aruco_debug.rviz)

Fixed frame: `map` (provided by SLAM via `/a300_00000/tf`).

| Display | Topic | Colour | Purpose |
|---------|-------|--------|---------|
| Robot Model | `/a300_00000/robot_description` | — | Robot 1 chassis |
| SLAM Map | `/a300_00000/map` | Grey | Live occupancy grid |
| LiDAR | `/a300_00000/sensors/lidar2d_0/scan` | Cyan | Obstacle sensor |
| ArUco Target Sphere | `/a300_00000/aruco_tracker/target_marker` | Red (live) / Orange (KF) | Tracker position |
| KF Rollout Path | `/a300_00000/aruco_tracker/predicted_path_odom` | Magenta | Kinematic prediction (debug only) |
| Planned Path | `/a300_00000/aruco_planned_path` | Green | Obstacle-aware A* path |
| Nav Goal Cylinder | `/a300_00000/aruco_goal_manager/goal_marker` | Green | Frozen goal during PLANNED_NAV |

---

## Troubleshooting

### Robot 2 platform_velocity_controller inactive
```bash
ros2 control switch_controllers \
    --activate platform_velocity_controller \
    --controller-manager /a300_00001/controller_manager
```

### ArUco follower not moving / always in "planned" mode
- Verify camera image is publishing: `ros2 topic hz /a300_00000/sensors/camera_0/color/image`
- Check nav_mode: `ros2 topic echo /a300_00000/aruco_goal_manager/nav_mode`
- If stuck in `"planned"`: goal manager entered planned state before the follower started. Restart with `launch_aruco_follower:=true` (default).

### No planned path (green line missing in RViz)
- Check SLAM: `ros2 topic hz /a300_00000/map` — should be ~1 Hz after t=12 s
- Check planner node: `ros2 node info /aruco_planner`
- Confirm `map→odom` TF: `ros2 run tf2_ros tf2_echo --ros-args -r /tf:=/a300_00000/tf map odom`

### Tracker never shows "measured"
- Gazebo camera publishes at ~1 Hz under load — this is expected
- Check detector: `ros2 topic hz /a300_00000/aruco_detector/target_pose`
- Ensure Robot 2 is within camera FOV (start position x=5 m; marker on rear)

### RViz "Fixed Frame [map] does not exist"
- SLAM has not initialised yet. Wait until t~14 s for the first `map→odom` transform.
- Check: `ros2 topic echo /a300_00000/map --once`

---

## Design Notes

### Why two separate cmd_vel topics?
The original design published directly to `/a300_00000/cmd_vel`. The new design uses separate topics for visual servo and planned navigation, routed through a dedicated twist_mux. This means:
- Switching between modes requires no explicit coordination — timeout-based priority handles it
- Both nodes can run continuously without conflict
- teleop (PS4 controller or keyboard) can override everything at any time by publishing to the teleop slot

### Why A* instead of Hybrid-A* for ArUco recovery?
Hybrid-A* produces smoother paths but takes 200–500 ms for short recovery paths. A* completes in under 50 ms. Since ArUco recovery paths are short (robot is never far from the predicted goal), A* is fast enough and avoids planning latency that would cause the robot to overshoot.

### Why decouple nav_timeout_s from prediction_timeout_s?
The ArUco tracker stops predicting after 10 s (`prediction_timeout_s`). The goal manager continues navigating for up to 30 s (`nav_timeout_s`). This allows the robot to navigate to the last known goal even after the KF prediction has expired — useful when Robot 2 goes around a corner.

### TF remapping
All nodes that need Robot 1's TF tree remap `/tf → /a300_00000/tf` and `/tf_static → /a300_00000/tf_static`. This is required because ROS 2 tf2 uses absolute topic names (`/tf`, `/tf_static`) regardless of node namespace. Without remapping, nodes would merge all robots' transforms into one tree.
