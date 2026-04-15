# Clearpath A300 Autonomous Navigation

ROS 2 Jazzy workspace for autonomous navigation and multi-robot coordination on Clearpath A300 platforms.

## Quick Start

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Integrated Goal-Pose + ArUco Follow Demo (Recommended)

Robot 2 navigates to clicked RViz goals while Robot 1 trails behind using ArUco marker detection with obstacle-aware recovery navigation.

```bash
ros2 launch autonomy_bringup two_robot_goal_follow.launch.py
```

**Role assignment:**
- **Robot 2 (`a300_00001`) = LEADER** — receives `2D Goal Pose`, runs the custom planner + path follower, marker on rear bumper
- **Robot 1 (`a300_00000`) = FOLLOWER** — camera detects the marker, follows with visual servo, falls back to obstacle-aware A* path planning when the marker is lost

**Workflow:**

1. Launch the integrated demo and wait about 20 seconds for both robots and SLAM to come up.
2. In RViz, click **2D Goal Pose** to command Robot 2.
3. Robot 1 follows automatically once the marker is visible.

Detailed setup notes and troubleshooting live in [docs/MULTI_ROBOT_SETUP.md](docs/MULTI_ROBOT_SETUP.md).

---

## Two-Robot ArUco Follow Demo (Manual Leader Control)

Same physical setup as the integrated demo, but you drive the leader manually while the follower uses the visual-servo + planned-recovery stack.

```bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py
```

**Startup timeline:**

| Time | Event |
|------|-------|
| t=0 s | Gazebo starts, Robot 1 generators start |
| t=8 s | Robot 2 (platform + sensors + spawn) |
| t~10 s | Robot 1 spawned in Gazebo |
| t=12 s | SLAM for Robot 1 + ArUco detector |
| t=13 s | ArUco Kalman tracker + camera optical frame TF |
| t=15 s | ArUco goal manager |
| t=16 s | Visual follower + planner + path follower + twist mux |
| t=17 s | RViz (ArUco debug view) |

**Drive the leader after ~16 s:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=a300_00001/base_link -r cmd_vel:=/a300_00001/cmd_vel
```

**Optional launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `follower_desired_standoff_m` | `1.5` | Standoff distance (m) from leader |
| `nav_timeout_s` | `30.0` | Max seconds to pursue frozen goal after ArUco lost |
| `launch_slam` | `true` | Set `false` if SLAM already running |
| `launch_rviz` | `true` | Open RViz ArUco debug view |
| `robot2_x` / `robot2_y` | `5.0` / `0.0` | Leader start pose |
| `world` | `warehouse` | Gazebo world |

Example with custom standoff:

```bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py \
    follower_desired_standoff_m:=2.0 \
    nav_timeout_s:=45.0
```

---

## Single-Robot Simulation

```bash
# Terminal 1 — Gazebo
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2 — SLAM
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3 — Autonomy (planner + controller + RViz)
ros2 launch autonomy_bringup autonomy.launch.py use_sim_time:=true
```

---

## System Architecture

### Two-Robot ArUco Following Pipeline

```
ArUco Detector (/a300_00000/aruco_detector/target_pose)
       │
       ▼
ArUco Tracker  — Kalman filter (odom frame)
   │        │
   │        └─ tracked_pose_odom ──► ArUco Goal Manager
   │                                      │ nav_mode ("visual"/"planned"/"idle")
   │                                      │ aruco_navigation_goal (map frame)
   │                                      │
   ▼                                      ▼
ArUco Follower              Trajectory Planner (A*)
(visual servo)                     │
   │                               ▼
   │                         Path Follower
   │                               │
   ▼                               ▼
 /a300_00000/aruco_follower/cmd_vel    /a300_00000/aruco_autonomous/cmd_vel
       │                                               │
       └──────────────┬────────────────────────────────┘
                      ▼
                  Twist Mux  (teleop slot wins; falls through on silence)
                      │
                      ▼
              /a300_00000/cmd_vel  →  Platform
```

**Following modes:**

| Tracker Status | nav_mode | Active Component |
|---------------|----------|-----------------|
| `measured` | `visual` | ArUco visual servo (direct camera-frame control) |
| `predicted` or `lost` | `planned` | Obstacle-aware A* → path follower |
| Timeout elapsed | `idle` | All motion stopped; waiting for reacquisition |

**Command arbitration (twist_mux):**
- Teleop slot: `/a300_00000/aruco_follower/cmd_vel` — wins immediately when active
- Autonomous slot: `/a300_00000/aruco_autonomous/cmd_vel` — wins after 1.5 s teleop silence
- When `nav_mode = "planned"`, ArUco follower goes completely silent → mux timeout fires → path follower takes command

### Packages

| Package | Role |
|---------|------|
| `trajectory_planner_pkg` | Path planning (A*, Hybrid-A*, RRT*) on occupancy grid |
| `simple_motion_pkg` | Path following with 5 controllers + twist_mux |
| `autonomy_bringup` | Launch orchestration + ArUco nodes |
| `navigation` | Nav2 integration + waypoint navigator |
| `state_estimation` | Standalone EKF/UKF/Particle Filter |

### ArUco Nodes (autonomy_bringup)

| Node | Executable | Role |
|------|-----------|------|
| `aruco_detector` | `aruco_detector` | OpenCV detection; publishes camera-frame pose |
| `aruco_tracker` | `aruco_tracker` | Kalman filter; smooths detections; coasts in odom frame |
| `aruco_follower` | `aruco_follower` | Visual servo; direct camera-frame P-controller |
| `aruco_goal_manager` | `aruco_goal_manager` | State machine; bridges tracker → planner |

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/a300_00000/aruco_detector/target_pose` | `PoseStamped` | Raw camera-frame marker pose |
| `/a300_00000/aruco_tracker/tracked_pose` | `PoseStamped` | KF-smoothed camera-frame pose |
| `/a300_00000/aruco_tracker/tracked_pose_odom` | `PoseStamped` | KF pose in odom frame |
| `/a300_00000/aruco_tracker/status` | `String` | `measured` / `predicted` / `lost` |
| `/a300_00000/aruco_goal_manager/nav_mode` | `String` | `visual` / `planned` / `idle` |
| `/a300_00000/aruco_navigation_goal` | `PoseStamped` | Frozen goal for planner (map frame) |
| `/a300_00000/aruco_planned_path` | `Path` | Obstacle-aware A* path |
| `/a300_00000/aruco_follower/cmd_vel` | `TwistStamped` | Visual servo commands (twist_mux teleop slot) |
| `/a300_00000/aruco_autonomous/cmd_vel` | `TwistStamped` | Path follower commands (twist_mux autonomous slot) |
| `/a300_00000/cmd_vel` | `TwistStamped` | Final command to platform |
| `/a300_00000/map` | `OccupancyGrid` | SLAM live map for planning |

---

## Collision Protection

The follower maintains a safe standoff from the leader via two mechanisms:

1. **Hard minimum approach distance** (`min_approach_dist_m = 1.2 m`): Visual servo stops unconditionally when the marker depth drops below this value, regardless of standoff setting or controller output.

2. **Planned-nav goal tolerance** (`goal_tolerance = 1.5 m`): Path follower stops at the same distance as the visual servo standoff so planned navigation never drives closer than desired.

---

## Troubleshooting

**Robot 2's `platform_velocity_controller` shows `inactive` after launch:**
```bash
ros2 control switch_controllers --activate platform_velocity_controller \
    --controller-manager /a300_00001/controller_manager
```

**ArUco follower not moving:**
- Check that ArUco marker is visible in the camera image (`/a300_00000/sensors/camera_0/color/image`)
- Check `nav_mode`: `ros2 topic echo /a300_00000/aruco_goal_manager/nav_mode`
- If stuck in `"planned"`, SLAM may not have initialised (no `map→odom` TF yet); wait a few seconds after t=12 s

**No planned path in RViz (green line missing):**
- Confirm SLAM is running: `ros2 topic hz /a300_00000/map`
- Check planner log: `ros2 node info /aruco_planner`

**RViz fixed frame error:**
- Fixed frame must be `map` (SLAM provides it). Do not change to `odom` — planned path and SLAM map are in map frame.

---

## Robot Hardware

- **Platform**: Clearpath A300, serial `a300-00000`, IP `192.168.131.1`
- **Sensors**: Microstrain IMU (100 Hz), Garmin 18x GPS (10 Hz), Intel RealSense D435 (30 Hz), Hokuyo UST LiDAR (40 Hz, 270° FOV)
- **Wheel radius**: 0.1625 m, **Wheel separation**: 0.562 m

---

## Build Notes

Build a single package:
```bash
colcon build --packages-select autonomy_bringup
```

Python dependencies not in rosdep:
```bash
pip3 install transforms3d scipy
```

Do **not** use `multi_robot_simulation.launch.py` or `working_multi_robot.launch.py` — both are broken (namespace collision, Gazebo never starts).

Do **not** re-run Clearpath generators for Robot 2 without re-adding the ArUco marker to `robot2/robot.urdf.xacro`.
