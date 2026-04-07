# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

**ROS 2 Jazzy workspace** — always source ROS 2 before building:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Build a single package:
```bashA
colcon build --packages-select <package_name>
# e.g.: colcon build --packages-select trajectory_planner_pkg simple_motion_pkg
```

Python dependencies not in rosdep:
```bash
pip3 install transforms3d scipy
```

## Running the System

### Integrated goal-pose + ArUco-follow demo (canonical, single command)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autonomy_bringup two_robot_goal_follow.launch.py
```

**Role assignment:**
- **Robot 2 (`a300_00001`) = LEADER** — receives `2D Goal Pose` from RViz → custom planner (A*/Hybrid-A*/RRT*) → custom path follower + obstacle avoidance → drives
- **Robot 1 (`a300_00000`) = FOLLOWER** — forward camera detects Robot 2's rear ArUco marker → ArUco follower drives Robot 1 to trail behind Robot 2

**Workflow once up (~15 s after launch):**
1. Open RViz (launched automatically).
2. Click **`2D Goal Pose`** anywhere on the map — Robot 2 plans and drives there.
3. Robot 1 follows Robot 2 automatically once the ArUco marker is in view.

**Optional args:**
```bash
ros2 launch autonomy_bringup two_robot_goal_follow.launch.py \
    follower_desired_standoff_m:=2.0 \
    follower_target_timeout_s:=2.0 \
    launch_slam:=false          # if SLAM already running in another terminal
```

Timeline: Gazebo + Robot 1 generators at t=0 → Robot 2 at t=8 s → Robot 1 at ~t=10 s → SLAM + ArUco detector at t=12 s → autonomy stack + ArUco follower + RViz at t=14 s.

---

### ArUco-only demo (no goal-pose autonomy)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py
```

Spawning sequence: Gazebo + Robot 1 generators at t=0 → Robot 2 platform+sensors+spawn at t=8 s → Robot 1 spawns at ~t=10 s (generators finish) → both robots fully up by ~t=12 s.
See `docs/MULTI_ROBOT_SETUP.md` for details, customisation args, and troubleshooting.

> **Do NOT use `multi_robot_simulation.launch.py` or `working_multi_robot.launch.py`** — both are broken (namespace collision, Gazebo never starts).

**ArUco marker follow workflow** (after the simulation is up):

```bash
# Drive Robot 2 (the one being followed) — all on ONE line, no backslashes
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=a300_00001/base_link -r cmd_vel:=/a300_00001/cmd_vel

# Start Robot 1 follower (separate terminal) — target_timeout_s>=2.0 required
# because Gazebo camera publishes at ~1 Hz under load
ros2 launch autonomy_bringup aruco_follow.launch.py target_timeout_s:=2.0

# Optional: adjust standoff distance (default 1.5 m)
ros2 launch autonomy_bringup aruco_follow.launch.py target_timeout_s:=2.0 desired_standoff_m:=2.0
```

Or launch everything including the follower in one command:
```bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py launch_aruco_follower:=true
```

**If Robot 2's `platform_velocity_controller` shows `inactive` after launch:**
```bash
ros2 control switch_controllers --activate platform_velocity_controller --controller-manager /a300_00001/controller_manager
```

---

**Single-robot simulation (original workflow):**

```bash
# Terminal 1: Gazebo simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2: SLAM mapping
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3: Autonomy (planner + controller + RViz)
ros2 launch autonomy_bringup autonomy.launch.py use_sim_time:=true
```

`autonomy.launch.py` launch args: `use_sim_time`, `launch_rviz` (default true), `launch_motion` (default true), `launch_plot` (default false), `planner_params`, `motion_params`.

**Nav2 waypoint navigation:**
```bash
ros2 run navigation waypoint_navigator \
    --ros-args \
    -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
    -p mode:=loop \
    -p use_sim_time:=true
```

**State estimation (standalone, no colcon build needed):**
```bash
python3 state_estimation/sensor_fusion_ekf.py   # EKF
python3 state_estimation/sensor_fusion_ukf.py   # UKF
python3 state_estimation/sensor_fusion_pf.py    # Particle Filter
python3 state_estimation/compare_filters.py     # Run all three + compare
```

## System Architecture

The system has four major subsystems that form a planning-execution loop:

```
Goal Pose → Trajectory Planner → Path → Path Follower → cmd_vel → Twist Mux → Robot Motors
                 ↑                                                       ↑
              /map (SLAM)                                         Teleop (PS4)
```

### Packages

| Package | Role |
|---|---|
| `trajectory_planner_pkg` | Path planning (A*, Hybrid-A*, RRT*) on occupancy grid |
| `simple_motion_pkg` | Path following with 5 selectable controllers + twist_mux |
| `autonomy_bringup` | Orchestration launch file + localization plotting |
| `navigation` | Nav2 integration + waypoint navigator |
| `state_estimation` | Standalone EKF/UKF/Particle Filter sensor fusion |
| `sensors` | Launch + config for IMU, GPS, LiDAR, Camera |
| `platform` | Motor control, teleop, diagnostics |

### Multi-robot directory layout

```
clearpath/
├── robot.yaml          # Robot 1 config  (namespace: a300_00000)
├── robot.urdf.xacro    # Robot 1 URDF    (generated by Clearpath tools, no marker)
├── platform/launch/    # Robot 1 generated platform launch files
├── sensors/launch/     # Robot 1 generated sensor launch files
│
└── robot2/             # Robot 2 setup directory (within the SAME workspace)
    ├── robot.yaml          # Robot 2 config  (namespace: a300_00001)
    ├── robot.urdf.xacro    # Robot 2 URDF    (ArUco marker link embedded — do NOT re-run generators without re-adding marker)
    ├── platform/launch/    # Robot 2 generated platform launch files
    └── sensors/launch/     # Robot 2 generated sensor launch files
```

The canonical launch file (`two_robot_aruco.launch.py`) uses `generate:=false` for Robot 2 to avoid the Clearpath generator overwriting the custom URDF. If `robot2/robot.yaml` changes, re-run the four generators manually (see `docs/MULTI_ROBOT_SETUP.md`) and re-add the ArUco marker to `robot2/robot.urdf.xacro`.

### trajectory_planner_pkg

- `planner_node.py` — ROS 2 node; subscribes to `/a300_00000/map` and `/goal_pose`, publishes `/planned_path`
- `planner_core.py` — Algorithm implementations: `astar()`, `hybrid_astar()`, `rrt_star()`; path smoothing via shortcut pruning
- `grid_utils.py` — Occupancy grid ↔ world coordinate transforms, grid inflation, clearance maps
- `config/planner_params.yaml` — Key params: `planner_type` (astar/hybrid_astar/rrt_star), `inflation_radius_m` (0.40m), `occupied_threshold` (85), `compare_mode`

Compare mode publishes all three planners simultaneously on separate topics.

### simple_motion_pkg

- `path_follower.py` — Main node; tracks `/planned_path`, publishes to `/autonomous/cmd_vel`; handles obstacle replanning, stuck detection (4s timeout), off-path recovery (6s cooldown)
- `velocity_profiler.py` — Speed scaling based on path curvature and obstacle proximity
- `twist_mux.py` — Arbitrates `/autonomous/cmd_vel` vs teleop; outputs `/a300_00000/cmd_vel`
- `controllers/` — stanley, pure_pursuit, pid, lqr, mpc (each ~50–200 lines)
- `config/motion_params.yaml` — Key params: `controller_type` (stanley/pid/pure_pursuit/lqr/mpc), `max_linear_vel` (0.4 m/s), `max_angular_vel` (1.0 rad/s), `controller_compare_mode`

### state_estimation

Standalone Python scripts (not a ROS package), each implementing an 8-state filter: `[x, y, θ, vx, vy, ω, ax, ay]`. Fuses IMU (100 Hz), GPS (10 Hz), Odometry, and LiDAR. Run directly with `python3`.

## Robot Hardware

- **Platform**: Clearpath A300 (4-wheel differential drive), serial `a300-00000`, IP `192.168.131.1`
- **Wheel radius**: 0.1625 m, **Wheel separation**: 0.562 m
- **Sensors**: Microstrain IMU (100 Hz, `/dev/microstrain_main`), Garmin 18x GPS (10 Hz), Intel RealSense D435 (30 Hz, 640×480), Hokuyo UST LiDAR (40 Hz, 270° FOV, `/dev/clearpath/lidar`)
- **All hardware config**: `robot.yaml`

## Key Topics

| Topic | Type | Direction |
|---|---|---|
| `/a300_00000/map` | `nav_msgs/OccupancyGrid` | Planner input |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Planner + follower input |
| `/planned_path` | `nav_msgs/Path` | Planner → Follower |
| `/a300_00000/sensors/lidar2d_0/scan` | `sensor_msgs/LaserScan` | Follower obstacle input |
| `/autonomous/cmd_vel` | `geometry_msgs/Twist` | Follower → Twist Mux |
| `/a300_00000/cmd_vel` | `geometry_msgs/Twist` | Twist Mux → Platform |
