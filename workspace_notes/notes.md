# Clearpath Autonomous Navigation - Engineering Log

---

## � Iteration 2 — 2026-03-07 (In Progress)

### Part 1: Official Clearpath Packages Discovery ✅ COMPLETE
**Status:** [CRITICAL] Custom configuration should defer to official Clearpath demos

After creating the custom navigation stack, discovered that **Clearpath Robotics provides official packages** optimized for Nav2, SLAM, and localization.

**Official Packages:**
1. `clearpath_gz` - Gazebo simulation for A300
2. `clearpath_nav2_demos` - Pre-configured Nav2 stack with SLAM and localization
3. `clearpath_viz` - Official Rviz visualization for navigation

### Part 2: Navigation Package Build Setup ✅ COMPLETE
**Task:** Make waypoint_navigator a proper ROS 2 package

**Issues Resolved:**
1. ✅ Created `package.xml` with all dependencies
2. ✅ Created `CMakeLists.txt` for package building
3. ✅ Reorganized Python code into proper package structure
4. ✅ Fixed timer bug in waypoint_navigator (`create_timer()` parameter)
5. ✅ Created wrapper executable for entry point
6. ✅ Successfully built package with colcon

**Package Structure:**
```
navigation/
├── package.xml                    # Package manifest
├── CMakeLists.txt                 # Build configuration
├── setup.py                       # Python setup (for reference)
├── bin/
│   └── waypoint_navigator         # Entry point wrapper
├── navigation/
│   ├── __init__.py               # Python package marker
│   └── waypoint_navigator.py     # Main implementation
├── config/
│   ├── nav2_params.yaml
│   ├── localization_gps.yaml
│   └── waypoints_*.yaml
├── launch/
│   └── navigation.launch.py
└── README.md
```

**Build & Run:**
```bash
# Build (one time)
cd /home/prajjwal/clearpath
colcon build --packages-select navigation

# Source workspace
source install/setup.bash

# Run waypoint navigator
ros2 run navigation waypoint_navigator --ros-args \
  -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
  -p mode:=loop
```

### Execution Flow (Official + Custom)
1. **Simulation:** `clearpath_gz/simulation.launch.py`
2. **Nav2 Setup:** `clearpath_nav2_demos/nav2.launch.py`
3. **Localization:** `clearpath_nav2_demos/slam.launch.py` OR `clearpath_nav2_demos/localization.launch.py`
4. **Visualization:** `clearpath_viz/view_navigation.launch.py`
5. **Missions:** `ros2 run navigation waypoint_navigator` (custom)

---

## �🔴 Iteration 2 — 2026-03-07 (CRITICAL UPDATE)

### Discovery: Use Official Clearpath Packages
**Status:** [CRITICAL] Custom configuration should defer to official Clearpath demos

After creating the custom navigation stack, discovered that **Clearpath Robotics provides official packages** optimized for Nav2, SLAM, and localization:

**Official Packages:**
1. `clearpath_gz` - Gazebo simulation for A300
2. `clearpath_nav2_demos` - Pre-configured Nav2 stack with SLAM and localization
3. `clearpath_viz` - Official Rviz visualization for navigation

**Official Launch Commands:**

*SLAM Mode (for mapping):*
```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

*Localization Mode (for autonomous navigation):*
```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

### Implications
- ✅ Custom `navigation/` package still useful for:
  - Waypoint navigation (FollowWaypoints action wrapper)
  - Mission-level autonomy
  - Custom path patterns
- ✅ Use official packages for core Nav2/SLAM/Localization
- ✅ Simpler maintenance (updates from Clearpath team)
- ✅ Proven in production environments
- ✅ Better integration with platform software

### Action Items
- [ ] Test custom waypoint navigator with official Nav2 launch
- [ ] Verify sensor integration works with official stack
- [ ] Document which configs come from official vs custom packages
- [ ] Update primary launch procedure to use official packages

### Updated Documentation
- ✅ Updated `navigation/README.md` with official launch commands
- ✅ Updated `workspace_notes/workspace_status.md` with correct quick start
- ✅ Updated `workspace_notes/iteration.md` with iteration 2 progress

---

## Iteration 1 — 2026-03-05

### Goal
Implement autonomous path planning and obstacle avoidance for Clearpath A300 Husky robot using:
- EKF-based state estimation (robot_localization)
- Nav2 stack for planning/control
- All 4 sensors integrated (IMU, GPS, LiDAR, Depth Camera)
- Waypoint navigation capability

### Initial Workspace Audit
**Existing packages:**
- `state_estimation/` - Custom EKF implementation (NOT using robot_localization)
- `platform/` - Platform control, twist_mux, EKF config (robot_localization)
- `sensors/` - All 4 sensors configured (IMU, GPS, LiDAR, Camera)

**Key files found:**
- `platform/config/localization.yaml` - robot_localization EKF config (odom + IMU only)
- `robot.urdf.xacro` - All 4 sensors present
- `sensors/config/*.yaml` - Gazebo bridge configs
- **No Nav2 configs exist yet** - need to create from scratch

**Current sensor topics (from configs):**
- IMU: `/a300_00000/sensors/imu_0/data`
- GPS: `/a300_00000/sensors/gps_0/fix`
- LiDAR: `/a300_00000/lidar2d_0/scan`
- Camera depth: `/a300_00000/camera_0/depth/image_rect_raw`
- Camera pointcloud: `/a300_00000/camera_0/points`
- Odometry: `/a300_00000/platform/odom`

**Current TF tree (expected):**
- `map` → `odom` → `base_link` → sensor frames
- EKF publishes `odom` → `base_link`
- Need map→odom from SLAM or AMCL

### Architecture Design

**Localization approach:**
- Use SLAM Toolbox in localization mode (user already runs it)
- robot_localization EKF for odom→base_link fusion
- navsat_transform_node for GPS→map conversion

**Sensor integration:**
1. **IMU** → EKF (angular velocity, linear acceleration)
2. **GPS** → navsat_transform_node → map frame origin
3. **LiDAR** → SLAM (localization) + Nav2 costmap (obstacles)
4. **Depth Camera** → Nav2 costmap (obstacles via pointcloud)

**Nav2 stack:**
- BT Navigator for mission control
- DWB controller for local planning
- NavFn for global planning
- Costmap layers: static map, obstacles (LiDAR + depth), inflation

### Files to Create/Modify

**New files:**
1. `navigation/config/nav2_params.yaml` - Full Nav2 configuration
2. `navigation/launch/navigation.launch.py` - Nav2 bringup
3. `navigation/scripts/waypoint_navigator.py` - Waypoint patrol node
4. `navigation/config/waypoints.yaml` - Example waypoint list

**Files to modify:**
1. `platform/config/localization.yaml` - Add GPS support to EKF
2. (Optional) Create navsat launch if needed

### Implementation Status
- [✓ COMPLETED] Creating navigation package structure
- [✓ COMPLETED] Nav2 configuration files
- [✓ COMPLETED] Waypoint navigator node
- [✓ COMPLETED] Launch file integration
- [✓ COMPLETED] Documentation and examples
- [PENDING] Real-world testing and validation

### Files Created/Modified

**New navigation package:**
- `navigation/config/nav2_params.yaml` - Full Nav2 stack configuration
  - Configured all Nav2 nodes: bt_navigator, controller_server, planner_server, etc.
  - DWB local planner: max_vel_x=0.8, max_vel_theta=1.0, acc_lim_x=2.5
  - NavFn global planner with Dijkstra algorithm
  - Local costmap: 3x3m rolling window, 5Hz update, voxel layer
  - Global costmap: full map, 1Hz update, obstacle+static+inflation layers
  - Both costmaps use LiDAR (`/a300_00000/lidar2d_0/scan`) and camera pointcloud (`/a300_00000/camera_0/points`)
  - Robot radius: 0.35m, inflation radius: 0.55m
  - Goal tolerance: 0.25m (xy), 0.25 rad (yaw)

- `navigation/config/localization_gps.yaml` - Dual EKF + GPS integration
  - ekf_local_node: fuses wheel odom + IMU for odom→base_link
  - ekf_global_node: fuses local estimate + GPS for map→odom
  - navsat_transform_node: converts GPS lat/lon to map x/y coordinates
  - All configured with `use_sim_time: true`

- `navigation/launch/navigation.launch.py` - Main navigation launcher
  - Launches Nav2 bringup with custom parameters
  - Optional GPS localization (use_gps parameter)
  - Optional map server for pre-built maps
  - Namespace support for multi-robot

- `navigation/scripts/waypoint_navigator.py` - Waypoint navigation node (executable)
  - Supports FollowWaypoints and NavigateToPose actions
  - Three modes: single (one-shot), loop (continuous), patrol (forward+backward)
  - Loads waypoints from YAML file
  - Configurable wait duration at waypoints
  - Status publishing and feedback monitoring
  - Auto-start capability

**Waypoint examples:**
- `navigation/config/waypoints_square.yaml` - 5x5m square pattern
- `navigation/config/waypoints_figure8.yaml` - Figure-8 patrol
- `navigation/config/waypoints_corridor.yaml` - Linear corridor patrol

**Documentation:**
- `navigation/README.md` - Complete navigation documentation
  - Architecture diagram showing sensor→EKF→Nav2 flow
  - Quick start guide
  - Sensor integration details (all 4 sensors documented)
  - Tuning parameters for speed, avoidance, planning
  - Troubleshooting section with specific fixes
  - Performance tips

- `navigation/start_navigation.sh` - Tmux-based quick start script (executable)
  - Launches all 5 terminals: gazebo, slam, nav2, rviz, waypoints
  - Automated startup with proper delays
  - Instructions for waypoint selection

**Modified files:**
- `README.md` - Updated with navigation section and new features

### Commands to Run

**Option 1: Quick Start (Recommended)**
```bash
cd /home/prajjwal/clearpath
./navigation/start_navigation.sh
# Follow instructions in the waypoints terminal
```

**Option 2: Manual Start**
```bash
# Terminal 1: Gazebo
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2: SLAM (wait 5s after Gazebo)
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3: Nav2 (wait 10s after SLAM)
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 4: RViz (wait 15s)
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true

# Terminal 5: Waypoint Navigator (wait for Nav2 to fully initialize ~30s)
python3 /home/prajjwal/clearpath/navigation/scripts/waypoint_navigator.py \
    --ros-args \
    -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
    -p mode:=loop \
    -p use_sim_time:=true
```

**Debug/Monitoring Commands:**
```bash
# Check Nav2 nodes running
ros2 node list | grep nav2

# Monitor costmap updates
ros2 topic hz /a300_00000/local_costmap/costmap

# View TF tree
ros2 run tf2_tools view_frames

# Check sensor topics
ros2 topic hz /a300_00000/lidar2d_0/scan
ros2 topic hz /a300_00000/camera_0/points
ros2 topic hz /a300_00000/sensors/imu_0/data
ros2 topic hz /a300_00000/sensors/gps_0/fix

# Monitor waypoint status
ros2 topic echo /waypoint_status

# Check EKF output
ros2 topic echo /a300_00000/odometry/filtered
```

### Validation Checklist

**Expected in RViz:**
- [ ] Robot model visible and moving
- [ ] LiDAR scan (red/white points) updating at 40Hz
- [ ] Map building/updating (gray grid)
- [ ] Local costmap (small blue/purple grid around robot)
- [ ] Global costmap (full map coverage)
- [ ] Planned global path (green line)
- [ ] Local trajectory (orange/yellow curve)
- [ ] TF frames visible (map→odom→base_link→sensors)
- [ ] Robot following waypoints sequentially

**Topics to verify (all should be publishing):**
- `/a300_00000/lidar2d_0/scan` → ~40 Hz
- `/a300_00000/camera_0/points` → ~30 Hz
- `/a300_00000/sensors/imu_0/data` → ~100 Hz
- `/a300_00000/sensors/gps_0/fix` → ~10 Hz (if GPS enabled)
- `/a300_00000/odometry/filtered` → ~50 Hz
- `/a300_00000/local_costmap/costmap` → ~5 Hz
- `/a300_00000/global_costmap/costmap` → ~1 Hz
- `/a300_00000/cmd_vel` → ~20 Hz (when navigating)

**TF tree expectations:**
```
map → odom → base_link → {imu_0_link, gps_0_link, laser_0_link, camera_0_link}
```
- `map→odom`: Published by SLAM (slam_toolbox)
- `odom→base_link`: Published by EKF (robot_localization)
- `base_link→sensors`: Static transforms from URDF

**Success criteria:**
1. Robot navigates to first waypoint without collision
2. Costmaps show obstacles from LiDAR and camera
3. Robot avoids obstacles while planning path
4. Reaches goal within tolerance (0.25m)
5. Continues to next waypoint in sequence
6. Loops/patrols according to mode selected

### Known Issues / TODO

**Current limitations:**
- GPS integration via navsat_transform not tested (requires outdoor GPS signal in sim)
- Costmap tuning may need adjustment for different environments
- Controller parameters optimized for moderate speed (not aggressive)
- No dynamic reconfigure support (must edit YAML and restart)

**Future improvements:**
- [ ] Test GPS integration with outdoor Gazebo worlds
- [ ] Add dynamic obstacle handling demonstration
- [ ] Implement smarter waypoint selection (e.g., nearest neighbor)
- [ ] Add RViz plugin for interactive waypoint editing
- [ ] Benchmark performance metrics (success rate, path efficiency)
- [ ] Test with real hardware (non-simulated sensors)
- [ ] Add recovery behavior configuration
- [ ] Integrate with state_estimation custom EKF (currently using robot_localization)

**Tuning needed for:**
- Real hardware (latencies, sensor noise different from sim)
- Different robot sizes (robot_radius, inflation_radius)
- Faster speeds (increase velocity limits and accelerations)
- Tighter spaces (reduce inflation, tune DWB critics)

### Troubleshooting Guide

**Problem: Robot not moving**
- Cause: Nav2 nodes not initialized or costmap not receiving data
- Fix: Check `ros2 node list`, verify sensor topics publishing, check TF tree
- Parameters: None specific

**Problem: Planner fails to find path**
- Cause: Goal in obstacle, inflation too large, unknown space not allowed
- Fix: Increase planner tolerance, reduce inflation_radius, set allow_unknown:true
- Parameters: `GridBased.tolerance` (increase to 1.0), `inflation_radius` (decrease to 0.4)

**Problem: Robot oscillates**
- Cause: DWB critic weights not balanced, goal tolerance too tight
- Fix: Increase PathAlign/GoalAlign scales, increase goal tolerances
- Parameters: `PathAlign.scale: 48.0`, `GoalAlign.scale: 36.0`, `xy_goal_tolerance: 0.35`

**Problem: Costmap not showing obstacles**
- Cause: Sensor topics not subscribed, frame_id mismatch, range limits wrong
- Fix: Verify observation_sources topics, check frame_ids, adjust obstacle_max_range
- Parameters: `scan.topic`, `scan.obstacle_max_range` (should be < sensor max range)

**Problem: TF lookup errors**
- Cause: EKF not publishing, SLAM not running, URDF not loaded
- Fix: Ensure ekf_node running, check SLAM/map_server, verify robot_state_publisher
- Parameters: Check `publish_tf: True` in EKF config

**Problem: Slow performance**
- Cause: High costmap resolution, large costmap size, high update rates
- Fix: Lower resolution to 0.1m, reduce costmap dimensions, lower update frequency
- Parameters: `resolution: 0.1`, `width: 5`, `update_frequency: 3.0`

### Mistakes to Avoid (Lessons Learned)

1. **Don't forget `use_sim_time:=true`** on ALL nodes when using Gazebo
   - Symptom: TF extrapolation errors, timeouts, old data warnings
   - Fix: Add to every launch file and node parameter

2. **Don't use absolute paths in launch files** (except for this workspace config)
   - Future: Convert to proper ROS2 package with ament_index
   - Current: Hard-coded paths acceptable for initial development

3. **Don't skip verifying sensor topics before testing navigation**
   - Always check: `ros2 topic list` and `ros2 topic hz <topic>`
   - Sensors must be publishing before Nav2 can use them

4. **Don't set costmap ranges larger than sensor ranges**
   - LiDAR max: 25m, so obstacle_max_range should be ≤2.5m for safety
   - Camera depth: limited range, keep obstacle_max_range ≤2.5m

5. **Don't forget robot namespace in topic names**
   - All topics prefixed with `/a300_00000/`
   - Costmap configs must include namespace

6. **Don't run waypoint navigator before Nav2 is fully initialized**
   - Wait ~30s after launching Nav2
   - Check: `ros2 action list | grep navigate_to_pose`

7. **Don't edit nav2_params.yaml without understanding nested structure**
   - Each node has its own section (e.g., `controller_server:`)
   - YAML indentation matters

8. **Don't test in empty Gazebo world**
   - Need obstacles for costmap testing
   - Need features for SLAM

### Architecture Decisions

**Why robot_localization EKF instead of custom EKF?**
- Standard, well-tested, maintained by ROS community
- Integrates seamlessly with Nav2
- Proper TF publishing with correct frames
- Custom EKF in state_estimation/ is educational but robot_localization is production-ready

**Why not dual EKF (local+global)?**
- GPS not critical in simulation
- Single EKF (odom frame) sufficient for current needs
- Dual EKF implementation provided but not launched by default
- Can enable with `use_gps:=true` in navigation launch

**Why DWB controller over TEB?**
- Simpler configuration
- Better documented
- Adequate performance for differential drive
- TEB better for car-like robots or complex maneuvers

**Why NavFn planner over others?**
- Simple Dijkstra-based planner
- Works well with 2D costmaps
- Fast for moderate-sized maps
- Alternatives (Smac, Theta*) can be swapped easily

**Why voxel layer instead of obstacle layer in local costmap?**
- Handles 3D data from depth camera
- Better for clearing obstacles below sensor
- Marking/clearing in 3D voxel grid
- Global costmap uses obstacle layer (2D sufficient)

---

## Next Steps (Recommended Priority)

1. **Test in simulation** - Verify full stack works end-to-end
2. **Tune for performance** - Adjust speeds and aggressiveness
3. **Test different waypoint patterns** - Validate all navigation modes
4. **Add safety features** - Collision monitor, emergency stop
5. **Benchmark performance** - Measure success rate, path efficiency
6. **Prepare for real hardware** - Sensor calibration, safety limits
7. **Git commit** - Save all changes to repository

---

## Iteration 3 — 2026-03-08

### Goal
Implement baseline A* trajectory planner as a standalone ROS 2 package, with optional simple motion controller and a unified bringup launch. Independent of Nav2 — publishes `nav_msgs/Path` from start→goal using occupancy grid.

### Hard Guard 0 — Detected Runtime Configuration
| Item | Value | Source |
|------|-------|--------|
| map_topic | `/map` (OccupancyGrid) | nav2_map_server / SLAM toolbox |
| cmd_vel_topic | `/a300_00000/cmd_vel` | twist_mux → Gazebo bridge |
| goal_topic | `/goal_pose` (PoseStamped) | RViz2 "2D Goal Pose" |
| map frame | `map` | nav2_params.yaml |
| odom frame | `odom` | nav2_params.yaml |
| base_link frame | `base_link` | nav2_params.yaml |
| Namespace | `a300_00000` | robot.yaml |
| Robot radius | 0.35 m, inflation 0.55 m | nav2_params.yaml |
| Max velocity | 0.8 m/s linear, 1.0 rad/s angular | nav2_params.yaml |
| Costmap resolution | 0.05 m | nav2_params.yaml |

### Files Created (full paths)

**Package 1 — `trajectory_planner_pkg/` (ament_python)**
- `trajectory_planner_pkg/package.xml`
- `trajectory_planner_pkg/setup.py`
- `trajectory_planner_pkg/setup.cfg`
- `trajectory_planner_pkg/resource/trajectory_planner_pkg`
- `trajectory_planner_pkg/trajectory_planner_pkg/__init__.py`
- `trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py` — ROS 2 node: subscribes `/map` + `/goal_pose`, publishes `/planned_path`
- `trajectory_planner_pkg/trajectory_planner_pkg/planner_core.py` — A*, Hybrid-A*, RRT* algorithms + post-processing (prune collinear, shortcut smooth)
- `trajectory_planner_pkg/trajectory_planner_pkg/grid_utils.py` — OccupancyGrid ↔ numpy, world↔grid coordinate conversion, obstacle inflation
- `trajectory_planner_pkg/config/planner_params.yaml` — Default parameters
- `trajectory_planner_pkg/launch/planner.launch.py`

**Package 2 — `simple_motion_pkg/` (ament_python)**
- `simple_motion_pkg/package.xml`
- `simple_motion_pkg/setup.py`
- `simple_motion_pkg/setup.cfg`
- `simple_motion_pkg/resource/simple_motion_pkg`
- `simple_motion_pkg/simple_motion_pkg/__init__.py`
- `simple_motion_pkg/simple_motion_pkg/path_follower.py` — P-controller with lookahead: subscribes `/planned_path`, publishes `cmd_vel`
- `simple_motion_pkg/config/motion_params.yaml`
- `simple_motion_pkg/launch/motion.launch.py`

**Package 3 — `autonomy_bringup/` (ament_python)**
- `autonomy_bringup/package.xml`
- `autonomy_bringup/setup.py`
- `autonomy_bringup/setup.cfg`
- `autonomy_bringup/resource/autonomy_bringup`
- `autonomy_bringup/autonomy_bringup/__init__.py`
- `autonomy_bringup/config/autonomy.rviz` — Minimal RViz config (Map, TF, RobotModel, Path, LaserScan, MarkerArray, Goal tool)
- `autonomy_bringup/launch/autonomy.launch.py` — Starts planner + follower + RViz

### Change Summary

**trajectory_planner_pkg:**
- A* (8-connected grid, octile heuristic) is the baseline — always works
- Hybrid-A* (heading-aware, 72 discrete headings) selectable via `planner_type: hybrid_astar`
- RRT* (sampling-based with rewiring) selectable via `planner_type: rrt_star`
- OccupancyGrid → binary numpy grid with `occupied_threshold` (default 65)
- Circular obstacle inflation by `inflation_radius_m` (default 0.35 m = robot radius)
- Unknown cells configurable via `allow_unknown` (default true = free)
- Start pose from TF lookup (`map` → `base_link`)
- Goal from `/goal_pose` (RViz "2D Goal Pose")
- Post-processing: collinear pruning + random shortcut smoothing
- Path orientations set along tangent direction
- Debug markers published to `/planner_debug` (start=green sphere, goal=red sphere)

**simple_motion_pkg:**
- Pure-pursuit-like P-controller with configurable lookahead (0.5 m)
- Slows linear velocity when heading error > 45°
- Stops and logs "Goal reached!" when within `goal_tolerance` (0.25 m)
- cmd_vel bounded: max 0.4 m/s linear, 1.0 rad/s angular
- 10 Hz control loop

**autonomy_bringup:**
- Single launch file starts all three components
- `launch_motion:=false` to skip path follower (visualization only)
- `launch_rviz:=false` to skip RViz
- RViz configured with: Grid, TF, RobotModel, Map (transient local), PlannedPath (green), PlannerDebug markers, LaserScan, 2D Goal Pose tool → `/goal_pose`

### Commands to Run

```bash
# Step 0: Build (one time)
cd /home/prajjwal/clearpath
colcon build --packages-select trajectory_planner_pkg simple_motion_pkg autonomy_bringup
source install/setup.bash

# Step 1: Gazebo simulation (separate terminal)
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Step 2: SLAM for /map (separate terminal, wait ~5s after Gazebo)
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Step 3: Autonomy bringup (planner + follower + RViz)
cd /home/prajjwal/clearpath && source install/setup.bash
ros2 launch autonomy_bringup autonomy.launch.py use_sim_time:=true

# Step 3-alt: Planner only (no motion, no RViz)
ros2 launch autonomy_bringup autonomy.launch.py launch_motion:=false launch_rviz:=false use_sim_time:=true

# Step 3-alt2: Switch to Hybrid-A*
ros2 launch trajectory_planner_pkg planner.launch.py use_sim_time:=true \
  --ros-args -p planner_type:=hybrid_astar
```

### Expected RViz Behavior
1. Fixed Frame: `map`
2. Map visible as gray/black occupancy grid
3. Click "2D Goal Pose" tool → click on map
4. Green path line appears from robot to goal on `/planned_path`
5. Green sphere at start, red sphere at goal on `/planner_debug`
6. Robot moves along path (if path follower enabled)
7. Console logs: "Planning astar: start=(x,y) → goal=(x,y)" and "Path found: N waypoints"

### Expected Topics
```
/map                          — nav_msgs/OccupancyGrid (from SLAM)
/goal_pose                    — geometry_msgs/PoseStamped (from RViz)
/planned_path                 — nav_msgs/Path (from trajectory_planner)
/planner_debug                — visualization_msgs/MarkerArray (from trajectory_planner)
/a300_00000/cmd_vel           — geometry_msgs/Twist (from path_follower)
```

### Known Issues / TODO
- ⏳ If `/map` is not being published, planner will log "Waiting for map…" — ensure SLAM or map_server is running
- ⏳ If TF `map→base_link` is unavailable, planner logs error — ensure localization is running
- ⏳ `scipy` must be installed for `inflate_grid` (binary_dilation) — `pip install scipy` if missing
- ⏳ Hybrid-A* and RRT* not yet tested end-to-end in sim (A* is the validated baseline)
- ⏳ Path follower uses simple P-control — not suitable for tight corridors without tuning

### "Do Not Repeat" Mistakes
- Do NOT use latched QoS for `/goal_pose` — it's volatile
- Do NOT forget `use_sim_time:=true` — planner timestamps will be wrong
- Map topic QoS MUST be `TRANSIENT_LOCAL` + `RELIABLE` — otherwise map is never received
- Always `source install/setup.bash` after building before running
- **Map topic is `/a300_00000/map`, NOT `/map`** — Clearpath namespaces everything under `a300_00000`
- **TF is on `/a300_00000/tf` and `/a300_00000/tf_static`**, NOT global `/tf` — must remap in launch files
- All nodes (planner, follower, RViz) need TF remappings: `/tf` → `/a300_00000/tf`, `/tf_static` → `/a300_00000/tf_static`

### Iteration 3 Fix — 2026-03-08 (Namespaced Topics & TF)

**Problem:** Planner showed "Waiting for map…" and no path generated on goal click.

**Root Causes (2):**
1. Map published on `/a300_00000/map` but planner subscribed to `/map`
2. TF transforms (odom→base_link, map→odom) published on `/a300_00000/tf` (6 publishers, 0 on global `/tf`) — planner TF buffer was empty

**Fixes Applied:**
- ✅ `planner_params.yaml`: `map_topic: "/a300_00000/map"`
- ✅ `planner_node.py`: default `map_topic` hardcoded to `/a300_00000/map`
- ✅ `autonomy.launch.py`: added TF remappings for planner, follower, AND rviz nodes
- ✅ `planner.launch.py`: added TF remappings
- ✅ `motion.launch.py`: added TF remappings
- ✅ `autonomy.rviz`: Map topic → `/a300_00000/map`, update topic → `/a300_00000/map_updates`
- ✅ `planner_node.py`: TF fallback chain: try `map→base_link`, then `odom→base_link`, then map center

### Iteration 3 Fix 2 — 2026-03-08 (Robot Model + TwistStamped)

**Problems:**
1. No Husky robot visible in RViz — only green debug blob at robot location
2. Robot not moving despite path being planned (cmd_vel published but ignored)

**Root Causes (2):**
1. RViz RobotModel subscribed to `/robot_description` (0 publishers) — actual topic is `/a300_00000/robot_description` (1 publisher)
2. **twist_mux uses `use_stamped: True`** — expects `TwistStamped`, but path_follower published `Twist` → **type mismatch, messages silently dropped**

**Fixes Applied:**
- ✅ `autonomy.rviz`: RobotModel topic → `/a300_00000/robot_description`
- ✅ `path_follower.py`: changed publisher from `Twist` to `TwistStamped` (with header stamp + frame_id)
- ✅ `path_follower.py`: `_stop()` also sends `TwistStamped` (not bare `Twist`)
- ✅ `path_follower.py`: added TF warning with throttle when lookup fails, stops robot on TF failure

**"Do Not Repeat" additions:**
- Clearpath twist_mux uses `use_stamped: True` → MUST publish `TwistStamped`, not `Twist`
- Robot description topic is always namespaced: `/a300_00000/robot_description`
- Always check `ros2 topic info -v` to verify message TYPE matches between pub/sub

---

