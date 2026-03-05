# Clearpath Autonomous Navigation - Engineering Log

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

