# Iteration Log & Change Tracker

**Project:** Clearpath A300 Autonomous Navigation  
**Workspace Notes Convention:** Established March 7, 2026

---

## Iteration Index

- [Iteration 1](#iteration-1-autonomous-navigation-framework) — Feb 5-Mar 5, 2026 ✅ COMPLETE
- [Iteration 2](#iteration-2) — Mar 7, 2026 🟡 IN PROGRESS
- [Iteration 3](#iteration-3-a-trajectory-planner) — Mar 8, 2026 ✅ COMPLETE

---

## Iteration 1: Autonomous Navigation Framework

**Date Range:** February 5 - March 5, 2026  
**Status:** ✅ COMPLETE

### Objectives
1. Implement autonomous path planning using Nav2
2. Integrate EKF-based state estimation
3. Fuse all 4 sensors (IMU, GPS, LiDAR, Camera)
4. Create waypoint navigation capability

### Completion Summary

#### Phase 1: Workspace Audit
- Discovered existing state_estimation, platform, and sensors packages
- Identified all 4 sensor configurations in URDF
- Found robot_localization EKF config (odom + IMU only)
- Confirmed absence of Nav2 configs

#### Phase 2: Architecture Design
**Localization Stack:**
- robot_localization EKF for odom→base_link (IMU + wheel odometry)
- EKF global node for map→odom (GPS integration)
- navsat_transform_node for GPS lat/lon to map coordinates

**Sensor Integration:**
1. IMU → EKF (angular velocity, linear acceleration)
2. GPS → navsat_transform_node → map frame
3. LiDAR → SLAM + Nav2 costmap (obstacles)
4. Depth Camera → Nav2 costmap (obstacles via pointcloud)

#### Phase 3: Implementation
**Files Created:**
1. `navigation/config/nav2_params.yaml`
   - Complete Nav2 stack configuration
   - BT Navigator, DWB controller, NavFn planner
   - Local/global costmaps with LiDAR + camera
   - Robot radius: 0.35m, inflation: 0.55m
   - Max velocity: 0.8 m/s

2. `navigation/config/localization_gps.yaml`
   - ekf_local_node: wheel odom + IMU → odom frame
   - ekf_global_node: local estimate + GPS → map frame
   - navsat_transform_node: GPS coordinate conversion

3. `navigation/launch/navigation.launch.py`
   - Nav2 bringup with custom parameters
   - GPS localization option (use_gps parameter)
   - Map server support for pre-built maps
   - Namespace support for multi-robot

4. `navigation/scripts/waypoint_navigator.py`
   - Autonomous waypoint patrol node
   - Sends consecutive goals to Nav2 stack
   - Reports success/failure for each waypoint

5. `navigation/config/waypoints_*.yaml`
   - `waypoints_corridor.yaml` - Straight line patrol
   - `waypoints_figure8.yaml` - Figure-8 pattern
   - `waypoints_square.yaml` - Square perimeter

**Files Modified:**
- None (used YAML configs instead of modifying existing files)

### Key Achievements
- ✅ Navigation package fully structured
- ✅ All 4 sensors configured for autonomous operation
- ✅ EKF + GPS localization setup complete
- ✅ Nav2 stack configured for DWB local planning and NavFn global planning
- ✅ Waypoint navigation system implemented
- ✅ Multiple pre-configured patrol patterns available

### Testing Status
- 🟡 Simulation testing pending
- 🟡 Real-world testing pending
- 🟡 Performance optimization pending

### Technical Specifications
- **DWB Local Planner:** max_vel_x=0.8, max_vel_theta=1.0, acc_lim_x=2.5
- **Global Planner:** NavFn with Dijkstra algorithm
- **Local Costmap:** 3x3m rolling window, 5Hz update
- **Global Costmap:** Full map, 1Hz update
- **Sensor Fusion:** Dual EKF (local+global) + navsat_transform
- **Goal Tolerance:** 0.25m (xy), 0.25 rad (yaw)

---

## Iteration 2: Align with Official Clearpath Packages

**Date Range:** March 7, 2026 - [ongoing]  
**Status:** 🟡 IN PROGRESS

### Key Discovery
**🔴 CRITICAL UPDATE:** User identified that official Clearpath packages should be used instead of custom configuration:
- `clearpath_gz` - Official Gazebo simulation
- `clearpath_nav2_demos` - Official Nav2 with SLAM and localization
- `clearpath_viz` - Official visualization tools

This is the recommended approach from Clearpath Robotics team.

### Objectives
- [✅] Workspace documentation system setup
- [✅] Establish iteration tracking workflow  
- [✅] Discover and implement official Clearpath nav2 commands
- [ ] Verify custom waypoint navigator works with official Nav2
- [ ] Test both SLAM and localization modes

### Changes Made This Session
- ✅ Updated `navigation/README.md` with official Clearpath launch procedures
- ✅ Updated `workspace_status.md` with correct quick start commands
- ✅ Documented two scenarios: SLAM mode and Localization mode
- ✅ Clarified that `clearpath_nav2_demos` manages Nav2, SLAM, and localization

### Important Distinctions

**Custom vs Official:**
| Aspect | Custom | Official |
|--------|--------|----------|
| **Nav2 Bringup** | `navigation.launch.py` | `clearpath_nav2_demos/nav2.launch.py` |
| **SLAM** | Manual with robot_localization | `clearpath_nav2_demos/slam.launch.py` |
| **Localization** | Manual EKF config | `clearpath_nav2_demos/localization.launch.py` |
| **Visualization** | Generic Rviz | `clearpath_viz/view_navigation.launch.py` |
| **Simulation** | Would need separate gazebo setup | `clearpath_gz/simulation.launch.py` |

### Testing Plan
- 🟡 Test official SLAM mode in simulation
- 🟡 Test official localization mode in simulation
- 🟡 Verify waypoint navigator integrates with official Nav2
- 🟡 Compare performance: custom vs official packages

### Session 2: Package Setup & Build Configuration

**Date:** March 7, 2026  
**Task:** Fix navigation package build and make waypoint_navigator executable

**Issues Found & Fixed:**
1. ✅ Navigation package missing `package.xml` and `CMakeLists.txt`
2. ✅ Waypoint navigator not registered as ROS 2 executable
3. ✅ Bug in `create_timer()` - removed invalid `oneshot` parameter

**Changes Made:**
- ✅ Created `package.xml` with proper dependencies
- ✅ Created `CMakeLists.txt` for Python package build
- ✅ Moved `waypoint_navigator.py` to `navigation/navigation/` package directory
- ✅ Created `navigation/__init__.py` for proper Python packaging
- ✅ Created wrapper script `bin/waypoint_navigator` as entry point
- ✅ Updated `CMakeLists.txt` to install wrapper executable
- ✅ Fixed timer bug in waypoint navigator
- ✅ Successfully built package: `colcon build --packages-select navigation`

**How to Use Now:**
```bash
# Build the package (one time)
cd /home/prajjwal/clearpath
colcon build --packages-select navigation

# Source the workspace
source install/setup.bash

# Run waypoint navigator
ros2 run navigation waypoint_navigator --ros-args \
  -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
  -p mode:=loop
```

### Session 3: Debugging - Robot Not Moving

**Date:** March 7, 2026  
**Issue:** Robot launches all components but doesn't move

**Root Causes Identified:**
1. ⏳ FollowWaypoints action server not ready when waypoint_navigator starts
2. 🔴 Initial pose not set - AMCL doesn't know robot location
3. ❌ cmd_vel not publishing - controller hasn't received valid goal
4. 📦 TF cache timing issues between planning and execution

**Troubleshooting Resources Created:**
- ✅ `NAV_TROUBLESHOOTING.md` - Complete debugging guide with fixes
- ✅ `nav_diagnostics.sh` - Automated diagnostics script

**Key Findings:**
- Nav2 needs 40+ seconds to fully initialize
- Must SET INITIAL POSE in RViz before navigation works
- Waypoint navigator must wait for action server to be ready
- Gazebo simulation has strict timing requirements

**Fix Sequence:**
1. Start Gazebo + Nav2 and **WAIT 40 SECONDS**
2. In RViz: Click "2D Pose Estimate" and set starting position
3. Verify `/a300_00000/follow_waypoints` action exists
4. Verify `cmd_vel` starts publishing when goal is sent
5. THEN start waypoint navigator

**Files Created/Modified:**
- ✅ Created `/home/prajjwal/clearpath/NAV_TROUBLESHOOTING.md`
- ✅ Created `/home/prajjwal/clearpath/nav_diagnostics.sh`
- ✅ Updated iteration notes

### In Progress
- 🟡 Testing with proper initialization sequence
- 🟡 Verifying action server readiness
- 🟡 Confirming cmd_vel publishing

### Pending
- [ ] Full autonomous patrol test with initial pose setting
- [ ] Waypoint following validation
- [ ] Performance metrics collection

---

## Change Tracking Template

For each session, use this format:

```
### Session [Date] - [Brief Title]
**Duration:** [Start - End]  
**Objectives:** 
- [ ] Objective 1
- [ ] Objective 2

**Changes Made:**
- ✅ Change 1
- ✅ Change 2

**Testing:**
- Status of any tests
- Issues encountered

**Outcome:**
- Summary of session results
```

---

## Notes for Future Reference

### Workspace Structure Conventions
- **workspace_notes/** - Always maintains current documentation
- **workspace_status.md** - Updated whenever significant changes occur
- **iteration.md** - Updated after each development session

### When to Update Documentation
1. After completing any major feature
2. When sensor configurations change
3. When parameters are tuned
4. After testing sessions (pass/fail)
5. Before starting a new iteration

### Quick Reference Links
- Navigation configs: `navigation/config/`
- Sensor topics: See workspace_status.md table
- Launch files: Each directory has `launch/` subdirectory
- Configuration: Each directory has `config/` subdirectory

---

## Iteration 3: A* Trajectory Planner

**Date:** March 8, 2026
**Status:** ✅ COMPLETE

### Objectives
- [✅] Implement baseline A* grid-based planner as standalone ROS 2 package
- [✅] Add optional Hybrid-A* and RRT* via `planner_type` parameter
- [✅] Create minimal path-following P-controller
- [✅] Create unified bringup launch with RViz config
- [✅] All packages build and smoke-test pass

### Packages Created

| Package | Type | Purpose |
|---------|------|---------|
| `trajectory_planner_pkg` | ament_python | A*/Hybrid-A*/RRT* planner → publishes `nav_msgs/Path` |
| `simple_motion_pkg` | ament_python | P-controller path follower → publishes `cmd_vel` |
| `autonomy_bringup` | ament_python | Launch file + RViz config for the full stack |

### Key Design Decisions
- Subscribes to `/map` (OccupancyGrid, transient_local QoS) for grid
- Start pose from TF (`map` → `base_link`), not a parameter
- Goal from `/goal_pose` (RViz "2D Goal Pose" tool)
- Obstacle inflation via scipy `binary_dilation` with circular kernel
- A* uses octile heuristic (8-connected grid)
- Post-processing: collinear pruning + random shortcut smoothing
- Path follower: lookahead-based P-controller, 10 Hz, max 0.4 m/s

### Testing Status
- ✅ All 3 packages build cleanly with `colcon build`
- ✅ A* algorithm smoke-tested on synthetic grid
- ✅ All Python imports verified
- 🟡 End-to-end simulation test pending
- 🟡 Hybrid-A* and RRT* not yet tested in sim

### Technical Specifications
- **A* baseline**: 8-connected, octile heuristic, grid-based
- **Hybrid-A***: 72 discrete headings (5° resolution), 3 steer angles
- **RRT***: 5000 default iterations, 5-cell step size, rewiring
- **Inflation**: Circular kernel, default radius = robot radius (0.35 m)
- **Path follower**: Kp_v=0.5, Kp_w=1.5, lookahead=0.5 m, goal_tol=0.25 m
- **Sim time**: All nodes respect `use_sim_time` parameter

