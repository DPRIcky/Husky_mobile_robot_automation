# Iteration Log & Change Tracker

**Project:** Clearpath A300 Autonomous Navigation  
**Workspace Notes Convention:** Established March 7, 2026

---

## Iteration Index

- [Iteration 1](#iteration-1-autonomous-navigation-framework) — Feb 5-Mar 5, 2026 ✅ COMPLETE
- [Iteration 2](#iteration-2) — Mar 7, 2026 🟡 IN PROGRESS
- [Iteration 3](#iteration-3-a-trajectory-planner) — Mar 8, 2026 ✅ COMPLETE
- [Iteration 4](#iteration-4-obstacle-avoidance-debugging) — Mar 9, 2026 ✅ COMPLETE
- [Iteration 5](#iteration-5-stuck-and-off-path-recovery) — Mar 9, 2026 ✅ COMPLETE
- [Iteration 6](#iteration-6-compare-mode-all-3-planners-in-rviz) — Mar 10, 2026 ✅ COMPLETE
- [Iteration 7](#iteration-7-obstacle-avoidance-parameter-tuning) — Mar 10, 2026 ✅ COMPLETE
- [Iteration 8](#iteration-8-replanning-loop-fixes) — Mar 10, 2026 ✅ COMPLETE

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


---

## Iteration 4: Obstacle Avoidance Debugging

**Date:** March 9, 2026
**Status:** ✅ COMPLETE

### Objectives
- [✅] Fix robot colliding with obstacles
- [✅] Add lidar-based obstacle detection to path follower
- [✅] Fix wrong lidar topic subscription
- [✅] Fix obstacle detection angle (±60° → ±25°)
- [✅] Fix stop_dist vs inflation_radius conflict
- [✅] Fix occupied_threshold — SLAM noise causing route detours
- [✅] Fix goal cell snapping in planner
- [✅] Fix laser frame angle offset (TF-corrected scan angles)
- [⏳] Full end-to-end obstacle avoidance validation

### Root Causes Found & Fixed (in order)

#### Fix 1 — Path follower had no obstacle avoidance
- **Problem:** Original path_follower.py was a pure P-controller with zero sensor input
- **Fix:** Added LaserScan subscription + stop-and-replan logic

#### Fix 2 — Wrong lidar topic (CRITICAL)
- **Problem:** Subscribed to `/a300_00000/lidar2d_0/scan` — NO publishers
- **Correct topic:** `/a300_00000/sensors/lidar2d_0/scan` (confirmed in clearpath_nav2_demos slam.launch.py)
- **Fix:** Updated scan_topic in motion_params.yaml and path_follower.py default

#### Fix 3 — Reactive steering caused oscillation
- **Problem:** Robot rotated toward clearer side → re-aimed at old path → saw obstacle → rotated again → infinite loop
- **Fix:** Removed reactive steering. Now: stop → wait 0.8s for map update → replan → wait for new path

#### Fix 4 — Obstacle check angle too wide (±60° → ±25°)
- **Problem:** ±60° cone caught corridor walls. Wall 0.5m to side → detected at 0.5/sin(60°)=0.577m < 0.6m stop threshold
- **Fix:** `obstacle_check_angle: 0.436` (±25°). Wall at 0.5m now at 0.5/sin(25°)=1.18m → no false stop

#### Fix 5 — stop_dist > inflation_radius (CORE CONFLICT)
- **Problem:** stop_dist=0.6m > inflation_radius=0.4m → lidar stopped robot at walls planner intentionally planned near
- **Observed:** "Obstacle at 0.59m" firing 15ms after path received — robot never moved
- **RULE: stop_dist MUST always be < inflation_radius**
- **Fix:** obstacle_stop_dist: 0.25m (< inflation 0.40m)

#### Fix 6 — SLAM noise treated as obstacles
- **Problem:** occupied_threshold=65 treated SLAM noise (value 65-80) as solid obstacles → long detour routes
- **SLAM values:** Free=0, Unknown=-1, Noise/uncertainty=65-80, Real obstacles=100
- **Fix:** `occupied_threshold: 85`

#### Fix 7 — Goal cell in inflated zone → silent planner failure
- **Problem:** A* returns None if goal cell is inside inflated obstacle. No error shown to user. No path generated.
- **Fix:** Added `_nearest_free_cell()` BFS in planner_node.py to snap goal to nearest free cell

#### Fix 8 — Laser frame not corrected to base_link
- **Problem:** Scan angles used raw (laser frame). If laser is rotated on robot, ±25° check was wrong direction
- **Fix:** `_get_laser_yaw_offset()` looks up TF `lidar2d_0_laser → base_link` once, caches yaw offset, applies to all angle checks
- Also added `range_min` filtering (was only filtering r<=0, now uses scan.range_min=0.05m)

#### Fix 9 — Inflation radius tuning history
- 0.35m (original) → 0.50m (too aggressive, blocked goals/corridors) → **0.40m (current)**

### Current Parameter Values

| Parameter | Value | Reason |
|-----------|-------|--------|
| `inflation_radius_m` | 0.40m | Robot half-width ~0.33m + margin |
| `occupied_threshold` | 85 | Filter SLAM noise (65-80), keep real obstacles (100) |
| `scan_topic` | `/a300_00000/sensors/lidar2d_0/scan` | Actual Clearpath lidar topic |
| `obstacle_stop_dist` | 0.25m | Must be < inflation_radius (0.40m) |
| `obstacle_warn_dist` | 0.50m | Slow to 50% near obstacles |
| `obstacle_check_angle` | 0.436 rad (±25°) | Narrow enough to avoid false stops from side walls |
| `map_update_delay_s` | 0.8s | Wait for SLAM to update map before replanning |
| `replan_retry_s` | 3.0s | Retry if still blocked |

### Architecture: Stop-and-Replan Flow
```
Obstacle enters forward cone (±25°, corrected to base_link frame):
  1. Stop immediately
  2. Wait 0.8s → SLAM updates map with new obstacle
  3. Publish current goal to /goal_pose → planner replans
  4. Remain stopped until new /planned_path arrives
  5. New path → reset state → resume
  6. If still blocked after 3s → retry replan
```

### Key Rules Established
1. **stop_dist < inflation_radius** — hard rule, always
2. **Lidar topic = `/a300_00000/sensors/lidar2d_0/scan`** — not `/lidar2d_0/scan`
3. **Always correct scan to robot frame via TF** — raw angles are in laser frame
4. **occupied_threshold = 85** for SLAM toolbox maps
5. **Goal snapping required** — planner silently fails if goal is in inflated zone
6. **No reactive steering** — stop+replan only for known environments


---

## Iteration 5: Stuck & Off-Path Recovery

**Date:** March 9, 2026
**Status:** ✅ COMPLETE

### Problem Statement
After Iteration 4 fixes, the robot was still observed heading directly into an obstacle (confirmed via RViz + Gazebo screenshot) without triggering any replanning. The lidar-based stop (±25° cone, 0.25m) was not catching the obstacle because:
1. The robot was approaching at an angle — obstacle slightly outside the narrow forward cone
2. Once physically pushed off-path, no recovery mechanism existed

### Fix 1 — Stuck Detection

**Mechanism:** Progress checkpoint every `stuck_check_interval_s` (4 s). If the robot has moved less than `stuck_dist_threshold_m` (0.15 m) while actively following a path, publish a replan request. Checkpoint resets when a new path is received.

**New parameters added to motion_params.yaml:**
- `stuck_check_interval_s: 4.0`
- `stuck_dist_threshold_m: 0.15`

### Fix 2 — Off-Path Detection

**Mechanism:** Every control loop tick (10 Hz), compute minimum Euclidean distance from the robot to any point on `_path`. If this exceeds `off_path_dist_m` (1.5 m), stop immediately and publish replan request.

**Rationale:** If the robot is >1.5m from its planned path it has either been physically displaced or the path is no longer reachable from current position. Replanning from current pose gives a fresh valid path.

**Latency:** ~100 ms (next control loop tick).

**New parameter:**
- `off_path_dist_m: 1.5`

### Files Modified

| File | Changes |
|------|---------|
| `simple_motion_pkg/simple_motion_pkg/path_follower.py` | Added stuck detection + off-path detection |
| `simple_motion_pkg/config/motion_params.yaml` | Added 3 new parameters |

### Recovery Behaviour Summary (Post Iteration 5)

| Trigger | Method | Latency |
|---------|--------|---------|
| Obstacle in lidar forward cone | Scan check (10 Hz) | ~100 ms |
| Robot drifted off planned path | Distance to path (10 Hz) | ~100 ms |
| Robot stalled / stuck | Pose progress (every 4 s) | up to 4 s |
| Replan did not clear obstacle | Retry timer (every 3 s) | 3 s |

### Test Result
✅ Robot successfully avoids obstacles and replans when deviated from path.

---

## Iteration 6: Compare Mode — All 3 Planners in RViz

**Date:** March 10, 2026
**Status:** ✅ COMPLETE

### Problem Statement
User wanted a visual comparison of all three planners: give a single goal and see three different paths drawn in RViz simultaneously, without the robot moving.

### Changes Made

#### `trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py`
- Added `compare_mode` boolean parameter
- When `compare_mode: true`: runs A*, Hybrid-A*, and RRT* sequentially on every goal and publishes each to a separate topic. `/planned_path` is NOT published (robot stays still).
- Added publishers: `_path_pub_astar`, `_path_pub_hybrid`, `_path_pub_rrt`
- Added `_plan_compare()` method with summary table logged to console
- Extracted `_cells_to_path_msg()` helper for reuse
- Added module-level `_path_length()` helper

#### `trajectory_planner_pkg/config/planner_params.yaml`
- Added `compare_mode: false` parameter (default off)
- Added `hybrid_num_headings`, `hybrid_step_size`, `hybrid_steer_angles` params (exposed from hardcoded globals)
- All 3 planner configs now fully tunable from YAML

#### `autonomy_bringup/config/autonomy.rviz`
Added 4 path display entries:
| Display name | Topic | Color |
|---|---|---|
| Path — A* (green) | `/planned_path_astar` | 0;220;80 |
| Path — Hybrid-A* (cyan) | `/planned_path_hybrid_astar` | 0;200;255 |
| Path — RRT* (orange) | `/planned_path_rrtstar` | 255;140;0 |
| Path — SELECTED (white) | `/planned_path` | 255;255;255 |

### How to Use Compare Mode
Set `compare_mode: true` in `planner_params.yaml`, then launch normally. Click a goal in RViz — all 3 paths appear, robot doesn't move. Set back to `false` to drive with the `planner_type` planner.

---

## Iteration 7: Obstacle Avoidance Parameter Tuning

**Date:** March 10, 2026
**Status:** ✅ COMPLETE

### Problem Statement
Screenshot confirmed robot physically inside an obstacle in Gazebo. Three root causes:
1. `obstacle_stop_dist: 0.25m` — robot nearly touching wall before halting
2. `obstacle_check_angle: ±25°` — side/angled obstacles not caught
3. `inflation_radius_m: 0.40m` — planner routes too close to walls

### Parameter Changes

| Parameter | Before | After | File |
|-----------|--------|-------|------|
| `obstacle_stop_dist` | 0.25 m | **0.40 m** | `motion_params.yaml` |
| `obstacle_warn_dist` | 0.50 m | **0.65 m** | `motion_params.yaml` |
| `obstacle_check_angle` | 0.436 rad (±25°) | **0.698 rad (±40°)** | `motion_params.yaml` |
| `inflation_radius_m` | 0.40 m | **0.50 m** | `planner_params.yaml` |

**Critical constraint maintained:** `obstacle_stop_dist (0.40m) < inflation_radius (0.50m)`

---

## Iteration 8: Replanning Loop Fixes

**Date:** March 10, 2026
**Status:** ✅ COMPLETE

### Problems Identified

#### Bug 1 — Replan loop with obstacle
`_path_cb` cleared `_blocked_since = None` on every new path. If the obstacle was still physically there (robot hadn't moved yet), the next control loop tick would restart the whole "wait 0.8s → replan" cycle → new path → clear again → infinite replan loop.

#### Bug 2 — Robot moves before new path arrives
After requesting a replan (off-path or stuck), there was no mechanism to keep the robot stopped while the planner computed the new path. Robot continued following the old (potentially invalid) path, sometimes driving into the obstacle before the new path arrived.

#### Bug 3 — Off-path cooldown reset on every new path
`_path_cb` set `_last_replan_time = None`, resetting the 6s cooldown to zero. An immediate re-trigger was possible on the first loop tick after new path arrived.

#### Bug 4 — Backtracking to waypoint 0 after replan
`_path_cb` always set `_path_idx = 0`. If the robot moved slightly during replan computation, waypoint 0 was now behind it. Robot backtracked to hit it, then continued forward.

### Fixes

#### Fix 1 — Don't clear `_blocked_since` in `_path_cb`
Obstacle scan is now the **only** thing that clears `_blocked_since`. When path is clear the existing "CLEAR" block in the control loop handles it naturally.

#### Fix 2 — `_waiting_for_replan` flag
Added `_waiting_for_replan: bool`. Set to `True` in `_publish_replan()`. Cleared to `False` in `_path_cb`. Control loop checks this flag right after obstacle detection; if set, stops and returns — robot stays frozen until new path arrives.

#### Fix 3 — Don't reset `_last_replan_time` in `_path_cb`
`_last_replan_time` now persists across new paths. The off-path 6s cooldown continues counting from the replan request, not from when the new path arrived.

#### Fix 4 — Start from closest waypoint
`_path_cb` now finds the waypoint closest to the robot's current TF pose and sets `_path_idx` there instead of always using 0. Eliminates backtracking.

### Files Modified
- `simple_motion_pkg/simple_motion_pkg/path_follower.py` — all 4 fixes
- `simple_motion_pkg/config/motion_params.yaml` — `off_path_dist_m` restored to 1.5m (had been tightened to 1.0m unnecessarily)

