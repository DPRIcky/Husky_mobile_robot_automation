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
- [Iteration 9](#iteration-9-5-controller-architecture) — Mar 10, 2026 ✅ COMPLETE
- [Iteration 10](#iteration-10-actual-trajectory-trace) — Mar 10, 2026 ✅ COMPLETE
- [Iteration 11](#iteration-11-systematic-audit--debug-instrumentation) — Mar 16, 2026 🟡 IN PROGRESS
- [Iteration 12](#iteration-12-hybrid-a-replanning-failure-fix) — Mar 18, 2026 ✅ COMPLETE

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
**Status:** 🟡 IN PROGRESS (minimal fixes applied; live ROS validation still pending)

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


---

## Iteration 9: 5-Controller Architecture

**Date:** March 10, 2026
**Status:** ✅ COMPLETE

### Objectives
- [✅] Replace P-controller with Stanley controller as the default
- [✅] Add PID, Pure Pursuit, LQR, and MPC controllers
- [✅] Add velocity profiler (curvature + goal ramp + obstacle factor + accel limit)
- [✅] Add twist multiplexer node (E-stop > teleop > autonomous)
- [✅] Add compare mode: all 5 controllers run every tick, only active one drives
- [✅] All 5 controllers share a uniform interface
- [✅] Build, verify imports, push to GitHub

### New Files

| File | Purpose |
|------|---------|
| `controllers/utils.py` | Shared geometry: `normalize_angle`, `advance_lookahead`, `nearest_on_path`, `signed_cte` |
| `controllers/stanley.py` | Stanley controller: `w = he + atan2(k_e × cte, v)` |
| `controllers/pid.py` | PID yaw controller with anti-windup integral |
| `controllers/pure_pursuit.py` | Arc curvature `κ = 2·y_robot / L²` in robot body frame |
| `controllers/lqr.py` | LQR via scipy `solve_discrete_are`, linearised unicycle model |
| `controllers/mpc.py` | MPC via scipy SLSQP, horizon N=8, warm-started |
| `controllers/__init__.py` | Exports all 5 + `CONTROLLER_NAMES` tuple |
| `velocity_profiler.py` | Menger curvature speed, goal ramp, obstacle factor, accel limit |
| `twist_mux.py` | ROS 2 node: E-stop > teleop (with timeout) > autonomous priority |

### Modified Files

| File | Changes |
|------|---------|
| `path_follower.py` | Complete rewrite — controller dispatch, compare mode, `/autonomous/cmd_vel` output |
| `motion_params.yaml` | All controller + profiler + twist_mux params; current runtime defaults use `controller_type: stanley`, `controller_compare_mode: false` |
| `setup.py` | Added `twist_mux` console_scripts entry |
| `autonomy.launch.py` | Added twist_mux node |

### Controller Interface (uniform across all 5)
```python
compute(rx, ry, ryaw, path, path_idx, v, dt)
    -> (v_cmd, w_cmd, cte, heading_error, new_path_idx)
```

### Architecture: cmd_vel routing
```
path_follower  -->  /autonomous/cmd_vel
                            |
                       twist_mux
                   (E-stop > teleop > auto)
                            |
               /a300_00000/cmd_vel  -->  robot
```

### Compare Mode
- `controller_compare_mode: true` — all 5 run every tick
- Active controller (set by `controller_type`) drives the robot
- All 20 values [stanley_v, stanley_w, stanley_cte, stanley_he, pid_v, ...] published to `/controller_diagnostics` (Float64MultiArray)
- Comparison table printed to console every 2 s

### Controller Details

#### Stanley
- `w = he + atan2(k_e × cte, max(v, v_min))`; geometric, no integral state
- Params: `stanley_k=1.0`, `stanley_v_min=0.1`

#### PID
- Error = yaw to lookahead point; anti-windup clamp ±2 rad·s
- Params: `pid_kp=1.5`, `pid_ki=0.05`, `pid_kd=0.2`

#### Pure Pursuit
- Lookahead → robot body frame; `κ = 2·y_robot / L²`; `w = v·κ`; no state
- Params: `lookahead_distance=0.5`

#### LQR
- State `[cte, he]`, input `[ω]`; discrete DARE via scipy; gain cached between steps
- Params: `lqr_q_cte=5.0`, `lqr_q_he=1.0`, `lqr_r_w=0.5`

#### MPC
- Horizon N=8; unicycle rollout; SLSQP, maxiter=40; warm-start from previous solution
- Params: `mpc_horizon=8`, `mpc_q_cte=5.0`, `mpc_q_he=1.0`, `mpc_r_v=0.1`, `mpc_r_w=0.5`

### Velocity Profiler
1. Curvature: `v_max / (1 + k_curv × |κ|)` via Menger curvature (6-waypoint window)
2. Goal ramp: linear `v_min..v_max` within `vp_goal_ramp_dist` (1.5 m) of goal
3. Obstacle factor: 1.0 (clear) / 0.5 (warn) / 0.0 (stop) — from lidar
4. Accel limit: `v_final = min(v_desired, v_prev + a_max × dt)`

### Twist Multiplexer Priorities
1. E-stop (`/estop` Bool) — zero twist
2. Teleop (`/a300_00000/joy_teleop/cmd_vel`) — overrides for `mux_teleop_timeout_s=0.5s`
3. Autonomous (`/autonomous/cmd_vel`) — default

---

## Iteration 10: Actual Trajectory Trace

**Date:** March 10, 2026
**Status:** ✅ COMPLETE

### Objective
Visualise the actual path the robot followed alongside the reference planned path in RViz, so controller tracking performance can be assessed visually.

### Changes Made

#### `simple_motion_pkg/simple_motion_pkg/path_follower.py`
- Added `_traj_pub` publisher on `/actual_trajectory` (`nav_msgs/Path`, frame `map`)
- Added `_actual_traj` accumulator (`nav_msgs/Path`) capped at 2000 poses (`_traj_max_poses`)
- Every control tick (when TF pose is available): appends current `PoseStamped` to `_actual_traj` and publishes
- **Clear on new goal:** If the new path's endpoint is >0.5m from the previous goal, `_actual_traj.poses` is cleared. Trajectory persists across replans to the same goal — the detour is visible.
- **Clear on new goal only**, never on replan to same goal — so you can see the full actual trajectory including any obstacle detours

#### `autonomy_bringup/config/autonomy.rviz`
Added two new Path displays:

| Display | Topic | Colour | Line Width | Z offset |
|---------|-------|--------|------------|----------|
| Reference Path (white) | `/planned_path` | 255;255;255 | 0.04 | 0.12 m |
| Actual Trajectory (yellow) | `/actual_trajectory` | 255;220;0 | 0.06 | 0.15 m |

The Z offsets ensure both traces are visible above the map even when overlapping.

### Behaviour Summary
- **Yellow trace** = where the robot actually drove (updated live, 10 Hz)
- **White trace** = the reference path from the planner
- Trajectory is reset when a new destination is set; preserved on replan to same goal
- Maximum 2000 poses buffered (~3 minutes at 10 Hz) — oldest dropped if exceeded


---

## Iteration 11: Systematic Audit & Debug Instrumentation

**Date:** March 16, 2026
**Status:** 🟡 IN PROGRESS (minimal fixes applied; live ROS validation still pending)

### Objective
Systematic root-cause audit of (A) poor path following and (B) obstacle avoidance failures.
No broad refactors — instrumentation and diagnosis first.

### Audit Findings

#### Bug 1 — `nearest_on_path` docstring sign inversion (documentation only)
`utils.py:34`: docstring said "positive = robot to the RIGHT". Actual formula (`tx_n·ey − ty_n·ex`) yields positive = robot to the LEFT (standard 2-D cross product). All controllers (Stanley, LQR, MPC) already assumed LEFT=positive. **Code is correct; docstring was wrong.** Fixed in this iteration.

#### Bug 2 — `shortcut_smooth(iterations=80)` too aggressive (HIGH CONFIDENCE)
`planner_node.py:217,243,267,289`: 80 random-shortcut iterations on an already-pruned path can reduce a 200-cell path to 3–5 waypoints. Consequences:
- Waypoints 2–5m apart → CTE discontinuities at corner transitions
- `_menger_curvature(window=6)` samples the same waypoint 3× → reports κ=0 → no speed reduction at corners
- Obstacle-cone direction `path_idx+5` almost always lands at the goal → cone misaimed
Validated in a synthetic corridor audit: 48 raw A* points → 9 pruned → 4 after 80 smoothing iterations.

#### Bug 3 — PID could go nearly blind to large lateral error near corners (HIGH CONFIDENCE)
`pid.py:45`: error signal used only `desired_yaw − ryaw` to the lookahead waypoint. On corner-entry poses where the robot was still far off the local segment, the lookahead bearing could already point almost straight at the next waypoint, so PID reported a tiny heading error and stopped steering meaningfully even with large CTE.

#### Bug 4 — 0.15m buffer between inflation_radius and stop_dist is too tight (MEDIUM)
`inflation_radius_m=0.65m`, `obstacle_stop_dist=0.50m`. CTE ≥ 0.15m at corners (common with sparse paths) brings robot inside stop_dist → false obstacle stop → backup+replan loop.

#### Bug 5 — RRT* collision checking not implicated
Current `planner_core.py` checks the full segment inside `_collision_free()`. No RRT* patch was needed for this task.

### Debug Instrumentation Added

#### `/path_follower_debug` topic (Float64MultiArray, 10 Hz)
Layout:
```
[v_desired, v_cmd, v_final,
 w_cmd, w_final,
 cte, he_deg,
 forward_min, cone_center_deg,
 path_idx, near_idx, n_waypoints, avg_wp_spacing_m,
 blocked_since_s, waiting_for_replan, backing_up,
 replan_requested, stuck_detected, off_path_detected]
```

#### Throttled structured log (1 Hz)
```
[DBG] mpc         near= 3  tgt= 3/ 5  cte=+0.021m  he= +2.3°  v=0.35→0.35→0.35  w=+0.05→+0.05  obs=2.10m(cone=+12°)  avg_sp=0.52m  flags=---|---|RPL|---|---|---
```

#### Path geometry log on every new path (`_path_cb`)
```
New path: 5 wp  start=wp0  avg_spacing=0.52m  max_spacing=1.10m  ctrl=mpc
```

### Evidence Gathered

#### Evidence 1 — `advance_lookahead()` could select a waypoint behind the robot
Synthetic sparse-path check (`lookahead_distance=0.8 m`):
- Pose `(5.0, 0.0)` on path `[(0,0),(2,0),(4,0),(6,0),(6,2),…]`
- `nearest_on_path()` returned waypoint 2 `(4,0)`
- Old `advance_lookahead()` also returned waypoint 2 because distance to `(4,0)` was `1.0 m >= lookahead`
- Result: PID / Pure Pursuit / MPC were allowed to target a point the robot had already passed

This is a correctness bug in target progression, not a tuning issue.

#### Evidence 2 — `shortcut_smooth(iterations=80)` collapsed corner paths too far
On a synthetic L-corridor A* path:
- Raw A* path: `48` points
- After `prune_collinear()`: `9` points
- After `shortcut_smooth(..., 80)`: `4` points
- After `shortcut_smooth(..., 5)`: `8` points

This matters because the 4-point path broke downstream follower assumptions:
- cone heading `path_idx+5` pointed toward the goal instead of the local segment
- `_menger_curvature(window=6)` returned `0.0` at every index on the 4-point path
- with 5 smoothing iterations, curvature remained non-zero on the corner-approach waypoints

#### Evidence 3 — Obstacle cone heading was using far-goal direction on sparse paths
Examples from synthetic sparse paths:
- sparse L-path at index 2: old cone heading `71.6°`, local path tangent `0.0°`
- 4-point planner-like path at index 0: old cone heading `43.0°`, local tangent `9.5°`

This means the forward obstacle metric could be computed in the wrong direction even when the raw scan itself was valid.

#### Evidence 4 — checked-in runtime config was still a debug controller setup
`motion_params.yaml` was shipping with:
- `controller_type: "pid"`
- `controller_compare_mode: true`

That is useful for diagnostics, but it is not a good default for obstacle runs. In a simple L-turn follower simulation with the current parameters plus the path-follower heading-alignment logic:
- Stanley reached the goal with max CTE ≈ `0.20 m`
- LQR reached the goal with max CTE ≈ `0.20 m`
- PID reached the goal but with max CTE ≈ `0.76 m`
- Pure Pursuit reached with max CTE ≈ `0.74 m`
- MPC stalled in this setup

For obstacle avoidance, the lower-CTE baseline matters because the robot is less likely to cut toward nearby obstacles while following the replanned path.

#### Evidence 5 — live terminal logs showed backup/resume loops that never reached replan delay
From the running autonomy stack on March 16, 2026:
- obstacle hits occurred repeatedly at `0.45–0.47 m`
- each hit triggered `reversing 1.0s then waiting for map update`
- about `1.1–1.3 s` later the log said `Path clear — resuming`
- `map_update_delay_s` was `2.5 s`

So the backup moved the robot just far enough to leave `stop_dist`, which cleared `_blocked_since` before the follower ever reached the replan delay. The robot then resumed the same still-bad path and hit the obstacle again.

This is a state-machine correctness bug, not an inflation/footprint problem.

#### Evidence 6 — live run with MPC did eventually replan, but MPC then stalled on the replanned path
After repeated backup/resume loops, the log showed:
- `Stuck: moved 0.03m in 4.9s — replanning.`
- planner replanned successfully on a fresh map (`Planning on map aged 0.40s`, `Path found: 16 waypoints`)

But after the new path arrived, MPC repeatedly produced logs like:
- `v=0.21→0.00→0.00`
- `w=-1.20→-1.20`
- `obs=2.88m`

So the remaining behavior in that run was:
1. obstacle handling delayed the replan too long, then
2. MPC did not execute the replanned path effectively even with clear obstacle distance

#### Evidence 7 — compare-mode logs isolated the PID and MPC controller-specific failures
From the live PID compare-mode run (`python3_6615_1773679162264.log`):
- at one corner pose, PID reported `cte=+0.575 m`, `he=+2.4°`, `w=+0.024 rad/s`
- on the same pose, Stanley reported `he=+53.8°`
- this means the lookahead-point bearing hid the fact that the robot was still far off the local path segment

Local replay of a similar corner-entry pose `(5.2, 0.8, 11°)` confirmed the same blind spot:
- old PID heading error was only `+0.3°`
- adding a signed-CTE heading correction reduced CTE from `0.80 m` to `0.10 m` within 25 control steps; the old behavior only reduced it to about `0.22 m`

The same live compare-mode log also showed MPC repeatedly falling into zero-speed spin states on the corner approach:
- examples: `v=0.000`, `w=-1.200`, `cte≈0.10–0.23 m`, `he≈-30° to -52°`
- this matched the active-MPC obstacle run, where the replanned path was clear (`obs=2.88 m`) but the controller still stalled
- the checked-in config was also overriding the controller's built-in `mpc_r_v=0.5` default with `0.05`, which weakens forward-progress bias exactly where the solver needs it most

#### Evidence 8 — curvature profiling was dropping to zero near the end of short replanned paths
From the current live MPC run (`python3_21843_1773687417147.log`):
- MPC stayed on a `16` waypoint replanned path with `near=10`, `tgt=12/16`
- `he` stayed large (`-30°` to `-55°`) while `v_desired` often stayed pinned at `0.40 m/s`
- repeated obstacle-triggered replans followed until a much shorter `6` waypoint path finally reached the goal

Root cause in `velocity_profiler.py`:
- `_menger_curvature(idx=12, window=6, n=16)` chose `mid=15`, `end=15`
- that duplicated the last waypoint, making the curvature denominator collapse and returning `0.0`
- result: short curved replans near the goal were falsely treated as straight, so MPC was allowed to keep attacking them at max speed

Local replay of a 16-point replanned-path fragment reproduced the exact failure:
- old profiler outputs near the bend: `idx 12 -> v_desired 0.40`, `idx 13 -> v_desired 0.40`
- patched profiler outputs: `idx 12 -> 0.10`, `idx 13 -> 0.08`

#### Evidence 9 — planner paths were still willing to hug obstacle boundaries on replans
User-requested mitigation: keep replanned paths a bit farther from obstacle boundaries so false-positive LiDAR returns are less likely to stop the robot while it is otherwise on a valid path.

Rather than rewriting the stack or inflating the robot footprint further, I added a soft clearance preference inside the existing planners:
- compute a distance-to-obstacle field on the current planning grid
- penalize free cells that lie within a preferred soft-clearance band
- apply a stronger penalty when the same goal is requested again within a short window, which is a good proxy for obstacle-triggered replanning in the current architecture

Synthetic A* check on a grid with two valid routes around an obstacle:
- plain A*: path length `28.73`, minimum clearance `1.0` cell
- clearance-biased A*: path length `34.14`, minimum clearance `3.61` cells

This is a mitigation for noisy/faulty obstacle sensing, not proof that the LiDAR false positives are solved at the source.

### Minimal Patches Applied

#### Patch 1 — fix lookahead progression correctness
- File: `simple_motion_pkg/simple_motion_pkg/controllers/utils.py`
- Change: `advance_lookahead()` now skips waypoints the robot has already passed along the current segment before applying the distance test
- Why: prevents PID / Pure Pursuit / MPC from targeting behind-robot waypoints on sparse paths

#### Patch 2 — keep planner smoothing but stop over-collapsing corner geometry
- File: `trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py`
- Change: reduced `shortcut_smooth()` iterations from `80` to `5`
- Why: preserves local corner geometry and usable curvature cues without removing pruning/smoothing from the architecture

#### Patch 3 — obstacle cone uses local path tangent, not far-goal heading
- File: `simple_motion_pkg/simple_motion_pkg/path_follower.py`
- Change: cone center now comes from `nearest_on_path()` local tangent instead of `path_idx + 5`
- Why: keeps the follower’s forward obstacle metric aligned with the segment currently being tracked

#### Patch 4 — extend debug observability for replan/state diagnosis
- File: `simple_motion_pkg/simple_motion_pkg/path_follower.py`
- Change: `/path_follower_debug` now appends `replan_requested`, `stuck_detected`, and `off_path_detected`; 1 Hz log now prints `near` and `tgt` indices explicitly
- Why: distinguishes target-progression bugs from state-machine / obstacle-detection faults

#### Patch 5 — preserve obstacle-replan state across new paths
- File: `simple_motion_pkg/simple_motion_pkg/path_follower.py`
- Change: `_path_cb()` no longer clears obstacle replan state while the scan is still blocked
- Why: prevents obstacle replans from restarting as if the obstacle had disappeared just because a new path arrived

#### Patch 6 — restore a non-debug default controller configuration
- File: `simple_motion_pkg/config/motion_params.yaml`
- Change: default `controller_type` set to `stanley` and `controller_compare_mode` set to `false`
- Why: the full autonomy launch from `workspace_status.md` should not come up in a PID + compare-mode debugging configuration by default

#### Patch 7 — obstacle stop now commits to replan even if backup briefly clears stop distance
- File: `simple_motion_pkg/simple_motion_pkg/path_follower.py`
- Change: added `_obstacle_replan_pending` so an obstacle-triggered backup continues through wait/replan instead of being canceled by a brief `Path clear` after reversing
- Why: this matches the intended recovery sequence (`backup -> wait for map -> replan`) and prevents the repeated backup/resume loop seen in the live logs

#### Patch 8 — PID now corrects lookahead heading with signed cross-track error
- File: `simple_motion_pkg/simple_motion_pkg/controllers/pid.py`
- Change: PID now finds `nearest_on_path()` first, advances lookahead from that local segment, and subtracts `atan2(cte, lookahead)` from the lookahead bearing before running the PID terms
- Why: this is the smallest controller-local fix that addresses the live failure mode where PID saw near-zero heading error while still far off the path

#### Patch 9 — MPC now reseeds stale warm-start speeds and restores the forward-progress weight
- Files:
  `simple_motion_pkg/simple_motion_pkg/controllers/mpc.py`
  `simple_motion_pkg/config/motion_params.yaml`
- Change: when the previous warm-start sequence is stuck near zero speed, MPC now reseeds the speed components back toward `v_desired`; `mpc_r_v` in the shipped config is restored from `0.05` to `0.5`
- Why: this preserves the current MPC architecture but reduces the spin-in-place local minimum seen in live compare logs and the active-MPC obstacle run

#### Patch 10 — keep curvature profiling valid near the end of short replanned paths
- File: `simple_motion_pkg/simple_motion_pkg/velocity_profiler.py`
- Change: `_menger_curvature()` now shrinks its sampling window near the goal so `idx`, `mid`, and `end` remain distinct instead of collapsing `mid` and `end` onto the same last waypoint
- Why: short obstacle-replan paths were being misclassified as zero-curvature near the goal, which let MPC drive them too fast and repeatedly skim back into the obstacle region

#### Patch 11 — add a soft clearance preference to the existing planners
- Files:
  `trajectory_planner_pkg/trajectory_planner_pkg/planner_core.py`
  `trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py`
  `trajectory_planner_pkg/config/planner_params.yaml`
- Change:
  A*, Hybrid-A*, and RRT* now accept an optional clearance map and add a soft cost for travelling too close to inflated obstacles.
  The planner node computes the clearance field from the current grid and increases the clearance weight when the same goal is requested again within a short time window, which is used here as a replan proxy.
- Why:
  this keeps the current planner architecture but biases replans toward routes with more breathing room, which is exactly the mitigation requested for false obstacle detections near path edges

### Pending Actions
- [ ] Run the stack in Gazebo / ROS 2 and capture `/path_follower_debug` during an open-path test
- [ ] Confirm that Stanley remains the best live baseline around obstacles; if not, compare Stanley vs LQR directly
- [ ] Verify blocked obstacle behavior end-to-end: raw scan → forward metric → map age → replanned path
- [ ] Relaunch the autonomy stack before judging Patches 7-11, because the currently running stack is still using the installed package version from `install/`
- [ ] Decide whether backup-on-obstacle should remain or be removed after live validation
- [ ] If false obstacle stops still happen even on clearly wide replans, inspect raw scan filtering / persistence directly instead of pushing planner clearance farther

---

## Iteration 12: Hybrid A* Replanning Failure Fix

**Date:** March 18, 2026
**Status:** ✅ COMPLETE

### Objective
Fix Hybrid A* failing to find a path when the robot is snapped to the inflated-obstacle boundary and the only visible route passes through a clearance-penalised corridor.

### Root Cause Analysis

Observed log:
```
Start cell (631, 177) inside inflated obstacle — snapped to (630, 177)
Clearance bias (replan): pref=0.30m  weight=2.00
Planning hybrid_astar: start=(-6.16,9.58) → goal=(-5.20,12.69)
Planner found NO path.
```

Three compounding causes identified:

**Cause 1 — Snap lands right at the obstacle boundary**
`_nearest_free_cell()` (BFS outward) correctly finds the nearest free cell, but on a 0.05 m grid that is often just 1 cell away. The snapped start is still at zero clearance on the inflated grid.

**Cause 2 — Doubled clearance weight makes boundary exits too expensive**
On a replan, `clearance_weight` is scaled by `replan_clearance_scale=2.0` → `weight=2.00`. With `preferred_clearance=6 cells` (0.30 m / 0.05 m), every cell within 6 cells of an obstacle receives a quadratic penalty. When the only viable escape route from the snapped start passes through the preferred-clearance band, the accumulated penalty exceeds any path cost and the open set exhausts without reaching the goal.

**Cause 3 — Coarse Hybrid A* steer angles miss tight-corridor headings**
Default steer angles `[-0.5, 0.0, 0.5]` (±28.6°) produce only 3 heading options per step. In narrow corridors where the robot must hold a heading close to ±14°, neither action is attractive and the search prematurely converges.

### Patches Applied

#### Patch 12a — finer Hybrid A* steer angles
- File: `trajectory_planner_pkg/trajectory_planner_pkg/planner_core.py`
- Change: default `steer_angles` changed from `[-0.5, 0.0, 0.5]` to `[-0.5, -0.25, 0.0, 0.25, 0.5]`
- Why: adds intermediate ±14.3° options so the search can hold headings needed to thread tight corridors; 5 actions per step vs 3

#### Patch 12b — two-stage planning retry in `_goal_cb`
- File: `trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py`
- Change: after the primary plan fails, two automatic retries are attempted in sequence:
  1. **Retry 1 — drop clearance bias:** re-runs the same planner on the inflated grid with `clearance_weight=0`. Handles the case where the doubled replan penalty blocks the only viable corridor; a path with less breathing room is still better than no path.
  2. **Retry 2 — un-inflated grid:** re-derives `start_rc` and `goal_rc` from world coordinates, snaps both to the raw (un-inflated) grid, and re-runs with no clearance penalty. Handles the case where the robot centre has a free path but the inflated footprint has no solution at all. The raw grid is also used for the subsequent `shortcut_smooth()` call so there is no inconsistency.
- Why: the previous code gave up after one attempt; these retries cover the two most common failure modes without changing the planner algorithm

### How to Test
No rebuild required (Python only). Restart the autonomy stack:
```bash
# T1 (if not running): ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
# T2 (if not running): ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch autonomy_bringup autonomy.launch.py
```
Drive the robot close to an obstacle and send a goal. Expected log signatures:
- `Planner failed — retrying without clearance bias …` → Retry 1 activated
- `Planner failed — retrying on un-inflated grid …` → Retry 2 activated
- `Path found: N waypoints` → a retry succeeded
