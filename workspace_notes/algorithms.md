# Algorithms & Controllers — Technical Reference

**Project:** Clearpath A300 Autonomous Navigation
**Last Updated:** March 10, 2026 (Iteration 9)

This document describes every planning algorithm and controller implemented in the custom autonomy stack (`trajectory_planner_pkg` + `simple_motion_pkg`).

---

## System Architecture Overview

```
/goal_pose (PoseStamped)
        │
        ▼
┌─────────────────────────┐    /planned_path            (single planner mode)
│  trajectory_planner_pkg │ ──────────────────────────────────────────────────┐
│  (A* / Hybrid-A* / RRT*)│    /planned_path_astar      (compare mode only)   │
│                         │    /planned_path_hybrid_astar                      │
│  compare_mode=false:    │    /planned_path_rrtstar                           │
│    single planner runs  │                                                    ▼
│  compare_mode=true:     │                                   ┌──────────────────────────────┐
│    all 3 run, no cmd_vel│                                   │   simple_motion_pkg          │
└─────────────────────────┘                                   │   path_follower              │
         ▲                                                     │   (Stanley/PID/PP/LQR/MPC + │
         │ replan requests (/goal_pose)                        │    VelocityProfiler +        │
         │◄──────────────────────────────────────────────────│    obstacle avoidance)        │
/a300_00000/map (OccupancyGrid) ──► planner                   └──────────────────────────────┘
/a300_00000/sensors/lidar2d_0/scan ──► follower                        │
                                                  /autonomous/cmd_vel  │
                                                                        ▼
                                                            ┌──────────────────┐
                                                            │   twist_mux      │
                                                            │ E-stop > teleop  │
                                                            │   > autonomous   │
                                                            └──────────────────┘
                                                                        │
                                                       /a300_00000/cmd_vel (TwistStamped)
                                                                        │
                                                                     robot
```

---

## 1. Trajectory Planner (`trajectory_planner_pkg`)

### 1.1 Map Preprocessing Pipeline

**File:** `trajectory_planner_pkg/grid_utils.py`

#### Step 1 — OccupancyGrid to Binary Grid
The ROS `OccupancyGrid` (int8, 0-100, -1=unknown) is converted to a binary NumPy array:
- Cells with value `>= occupied_threshold` (default 85) → **obstacle (1)**
- Unknown cells (-1) → **free (0)** when `allow_unknown=true` (avoids detours through unexplored space during SLAM)
- All other cells → **free (0)**

> **Why threshold=85?** SLAM Toolbox generates noise values in the 65–80 range. Real obstacles hit 100. A threshold of 85 cleanly separates them.

#### Step 2 — Obstacle Inflation
Obstacles are expanded using `scipy.ndimage.binary_dilation` with a **circular structuring element** of radius `inflation_radius_m` (default 0.40 m):

```
radius_cells = ceil(inflation_radius_m / map_resolution)
```

This ensures the planner keeps the robot (half-width ~0.33 m) away from walls. The inflated grid is used for all planning; the raw un-inflated grid is used as a fallback if the robot's own start cell falls inside the inflated zone.

#### Step 3 — Goal Cell Snapping
If the clicked goal cell falls inside an inflated obstacle, a **BFS outward search** (`_nearest_free_cell`) finds the nearest free cell within a 20-cell search radius. This prevents silent planner failures when the user clicks near a wall.

---

### 1.2 A* Planner (default)

**File:** `trajectory_planner_pkg/planner_core.py` — `astar()`

**Algorithm:** Standard A* on a 2D grid with **8-connected neighbours**.

**Data structures:**
- `open_set`: min-heap of `(f_score, (row, col))`
- `g_score`: dict mapping cell → cost-from-start
- `came_from`: dict for path reconstruction

**Heuristic — Octile distance:**
```
h(a, b) = √2 × min(Δrow, Δcol) + |Δrow − Δcol|
```
Octile distance is admissible and consistent for 8-connected grids. It is more accurate than Euclidean (which underestimates diagonal costs) and tighter than Chebyshev.

**Edge costs:**
- Cardinal move (N/S/E/W): **1.0**
- Diagonal move: **√2 ≈ 1.414**

**Complexity:** O((W × H) log(W × H)) in the worst case.

**When to use:** Default for all planning. Fast, complete, and optimal on the inflated binary grid.

---

### 1.3 Hybrid A* Planner (optional)

**File:** `trajectory_planner_pkg/planner_core.py` — `hybrid_astar()`

**Algorithm:** Hybrid A* planning in continuous (x, y, θ) space while using the discrete grid for collision checking.

**State space:** `(row, col, heading_bin)` where heading is discretized to **72 bins** (5° resolution).

**Motion model:** At each step, 3 steering actions are applied:
```
steer ∈ {-0.5, 0.0, +0.5}  rad
new_θ = θ + steer
new_pos = pos + step_size × (cos(new_θ), sin(new_θ))
```

**Heuristic:** Euclidean distance to goal (ignoring heading).

**Goal tolerance:** 2 cells spatial + 1 heading bin.

**Configurable parameters (planner_params.yaml):**
- `hybrid_num_headings` (default 72) — heading resolution
- `hybrid_step_size` (default 1.0) — step length in grid cells
- `hybrid_steer_angles` (default [-0.5, 0.0, 0.5]) — steering options in radians

**When to use:** When path curvature matters (e.g., tight corridors, car-like robots). Currently set via `planner_type: hybrid_astar` in `planner_params.yaml`.

---

### 1.4 RRT* Planner (optional)

**File:** `trajectory_planner_pkg/planner_core.py` — `rrt_star()`

**Algorithm:** Rapidly-exploring Random Tree with rewiring (RRT*) — asymptotically optimal.

**Key parameters:**
| Parameter | Value | Purpose |
|-----------|-------|---------|
| `max_iter` | 8000 | Maximum tree nodes |
| `step_size` | 5.0 cells | Max extension per step |
| `goal_sample_rate` | 0.10 | 10% chance to sample goal directly |
| `search_radius` | 10.0 cells | Rewire neighbourhood radius |

**Algorithm steps per iteration:**
1. **Sample** a random cell (or goal with probability 0.10)
2. **Nearest** node: brute-force O(n) nearest-neighbour search
3. **Steer**: extend at most `step_size` toward sample
4. **Collision check**: linear interpolation along the segment
5. **Choose parent**: find best parent within `search_radius` (min cost)
6. **Rewire**: update any neighbour that can be reached cheaper via new node
7. **Goal check**: connect to goal if within `step_size` and collision-free

**Path extraction:** Trace `parent` pointers from best goal node to root.

**When to use:** Complex environments with many narrow passages where A* may find suboptimal paths. Slower than A* (probabilistic, not guaranteed to find path within iteration limit).

---

### 1.5 Path Post-Processing

Applied to the raw planner output before publishing:

#### Collinear Pruning (`prune_collinear`)
Removes intermediate waypoints that lie on the same line as their neighbours. Uses the 2D cross-product magnitude to measure deviation from collinearity (tolerance: 0.5 cells).

**Before:** 50 points along a straight corridor
**After:** 2 points (start + end of segment)

#### Shortcut Smoothing (`shortcut_smooth`)
Iteratively tries random shortcuts: pick two path indices i < j, check if the direct line between them is collision-free, and if so remove all intermediate points. Runs for **80 iterations**.

This reduces path length and produces smoother trajectories for the follower.

---

## 2. Path Follower (`simple_motion_pkg`)

**Files:** `simple_motion_pkg/path_follower.py`, `controllers/`, `velocity_profiler.py`, `twist_mux.py`

### 2.0 Controller Architecture

All 5 controllers share a uniform interface:
```python
compute(rx, ry, ryaw, path, path_idx, v, dt)
    -> (v_cmd, w_cmd, cte, heading_error, new_path_idx)
```

The `VelocityProfiler` computes the desired speed `v` before calling the active controller. The controller then scales `v` further (e.g., by `cos(heading_error)`) and computes `w`.

**Compare mode** (`controller_compare_mode: true`): all 5 controllers run every tick. Only the one selected by `controller_type` drives the robot. All 20 values `[v,w,cte,he] × 5` are published to `/controller_diagnostics`.

### 2.1 Stanley Controller

**File:** `controllers/stanley.py`

```
nearest_on_path → (cte, path_heading)
heading_error = normalize(path_heading − ryaw)
w = heading_error + atan2(k_e × cte, max(v, v_min))
v_cmd = v × max(0, cos(heading_error))
```

Geometric controller. Steering combines heading alignment and cross-track correction. No integral state — robust, deterministic.

**Parameters:** `stanley_k=1.0`, `stanley_v_min=0.1`

### 2.2 PID Controller

**File:** `controllers/pid.py`

```
target = advance_lookahead(path, path_idx, lookahead_distance)
error = normalize(atan2(ty−ry, tx−rx) − ryaw)
integral += error × dt  (clamped ±windup_limit)
derivative = (error − prev_error) / dt
w = kp×error + ki×integral + kd×derivative
v_cmd = v × max(0.1, 1 − |error|/π)
```

Anti-windup clamp prevents integral saturation during long straight sections.

**Parameters:** `pid_kp=1.5`, `pid_ki=0.05`, `pid_kd=0.2`, `pid_windup_limit=2.0`

### 2.3 Pure Pursuit Controller

**File:** `controllers/pure_pursuit.py`

```
target = advance_lookahead(path, path_idx, L)
# Transform to robot body frame:
x_robot =  (tx−rx)×cos(ryaw) + (ty−ry)×sin(ryaw)
y_robot = −(tx−rx)×sin(ryaw) + (ty−ry)×cos(ryaw)
κ = 2×y_robot / L²
w = v × κ
```

No internal state. Classic curvature-based controller.

**Parameters:** `lookahead_distance=0.5`

### 2.4 LQR Controller

**File:** `controllers/lqr.py`

Linearised unicycle at operating point (v, 0):
```
A = [[0, v], [0, 0]],  B = [[0], [1]]
Ad = I + A×dt,  Bd = B×dt
Solve: P = solve_discrete_are(Ad, Bd, Q, R)
K = (R + Bd'×P×Bd)^-1 × Bd'×P×Ad
u = -(K @ [cte, he])[0]
```

Gain recomputed only when `v` or `dt` changes beyond threshold (avoids DARE overhead every tick). Fallback `K = [[0, 2]]` if DARE fails (e.g., v ≈ 0).

**Parameters:** `lqr_q_cte=5.0`, `lqr_q_he=1.0`, `lqr_r_w=0.5`

### 2.5 MPC Controller

**File:** `controllers/mpc.py`

```
Optimise over U = [v_0, w_0, ..., v_{N-1}, w_{N-1}]:
  minimise: Σ (Q_cte×cte_k² + Q_he×he_k² + R_v×v_k² + R_w×w_k²)
  subject to: 0 ≤ v_k ≤ max_v,  -max_w ≤ w_k ≤ max_w
  unicycle model: x_{k+1} = x_k + v_k×cos(θ_k)×dt
                  y_{k+1} = y_k + v_k×sin(θ_k)×dt
                  θ_{k+1} = θ_k + w_k×dt
Solver: scipy SLSQP, maxiter=40
Warm start: shift previous solution 1 step forward
```

**Parameters:** `mpc_horizon=8`, `mpc_q_cte=5.0`, `mpc_q_he=1.0`, `mpc_r_v=0.1`, `mpc_r_w=0.5`

### 2.6 Velocity Profiler

**File:** `velocity_profiler.py`

Three factors combined multiplicatively:

```
1. Curvature:    v_curv = v_max / (1 + k_curv × |κ|)
                 κ computed via Menger formula over 6-waypoint window

2. Goal ramp:    v_goal = v_min + (v_max − v_min) × (dist/goal_ramp_dist)
                 applied when dist_to_goal < goal_ramp_dist (1.5 m)

3. Obstacle:     v_desired = min(v_curv, v_goal) × obstacle_factor
                 obstacle_factor: 1.0 (clear) | 0.5 (warn) | 0.0 (stop)

4. Accel limit:  v_final = min(v_desired, v_prev + a_max × dt)
                 (deceleration is instantaneous — safety stops not delayed)
```

**Parameters:** `vp_curvature_gain=3.0`, `vp_goal_ramp_dist=1.5`, `vp_min_vel=0.05`, `vp_accel_limit=0.5`

---

### 2.2 Obstacle Detection

**Source:** `/a300_00000/sensors/lidar2d_0/scan` (LaserScan, 40 Hz)

**Method:** Forward cone scan check

```
For each laser ray i:
    angle_laser = angle_min + i × angle_increment
    angle_base  = normalize(angle_laser + laser_yaw_offset)  ← TF-corrected
    if |angle_base| > check_angle (±25°): skip
    if r <= range_min or r >= range_max or not finite: skip
    forward_min = min(forward_min, r)
```

**TF correction:** The laser frame (`lidar2d_0_laser`) may be mounted at a yaw offset relative to `base_link`. This offset is looked up once via TF and cached as `_laser_yaw_offset`, ensuring the forward cone is always aligned with the robot's heading.

**Zones:**
| Zone | Distance | Action |
|------|----------|--------|
| Warn zone | < 0.65 m | Halve linear speed |
| Danger zone | < 0.40 m | Stop + trigger replan state machine |

> **Critical constraint:** `stop_dist (0.40 m) < inflation_radius (0.50 m)`.
> If stop_dist ≥ inflation_radius, the planner can legitimately plan paths past walls that the follower then stops at, causing infinite replanning loops.

---

### 2.3 Stop-and-Replan State Machine

Triggered when obstacle enters danger zone (< 0.40 m):

```
State: BLOCKED
  t=0:             Stop robot. Record blocked_since.
  t=map_upd_delay (0.8s): _publish_replan() → /goal_pose, sets
                           _waiting_for_replan=True, replan_requested=True.
  t=last_replan + replan_retry (3.0s) intervals:
                   Retry if still blocked.

On new /planned_path received (_path_cb):
  _waiting_for_replan = False  → robot unfreezes
  _replan_requested   = False  → retry timer resets
  _blocked_since      UNCHANGED — only cleared by scan going clear
  _last_replan_time   UNCHANGED — off-path cooldown continues

On obstacle cleared (forward_min > stop_dist):
  _blocked_since = None → resume immediately without replan.
```

**Key design rules:**
- `_blocked_since` is ONLY cleared when the laser scan shows the path clear. `_path_cb` must not reset it, otherwise a new path while obstacle is still present restarts the 0.8s wait loop repeatedly.
- `_waiting_for_replan` freezes the robot between replan request and new path arrival, preventing the robot from moving toward the obstacle during planner computation.

The `map_update_delay` (0.8 s) gives SLAM Toolbox time to incorporate the newly detected obstacle into the costmap before the planner is triggered. Without this delay, the planner would replan using an outdated map and produce the same path.

---

### 2.4 Stuck Detection

**Purpose:** Catches cases where the robot is heading toward an obstacle that doesn't enter the lidar's narrow forward cone (e.g., approaching at an angle), or where the robot is physically stalled.

**Algorithm:**
```
Every stuck_check_interval (4.0 s):
    moved = euclidean_dist(current_pose, pose_4s_ago)
    if moved < stuck_dist_threshold (0.15 m) and active:
        → publish /goal_pose (replan request)
    Update checkpoint.

On new path received: reset checkpoint.
```

---

### 2.5 Off-Path Detection

**Purpose:** Catches cases where the robot has been physically displaced from its planned path (e.g., collision pushed it sideways, or path became invalid due to map update).

**Algorithm:**
```
Every control cycle (10 Hz):
    min_path_dist = min(euclidean_dist(robot, p) for p in path)
    replan_cooldown = replan_retry_s × 2  (= 6 s)
    since_last_replan = now - _last_replan_time  (∞ if never replanned)
    if min_path_dist > off_path_dist (1.5 m) AND since_last_replan > cooldown:
        → _publish_replan()   (sets _waiting_for_replan=True)
        # robot stops on the next tick via _waiting_for_replan check
```

**Cooldown:** 6 s after any replan, off-path detection is suppressed. This prevents a newly replanned path (which starts at the robot's position at replan-time) from immediately triggering another replan because the robot moved a few centimetres during computation.

This is the fastest-acting recovery: triggers within 100 ms of the robot drifting off-path, vs. 4 s for stuck detection.

### 2.6 Closest-Waypoint Start

When `_path_cb` receives a new path (initial plan or replan), it computes the Euclidean distance from the robot's current pose to every waypoint except the last and sets `_path_idx` to the nearest one:

```python
dists = [hypot(px − rx, py − ry) for px, py in self._path[:-1]]
self._path_idx = dists.index(min(dists))
```

**Why:** During a replan, the planner uses the robot's pose at request time. By the time the path arrives (~0.5–1s later) the robot has moved. Without this fix, `_path_idx=0` would be behind the robot, causing a backtrack manoeuvre before proceeding to goal.

---

## 3. Parameter Summary

### Planner (`planner_params.yaml`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `planner_type` | `hybrid_astar` | Active planner: `astar` / `hybrid_astar` / `rrt_star` |
| `compare_mode` | false | When true: run all 3 planners, publish 3 paths, robot does NOT move |
| `inflation_radius_m` | 0.50 | Obstacle inflation radius (must be > obstacle_stop_dist) |
| `occupied_threshold` | 85 | SLAM values ≥85 treated as obstacle (noise is 65-80) |
| `allow_unknown` | true | Unknown cells (-1) treated as free for planning |
| `hybrid_num_headings` | 72 | Heading bins for Hybrid-A* (360/72 = 5° resolution) |
| `hybrid_step_size` | 1.0 | Hybrid-A* step length in grid cells |
| `hybrid_steer_angles` | [-0.5, 0.0, 0.5] | Steering options in radians |
| `rrt_max_iter` | 8000 | Max RRT* tree nodes |
| `rrt_step_size` | 5.0 | Max RRT* extension per step (cells) |
| `rrt_goal_sample_rate` | 0.10 | Probability of sampling goal directly |
| `rrt_search_radius` | 10.0 | RRT* rewire neighbourhood radius (cells) |

### Follower / Controller (`motion_params.yaml`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `controller_type` | `mpc` | Active controller: stanley/pid/pure_pursuit/lqr/mpc |
| `controller_compare_mode` | true | Run all 5, publish /controller_diagnostics |
| `lookahead_distance` | 0.5 m | Carrot distance ahead on path |
| `max_linear_vel` | 0.4 m/s | Speed cap |
| `max_angular_vel` | 1.0 rad/s | Turn rate cap |
| `goal_tolerance` | 0.25 m | Radius to declare goal reached |
| `obstacle_warn_dist` | 0.65 m | Slow to 50% in this zone |
| `obstacle_stop_dist` | 0.40 m | Stop and replan in this zone (must be < inflation_radius) |
| `obstacle_check_angle` | ±40° (0.698 rad) | Forward cone half-angle |
| `map_update_delay_s` | 0.8 s | Wait for SLAM before first replan |
| `replan_retry_s` | 3.0 s | Replan retry interval when still blocked |
| `stuck_check_interval_s` | 4.0 s | Stuck detection interval |
| `stuck_dist_threshold_m` | 0.15 m | Minimum movement to not be stuck |
| `off_path_dist_m` | 1.5 m | Max allowed distance from planned path |
| `stanley_k` | 1.0 | Stanley cross-track error gain |
| `stanley_v_min` | 0.1 m/s | Stanley minimum speed for CTE denominator |
| `pid_kp` | 1.5 | PID proportional gain |
| `pid_ki` | 0.05 | PID integral gain |
| `pid_kd` | 0.2 | PID derivative gain |
| `pid_windup_limit` | 2.0 rad·s | PID anti-windup clamp |
| `lqr_q_cte` | 5.0 | LQR state cost for CTE |
| `lqr_q_he` | 1.0 | LQR state cost for heading error |
| `lqr_r_w` | 0.5 | LQR control cost for ω |
| `mpc_horizon` | 8 | MPC prediction horizon |
| `mpc_q_cte` | 5.0 | MPC stage cost for CTE |
| `mpc_q_he` | 1.0 | MPC stage cost for heading error |
| `mpc_r_v` | 0.1 | MPC stage cost for linear velocity |
| `mpc_r_w` | 0.5 | MPC stage cost for angular velocity |
| `vp_curvature_gain` | 3.0 | VelocityProfiler curvature divisor sensitivity |
| `vp_goal_ramp_dist` | 1.5 m | Deceleration start distance from goal |
| `vp_min_vel` | 0.05 m/s | Minimum speed while active |
| `vp_accel_limit` | 0.5 m/s² | Max speed increase per second |

### Twist Mux (`motion_params.yaml` under `twist_mux`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `mux_autonomous_topic` | `/autonomous/cmd_vel` | Input from path follower |
| `mux_teleop_topic` | `/a300_00000/joy_teleop/cmd_vel` | Input from teleop |
| `mux_output_topic` | `/a300_00000/cmd_vel` | Output to robot |
| `mux_teleop_timeout_s` | 0.5 s | Teleop fallback timeout |
| `mux_rate_hz` | 20.0 Hz | Mux output rate |

---

## 4. Recovery Behaviour Summary

| Trigger | Detection Method | Latency | Action |
|---------|-----------------|---------|--------|
| Obstacle in forward cone | LiDAR scan (10 Hz) | ~100 ms | Stop → wait 0.8 s → replan → freeze until new path |
| Robot drifted off path | Distance to path (10 Hz) | ~100 ms | Replan → freeze until new path (6 s cooldown) |
| Robot stuck / stalled | Pose progress (every 4 s) | ≤4 s | Replan → freeze until new path |
| Still blocked after replan | Timer (every 3 s) | 3 s | Retry replan |

**Robot freeze:** whenever `_publish_replan()` is called, `_waiting_for_replan=True` is set. The control loop stops the robot and returns early until `_path_cb` clears this flag when the new path arrives. This prevents the robot from moving toward obstacles during planner computation.

---

## 5. Coordinate System Notes

- **Map frame:** `map` — global fixed frame, origin at SLAM start
- **Robot frame:** `base_link` — robot centre
- **Laser frame:** `lidar2d_0_laser` — mounted with potential yaw offset
- **Grid convention:** `(row, col)` = `(y-axis, x-axis)` in the map frame
- **World ↔ Grid conversion:**
  ```
  col = floor((world_x − origin_x) / resolution)
  row = floor((world_y − origin_y) / resolution)
  world_x = origin_x + (col + 0.5) × resolution  ← cell centre
  ```
