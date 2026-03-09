# Workspace Status & Component Overview

**Last Updated:** March 7, 2026  
**Project:** Clearpath A300 Autonomous Navigation System

---

## 📋 Quick Summary

- **Robot:** Clearpath A300 (a300-00000)
- **ROS Version:** ROS 2 Jazzy
- **Main Goal:** Autonomous path planning and obstacle avoidance with multi-sensor fusion
- **Current Status:** Iteration 1 completed | Navigation system framework in place

---

## 🔧 System Components

### 1. **State Estimation** (`/state_estimation`)
**Status:** ✅ Completed  
- Custom EKF, UKF, and Particle Filter implementations
- Sensor fusion for localization
- Files: `sensor_fusion_ekf.py`, `sensor_fusion_ukf.py`, `sensor_fusion_pf.py`
- Config: `sensor_fusion.yaml`
- Launch: `state_estimation.launch.py`

### 2. **Navigation** (`/navigation`)
**Status:** ✅ Iteration 1 Complete
- Nav2 stack integration
- Multi-sensor input to costmaps
- Waypoint navigation capability
- EKF + GPS localization setup

**Key Files:**
- `nav2_params.yaml` - Full Nav2 configuration (DWB, NavFn, costmaps)
- `localization_gps.yaml` - Dual EKF + GPS integration
- `navigation.launch.py` - Nav2 bringup launcher
- `waypoint_navigator.py` - Autonomous waypoint mission node
- `waypoints_*.yaml` - Pre-built waypoint paths (corridor, figure8, square)

### 3. **Sensors** (`/sensors`)
**Status:** ✅ Complete
- **IMU** (`imu_0.yaml`, `imu_0.launch.py`)
  - Topic: `/a300_00000/sensors/imu_0/data`
  - Rate: 100 Hz
  
- **GPS** (`gps_0.yaml`, `gps_0.launch.py`)
  - Topic: `/a300_00000/sensors/gps_0/fix`
  - Rate: 10 Hz
  
- **LiDAR 2D** (`lidar2d_0.yaml`, `lidar2d_0.launch.py`)
  - Topic: `/a300_00000/lidar2d_0/scan`
  - Rate: 40 Hz
  
- **Depth Camera** (`camera_0.yaml`, `camera_0.launch.py`)
  - Topics: `/a300_00000/camera_0/depth/image_rect_raw`, `/a300_00000/camera_0/points`
  - Rate: 30 Hz

### 4. **Platform Control** (`/platform`)
**Status:** ✅ Complete
- Platform initialization and control
- Twist multiplexer (teleop/autonomous switching)
- Localization (robot_localization EKF)
- Key configs: `control.yaml`, `localization.yaml`, `twist_mux.yaml`

### 5. **Manipulators** (`/manipulators`)
**Status:** ✅ Complete
- MoveIt configuration for robot arms
- Config: `moveit.yaml`
- Launch: `manipulators-service.launch.py`

### 6. **Trajectory Planner** (`/trajectory_planner_pkg`) — NEW
**Status:** ✅ Complete (Iteration 3, Mar 8 2026)
- A* baseline planner (grid-based, 8-connected)
- Optional Hybrid-A* and RRT* via `planner_type` parameter
- Subscribes: `/map` (OccupancyGrid), `/goal_pose` (PoseStamped)
- Publishes: `/planned_path` (nav_msgs/Path), `/planner_debug` (MarkerArray)
- Start from TF (`map` → `base_link`)
- Config: `planner_params.yaml`
- Launch: `planner.launch.py`

### 7. **Simple Motion** (`/simple_motion_pkg`) — NEW
**Status:** ✅ Complete (Iteration 3, Mar 8 2026)
- P-controller path follower with lookahead
- Subscribes: `/planned_path`
- Publishes: `/a300_00000/cmd_vel`
- Config: `motion_params.yaml`
- Launch: `motion.launch.py`

### 8. **Autonomy Bringup** (`/autonomy_bringup`) — NEW
**Status:** ✅ Complete (Iteration 3, Mar 8 2026)
- Unified launch: planner + follower + RViz
- RViz config: Map, TF, RobotModel, Path, LaserScan, Goal tool
- Launch: `autonomy.launch.py`

---

## 🗂️ Key Configuration Files

| Component | Config File | Purpose |
|-----------|-------------|---------|
| Navigation | `navigation/config/nav2_params.yaml` | Full Nav2 stack parameters |
| Localization | `navigation/config/localization_gps.yaml` | EKF + GPS sensor fusion |
| Localization | `platform/config/localization.yaml` | Primary robot_localization config |
| Control | `platform/config/control.yaml` | Motor control parameters |
| Twist Mux | `platform/config/twist_mux.yaml` | Command velocity arbitration |

---

## 🚀 Quick Start Commands

### Using Official Clearpath Packages (Recommended)

**SLAM Mode (for mapping new environments):**
```bash
# T1: Simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# T2: Nav2 + SLAM
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# T3: Visualization
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

**Localization Mode (for autonomous navigation with pre-built map):**
```bash
# T1: Simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# T2: Nav2 + Localization
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# T3: Visualization
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

**Autonomous Waypoint Patrol (custom):**
```bash
# T4: Waypoint navigator (run after nav2 is ready)
ros2 run navigation waypoint_navigator --ros-args \
  -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
  -p mode:=loop
```

**A* Trajectory Planner (Iteration 3):**
```bash
# Build first (one time):
cd /home/prajjwal/clearpath
colcon build --packages-select trajectory_planner_pkg simple_motion_pkg autonomy_bringup
source install/setup.bash

# T1: Gazebo
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# T2: SLAM (wait ~5s)
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# T3: Planner + Follower + RViz
ros2 launch autonomy_bringup autonomy.launch.py use_sim_time:=true

# Then click "2D Goal Pose" in RViz to plan & follow a path
```

---

## 📊 Sensor Topics Reference

| Sensor | Topic | Type | Rate |
|--------|-------|------|------|
| IMU | `/a300_00000/sensors/imu_0/data` | sensor_msgs/Imu | 100 Hz |
| GPS | `/a300_00000/sensors/gps_0/fix` | sensor_msgs/NavSatFix | 10 Hz |
| LiDAR | `/a300_00000/lidar2d_0/scan` | sensor_msgs/LaserScan | 40 Hz |
| Camera Depth | `/a300_00000/camera_0/depth/image_rect_raw` | sensor_msgs/Image | 30 Hz |
| Camera Pointcloud | `/a300_00000/camera_0/points` | sensor_msgs/PointCloud2 | 30 Hz |
| Odometry | `/a300_00000/platform/odom` | nav_msgs/Odometry | 50 Hz |

---

## 🔄 TF Tree Structure

```
map
  └── odom (published by EKF global node + navsat_transform)
      └── base_link (published by EKF local node)
          ├── imu_link (IMU sensor frame)
          ├── lidar2d_0 (LiDAR sensor frame)
          ├── camera_0 (Camera sensor frame)
          └── [other sensor frames]
```

---

## 📝 Ongoing Work

### Current Iteration: 3 (A* Trajectory Planner)
- ✅ Navigation package structure created (Iter 1)
- ✅ Nav2 configuration completed (Iter 1)
- ✅ Aligned with official Clearpath packages (Iter 2)
- ✅ A* trajectory planner package created (Iter 3)
- ✅ Simple motion controller created (Iter 3)
- ✅ Autonomy bringup launch created (Iter 3)
- ✅ All 3 new packages build and pass smoke tests
- ⏳ End-to-end simulation testing pending
- ⏳ Hybrid-A* / RRT* sim testing pending

---

## 🔗 Related Documents

- [Iteration Log](iteration.md) - Track all changes and updates
- [main README](../README.md) - Project overview
- [Navigation README](../navigation/README.md) - Detailed navigation setup
- [System Overview](../SYSTEM_OVERVIEW.md) - Complete system architecture
- [Engineering Log](../notes.md) - Original iteration notes

