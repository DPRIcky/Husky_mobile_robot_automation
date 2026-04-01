# Husky Mobile Robot Automation

A comprehensive ROS 2 Jazzy workspace for the Clearpath A300 (Husky) mobile robot platform featuring multi-robot simulation, ArUco-based robot following, path planning, and advanced sensor fusion.

## Overview

This repository contains a complete robotics system for the Clearpath A300 outdoor mobile robot, including:
- **Multi-Robot Simulation**: Two A300 robots in Gazebo — one leads, one follows using visual marker tracking
- **ArUco Marker Following**: Robot 1 autonomously tracks and follows Robot 2 using its front camera and an ArUco marker mounted on Robot 2's rear
- **Path Planning**: A*, Hybrid-A*, and RRT* planners on occupancy grid with path smoothing
- **Path Following**: 5 selectable controllers (Stanley, Pure Pursuit, PID, LQR, MPC)
- **State Estimation**: Multiple Kalman filter implementations (EKF, UKF, Particle Filter)
- **Sensor Integration**: IMU, GPS, Lidar, Camera, and Wheel Odometry

## Repository Structure

```
clearpath/
├── robot.yaml              # Robot 1 config (namespace: a300_00000)
├── robot.urdf.xacro        # Robot 1 URDF (no marker)
├── platform/launch/        # Robot 1 generated platform launch files
├── sensors/launch/         # Robot 1 generated sensor launch files
│
├── robot2/                 # Robot 2 setup
│   ├── robot.yaml              # Robot 2 config (namespace: a300_00001)
│   ├── robot.urdf.xacro        # Robot 2 URDF (ArUco marker embedded on rear)
│   ├── platform/launch/
│   └── sensors/launch/
│
├── autonomy_bringup/       # Main autonomy package
│   ├── launch/
│   │   ├── two_robot_aruco.launch.py   # Canonical multi-robot launch
│   │   ├── aruco_detection.launch.py   # Standalone detector
│   │   ├── aruco_follow.launch.py      # Standalone follower
│   │   └── autonomy.launch.py          # Single-robot planning stack
│   ├── autonomy_bringup/
│   │   ├── aruco_detector.py           # OpenCV ArUco detector node
│   │   └── aruco_follower.py           # Visual following controller node
│   └── models/aruco_marker_0/          # ArUco marker mesh + texture
│
├── trajectory_planner_pkg/ # A*, Hybrid-A*, RRT* path planners
├── simple_motion_pkg/      # Path following controllers + twist mux
├── state_estimation/       # Standalone EKF/UKF/Particle Filter scripts
├── navigation/             # Nav2 integration + waypoint navigator
├── sensors/                # Sensor configs and launch files
├── platform/               # Platform control launch files
└── docs/                   # Detailed documentation
    ├── MULTI_ROBOT_SETUP.md
    ├── SYSTEM_OVERVIEW.md
    └── PROJECT_REPORT.md
```

## Quick Start

### Prerequisites

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-clearpath-gz ros-jazzy-nav2-bringup
pip3 install transforms3d scipy
```

### Multi-Robot ArUco Follow (primary demo)

Two A300 robots spawn in the Gazebo warehouse world. Robot 2 carries an ArUco marker on its rear bumper. Robot 1 detects and follows it.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py
```

After ~13 s both robots are fully up. Then in separate terminals:

**Drive Robot 2** (the one being followed):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=a300_00001/base_link -r cmd_vel:=/a300_00001/cmd_vel
```

**Start Robot 1 follower**:
```bash
ros2 launch autonomy_bringup aruco_follow.launch.py target_timeout_s:=2.0
```

Robot 1 will drive toward Robot 2, maintain a ~1.5 m standoff, and steer to keep the marker centered. Adjust standoff with `desired_standoff_m:=2.0`.

Launch with follower enabled from the start:
```bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py launch_aruco_follower:=true
```

See [docs/MULTI_ROBOT_SETUP.md](docs/MULTI_ROBOT_SETUP.md) for full details, customisation args, and troubleshooting.

### Single-Robot Path Planning

```bash
# Terminal 1: Gazebo simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2: SLAM mapping
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3: Autonomy (planner + controller + RViz)
ros2 launch autonomy_bringup autonomy.launch.py use_sim_time:=true
```

### State Estimation (standalone)

```bash
python3 state_estimation/sensor_fusion_ekf.py   # EKF
python3 state_estimation/sensor_fusion_ukf.py   # UKF
python3 state_estimation/sensor_fusion_pf.py    # Particle Filter
python3 state_estimation/compare_filters.py     # Run all three + compare
```

## Features

- Multi-robot Gazebo simulation (2 × Clearpath A300)
- Vision-based robot following via ArUco marker detection (OpenCV, no external ROS package needed)
- Path planning: A*, Hybrid-A*, RRT* on live occupancy grid
- 5 path following controllers: Stanley, Pure Pursuit, PID, LQR, MPC
- Multi-sensor state estimation: EKF, UKF, Particle Filter (8-state: x, y, θ, vx, vy, ω, ax, ay)
- SLAM-based mapping with Nav2 integration
- Waypoint-based mission planning (single, loop, patrol modes)

## Robot Specifications

- **Platform**: Clearpath A300 (4-wheel differential drive)
- **Wheel Radius**: 0.1625 m, **Wheel Separation**: 0.562 m
- **Serial**: a300-00000, **IP**: 192.168.131.1
- **Sensors**: Microstrain IMU, Garmin GPS, Intel RealSense D435, Hokuyo UST LiDAR

## Documentation

- [docs/MULTI_ROBOT_SETUP.md](docs/MULTI_ROBOT_SETUP.md) — Two-robot ArUco follow setup and troubleshooting
- [docs/SYSTEM_OVERVIEW.md](docs/SYSTEM_OVERVIEW.md) — Hardware and software architecture
- [docs/PROJECT_REPORT.md](docs/PROJECT_REPORT.md) — Implementation details and results
- [navigation/README.md](navigation/README.md) — Nav2 waypoint navigation

## Author

**DPRicky**

---

*Last Updated: March 2026*
