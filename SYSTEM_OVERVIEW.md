# Clearpath Robotics System Overview

**Date:** February 4, 2026  
**ROS Version:** ROS 2 Jazzy  
**Platform:** Clearpath A300 (Serial: a300-00000)

---

## Table of Contents
- [System Introduction](#system-introduction)
- [Hardware Platform](#hardware-platform)
- [Installed ROS Packages](#installed-ros-packages)
- [Workspace Structure](#workspace-structure)
- [Sensors](#sensors)
- [Control System](#control-system)
- [Navigation & Localization](#navigation--localization)
- [Simulation Environment](#simulation-environment)
- [Development Components](#development-components)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [Getting Started Guide](#getting-started-guide)

---

## System Introduction

This system is based on the **Clearpath A300** outdoor mobile robot platform with integrated sensors and navigation capabilities. The workspace is configured for both simulation and real robot deployment using ROS 2 Jazzy.

### Key System Information
- **Robot Model:** A300 (Outdoor Differential Drive 4WD)
- **Serial Number:** a300-00000
- **Namespace:** `a300_00000`
- **Hostname:** cpr-a300-00000
- **IP Address:** 192.168.131.1
- **Controller:** PS4 gamepad

---

## Hardware Platform

### Platform Specifications
- **Type:** A300 Differential Drive Robot
- **Drive System:** 4-wheel differential drive (`diff_4wd`)
- **Wheel Configuration:**
  - Front wheels: Outdoor
  - Rear wheels: Outdoor
  - Wheel radius: 0.1625m
  - Wheel separation: 0.562m

### Battery
- **Model:** S_24V20_U1
- **Configuration:** S1P2

### Attachments
1. **Top Plate** (a200.top_plate)
   - Default model
   - Mounted on default mount
   
2. **Sensor Arch** (sensor_arch_300)
   - Mounted at (0.05, 0.0, 0.025)
   - Hosts elevated sensors
   
3. **Front Bumper**
   - Default model
   - Mounted on front bumper mount
   
4. **Rear Bumper**
   - Default model
   - Mounted on rear bumper mount

---

## Installed ROS Packages

The following Clearpath packages are installed in `/opt/ros/jazzy`:

### Core Packages
- **clearpath_common** - Common utilities and base functionality
- **clearpath_config** - Configuration management tools
- **clearpath_description** - Robot URDF descriptions
- **clearpath_platform_msgs** - Platform-specific message definitions

### Platform-Specific
- **clearpath_platform_description** - A300 platform URDF models
- **clearpath_control** - Motor control and velocity controllers
- **clearpath_diagnostics** - System health monitoring

### Sensors
- **clearpath_sensors_description** - Sensor URDF models and configurations

### Manipulators
- **clearpath_manipulators** - Manipulator control and integration
- **clearpath_manipulators_description** - Manipulator URDF models
- **clearpath_mounts_description** - Mounting bracket descriptions

### Simulation
- **clearpath_simulator** - Main simulation package
- **clearpath_gz** - Gazebo simulation integration
- **clearpath_generator_gz** - Gazebo world/model generators
- **clearpath_generator_common** - Common generator utilities

### Navigation & Autonomy
- **clearpath_nav2_demos** - Nav2 navigation demos and configurations
- **clearpath_bt_joy** - Behavior tree joystick integration

### Visualization
- **clearpath_viz** - RViz configurations and visualization tools

---

## Workspace Structure

Your workspace at `/home/prajjwal/clearpath/` is organized as follows:

```
clearpath/
├── robot.urdf.xacro          # Main robot description
├── robot.srdf                # Semantic robot description (MoveIt)
├── robot.srdf.xacro          # SRDF xacro template
├── robot.yaml                # Primary robot configuration
│
├── platform/                 # Platform control & configuration
│   ├── config/
│   │   ├── control.yaml             # Motor controller settings
│   │   ├── localization.yaml        # EKF localization config
│   │   ├── twist_mux.yaml           # Velocity command multiplexer
│   │   ├── teleop_joy.yaml          # Joystick teleop settings
│   │   ├── teleop_interactive_markers.yaml
│   │   ├── diagnostic_aggregator.yaml
│   │   ├── diagnostic_updater.yaml
│   │   ├── foxglove_bridge.yaml
│   │   └── imu_filter.yaml
│   └── launch/
│       └── platform-service.launch.py  # Main platform launch file
│
├── sensors/                  # Sensor configuration & launch files
│   ├── config/
│   │   ├── camera_0.yaml            # Intel RealSense D435
│   │   ├── gps_0.yaml               # Garmin 18x GPS
│   │   ├── imu_0.yaml               # Microstrain IMU
│   │   └── lidar2d_0.yaml           # Hokuyo UST LiDAR
│   └── launch/
│       ├── camera_0.launch.py
│       ├── gps_0.launch.py
│       ├── imu_0.launch.py
│       ├── lidar2d_0.launch.py
│       └── sensors-service.launch.py   # Launches all sensors
│
├── manipulators/             # Manipulator control (currently empty)
│   ├── config/
│   │   └── moveit.yaml              # MoveIt configuration
│   └── launch/
│       └── manipulators-service.launch.py
│
└── platform-extras/          # Additional platform extensions
    └── launch/
```

---

## Sensors

Your A300 is equipped with the following sensors:

### 1. 2D LiDAR (lidar2d_0)
- **Model:** Hokuyo UST
- **Mount Location:** sensor_arch_mount
- **Specifications:**
  - Angular resolution: 0.5°
  - Scan range: -180° to +180°
  - Min range: 0.05m
  - Max range: 25.0m
  - Update rate: 40 Hz
  - Port: `/dev/clearpath/lidar`
- **Frame ID:** `laser_0_link`
- **Topics:**
  - `/a300_00000/sensors/lidar2d_0/scan` (sensor_msgs/LaserScan)

### 2. RGB-D Camera (camera_0)
- **Model:** Intel RealSense D435
- **Mount Location:** sensor_arch_mount
- **Specifications:**
  - Resolution: 640x480
  - Update rate: 30 Hz
  - Color + Depth + Point cloud enabled
- **Frame ID:** `camera_0_link`
- **Topics:**
  - `/a300_00000/camera_0/color/image_raw`
  - `/a300_00000/camera_0/depth/image_rect_raw`
  - `/a300_00000/camera_0/depth/color/points` (PointCloud2)

### 3. IMU (imu_0)
- **Model:** Microstrain IMU
- **Mount Location:** base_link (robot center)
- **Specifications:**
  - Update rate: 100 Hz
  - ENU frame orientation enabled
  - Port: `/dev/microstrain_main`
- **Frame ID:** `imu_0_link`
- **Topics:**
  - `/a300_00000/sensors/imu_0/data` (sensor_msgs/Imu)

### 4. GPS (gps_0)
- **Model:** Garmin 18x
- **Mount Location:** sensor_arch_mount (0.0, 0.2, 0.01)
- **Specifications:**
  - NMEA protocol
  - Baud rate: 115200
  - Port: `/dev/clearpath/gps`
- **Frame ID:** `gps_0_link`
- **Topics:**
  - `/a300_00000/sensors/gps_0/fix` (sensor_msgs/NavSatFix)

---

## Control System

### Differential Drive Controller
The platform uses a **diff_drive_controller** for motion control:

#### Configuration
- **Update rate:** 50 Hz
- **Left wheels:** front_left_wheel_joint, rear_left_wheel_joint
- **Right wheels:** front_right_wheel_joint, rear_right_wheel_joint
- **Control mode:** Closed-loop (open_loop: False)

#### Velocity Limits
**Linear (X-axis):**
- Max velocity: ±2.0 m/s
- Max acceleration: 2.0 m/s²
- Max deceleration: -4.0 m/s²

**Angular (Z-axis):**
- Max velocity: ±2.0 rad/s
- Max acceleration: ±4.0 rad/s²

#### Published Topics
- `/a300_00000/platform/odom` - Odometry data
- `/a300_00000/platform/cmd_vel` - Velocity commands (input)

### Twist Multiplexer (twist_mux)
Manages multiple velocity command sources with priorities:

| Source | Topic | Priority | Timeout |
|--------|-------|----------|---------|
| RC Teleop | `rc_teleop/cmd_vel` | 12 (highest) | 0.5s |
| Joy Teleop | `joy_teleop/cmd_vel` | 10 | 0.5s |
| Interactive Marker | `twist_marker_server/cmd_vel` | 8 | 0.5s |
| External/Nav | `cmd_vel` | 1 (lowest) | 0.5s |

**Safety Locks:**
- Emergency stop: Priority 255
- Safety stop: Priority 254

---

## Navigation & Localization

### Extended Kalman Filter (EKF)
Sensor fusion for state estimation:

**Configuration:**
- **Frequency:** 50 Hz
- **Mode:** 2D navigation (two_d_mode: True)
- **Frames:**
  - World frame: `odom`
  - Base frame: `base_link`
  - Publishes TF: Yes

**Fused Sensors:**
1. **Odometry** (`platform/odom`)
   - Position: X, Y, Yaw
   - Velocity: X, Y, Yaw_rate
   
2. **IMU** (`sensors/imu_0/data`)
   - Angular velocity: Yaw_rate
   - Linear acceleration (gravity removed)

**Published Topics:**
- `/a300_00000/odometry/filtered` - Fused odometry estimate

### Nav2 Integration
The `clearpath_nav2_demos` package provides:
- Pre-configured Nav2 parameter files
- Autonomous navigation demonstrations
- Path planning capabilities
- Obstacle avoidance using LiDAR

---

## Simulation Environment

### Gazebo Simulation
Launch simulations using the Clearpath Gazebo package:

```bash
ros2 launch clearpath_gz simulation.launch.py
```

#### Available Worlds
- `construction` - Construction site environment
- `office` - Indoor office space
- `orchard` - Outdoor agricultural setting
- `pipeline` - Industrial pipeline inspection
- `solar_farm` - Solar panel array environment
- `warehouse` - Indoor warehouse (default)

#### Launch Arguments
- `world:=<world_name>` - Select simulation world
- `rviz:=true/false` - Launch RViz (default: false)
- `x:=<value>` - Initial X position (default: 0.0)
- `y:=<value>` - Initial Y position (default: 0.0)
- `yaw:=<value>` - Initial orientation (default: 0.0)
- `setup_path:=<path>` - Custom robot configuration path
- `auto_start:=true/false` - Auto-start simulation

#### Example Commands
```bash
# Launch with RViz in office world
ros2 launch clearpath_gz simulation.launch.py world:=office rviz:=true

# Start at specific position
ros2 launch clearpath_gz simulation.launch.py x:=2.0 y:=1.0 yaw:=1.57

# Use custom setup path
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
```

---

## Development Components

### 1. Robot Description (URDF)
**File:** [robot.urdf.xacro](robot.urdf.xacro)

This xacro file defines the complete robot model including:
- A300 base platform with 4-wheel differential drive
- All sensor mounts and attachments
- Sensor models (LiDAR, camera, IMU, GPS)
- Physical properties and collision models

**Included packages:**
- `clearpath_platform_description` - Base platform
- `clearpath_sensors_description` - Sensor models
- `clearpath_mounts_description` - Mounting brackets

### 2. Configuration File (YAML)
**File:** [robot.yaml](robot.yaml)

Central configuration for all robot parameters:
- System settings (namespace, network)
- Platform configuration (controller, battery)
- Sensor parameters and ROS settings
- Attachment definitions

### 3. Semantic Description (SRDF)
**Files:** [robot.srdf](robot.srdf), [robot.srdf.xacro](robot.srdf.xacro)

Used by MoveIt for manipulation planning (when manipulators are added).

### 4. Custom Launch Files

#### Platform Service
**File:** [platform/launch/platform-service.launch.py](platform/launch/platform-service.launch.py)

Launches:
- Platform control system
- EKF localization
- Gazebo bridges (for simulation)
- Odometry and TF publishers

#### Sensors Service
**File:** [sensors/launch/sensors-service.launch.py](sensors/launch/sensors-service.launch.py)

Launches all four sensors:
- LiDAR driver
- RealSense camera driver
- IMU driver
- GPS driver

#### Individual Sensor Launches
- `camera_0.launch.py` - RGB-D camera only
- `lidar2d_0.launch.py` - 2D LiDAR only
- `imu_0.launch.py` - IMU only
- `gps_0.launch.py` - GPS only

### 5. Configuration Files

All configuration files use the namespace `a300_00000`:

**Platform Configs:**
- `control.yaml` - Controller parameters, velocity limits
- `localization.yaml` - EKF sensor fusion settings
- `twist_mux.yaml` - Command source priorities
- `teleop_joy.yaml` - PS4 controller mapping
- `diagnostic_*.yaml` - System health monitoring
- `foxglove_bridge.yaml` - Foxglove Studio integration

**Sensor Configs:**
- Individual YAML files for each sensor's ROS parameters

---

## Getting Started Guide

### 1. Running the Simulation

Launch the complete simulated robot:
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch clearpath_gz simulation.launch.py rviz:=true

# Terminal 2: Launch sensors (if not auto-started)
ros2 launch /home/prajjwal/clearpath/sensors/launch/sensors-service.launch.py

# Terminal 3: Teleop control with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/a300_00000/cmd_vel
```

### 2. Viewing Sensor Data

#### List all topics
```bash
ros2 topic list | grep a300_00000
```

#### View LiDAR scans
```bash
ros2 topic echo /a300_00000/sensors/lidar2d_0/scan
```

#### View camera images
```bash
ros2 run rqt_image_view rqt_image_view /a300_00000/camera_0/color/image_raw
```

#### View IMU data
```bash
ros2 topic echo /a300_00000/sensors/imu_0/data
```

#### View GPS position
```bash
ros2 topic echo /a300_00000/sensors/gps_0/fix
```

### 3. Inspecting the Robot Model

View the robot in RViz:
```bash
ros2 launch clearpath_viz view_robot.launch.py
```

Convert URDF to viewable format:
```bash
cd /home/prajjwal/clearpath
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
```

### 4. Navigation Setup

Launch Nav2 navigation stack:
```bash
# Requires Nav2 to be installed
ros2 launch clearpath_nav2_demos nav2_bringup.launch.py
```

### 5. Custom Development

#### Modify Robot Configuration
1. Edit [robot.yaml](robot.yaml) to change sensors, parameters
2. Run generation (if using generator tools)
3. Restart platform services

#### Add New Sensors
1. Add sensor entry to [robot.yaml](robot.yaml)
2. Create config file in `sensors/config/`
3. Create launch file in `sensors/launch/`
4. Update [sensors-service.launch.py](sensors/launch/sensors-service.launch.py)

#### Tune Control Parameters
Edit [platform/config/control.yaml](platform/config/control.yaml) to adjust:
- Velocity limits
- Acceleration profiles
- Controller gains

#### Modify Localization
Edit [platform/config/localization.yaml](platform/config/localization.yaml) to:
- Change sensor fusion weights
- Adjust covariance matrices
- Enable/disable sensors in EKF

### 6. Useful ROS Commands

```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /a300_00000/platform_velocity_controller

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor diagnostics
ros2 topic echo /diagnostics

# Check robot state
ros2 topic echo /a300_00000/joint_states

# Send velocity command
ros2 topic pub /a300_00000/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## Development Capabilities

### What You Can Develop

1. **Autonomous Navigation**
   - Use Nav2 stack with LiDAR for obstacle avoidance
   - Implement custom path planning algorithms
   - Create waypoint navigation systems

2. **Sensor Processing**
   - Point cloud processing from RealSense camera
   - LiDAR-based SLAM and mapping
   - IMU-based orientation tracking
   - GPS waypoint navigation

3. **Computer Vision**
   - Object detection using RGB-D camera
   - Visual SLAM
   - Person following algorithms
   - Semantic segmentation

4. **Manipulation** (when arm is added)
   - MoveIt-based motion planning
   - Pick and place operations
   - Visual servoing

5. **Multi-Robot Systems**
   - Fleet coordination (namespace: a300_00000)
   - Distributed task allocation

6. **Custom Behaviors**
   - Behavior trees with BT.CPP
   - State machines
   - Teleoperation interfaces

7. **Simulation Testing**
   - Test algorithms in 6 different environments
   - Rapid prototyping without hardware
   - Integration testing

### Key Tools Available
- **RViz** - 3D visualization
- **Gazebo** - Physics simulation
- **Nav2** - Navigation framework
- **MoveIt** - Manipulation planning (ready for arms)
- **TF2** - Coordinate frame management
- **Foxglove** - Modern visualization (config included)
- **RQt** - Plugin-based GUI tools

---

## Quick Reference

### Important Topics
```
/a300_00000/cmd_vel                          # Velocity commands
/a300_00000/platform/odom                    # Wheel odometry
/a300_00000/odometry/filtered                # Fused odometry
/a300_00000/sensors/lidar2d_0/scan          # LiDAR scans
/a300_00000/camera_0/depth/color/points     # Point cloud
/a300_00000/sensors/imu_0/data              # IMU measurements
/a300_00000/sensors/gps_0/fix               # GPS position
/a300_00000/joint_states                     # Joint positions
/diagnostics                                 # System health
```

### Important Frames
```
odom          # Odometry frame (world frame)
base_link     # Robot base center
laser_0_link  # LiDAR frame
camera_0_link # Camera frame
imu_0_link    # IMU frame
gps_0_link    # GPS antenna frame
```

### Package Locations
- **Installed packages:** `/opt/ros/jazzy/share/clearpath_*/`
- **Workspace:** `/home/prajjwal/clearpath/`
- **Configuration:** `~/clearpath/{platform,sensors,manipulators}/config/`
- **Launch files:** `~/clearpath/{platform,sensors,manipulators}/launch/`

---

## Support Resources

### Documentation
- [Clearpath Robotics Docs](https://docs.clearpathrobotics.com/)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/)

### Community
- [Clearpath Robotics Support](https://support.clearpathrobotics.com/)
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

---

**Last Updated:** February 4, 2026  
**System Version:** Clearpath ROS 2 Jazzy
