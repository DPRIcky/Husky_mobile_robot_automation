# Husky Mobile Robot Automation

A comprehensive ROS 2 Jazzy workspace for the Clearpath A300 (Husky) mobile robot platform with advanced sensor fusion and state estimation capabilities.

## 🤖 Overview

This repository contains a complete robotics system for the Clearpath A300 outdoor mobile robot, including:
- **State Estimation**: Multiple Kalman filter implementations (EKF, UKF, Particle Filter)
- **Sensor Integration**: IMU, GPS, Lidar, Camera, and Wheel Odometry
- **Robot Configuration**: URDF/XACRO models, SRDF for MoveIt
- **Launch Files**: Modular launch system for sensors, platform, and manipulators

## 📁 Repository Structure

```
├── state_estimation/          # Sensor fusion and state estimation
│   ├── sensor_fusion_ekf.py   # Extended Kalman Filter implementation
│   ├── sensor_fusion_ukf.py   # Unscented Kalman Filter implementation
│   ├── sensor_fusion_pf.py    # Particle Filter implementation
│   ├── compare_filters.py     # Filter comparison tool
│   └── config/                # Configuration files
├── sensors/                   # Sensor configurations and launch files
│   ├── config/                # Camera, GPS, IMU, Lidar configs
│   └── launch/                # Individual sensor launch files
├── platform/                  # Platform control and configuration
│   ├── config/                # Control, localization, teleop configs
│   └── launch/                # Platform service launch files
├── manipulators/              # Arm/manipulator configuration
│   ├── config/                # MoveIt configuration
│   └── launch/                # Manipulator launch files
├── robot.urdf.xacro          # Robot description
├── robot.srdf                # Semantic robot description
└── SYSTEM_OVERVIEW.md        # Detailed system documentation

```

## 🚀 Quick Start

### Prerequisites

- ROS 2 Jazzy
- Python 3.10+
- Required Python packages: `numpy`, `matplotlib`, `transforms3d`

```bash
# Install dependencies
sudo apt install python3-numpy python3-matplotlib
pip3 install transforms3d
```

### Running State Estimation

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Run the EKF-based state estimator
python3 state_estimation/sensor_fusion_ekf.py
```

### Launch Sensor Suite

```bash
# Launch all sensors
ros2 launch sensors/launch/sensors-service.launch.py

# Launch individual sensors
ros2 launch sensors/launch/imu_0.launch.py
ros2 launch sensors/launch/gps_0.launch.py
ros2 launch sensors/launch/lidar2d_0.launch.py
```

## 🔬 State Estimation System

This repository includes three different sensor fusion implementations:

### Extended Kalman Filter (EKF)
- **File**: `sensor_fusion_ekf.py`
- Best for linear/near-linear systems
- Fast and efficient
- **State Vector**: [x, y, θ, vx, vy, ω, ax, ay]

### Unscented Kalman Filter (UKF)
- **File**: `sensor_fusion_ukf.py`
- Better for non-linear systems
- Sigma point approach
- More accurate for complex motion

### Particle Filter (PF)
- **File**: `sensor_fusion_pf.py`
- Non-parametric approach
- Handles multi-modal distributions
- Good for global localization

### Compare Filters

```bash
# Run comparison across all three filters
python3 state_estimation/compare_filters.py
```

## 📊 Features

- ✅ Multi-sensor fusion (IMU, GPS, Odometry, Lidar)
- ✅ Real-time state estimation at 50 Hz
- ✅ Covariance estimation and uncertainty tracking
- ✅ Visualization tools for monitoring
- ✅ Configurable through YAML files
- ✅ ROS 2 native implementation
- ✅ Modular and extensible architecture

## 🛠️ Robot Specifications

- **Platform**: Clearpath A300 (Husky)
- **Drive**: 4-wheel differential drive
- **Wheel Radius**: 0.1625m
- **Wheel Separation**: 0.562m
- **Serial**: a300-00000
- **Controller**: PS4 gamepad

## 📖 Documentation

For detailed system documentation, see:
- [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md) - Complete system architecture
- [state_estimation/README.md](state_estimation/README.md) - State estimation details
- [state_estimation/QUICK_START.md](state_estimation/QUICK_START.md) - Quick start guide

## 🤝 Contributing

This is a project workspace for the Clearpath Husky mobile robot automation system.

## 📝 License

See individual package licenses.

## 👤 Author

**DPRicky**

---

*Last Updated: March 2026*
