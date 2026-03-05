# State Estimation using Extended Kalman Filter (EKF)

## Overview
This package implements sensor fusion for robot state estimation using an Extended Kalman Filter. It fuses data from multiple sensors (IMU, Odometry, GPS, Lidar) to provide robust state estimates.

## Features
- **Extended Kalman Filter (EKF)** implementation for sensor fusion
- Supports multiple sensors:
  - IMU (Inertial Measurement Unit)
  - Wheel Odometry
  - GPS (Global Positioning System)
  - Lidar (2D scan matching for position/orientation correction)
- Real-time state estimation at 50 Hz
- Publishes fused state estimates with covariance
- Visualization tools for monitoring

## State Vector
The filter estimates an 8-dimensional state vector:
- `x, y`: Position in 2D (meters)
- `theta`: Orientation/yaw angle (radians)
- `vx, vy`: Linear velocities (m/s)
- `omega`: Angular velocity (rad/s)
- `ax, ay`: Linear accelerations (m/s²)

## Installation

### Dependencies
```bash
sudo apt install python3-numpy python3-matplotlib
pip3 install transforms3d
```

### Make scripts executable
```bash
chmod +x state_estimation/sensor_fusion_ekf.py
chmod +x state_estimation/visualize_state.py
```

## Usage

### Running the State Estimator

#### Option 1: Run directly with Python
```bash
# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Run the sensor fusion node
python3 state_estimation/sensor_fusion_ekf.py
```

#### Option 2: Using launch file
```bash
ros2 launch state_estimation/launch/state_estimation.launch.py
```

### Visualization
In a separate terminal, run the visualization tool:
```bash
python3 state_estimation/visualize_state.py
```

This will open a matplotlib window showing:
- Position trajectory (X-Y plot)
- Position over time
- Orientation over time
- Velocities over time
- Angular velocity over time
- Position uncertainty (covariance)

## Configuration

Edit [config/sensor_fusion.yaml](config/sensor_fusion.yaml) to configure:

```yaml
update_rate: 50.0        # Filter update rate in Hz
use_imu: true            # Enable/disable IMU
use_odometry: true       # Enable/disable odometry
use_gps: true            # Enable/disable GPS
use_lidar: true          # Enable/disable lidar scan matching
robot_namespace: 'a300_00000'  # Robot namespace
```

### Sensor Noise Parameters
You can tune the filter performance by adjusting noise parameters:
- `process_noise`: Model uncertainty
- `imu_noise`: IMU measurement noise
- `odom_noise`: Odometry measurement noise
- `gps_noise`: GPS measurement noise
- `lidar_noise`: LIDAR scan matching noise

## Topics

### Subscribed Topics
- `/{robot_namespace}/sensors/imu_0/data` (sensor_msgs/Imu)
- `/{robot_namespace}/platform/odom` (nav_msgs/Odometry)
- `/{robot_namespace}/sensors/gps_0/fix` (sensor_msgs/NavSatFix)
- `/{robot_namespace}/sensors/lidar2d_0/scan` (sensor_msgs/LaserScan)

### Published Topics
- `/state_estimate/pose` (geometry_msgs/PoseWithCovarianceStamped)
- `/state_estimate/twist` (geometry_msgs/TwistStamped)
- `/state_estimate/odom` (nav_msgs/Odometry)

## Algorithm Details

### Extended Kalman Filter
The EKF operates in two steps:

1. **Prediction Step** (runs at update_rate):
   - Uses motion model to predict next state
   - Motion model: constant acceleration model
   ```
   x_{k+1} = x_k + vx*dt + 0.5*ax*dt²
   y_{k+1} = y_k + vy*dt + 0.5*ay*dt²
   theta_{k+1} = theta_k + omega*dt
   vx_{k+1} = vx_k + ax*dt
   vy_{k+1} = vy_k + ay*dt
   ```

2. **Update Step** (runs when sensor data arrives):
   - Corrects prediction using sensor measurements
   - Each sensor provides partial state observations
   - Kalman gain weights prediction vs measurement based on uncertainties

### Sensor Integration
- **IMU**: Provides orientation (theta), angular velocity (omega), and linear acceleration (ax, ay)
- **Odometry**: Provides position (x, y), orientation (theta), and velocities (vx, vy, omega)
- **GPS**: Provides absolute position (x, y) in global frame - corrects long-term drift
- **Lidar**: Provides position (x, y) and orientation (theta) corrections via scan matching - improves local accuracy

### LIDAR Scan Matching
The LIDAR integration uses a simplified Iterative Closest Point (ICP) algorithm:
1. **Scan Conversion**: Converts polar laser scan data to Cartesian point cloud
2. **Point Correspondence**: Matches current scan points to previous scan points
3. **Displacement Estimation**: Calculates relative displacement (dx, dy, dtheta)
4. **EKF Update**: Fuses the displacement as a position/orientation measurement

**Benefits**:
- Corrects odometry drift in environments with static features
- Provides more frequent position updates than GPS
- More accurate than odometry for local positioning
- Works indoors where GPS is unavailable

**Limitations**:
- Requires static environment (moving objects can cause errors)
- Performance degrades in featureless environments (long corridors, open spaces)
- Computationally more expensive than other sensors

## Testing

### Monitor State Estimates
```bash
# View pose estimates
ros2 topic echo /state_estimate/pose

# View velocity estimates
ros2 topic echo /state_estimate/twist

# View full odometry
ros2 topic echo /state_estimate/odom
```

### Check Topics
```bash
# List all active topics
ros2 topic list

# Check sensor data rates
ros2 topic hz /a300_00000/sensors/imu_0/data
ros2 topic hz /a300_00000/platform/odom
```

## Troubleshooting

### No sensor data received
1. Check if sensors are publishing:
   ```bash
   ros2 topic list | grep sensors
   ros2 topic echo /a300_00000/sensors/imu_0/data
   ```

2. Verify robot namespace in config file matches your robot

### Filter divergence
1. Check sensor noise parameters - they may need tuning
2. Ensure sensors are properly calibrated
3. Verify sensor data quality

### High uncertainty
1. More sensors = lower uncertainty
2. Enable GPS if available for absolute position reference
3. Increase update rate if computational resources allow

## Future Improvements
- [ ] Implement Unscented Kalman Filter (UKF) for better nonlinear handling
- [ ] Add more sophisticated scan matching algorithms (e.g., NDT, GICP)
- [ ] Implement particle filter for multi-modal distributions
- [ ] Add adaptive noise covariance estimation
- [ ] Support for 3D state estimation
- [ ] Integration with SLAM algorithms
- [ ] Loop closure detection with LIDAR

## References
- Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics
- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). Estimation with Applications to Tracking and Navigation
- ROS 2 robot_localization package: http://docs.ros.org/en/jazzy/p/robot_localization/

## License
[Specify your license here]

## Author
Created for Clearpath A300 Robot Platform
