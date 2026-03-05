# State Estimation System - Quick Reference

## 📁 Project Structure
```
state_estimation/
├── sensor_fusion_ekf.py          # Main EKF implementation
├── visualize_state.py            # Real-time visualization tool
├── test_mock_sensors.py          # Mock sensor data generator
├── run_test.sh                   # Quick test script
├── README.md                     # Full documentation
├── config/
│   └── sensor_fusion.yaml        # Configuration parameters
└── launch/
    └── state_estimation.launch.py # ROS2 launch file
```

## 🚀 Quick Start

### Option 1: Interactive Test Script (Recommended)
```bash
cd /home/prajjwal/clearpath
./state_estimation/run_test.sh
```
Then select:
- **Option 1**: Test with mock sensors (no robot needed)
- **Option 2**: Test with real robot/simulation

### Option 2: Manual Commands

**Test with mock sensors:**
```bash
# Terminal 1: Mock sensors (simulates circular motion)
python3 state_estimation/test_mock_sensors.py

# Terminal 2: State estimator
python3 state_estimation/sensor_fusion_ekf.py

# Terminal 3: Visualizer
python3 state_estimation/visualize_state.py
```

**Test with real robot:**
```bash
# Terminal 1: State estimator
python3 state_estimation/sensor_fusion_ekf.py

# Terminal 2: Visualizer
python3 state_estimation/visualize_state.py
```

## 📊 What Does It Do?

### Input Sensors
- **IMU**: Orientation, angular velocity, acceleration
- **Odometry**: Position, velocity from wheel encoders
- **GPS**: Absolute position (optional)
- **Lidar**: For scan matching (future)

### Output State Estimate
- Position: `(x, y)` in meters
- Orientation: `theta` in radians
- Linear velocity: `(vx, vy)` in m/s
- Angular velocity: `omega` in rad/s
- Linear acceleration: `(ax, ay)` in m/s²
- Covariance: Uncertainty estimates

### Published Topics
```bash
/state_estimate/pose     # Pose with covariance
/state_estimate/twist    # Velocity
/state_estimate/odom     # Combined odometry
```

## 🔧 Configuration

Edit `state_estimation/config/sensor_fusion.yaml`:

```yaml
update_rate: 50.0        # Filter frequency (Hz)
use_imu: true           # Enable IMU
use_odometry: true      # Enable odometry
use_gps: false          # Enable GPS
robot_namespace: 'a300_00000'
```

## 📈 Monitoring

### View Topics
```bash
# List state estimate topics
ros2 topic list | grep state_estimate

# View pose estimates
ros2 topic echo /state_estimate/pose

# Check update rate
ros2 topic hz /state_estimate/odom
```

### View Plots
The visualizer shows 6 real-time plots:
1. **Position Trajectory**: 2D path (X-Y)
2. **Position vs Time**: X and Y over time
3. **Orientation vs Time**: Theta (yaw angle)
4. **Velocity vs Time**: Vx and Vy
5. **Angular Velocity**: Omega
6. **Uncertainty**: Standard deviation of position

## 🧪 Testing Scenarios

### 1. Stationary Robot
- State should remain stable
- Uncertainty should decrease over time
- Velocity should be near zero

### 2. Circular Motion (Mock Sensors)
- Should see smooth circular trajectory
- Constant angular velocity
- Centripetal acceleration visible

### 3. Straight Line Motion
- Position should increase linearly
- Orientation should remain constant
- Velocity should be steady

## 🎯 Key Features

### Extended Kalman Filter
- **Prediction**: Uses motion model (constant acceleration)
- **Update**: Fuses sensor measurements with Kalman gain
- **Covariance**: Tracks estimate uncertainty

### Sensor Fusion Strategy
1. **Prediction Step** (50 Hz):
   - Predicts next state using motion model
   - Increases uncertainty

2. **IMU Update** (High frequency):
   - Corrects orientation and acceleration
   - Low latency

3. **Odometry Update** (Medium frequency):
   - Corrects position and velocity
   - Good relative accuracy

4. **GPS Update** (Low frequency, optional):
   - Corrects absolute position
   - Prevents drift

## ⚙️ Tuning Tips

### If filter is too smooth (lags behind):
- Decrease measurement noise values
- Increase process noise

### If filter is too noisy (jittery):
- Increase measurement noise values
- Decrease process noise

### If uncertainty grows too fast:
- Decrease process noise
- Check sensor data quality

### If estimates drift:
- Enable GPS for absolute reference
- Check odometry calibration
- Verify IMU bias compensation

## 🐛 Troubleshooting

### No visualization appearing
```bash
# Install matplotlib
pip3 install matplotlib numpy
```

### No sensor data
```bash
# Check sensor topics
ros2 topic list | grep sensors
ros2 topic echo /a300_00000/sensors/imu_0/data
```

### Import errors
```bash
# Install dependencies
pip3 install numpy transforms3d matplotlib
```

### Filter not converging
1. Check sensor data quality
2. Verify noise parameters in config
3. Ensure sensors are publishing at expected rates

## 📚 Algorithm Details

### State Vector (8D)
```
[x, y, theta, vx, vy, omega, ax, ay]
```

### Motion Model
```
x_new = x + vx*dt + 0.5*ax*dt²
y_new = y + vy*dt + 0.5*ay*dt²
theta_new = theta + omega*dt
vx_new = vx + ax*dt
vy_new = vy + ay*dt
```

### Update Equations
```
Innovation: y = z - H*x
Kalman Gain: K = P*H'*inv(H*P*H' + R)
State Update: x = x + K*y
Covariance Update: P = (I - K*H)*P
```

## 🔗 Next Steps

1. **Tune Parameters**: Adjust noise covariance in config
2. **Add Scan Matching**: Implement lidar-based corrections
3. **Compare with robot_localization**: Benchmark against ROS2 EKF
4. **Log Data**: Record and analyze filter performance
5. **Add UKF**: Try Unscented Kalman Filter for better nonlinearity handling

## 📞 Support

For issues or questions:
- Check README.md for detailed documentation
- Review configuration in sensor_fusion.yaml
- Monitor ROS2 topics for data flow
- Use visualization for debugging

---
**Created**: February 18, 2026  
**Platform**: Clearpath A300 - ROS2 Jazzy
