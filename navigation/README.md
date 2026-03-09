# Autonomous Navigation for Clearpath A300

Complete autonomous navigation system with Nav2, EKF state estimation, and multi-sensor fusion.

## Overview

This package provides autonomous path planning and obstacle avoidance for the Clearpath A300 (Husky) robot using:

- **Nav2** - Navigation2 stack for planning and control
- **EKF** - Extended Kalman Filter for state estimation (robot_localization)
- **All 4 Sensors Integrated**:
  - IMU (6-axis) - Angular velocity and linear acceleration
  - GPS - Global positioning (optional)
  - LiDAR - 2D laser scan for obstacle detection
  - Depth Camera - 3D pointcloud for obstacle detection

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      SENSOR INPUTS                           │
├──────────┬──────────┬──────────┬─────────────────────────────┤
│   IMU    │   GPS    │  LiDAR   │  Depth Camera               │
│ 100 Hz   │  10 Hz   │  40 Hz   │    30 Hz                    │
└────┬─────┴────┬─────┴────┬─────┴──────┬──────────────────────┘
     │          │          │            │
     │          │          │            │
     v          v          v            v
┌────────────────────┐  ┌───────────────────────────┐
│  robot_localization│  │   Nav2 Costmaps           │
│       (EKF)        │  │  - Obstacle Layer (LiDAR) │
│                    │  │  - Voxel Layer (Camera)   │
│  odom→base_link TF │  │  - Inflation Layer        │
└─────────┬──────────┘  └────────────┬──────────────┘
          │                          │
          v                          v
     ┌─────────────────────────────────────┐
     │         Nav2 Stack                  │
     │  - Global Planner (NavFn)           │
     │  - Local Planner (DWB)              │
     │  - Recovery Behaviors               │
     │  - Behavior Tree Navigator          │
     └──────────────┬──────────────────────┘
                    │
                    v
            ┌───────────────────┐
            │  cmd_vel commands │
            │  to robot         │
            └───────────────────┘
```

## Files Structure

```
navigation/
├── config/
│   ├── nav2_params.yaml              # Nav2 configuration
│   ├── localization_gps.yaml         # GPS-enhanced EKF config
│   ├── waypoints_square.yaml         # Example square pattern
│   ├── waypoints_figure8.yaml        # Example figure-8 pattern
│   └── waypoints_corridor.yaml       # Example corridor patrol
├── launch/
│   └── navigation.launch.py          # Main navigation launch
└── scripts/
    └── waypoint_navigator.py         # Waypoint navigation node
```

## Quick Start - Official Clearpath Packages

### Scenario 1: Nav2 + SLAM (Mapping)
Use this for exploring new environments and building maps:

```bash
# Terminal 1: Gazebo simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2: Nav2 stack initialization
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3: SLAM for mapping
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 4: Visualization (RViz)
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

### Scenario 2: Nav2 + Localization (Known Map)
Use this for autonomous navigation with a pre-built map:

```bash
# Terminal 1: Gazebo simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2: Nav2 stack initialization
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3: Localization (MCL - Monte Carlo Localization)
ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 4: Visualization (RViz)
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

### Scenario 3: Run Waypoint Navigation (Custom)
Once Nav2 is running (either SLAM or localization), send autonomous goals:

**First time setup (build the package):**
```bash
cd /home/prajjwal/clearpath
colcon build --packages-select navigation
```

**Then run the waypoint navigator:**
```bash
# Terminal 5: Waypoint navigator
source /home/prajjwal/clearpath/install/setup.bash
ros2 run navigation waypoint_navigator \
    --ros-args \
    -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
    -p mode:=loop
```

**Or use convenience alias after sourcing:**
```bash
ros2 run navigation waypoint_navigator \
    --ros-args \
    -p waypoints_file:=./navigation/config/waypoints_square.yaml \
    -p mode:=loop
```

## Sensor Integration Details

### IMU (Microstrain)
- **Topic**: `/a300_00000/sensors/imu_0/data`
- **Rate**: 100 Hz
- **Used For**: 
  - EKF: Angular velocity, linear acceleration
  - Orientation stabilization

### GPS (Garmin 18x)
- **Topic**: `/a300_00000/sensors/gps_0/fix`
- **Rate**: 10 Hz
- **Used For**: 
  - Global position (via navsat_transform)
  - Long-term drift correction
  - Outdoor navigation

### LiDAR (Hokuyo UST)
- **Topic**: `/a300_00000/lidar2d_0/scan`
- **Rate**: 40 Hz
- **Range**: 0.05m - 25m
- **Used For**:
  - SLAM/Localization
  - Costmap obstacle detection
  - Collision avoidance

### Depth Camera (RealSense D435)
- **Topic**: `/a300_00000/camera_0/points`
- **Rate**: 30 Hz
- **Used For**:
  - 3D obstacle detection
  - Costmap voxel layer
  - Low-height obstacle detection

## Navigation Modes

### Single Run
Visit waypoints once then stop:
```bash
-p mode:=single
```

### Loop Mode
Continuously repeat waypoints:
```bash
-p mode:=loop
```

### Patrol Mode
Go forward through waypoints, then backward (patrol):
```bash
-p mode:=patrol
```

## Creating Custom Waypoints

Create a YAML file with waypoints in map frame:

```yaml
waypoints:
  - x: 0.0
    y: 0.0
    yaw: 0.0
    
  - x: 5.0
    y: 0.0
    yaw: 1.5708  # 90 degrees
    
  - x: 5.0
    y: 5.0
    yaw: 3.14159  # 180 degrees
```

### Coordinate System
- **Frame**: `map` (from SLAM or pre-built map)
- **Units**: meters
- **Yaw**: radians (0 = east, π/2 = north)

## Nav2 Configuration

### Global Costmap
- **Frame**: `map`
- **Update Rate**: 1 Hz
- **Layers**: Static map, obstacles (LiDAR + camera), inflation
- **Robot Radius**: 0.35m
- **Inflation Radius**: 0.55m

### Local Costmap
- **Frame**: `odom`
- **Update Rate**: 5 Hz
- **Rolling Window**: 3m x 3m
- **Layers**: Voxel (obstacles), inflation
- **Resolution**: 0.05m

### Controller (DWB)
- **Max Linear Velocity**: 0.8 m/s
- **Max Angular Velocity**: 1.0 rad/s
- **Control Frequency**: 20 Hz
- **Goal Tolerance**: 0.25m (xy), 0.25 rad (yaw)

### Planner (NavFn)
- **Algorithm**: Dijkstra (use_astar: false)
- **Allows Unknown**: true
- **Tolerance**: 0.5m

## Tuning Parameters

### Increase Speed
Edit `navigation/config/nav2_params.yaml`:
```yaml
FollowPath:
  max_vel_x: 1.2  # Increase from 0.8
  max_vel_theta: 1.5  # Increase from 1.0
```

### Tighter Obstacle Avoidance
```yaml
inflation_layer:
  inflation_radius: 0.75  # Increase from 0.55
  cost_scaling_factor: 5.0  # Increase from 3.0
```

### More Aggressive Planning
```yaml
FollowPath:
  acc_lim_x: 3.5  # Increase from 2.5
  decel_lim_x: -3.5  # Increase from -2.5
```

## Troubleshooting

### Robot Not Moving
1. Check that Nav2 nodes are running:
   ```bash
   ros2 node list | grep nav2
   ```

2. Check costmaps are updating:
   ```bash
   ros2 topic hz /a300_00000/local_costmap/costmap
   ```

3. Verify TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Planner Fails to Find Path
1. Check if goal is in obstacle:
   - View costmaps in RViz
   - Reduce inflation radius

2. Increase planner tolerance:
   ```yaml
   GridBased:
     tolerance: 1.0  # Increase from 0.5
   ```

### Robot Oscillates or Gets Stuck
1. Tune DWB critics:
   ```yaml
   PathAlign.scale: 48.0  # Increase from 32.0
   GoalAlign.scale: 36.0  # Increase from 24.0
   ```

2. Enable recovery behaviors in RViz
3. Check for TF delays (should be < 200ms)

### Costmap Not Updating from Sensors
1. Check sensor topics are publishing:
   ```bash
   ros2 topic hz /a300_00000/lidar2d_0/scan
   ros2 topic hz /a300_00000/camera_0/points
   ```

2. Verify frame_ids match between sensors and costmap config

3. Check observation source ranges:
   ```yaml
   scan:
     obstacle_max_range: 2.5  # Should be less than sensor max
     raytrace_max_range: 3.0
   ```

### GPS Not Working
1. Ensure navsat_transform is running:
   ```bash
   ros2 node list | grep navsat
   ```

2. Check GPS fix quality:
   ```bash
   ros2 topic echo /a300_00000/sensors/gps_0/fix
   ```

3. Set proper datum (first GPS position)

## Advanced Features

### Save Map After SLAM
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Use Pre-Built Map
```bash
ros2 launch /home/prajjwal/clearpath/navigation/launch/navigation.launch.py \
    map_file:=/path/to/my_map.yaml
```

### Monitor Navigation Status
```bash
# Waypoint progress
ros2 topic echo /waypoint_status

# Current goal
ros2 topic echo /goal_pose

# Path visualization
# Add /plan topic in RViz
```

### Interactive Goal Setting
In RViz:
1. Click "2D Goal Pose" button
2. Click and drag on map to set goal position and orientation
3. Robot will plan and execute path

## Performance Tips

1. **Use composition** for better performance:
   ```yaml
   use_composition: True  # In launch file
   ```

2. **Reduce costmap size** for faster planning:
   ```yaml
   width: 5  # Instead of 10
   height: 5
   ```

3. **Lower update rates** if CPU limited:
   ```yaml
   update_frequency: 3.0  # Instead of 5.0
   ```

4. **Disable unused sensors**:
   - Comment out pointcloud if not needed
   - Use only LiDAR for 2D navigation

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [robot_localization](http://docs.ros.org/en/jazzy/p/robot_localization/)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)

## License

See project root for license information.

---

**Last Updated**: 2026-03-05
