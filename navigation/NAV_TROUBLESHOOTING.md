# Navigation Stack Troubleshooting Guide

**Issue:** Robot is not moving - No cmd_vel commands being published

## Root Causes & Fixes

### ❌ **Problem 1: FollowWaypoints Action Server Not Ready**

**Symptoms:**
- Waypoint navigator logs show: `Waiting for FollowWaypoints action server`
- No movement happens

**Causes:**
- Nav2 stack hasn't fully initialized yet
- BT Navigator not ready
- Action servers still binding

**Fix:**
```bash
# Give Nav2 MORE TIME to initialize (30+ seconds after launch)
# OR wait for this log message:
# [bt_navigator]: BT Navigator initialized successfully

# Then wait additional 10 seconds before starting waypoint navigator

# Terminal 1: Start everything and WAIT 30 seconds
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
# WAIT 30+ SECONDS HERE

# Terminal 2: Check if nav2 is ready
source /home/prajjwal/clearpath/install/setup.bash
ros2 action list | grep follow

# Terminal 3: ONLY AFTER you see /a300_00000/follow_waypoints, run:
ros2 run navigation waypoint_navigator --ros-args \
  -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
  -p mode:=loop
```

---

### ❌ **Problem 2: Initial Pose Not Set**

**Symptoms:**
- AMCL errors about transform extrapolation
- TF tree errors
- costmap not initializing

**Causes:**
- Robot doesn't know where it is on the map
- AMCL needs initial pose estimate

**Fix:**

In **RViz**, set initial pose:
1. Click **"2D Pose Estimate"** button (green arrow button)
2. Click and drag on the map where the robot should start
3. Rotate to set orientation
4. Release

**Or via ROS:**
```bash
ros2 topic pub -1 /a300_00000/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
   pose: {
     pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
            orientation: {x: 0, y: 0, z: 0, w: 1.0}},
     covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
   }"
```

---

### ❌ **Problem 3: Nav2 Not Publishing cmd_vel**

**Symptoms:**
- `ros2 topic hz /a300_00000/cmd_vel_bridge/input/cmd_vel` shows NO DATA
- Robot stays still even after setting pose

**Causes:**
- Controller server hasn't received valid goal yet
- Goal tolerance too tight
- Planner couldn't find valid path

**Fix:**

Check if controller is working:
```bash
# Manually send a test goal
ros2 action send_goal /a300_00000/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Watch for:
- Goal accepted?
- Path computed?
- cmd_vel starts publishing?

---

### ❌ **Problem 4: Transform Cache Too Small**

**Symptoms:**
- Messages: `"Message is so big that it took longer than all the data in the transform cache"`
- TF lookup failures
- Controller can't compute path

**Causes:**
- Too much delay between planning and execution
- Slow computer or high load
- Timestamp skew

**Fix:**

Reduce update frequencies in navigation:
```bash
# Edit nav2_params.yaml:
bt_navigator:
  ros__parameters:
    bt_loop_duration: 20      # Increase from 10 (slower updates = better stability)
    
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Reduce from 20 (more time per update)

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 2.0   # Reduce from 5.0
      publish_frequency: 1.0  # Reduce from 2.0
```

---

## ✅ **Step-by-Step Fix Sequence**

### **Step 1: Start Gazebo & Nav2 (Wait 40 seconds)**
```bash
# T1: Gazebo + Nav2
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# T2: Visualization
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true
```

**Wait 40 seconds while Nav2 initializes...**

### **Step 2: Verify Action Servers Are Ready**
```bash
# T3: Diagnostics terminal
source /home/prajjwal/clearpath/install/setup.bash
ros2 action list | grep follow
# Should see: /a300_00000/follow_waypoints
```

**If you DON'T see it, go back to Step 1 and wait longer**

### **Step 3: Set Initial Pose in RViz**
- Click "2D Pose Estimate" (green arrow)
- Click on map at robot's location
- Drag to set heading
- Release

### **Step 4: Verify Costmaps Are Loaded**
```bash
# Check costmap topics
ros2 topic hz /a300_00000/local_costmap/costmap
# Should see: ~5Hz
```

### **Step 5: Check cmd_vel Publishing**
```bash
# Send test goal
ros2 action send_goal /a300_00000/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# Watch for cmd_vel (in another terminal)
ros2 topic echo /a300_00000/cmd_vel_bridge/input/cmd_vel
```

### **Step 6: Start Waypoint Navigator**
```bash
# ONLY after all above checks pass
ros2 run navigation waypoint_navigator --ros-args \
  -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \
  -p mode:=loop
```

---

## 🔧 **Quick Diagnostics Commands**

```bash
# See all nodes
ros2 node list

# See all action servers
ros2 action list

# Test FollowWaypoints manually
ros2 action send_goal /a300_00000/follow_waypoints \
  nav2_msgs/action/FollowWaypoints \
  "poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}, {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0}}}]"

# Monitor TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link

# Monitor costmap updates
ros2 topic hz /a300_00000/local_costmap/costmap
ros2 topic hz /a300_00000/global_costmap/costmap

# See robot state
ros2 topic echo /a300_00000/amcl_pose
```

---

## 🎯 **Expected Behavior**

✅ **Correct sequence:**
1. AMCL shows good pose estimate
2. Costmaps publish regularly
3. BT Navigator accepts goals
4. Controller publishes cmd_vel
5. Robot moves smoothly to waypoint
6. Repeats for each waypoint

❌ **Problem indicators:**
- Any "action server not available"
- Any TF lookup failures
- cmd_vel not publishing
- Costmaps not updating
- AMCL showing strange poses

---

## 📋 **Use the Diagnostics Script**

```bash
chmod +x /home/prajjwal/clearpath/nav_diagnostics.sh
/home/prajjwal/clearpath/nav_diagnostics.sh
```

This will show you **exactly** which components are missing.
