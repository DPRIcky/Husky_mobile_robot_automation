# Why Robot Isn't Moving - Debugging Guide

## Current Status from Diagnostics

✅ **Working:**
- All Nav2 nodes running (bt_navigator, controller_server, planner_server)
- Action servers ready (follow_waypoints, navigate_to_pose)
- Costmap services available

❌ **NOT Working:**
- cmd_vel NOT publishing
- AMCL pose NOT available

---

## The Problem Chain

```
No AMCL Pose
    ↓
Robot doesn't know its location
    ↓
Planner won't plan (no starting point)
    ↓
Controller has no path to follow
    ↓
cmd_vel NOT published
    ↓
❌ ROBOT DOESN'T MOVE
```

---

## Fix 1: Set Initial Pose (Most Important)

### In RViz:
1. **Find the "2D Pose Estimate" button**
   - Look in toolbar at top of RViz window
   - It's a green arrow icon (looks like: →)
   
2. **Click the button** (Select 2D Pose Estimate tool)

3. **Click on the map**
   - Click near where the Gazebo robot appears
   - Usually around coordinates (0, 0) in corner of map

4. **Drag to set heading**
   - Hold mouse button and drag
   - Arrow should rotate to point forward

5. **Release mouse**

### Verify in terminal:
```bash
# This should show robot position:
ros2 topic echo --once /a300_00000/amcl_pose

# Should output something like:
# header:
#   frame_id: map
# pose:
#   pose:
#     position:
#       x: 0.15    # Robot X position
#       y: -0.08   # Robot Y position
```

---

## Fix 2: Check If Localization Is Working

```bash
# Terminal: Check if AMCL is updating
source /home/prajjwal/clearpath/install/setup.bash

# Watch AMCL pose updates
ros2 topic hz /a300_00000/amcl_pose
# Should show updates like: average rate: 2.00

# If NO updates, AMCL isn't running or localized
```

---

## Fix 3: Manually Test Path Planning

If AMCL pose is set but cmd_vel still doesn't publish:

```bash
# Send a simple test goal
ros2 action send_goal /a300_00000/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  'pose: {header: {frame_id: "map"}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}'

# Watch for:
# 1. Goal accepted → Planner working
# 2. Feedback updates → Path being computed
# 3. cmd_vel starts publishing → Controller working
```

---

## Fix 4: Check Individual Components

### Check if planner is computing paths:
```bash
ros2 topic echo /a300_00000/plan
# Should see path messages when you send goals
```

### Check if controller receives paths:
```bash
ros2 topic echo /a300_00000/local_plan
# Should see local trajectory updates
```

### Check costmaps are updating:
```bash
ros2 topic hz /a300_00000/local_costmap/costmap
# Should see ~5 Hz updates
ros2 topic hz /a300_00000/global_costmap/costmap
# Should see ~1 Hz updates
```

---

## Fix 5: Check Nav2 Logs

Look for errors:

```bash
# Watch controller output
ros2 node info /a300_00000/controller_server

# Watch planner output  
ros2 node info /a300_00000/planner_server

# Watch BT Navigator
ros2 node info /a300_00000/bt_navigator
```

---

## Common Issues & Solutions

### Issue: "No valid costmap"
**Solution:** Localization system hasn't converged
- Ensure AMCL pose is set in RViz
- Wait 5+ seconds for AMCL to get good estimate
- Check: `ros2 topic hz /a300_00000/amcl_pose`

### Issue: "Path planning failed"
**Solution:** Goal might be unreachable
- Try goal closer to current position (0.5m away)
- Ensure goal is in map frame and map is loaded
- Check: `ros2 topic echo /a300_00000/global_costmap/costmap`

### Issue: "Planner timeout"
**Solution:** Planner needs more time
- Increase `/a300_00000/planner_server` timeout
- Try simpler paths first (straight line)
- Check CPU load: `top`

### Issue: "Transform failed"
**Solution:** TF tree issue
- Check: `ros2 run tf2_tools view_frames`
- Should show: `map` → `odom` → `base_link`
- If missing links, restart localization system

---

## Quick Recovery Sequence

Run this for automatic diagnostic + attempted fix:

```bash
chmod +x /home/prajjwal/clearpath/nav_recovery.sh
bash /home/prajjwal/clearpath/nav_recovery.sh
```

This will:
1. Prompt you to set initial pose if needed
2. Send a test goal to verify everything works
3. Tell you if robot is ready for waypoint navigation

---

## If All Else Fails

**Nuclear Option - Restart Everything:**

```bash
# Terminal 1: Kill all current processes
pkill -f "ros2 launch" || true
sleep 2

# Terminal 1: Fresh start
roscd && cd /home/prajjwal/clearpath
source ./install/setup.bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath

# Terminal 2 (after 10s)
source ./install/setup.bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath
ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath

# Terminal 3 (after 30s)
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true

# WAIT 30 MORE SECONDS

# Terminal 4
source ./install/setup.bash
bash nav_recovery.sh
```

---

## Success Indicators

You'll know it's working when:

✅ AMCL pose shows robot position  
✅ Sending a goal → cmd_vel publishes  
✅ Robot moves toward goal in Gazebo  
✅ Robot stops at goal  
✅ Ready for waypoint navigation  

