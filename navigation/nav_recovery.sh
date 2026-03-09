#!/bin/bash
# Navigation Recovery Script
# Run this to get the robot moving

source /home/prajjwal/clearpath/install/setup.bash

echo "=========================================="
echo "STEP 1: VERIFY INITIAL POSE IS SET"
echo "=========================================="
echo ""
echo "🔍 Checking AMCL pose..."
timeout 3 ros2 topic echo --once /a300_00000/amcl_pose 2>/dev/null

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ NO AMCL POSE FOUND!"
    echo ""
    echo "⚠️  YOU MUST SET THE INITIAL POSE IN RVIZ:"
    echo "   1. Look for the GREEN ARROW button labeled '2D Pose Estimate'"
    echo "   2. CLICK the button"
    echo "   3. CLICK on the map to place robot"
    echo "   4. DRAG to rotate the arrow to set heading"
    echo "   5. RELEASE the mouse"
    echo ""
    echo "📍 Try clicking near coordinates (0, 0) on the map"
    read -p "Press ENTER after setting pose in RViz, then I'll check again..."
    timeout 3 ros2 topic echo --once /a300_00000/amcl_pose 2>/dev/null
    
    if [ $? -ne 0 ]; then
        echo "❌ Still no pose detected. Retry RViz step above."
        exit 1
    fi
fi

echo ""
echo "✅ AMCL pose detected!"
echo ""

echo "=========================================="
echo "STEP 2: TEST cmd_vel PUBLISHING"
echo "=========================================="
echo ""
echo "🔍 Attempting to send a test navigation goal..."
echo ""

# Send a simple test goal 1 meter forward
ros2 action send_goal /a300_00000/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  'pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' \
  --feedback &

GOAL_PID=$!

echo ""
echo "⏳ Waiting 3 seconds for goal to be processed..."
sleep 3

echo ""
echo "🔍 Checking if cmd_vel is now publishing..."
timeout 2 ros2 topic hz /a300_00000/cmd_vel_bridge/input/cmd_vel 2>/dev/null

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ SUCCESS! cmd_vel is publishing at expected rate!"
    echo ""
    echo "📊 Now you can start waypoint navigator:"
    echo ""
    echo "   ros2 run navigation waypoint_navigator --ros-args \\"
    echo "     -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml \\"
    echo "     -p mode:=loop"
    echo ""
else
    echo ""
    echo "❌ cmd_vel still not publishing after test goal"
    echo ""
    echo "📋 DEBUGGING NEEDED:"
    echo "   1. Check if planner could compute a path"
    echo "   2. Check controller state"
    echo "   3. Check for errors in logs"
    echo ""
    echo "Try these commands:"
    echo "   ros2 node list | grep planner"
    echo "   ros2 node list | grep controller"
    echo "   ros2 topic echo /a300_00000/plan"
    exit 1
fi

# Stop the goal action
wait $GOAL_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "✅ RECOVERY COMPLETE"
echo "=========================================="
