#!/bin/bash
# Navigation Stack Diagnostics
# Run in new terminal after sourcing install

source /home/prajjwal/clearpath/install/setup.bash

echo "=========================================="
echo "NAV2 DIAGNOSTICS - $(date)"
echo "=========================================="

echo ""
echo "1️⃣ CHECKING NODES..."
echo "---"
ros2 node list | grep -E "(nav2|controller|planner|follow|navigate)" | head -15

echo ""
echo "2️⃣ CHECKING ACTION SERVERS..."
echo "---"
ros2 action list | grep -E "(navigate|follow)" | head -10

echo ""
echo "3️⃣ CHECKING CMD_VEL PUBLISHING..."
echo "---"
timeout 2 ros2 topic hz /a300_00000/cmd_vel_bridge/input/cmd_vel 2>/dev/null || echo "❌ NO cmd_vel BEING PUBLISHED"

echo ""
echo "4️⃣ CHECKING TF TREE..."
echo "---"
ros2 tf2_tools view_frames 2>&1 | grep -E "(map|odom|base_link)" | head -5

echo ""
echo "5️⃣ CHECKING AMCL POSE..."
echo "---"
timeout 2 ros2 topic echo --once /a300_00000/amcl_pose 2>/dev/null || echo "❌ NO AMCL POSE"

echo ""
echo "6️⃣ CHECKING COSTMAP STATUS..."
echo "---"
ros2 service list | grep -E "(costmap|clear)" | head -5

echo ""
echo "=========================================="
echo "✅ Diagnostics Complete"
echo "=========================================="
