#!/bin/bash
# Quick Start Script for Autonomous Navigation
# Run all necessary components in tmux windows

SESSION="clearpath_nav"

# Check if tmux session exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
    echo "Creating new tmux session: $SESSION"
    
    # Create session with first window
    tmux new-session -d -s $SESSION -n "gazebo"
    tmux send-keys -t $SESSION:gazebo "cd /home/prajjwal/clearpath" C-m
    tmux send-keys -t $SESSION:gazebo "echo 'Starting Gazebo simulation...'" C-m
    tmux send-keys -t $SESSION:gazebo "ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/prajjwal/clearpath" C-m
    
    # Window 2: SLAM
    tmux new-window -t $SESSION -n "slam"
    tmux send-keys -t $SESSION:slam "cd /home/prajjwal/clearpath" C-m
    tmux send-keys -t $SESSION:slam "sleep 5 && echo 'Starting SLAM...'" C-m
    tmux send-keys -t $SESSION:slam "sleep 5 && ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath" C-m
    
    # Window 3: Nav2
    tmux new-window -t $SESSION -n "nav2"
    tmux send-keys -t $SESSION:nav2 "cd /home/prajjwal/clearpath" C-m
    tmux send-keys -t $SESSION:nav2 "sleep 10 && echo 'Starting Nav2...'" C-m
    tmux send-keys -t $SESSION:nav2 "sleep 10 && ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/prajjwal/clearpath" C-m
    
    # Window 4: RViz
    tmux new-window -t $SESSION -n "rviz"
    tmux send-keys -t $SESSION:rviz "cd /home/prajjwal/clearpath" C-m
    tmux send-keys -t $SESSION:rviz "sleep 15 && echo 'Starting RViz...'" C-m
    tmux send-keys -t $SESSION:rviz "sleep 15 && ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a300_00000 use_sim_time:=true" C-m
    
    # Window 5: Waypoint Navigator (manual start)
    tmux new-window -t $SESSION -n "waypoints"
    tmux send-keys -t $SESSION:waypoints "cd /home/prajjwal/clearpath" C-m
    tmux send-keys -t $SESSION:waypoints "echo ''" C-m
    tmux send-keys -t $SESSION:waypoints "echo '=== Waypoint Navigator ===' " C-m
    tmux send-keys -t $SESSION:waypoints "echo 'Wait for Nav2 to fully initialize (check other windows)'" C-m
    tmux send-keys -t $SESSION:waypoints "echo ''" C-m
    tmux send-keys -t $SESSION:waypoints "echo 'Available waypoint files:'" C-m
    tmux send-keys -t $SESSION:waypoints "echo '  - waypoints_square.yaml (5x5m square)'" C-m
    tmux send-keys -t $SESSION:waypoints "echo '  - waypoints_figure8.yaml (figure-8 pattern)'" C-m
    tmux send-keys -t $SESSION:waypoints "echo '  - waypoints_corridor.yaml (corridor patrol)'" C-m
    tmux send-keys -t $SESSION:waypoints "echo ''" C-m
    tmux send-keys -t $SESSION:waypoints "echo 'Example command (copy/paste):'" C-m
    tmux send-keys -t $SESSION:waypoints "echo 'python3 navigation/scripts/waypoint_navigator.py --ros-args -p waypoints_file:=/home/prajjwal/clearpath/navigation/config/waypoints_square.yaml -p mode:=loop -p use_sim_time:=true'" C-m
    tmux send-keys -t $SESSION:waypoints "echo ''" C-m
    
    echo "Session created! Attach with: tmux attach -t $SESSION"
    echo "Navigate windows: Ctrl+b then n (next) or p (previous)"
    echo "Detach: Ctrl+b then d"
    
    # Attach to session
    tmux attach -t $SESSION
else
    echo "Session $SESSION already exists. Attaching..."
    tmux attach -t $SESSION
fi
