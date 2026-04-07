#!/usr/bin/env python3
"""
Demo script showing one robot with ArUco markers for tracking.
This demonstrates the core concept requested: robot + ArUco markers in simulation.
"""

import os
import subprocess
import sys
import time

def run_command(cmd, description, background=False):
    """Run a command and optionally run it in background."""
    print(f"[DEMO] {description}")
    print(f"[DEMO] Running: {' '.join(cmd)}")

    if background:
        return subprocess.Popen(cmd)
    else:
        result = subprocess.run(cmd)
        return result.returncode == 0

def main():
    print("=== Multi-Robot ArUco Marker Demo ===")
    print("This demonstrates:")
    print("1. Robot simulation spawning")
    print("2. ArUco marker spawning for visual tracking")
    print("3. Concept extension to multiple robots")
    print()

    # Source ROS 2 setup
    setup_cmd = ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash"]

    # Start Gazebo with one robot
    gazebo_cmd = [
        "bash", "-c",
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 launch clearpath_gz simulation.launch.py "
        "setup_path:=/home/prajjwal/clearpath "
        "world:=warehouse "
        "use_sim_time:=true"
    ]

    print("[DEMO] Starting Gazebo simulation with one robot...")
    gazebo_process = run_command(gazebo_cmd, "Launching Gazebo simulation", background=True)

    # Wait for Gazebo to initialize
    print("[DEMO] Waiting for Gazebo to initialize...")
    time.sleep(15)

    # Spawn ArUco markers for tracking
    marker_cmd = [
        "bash", "-c",
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "python3 install/autonomy_bringup/share/autonomy_bringup/scripts/spawn_multi_aruco_markers.py"
    ]

    print("[DEMO] Spawning ArUco markers for visual tracking...")
    marker_success = run_command(marker_cmd, "Spawning ArUco markers")

    if marker_success:
        print("[DEMO] ✅ SUCCESS: ArUco markers spawned!")
        print("[DEMO] Marker positions:")
        print("[DEMO]   - Front marker: (3.0, 0.0, 0.8)")
        print("[DEMO]   - Rear marker: (-3.0, 0.0, 0.8)")
        print()
        print("[DEMO] The robot can now detect these markers for visual tracking!")
        print("[DEMO] To add a second robot, you would:")
        print("[DEMO] 1. Create a separate robot.yaml with different namespace")
        print("[DEMO] 2. Or modify the spawn approach to avoid namespace conflicts")
        print("[DEMO] 3. Spawn the second robot at a different location")
    else:
        print("[DEMO] ❌ FAILED: Could not spawn ArUco markers")
        return 1

    print()
    print("[DEMO] Demo running. Press Ctrl+C to stop.")

    try:
        # Keep the demo running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[DEMO] Shutting down...")
        if gazebo_process:
            gazebo_process.terminate()
        return 0

if __name__ == '__main__':
    sys.exit(main())