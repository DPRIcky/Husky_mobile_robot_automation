#!/usr/bin/env python3
"""
Spawn a second robot in Gazebo simulation with different namespace.
This script spawns a second Clearpath A300 robot alongside the first one.
"""

import os
import subprocess
import sys

def main():
    # Get the setup path from environment or use default
    setup_path = os.environ.get('CLEARPATH_SETUP_PATH',
                               os.path.join(os.path.expanduser('~'), 'clearpath'))

    # Spawn second robot with different namespace and position
    # Robot 2 will be named a300_00001 and positioned at x=5.0, y=0.0

    cmd = [
        'ros2', 'launch', 'clearpath_gz', 'simulation.launch.py',
        'setup_path:=' + setup_path,
        'world:=warehouse',
        'use_sim_time:=true',
        'x:=5.0',  # Position robot 2 5 meters ahead
        'y:=0.0',
        'z:=0.15',
        'yaw:=3.14159',  # 180 degrees - facing opposite direction
        'namespace:=a300_00001'  # Different namespace for second robot
    ]

    print(f"[spawn_second_robot] Spawning second robot with command:")
    print(' '.join(cmd))

    try:
        # Note: This will block as it launches the full simulation
        # For a true spawn-only approach, we'd use the robot_spawn.launch.py directly
        result = subprocess.run(cmd)
        return result.returncode
    except KeyboardInterrupt:
        print("\n[spawn_second_robot] Simulation interrupted by user")
        return 0
    except Exception as e:
        print(f"[spawn_second_robot] Error: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())