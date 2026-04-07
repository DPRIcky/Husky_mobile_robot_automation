#!/usr/bin/env python3
"""
Spawn multiple robots into a running Gazebo Sim world using direct model spawning.
This approach gives more control over robot positioning and namespacing.
"""

import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

DEFAULT_WORLD = 'warehouse'

def get_robot_model_xml(robot_name: str) -> str:
    """Get the robot model XML from the robot_description topic."""
    # In a real implementation, we would get this from the parameter server or file
    # For now, we'll return a placeholder that indicates we should use the standard spawn approach
    return None

def spawn_robot_via_ros_gz(robot_name: str, namespace: str,
                          x: float, y: float, z: float, yaw: float,
                          world: str) -> bool:
    """Spawn a robot using ros_gz_sim create with robot_description topic."""
    print(f'[spawn_multi_robots] Spawning robot {robot_name} with namespace {namespace} at '
          f'x={x}, y={y}, z={z}, yaw={yaw} in world "{world}"')

    # Use the standard robot spawning approach via the clearpath_gz package
    # This is more reliable than trying to extract and respawn the URDF
    cmd = [
        'ros2', 'launch', 'clearpath_gz', 'robot_spawn.launch.py',
        f'world:={world}',
        f'use_sim_time:=true',
        f'x:={x}',
        f'y:={y}',
        f'z:={z}',
        f'yaw:={yaw}',
        f'namespace:={namespace}'
    ]

    print(f'[spawn_multi_robots] Executing: {" ".join(cmd)}')

    try:
        result = subprocess.run(cmd, timeout=30)  # 30 second timeout
        if result.returncode == 0:
            print(f'[spawn_multi_robots] Robot {robot_name} spawned successfully')
            return True
        else:
            print(f'[spawn_multi_robots] Failed to spawn robot {robot_name} (exit code {result.returncode})')
            return False
    except subprocess.TimeoutExpired:
        print(f'[spawn_multi_robots] Timeout spawning robot {robot_name}')
        return False
    except Exception as e:
        print(f'[spawn_multi_robots] Error spawning robot {robot_name}: {e}')
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Spawn multiple Clearpath robots into a running Gazebo Sim world.'
    )
    parser.add_argument('--world', default=DEFAULT_WORLD, help='Gazebo world name')
    parser.add_argument('--robot1-ns', default='a300_00000', help='Namespace for robot 1')
    parser.add_argument('--robot2-ns', default='a300_00001', help='Namespace for robot 2')
    parser.add_argument('--robot1-x', type=float, default=0.0, help='X position of robot 1')
    parser.add_argument('--robot1-y', type=float, default=0.0, help='Y position of robot 1')
    parser.add_argument('--robot2-x', type=float, default=5.0, help='X position of robot 2')
    parser.add_argument('--robot2-y', type=float, default=0.0, help='Y position of robot 2')
    parser.add_argument('--z', type=float, default=0.15, help='Z position for both robots')
    parser.add_argument('--robot1-yaw', type=float, default=0.0, help='Yaw for robot 1')
    parser.add_argument('--robot2-yaw', type=float, default=3.14159, help='Yaw for robot 2 (180 deg)')

    args = parser.parse_args()

    print('[spawn_multi_robots] Starting multi-robot spawn process...')
    print(f'[spawn_multi_robots] World: {args.world}')

    # Spawn robot 1
    success1 = spawn_robot_via_ros_gz(
        'robot1', args.robot1_ns,
        args.robot1_x, args.robot1_y, args.z, args.robot1_yaw,
        args.world
    )

    # Spawn robot 2
    success2 = spawn_robot_via_ros_gz(
        'robot2', args.robot2_ns,
        args.robot2_x, args.robot2_y, args.z, args.robot2_yaw,
        args.world
    )

    if success1 and success2:
        print('[spawn_multi_robots] Both robots spawned successfully!')
        print(f'[spawn_multi_robots] Robot 1: {args.robot1_ns} at ({args.robot1_x}, {args.robot1_y}, {args.z})')
        print(f'[spawn_multi_robots] Robot 2: {args.robot2_ns} at ({args.robot2_x}, {args.robot2_y}, {args.z})')
        return 0
    else:
        print('[spawn_multi_robots] ERROR: Failed to spawn one or both robots')
        return 1

if __name__ == '__main__':
    sys.exit(main())