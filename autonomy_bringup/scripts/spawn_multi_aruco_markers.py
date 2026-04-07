#!/usr/bin/env python3
"""
Spawn multiple static ArUco marker panels into a running Gazebo Sim world.

Marker spec: DICT_4X4_50, ID 0, 0.4 m × 0.4 m face, 0.01 m thick.
Default poses:
  Marker 1 (front): x=3.0 y=0.0 z=0.8 (panel centre), facing the robot at origin.
  Marker 2 (rear): x=-3.0 y=0.0 z=0.8 (panel centre), facing away from origin.

Usage (Gazebo + ROS 2 must already be running):
  python3 spawn_multi_aruco_markers.py
  python3 spawn_multi_aruco_markers.py --world warehouse

The script uses 'ros2 run ros_gz_sim create -string' so the absolute texture
path is embedded directly — no GZ_SIM_RESOURCE_PATH setup required.
"""

import argparse
import os
import subprocess
import sys

DEFAULT_WORLD = 'warehouse'

# Marker positions
MARKER_1_POS = {'x': 3.0, 'y': 0.0, 'z': 0.8}   # Front marker
MARKER_2_POS = {'x': -3.0, 'y': 0.0, 'z': 0.8}  # Rear marker (behind robot)

def _models_dir() -> str:
    """Return the autonomy_bringup models/ directory (install tree preferred)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('autonomy_bringup'), 'models')
    except Exception:
        # Running directly from the source tree (no colcon build required)
        return os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models'))


def _ensure_texture(tex_path: str) -> None:
    """Generate the ArUco PNG if it is missing (e.g. first run from source)."""
    if os.path.isfile(tex_path):
        return
    print(f'[spawn_aruco] Texture not found — generating: {tex_path}')
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit('[spawn_aruco] ERROR: opencv-python is required to regenerate '
                 'the texture.  Install with: pip install opencv-python')

    os.makedirs(os.path.dirname(tex_path), exist_ok=True)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    marker_px, border_px = 400, 56
    total = marker_px + 2 * border_px  # 512
    marker_img = cv2.aruco.drawMarker(aruco_dict, 0, marker_px)
    canvas = np.full((total, total), 255, dtype=np.uint8)
    canvas[border_px:border_px + marker_px, border_px:border_px + marker_px] = marker_img
    cv2.imwrite(tex_path, canvas)
    print(f'[spawn_aruco] Texture written ({total}×{total} px).')


def _build_sdf(tex_abs_path: str, model_name: str) -> str:
    """Return an SDF string with the texture path embedded as a file:// URI."""
    return f"""\
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>0.01 0.4 0.4</size></box>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>file://{tex_abs_path}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>0.01 0.4 0.4</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def spawn_marker(x: float, y: float, z: float, world: str, model_name: str) -> bool:
    """Spawn a single ArUco marker at the specified position with a unique model name."""
    models_dir = _models_dir()
    tex_path = os.path.join(models_dir, 'aruco_marker_0', 'materials', 'textures', 'aruco_marker_0.png')

    _ensure_texture(tex_path)
    sdf_str = _build_sdf(os.path.abspath(tex_path), model_name)

    print(f'[spawn_aruco] Spawning {model_name} at x={x} y={y} z={z} in world "{world}" …')

    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world,
        '-string', sdf_str,
        '-name', model_name,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z),
        '-R', '0', '-P', '0', '-Y', '0',
    ]

    result = subprocess.run(cmd)
    if result.returncode == 0:
        print(f'[spawn_aruco] {model_name} spawned successfully at ({x}, {y}, {z}).')
        return True
    else:
        print(f'[spawn_aruco] Spawn failed at ({x}, {y}, {z}) (exit code {result.returncode}).')
        print('[spawn_aruco] Is Gazebo running? Check: gz sim status')
        return False


def main() -> None:
    ap = argparse.ArgumentParser(
        description='Spawn multiple ArUco marker panels into a running Gazebo Sim world.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument('--world', default=DEFAULT_WORLD, help='Gazebo world name')
    ap.add_argument('--marker1-x', type=float, default=MARKER_1_POS['x'], help='X position of marker 1 (front)')
    ap.add_argument('--marker1-y', type=float, default=MARKER_1_POS['y'], help='Y position of marker 1 (front)')
    ap.add_argument('--marker1-z', type=float, default=MARKER_1_POS['z'], help='Z position of marker 1 (front)')
    ap.add_argument('--marker2-x', type=float, default=MARKER_2_POS['x'], help='X position of marker 2 (rear)')
    ap.add_argument('--marker2-y', type=float, default=MARKER_2_POS['y'], help='Y position of marker 2 (rear)')
    ap.add_argument('--marker2-z', type=float, default=MARKER_2_POS['z'], help='Z position of marker 2 (rear)')
    args = ap.parse_args()

    print('[spawn_multi_aruco] Spawning ArUco markers for multi-robot tracking...')

    # Spawn first marker — unique Gazebo model name 'aruco_marker_front'
    success1 = spawn_marker(args.marker1_x, args.marker1_y, args.marker1_z, args.world,
                            model_name='aruco_marker_front')

    # Spawn second marker — unique Gazebo model name 'aruco_marker_rear'
    success2 = spawn_marker(args.marker2_x, args.marker2_y, args.marker2_z, args.world,
                            model_name='aruco_marker_rear')

    if success1 and success2:
        print('[spawn_multi_aruco] Both ArUco markers spawned successfully!')
        print(f'[spawn_multi_aruco] Marker 1 (front): x={args.marker1_x}, y={args.marker1_y}, z={args.marker1_z}')
        print(f'[spawn_multi_aruco] Marker 2 (rear): x={args.marker2_x}, y={args.marker2_y}, z={args.marker2_z}')
        print('[spawn_multi_aruco] Markers should now be visible in Gazebo for robot tracking')
    elif success1 or success2:
        print('[spawn_multi_aruco] WARNING: Only one marker spawned successfully')
        sys.exit(1)
    else:
        print('[spawn_multi_aruco] ERROR: Failed to spawn any ArUco markers')
        print('[spawn_multi_aruco] Troubleshooting tips:')
        print('[spawn_multi_aruco] 1. Ensure Gazebo is running: gz sim')
        print('[spawn_multi_aruco] 2. Check that the world is loaded: gz topic -l')
        print('[spawn_multi_aruco] 3. Verify ROS 2 environment is sourced')
        sys.exit(1)


if __name__ == '__main__':
    main()