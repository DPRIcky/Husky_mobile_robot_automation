#!/usr/bin/env python3
"""
Spawn a static ArUco marker panel into a running Gazebo Sim world.

Marker spec: DICT_4X4_50, ID 0, 0.4 m × 0.4 m face, 0.01 m thick.
Default pose: x=3.0 y=0.0 z=0.8 (panel centre), facing the robot at origin.

Usage (Gazebo + ROS 2 must already be running):
  python3 spawn_aruco_marker.py
  python3 spawn_aruco_marker.py --x 3.0 --y 0.0 --z 0.8 --world warehouse

The script uses 'ros2 run ros_gz_sim create -string' so the absolute texture
path is embedded directly — no GZ_SIM_RESOURCE_PATH setup required.
"""

import argparse
import os
import subprocess
import sys

DEFAULT_X = 3.0
DEFAULT_Y = 0.0
DEFAULT_Z = 0.8
DEFAULT_WORLD = 'warehouse'
MODEL_NAME = 'aruco_marker_0'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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


def _build_sdf(tex_abs_path: str) -> str:
    """Return an SDF string with the texture path embedded as a file:// URI."""
    return f"""\
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{MODEL_NAME}">
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


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description='Spawn ArUco marker panel into a running Gazebo Sim world.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument('--x',     type=float, default=DEFAULT_X,     help='X position (m)')
    ap.add_argument('--y',     type=float, default=DEFAULT_Y,     help='Y position (m)')
    ap.add_argument('--z',     type=float, default=DEFAULT_Z,     help='Z position (m, panel centre)')
    ap.add_argument('--world', default=DEFAULT_WORLD,             help='Gazebo world name')
    args = ap.parse_args()

    models_dir = _models_dir()
    tex_path   = os.path.join(models_dir, MODEL_NAME,
                              'materials', 'textures', 'aruco_marker_0.png')

    _ensure_texture(tex_path)

    sdf_str = _build_sdf(os.path.abspath(tex_path))

    print(f'[spawn_aruco] Spawning {MODEL_NAME} at '
          f'x={args.x} y={args.y} z={args.z} in world "{args.world}" …')

    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world',  args.world,
        '-string', sdf_str,
        '-name',   MODEL_NAME,
        '-x', str(args.x),
        '-y', str(args.y),
        '-z', str(args.z),
        '-R', '0', '-P', '0', '-Y', '0',
    ]

    result = subprocess.run(cmd)
    if result.returncode == 0:
        print(f'[spawn_aruco] {MODEL_NAME} spawned successfully.')
    else:
        sys.exit(f'[spawn_aruco] Spawn failed (exit code {result.returncode}).\n'
                 'Is Gazebo running?  Check: gz sim status')


if __name__ == '__main__':
    main()
