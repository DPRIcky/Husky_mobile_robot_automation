# Multi-Robot Simulation Setup

Two Clearpath A300 robots in the Gazebo warehouse world:

| Robot | Namespace | Gazebo model name | ArUco marker |
|-------|-----------|-------------------|--------------|
| Robot 1 | `a300_00000` | `a300_00000/robot` | — |
| Robot 2 | `a300_00001` | `a300_00001/robot` | ID 0, DICT\_4X4\_50, rear-mounted |

The ArUco marker is a **link in Robot 2's URDF** (`aruco_marker_link`) fixed to
`rear_bumper_mount`. It moves with the robot and is never a separate static
world model. The marker now renders from a textured mesh asset so Gazebo sees
an actual ArUco panel instead of a plain white box.

---

## Quick start (one command)

```bash
source /opt/ros/jazzy/setup.bash
source /home/prajjwal/clearpath/install/setup.bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py
```

Optional arguments:

```bash
ros2 launch autonomy_bringup two_robot_aruco.launch.py \
    robot1_x:=-2.5  robot1_y:=0.0 \
    robot2_x:= 2.5  robot2_y:=0.0 \
    world:=warehouse
```

---

## What happens at launch

| Time | Event |
|------|-------|
| 0 s | Gazebo starts; Robot 1 generators start in parallel |
| 8 s | Robot 2: platform-service, sensors-service, and `ros_gz_sim create` all fire together (same TimerAction) |
| ~10 s | Robot 1 generators finish → platform-service + sensors-service + `ros_gz_sim create` fire together |

Robot 1 and Robot 2 end up spawning at roughly the same time (~10 s).
The reason Robot 2 uses a single TimerAction is to avoid the QoS/timing issue where `ros_gz_sim create` starts 15 s after `robot_state_publisher` and misses the `robot_description` message.

---

## Directory layout

```
clearpath/
├── robot.yaml                     # Robot 1 config  (namespace: a300_00000)
├── robot.urdf.xacro               # Robot 1 URDF    (no marker)
├── platform/launch/               # Robot 1 generated launch files
├── sensors/launch/
│
├── robot2/                        # Robot 2 setup directory
│   ├── robot.yaml                 # Robot 2 config  (namespace: a300_00001)
│   ├── robot.urdf.xacro           # Robot 2 URDF    (ArUco marker embedded)
│   ├── platform/launch/           # Robot 2 generated launch files
│   └── sensors/launch/
│
└── autonomy_bringup/
    ├── launch/
    │   ├── two_robot_aruco.launch.py   ← canonical launch file
    │   └── aruco_detection.launch.py   ← standalone detector launch
    └── models/aruco_marker_0/
        ├── meshes/aruco_marker_panel.obj
        └── materials/textures/aruco_marker_0.png
```

---

## ArUco marker details

| Property | Value |
|----------|-------|
| Dictionary | DICT\_4X4\_50 |
| ID | 0 |
| Face size | 0.4 m × 0.4 m |
| Thickness | 0.01 m |
| Parent link | `rear_bumper_mount` |
| Offset from mount | xyz = (0.05, 0, 0.30) m — 5 cm behind bumper, 30 cm above |
| Joint type | fixed (moves rigidly with robot) |
| Visual asset | `package://autonomy_bringup/models/aruco_marker_0/meshes/aruco_marker_panel.obj` |

The marker link is declared in
`robot2/robot.urdf.xacro` under `<!-- Extras -->`:

```xml
<link name="aruco_marker_link"> ... </link>
<joint name="aruco_marker_joint" type="fixed">
  <parent link="rear_bumper_mount"/>
  <child  link="aruco_marker_link"/>
  <origin xyz="0.05 0.0 0.30" rpy="0 0 0"/>
</joint>
<visual name="aruco_marker_visual">
  <geometry>
    <mesh filename="package://autonomy_bringup/models/aruco_marker_0/meshes/aruco_marker_panel.obj"/>
  </geometry>
</visual>
```

---

## Why Robot 2 uses a pre-generated setup directory

The Clearpath generator (`generate_description`) reads `robot2/robot.yaml` and
**overwrites** `robot2/robot.urdf.xacro`.  Our ArUco marker additions live at
the bottom of that file. Running the generator again would erase them.

The robot2 files were pre-generated once with:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run clearpath_generator_common generate_description        -s robot2/
ros2 run clearpath_generator_common generate_semantic_description -s robot2/
ros2 run clearpath_generator_gz     generate_launch             -s robot2/
ros2 run clearpath_generator_gz     generate_param              -s robot2/
```

After that, `robot2/robot.urdf.xacro` was edited to add the marker. The
canonical launch file uses the generated `robot2/platform/launch` and
`robot2/sensors/launch` files directly, then spawns Robot 2 with a
`ros_gz_sim create` node in the `a300_00001` namespace. This preserves the
custom URDF while avoiding the shared-namespace problems of the stock
`robot_spawn.launch.py` flow.

If you ever need to regenerate (e.g. after changing `robot2/robot.yaml`):

```bash
cd /home/prajjwal/clearpath
source /opt/ros/jazzy/setup.bash
ros2 run clearpath_generator_common generate_description        -s robot2/
ros2 run clearpath_generator_common generate_semantic_description -s robot2/
ros2 run clearpath_generator_gz     generate_launch             -s robot2/
ros2 run clearpath_generator_gz     generate_param              -s robot2/
# Then re-add the ArUco marker to robot2/robot.urdf.xacro
```

---

## Topic namespaces

**Robot 1 (`a300_00000`):**
- `/a300_00000/cmd_vel`
- `/a300_00000/odom`
- `/a300_00000/sensors/lidar2d_0/scan`
- `/a300_00000/sensors/camera_0/color/image`
- TF root: `a300_00000/base_link`

**Robot 2 (`a300_00001`):**
- `/a300_00001/cmd_vel`
- `/a300_00001/odom`
- `/a300_00001/sensors/lidar2d_0/scan`
- `/a300_00001/sensors/camera_0/color/image`
- TF root: `a300_00001/base_link`
- Extra TF frame: `a300_00001/aruco_marker_link`

---

## ArUco detection

This workspace now includes a lightweight OpenCV-based detector node in
`autonomy_bringup` because there is no preinstalled ROS ArUco package in this
environment.

Default behavior:
- `two_robot_aruco.launch.py` starts the detector automatically after both
  robots come up
- Detector watches Robot 1's front camera:
  `/a300_00000/sensors/camera_0/color/image`
- Camera info topic:
  `/a300_00000/sensors/camera_0/color/camera_info`

Published outputs:
- `/a300_00000/aruco_detector/poses`
- `/a300_00000/aruco_detector/markers`
- `/a300_00000/aruco_detector/debug_image`
- TF frames like `aruco_marker_0` in the observing camera frame

Standalone detector launch:

```bash
ros2 launch autonomy_bringup aruco_detection.launch.py
```

---

## Root causes of the previous broken setup

| Bug | Location | Effect |
|-----|----------|--------|
| Both markers used `MODEL_NAME = 'aruco_marker_0'` | `spawn_multi_aruco_markers.py` | Gazebo rejected second spawn — name collision |
| Markers were `<static>true</static>` world objects | `spawn_multi_aruco_markers.py` | Markers sat at fixed world coords, never moved with any robot |
| `robot_spawn.launch.py` reads namespace from `robot.yaml`, ignores the `namespace` arg | `clearpath_gz/launch/robot_spawn.launch.py` | Both robots got `a300_00000` — namespace collision |
| `gz_sim` action never added to `LaunchDescription` | `working_multi_robot.launch.py` | Gazebo never started |

---

## Troubleshooting

**Robot 2 spawns with wrong namespace / no marker**
→ Verify `robot2/robot.yaml` has `namespace: a300_00001` and
  `robot2/robot.urdf.xacro` contains `aruco_marker_link`.

**ArUco texture not visible in Gazebo**
→ Confirm the PNG exists:
```bash
ls /home/prajjwal/clearpath/autonomy_bringup/models/aruco_marker_0/materials/textures/
```

**"Model already exists" error**
→ Restart Gazebo — a previous run left the model in the world.

**TF tree missing robot 2 frames**
→ Robot 2's `platform-service.launch.py` publishes `robot_state_publisher`
  under namespace `a300_00001`.  Check that the node is running:
```bash
ros2 node list | grep a300_00001
```
