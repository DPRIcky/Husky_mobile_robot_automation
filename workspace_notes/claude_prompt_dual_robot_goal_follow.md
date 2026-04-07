## Primary Claude Code Prompt

You are working inside this repository:

- `/home/prajjwal/clearpath`

This is a ROS 2 Jazzy workspace for the Clearpath A300 with:

- a custom planning stack in `trajectory_planner_pkg`
- a custom path-following + obstacle avoidance stack in `simple_motion_pkg`
- a current two-robot ArUco demo in `autonomy_bringup`

Read these files first before planning or editing:

- `/home/prajjwal/clearpath/CLAUDE.md`
- `/home/prajjwal/clearpath/autonomy_bringup/launch/autonomy.launch.py`
- `/home/prajjwal/clearpath/autonomy_bringup/launch/two_robot_aruco.launch.py`
- `/home/prajjwal/clearpath/autonomy_bringup/config/autonomy.rviz`
- `/home/prajjwal/clearpath/trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py`
- `/home/prajjwal/clearpath/trajectory_planner_pkg/config/planner_params.yaml`
- `/home/prajjwal/clearpath/simple_motion_pkg/simple_motion_pkg/path_follower.py`
- `/home/prajjwal/clearpath/simple_motion_pkg/simple_motion_pkg/twist_mux.py`
- `/home/prajjwal/clearpath/simple_motion_pkg/config/motion_params.yaml`
- `/home/prajjwal/clearpath/autonomy_bringup/autonomy_bringup/aruco_detector.py`
- `/home/prajjwal/clearpath/autonomy_bringup/autonomy_bringup/aruco_follower.py`
- `/home/prajjwal/clearpath/docs/MULTI_ROBOT_SETUP.md`
- `/home/prajjwal/clearpath/README.md`

Do not use these as the implementation base:

- `autonomy_bringup/launch/multi_robot_simulation.launch.py`
- `autonomy_bringup/launch/working_multi_robot.launch.py`

They are documented as broken.

## Goal

Implement the final integrated behavior:

1. Restore the previous RViz goal-pose workflow so that clicking `2D Goal Pose` causes the existing custom planner to generate a path.
2. Make the existing custom path follower execute that path using the current controller and obstacle avoidance / replan logic.
3. Upgrade that workflow from one robot to two robots in the same simulation.
4. The updated system must include a follower robot, and the user requirement says the second robot should be the follower.

## Important current repo facts

- The old single-robot autonomy flow already exists in:
  - `autonomy_bringup/launch/autonomy.launch.py`
  - `autonomy_bringup/config/autonomy.rviz`
- The custom planner and controller stack already exists and should be reused:
  - planner: `trajectory_planner_pkg`
  - controller + obstacle handling: `simple_motion_pkg`
- The current two-robot ArUco demo exists in:
  - `autonomy_bringup/launch/two_robot_aruco.launch.py`
- Right now the ArUco setup is physically arranged so Robot 1 detects Robot 2:
  - Robot 2 has the rear-mounted marker in `robot2/robot.urdf.xacro`
  - the current detector/follower flow is built around Robot 1 observing Robot 2
- Many topics are still hard-coded to `/a300_00000/...` or `/goal_pose`, so this probably needs parameterization.

## Critical ambiguity to resolve first

The requirement says: "add the 2 robots and the 2nd robot will be the follower."

But the current ArUco implementation is wired the other way around: Robot 1 follows Robot 2 because Robot 2 carries the marker.

Before making edits, do a short architecture note that clearly answers:

1. Which robot is the leader receiving the RViz goal pose?
2. Which robot is the follower?
3. Where must the ArUco marker live for that to be physically correct?
4. Which robot runs the detector and which robot publishes the follow cmd_vel?

If Robot 2 truly must be the follower, then do not ignore the physical inconsistency. Fix it properly, which likely means moving or duplicating the marker arrangement so Robot 2 can visually follow Robot 1.

## Hard requirements

- Reuse the existing custom planning + control stack. Do not silently replace the main behavior with Nav2 `NavigateToPose`.
- Preserve the obstacle stop / reverse / replan behavior already implemented in `simple_motion_pkg/path_follower.py`.
- Keep namespace isolation correct for both robots.
- RViz goal tool must drive the leader robot's planner path generation.
- Avoid breaking the currently working ArUco detection pipeline unless a role reversal makes changes necessary.
- Prefer adding a new clean canonical launch for the integrated demo instead of destabilizing the known-good ArUco-only launch, unless you can safely extend the existing launch.
- Maintain readable launch arguments and topic remaps.

## What I want you to do

1. Inspect the current code and write a short implementation plan.
2. Parameterize the custom autonomy stack where needed so it can target a selected robot namespace cleanly.
3. Implement a two-robot launch flow that combines:
   - Gazebo two-robot spawning
   - RViz goal-pose autonomy for the leader
   - ArUco-based follower behavior for the follower robot
4. Update RViz so it is useful again for this integrated workflow:
   - both robots visible
   - leader path visible
   - map visible
   - goal tool publishes to the correct topic
   - any useful follower/debug overlays preserved if practical
5. Update documentation for the new canonical run command and explain which robot is leader vs follower.
6. Build and run validation commands and report exactly what passed and what still needs manual simulation verification.

## Expected implementation direction

Unless code inspection proves a better approach, I expect something close to this:

- Leader robot:
  - receives RViz goal pose
  - planner publishes planned path
  - path follower executes planned path
  - obstacle avoidance and replanning remain active
- Follower robot:
  - uses ArUco detection to follow the leader robot
  - should not interfere with the leader's planner/controller topics
- Launch:
  - should bring up both robots with clean namespaces
  - should start the right detector/follower nodes in the right namespace
  - should start RViz with the correct config

## Likely code areas to modify

- `autonomy_bringup/launch/autonomy.launch.py`
- `autonomy_bringup/launch/two_robot_aruco.launch.py`
- possibly a new launch such as `autonomy_bringup/launch/two_robot_goal_follow.launch.py`
- `autonomy_bringup/config/autonomy.rviz`
- `trajectory_planner_pkg/config/planner_params.yaml`
- `simple_motion_pkg/config/motion_params.yaml`
- `trajectory_planner_pkg/trajectory_planner_pkg/planner_node.py`
- `simple_motion_pkg/simple_motion_pkg/path_follower.py`
- `simple_motion_pkg/simple_motion_pkg/twist_mux.py`
- `autonomy_bringup/autonomy_bringup/aruco_follower.py`
- whichever robot URDF must carry the ArUco marker for the chosen follower topology
- README/docs files

## Design constraints

- Minimize risky rewrites.
- Prefer parameterization and launch composition over duplicating node logic.
- Keep the one-robot workflow working if possible.
- Keep the current working two-robot ArUco demo available if possible.
- Do not hard-code everything to `a300_00000`; make the leader/follower roles explicit.
- Be careful with TF remaps and namespaced topics.
- Be careful with `cmd_vel` routing so the wrong robot does not move.
- Be careful with Clearpath generator limitations described in `docs/MULTI_ROBOT_SETUP.md`.

## Acceptance criteria

The task is done only if the repo supports this workflow:

1. Launch the integrated two-robot simulation.
2. Open RViz.
3. Use `2D Goal Pose`.
4. The leader robot gets a planned path and drives it using the existing custom controller stack.
5. Obstacle handling from the custom path follower still works.
6. The follower robot follows the leader using the ArUco-based mechanism in a role-consistent way.
7. Namespaces, TF, and cmd_vel topics are clean and non-conflicting.

## Validation expectations

After edits, run at least:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/prajjwal/clearpath
colcon build --packages-select autonomy_bringup trajectory_planner_pkg simple_motion_pkg
source install/setup.bash
```

Then give the exact launch command for the integrated demo and list useful verification commands such as:

- `ros2 node list`
- `ros2 topic list | grep a300_0000`
- `ros2 topic echo /planned_path`
- `ros2 topic echo <leader or follower debug topic>`
- any controller-manager checks if relevant

If full simulation validation is not possible in your environment, say exactly what you verified and what remains to be checked manually in Gazebo/RViz.

## Output format

Work in this order:

1. Short architecture note resolving leader/follower roles.
2. Short implementation plan.
3. Code changes.
4. Build/test results.
5. Final summary with run instructions.

## When to use Opus

Use Claude Opus if either of these happens:

- the leader/follower role reversal requires a nontrivial architecture decision around marker placement, detector namespace, or launch design
- you get stuck reconciling the old RViz goal-pose flow with the new two-robot ArUco setup and need a stronger whole-system design pass

If Sonnet can handle it cleanly after the initial inspection, that is fine too.


## Optional Opus Escalation Prompt

If you want to hand only the hardest part to Opus, use this:

Please do a deep architecture pass on this ROS 2 Jazzy Clearpath A300 repo at `/home/prajjwal/clearpath`.

Context:

- Single-robot RViz goal-pose autonomy already exists via `autonomy.launch.py`, `autonomy.rviz`, `trajectory_planner_pkg`, and `simple_motion_pkg`.
- Two-robot ArUco following already exists via `two_robot_aruco.launch.py`, but it is currently arranged so Robot 1 follows Robot 2 because Robot 2 carries the marker.
- New requirement: integrate both systems and make the second robot the follower while preserving the old RViz goal-pose planner/controller/obstacle-avoidance workflow.

I need you to:

1. Resolve the correct physical/software role assignment for leader and follower.
2. Propose the cleanest launch and parameterization architecture.
3. Identify the minimum safe code changes needed.
4. Call out any hidden risk around namespaces, TF, `cmd_vel`, RViz topic wiring, or ArUco marker placement.
5. Recommend whether to extend `two_robot_aruco.launch.py` or add a new canonical integrated launch.

Be concrete and repo-specific. Reference exact files to change and explain why.
