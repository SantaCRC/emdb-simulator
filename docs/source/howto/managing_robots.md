# Viewing, Editing, and Creating Robots

`emdb_simulator` robots are composite robosuite robot classes -- an arm, a
base, and a gripper, registered with `register_robot_class` (see
{doc}`../architecture` and the project's own `UR5eOmron`,
{py:class}`emdb_simulator.core.robot_loader.UR5eOmron`). `scripts/robot_tool.py`
is a menu-driven tool to list, inspect, edit, preview, and create robots
without needing to read robosuite's registration internals.

## Run it

```bash
cd ros_packages/src/emdb_simulator
python3 scripts/robot_tool.py
```

List/Inspect/Edit/Create work with plain Python 3, no ROS2/robosuite
environment needed. **Preview a robot in 3D** is the one action that needs
`source env.sh` first (it builds a real robosuite environment to render).

```{tip}
Asking an LLM/coding agent to manage robots for you? Point it at
{doc}`../llms` -- the script also has non-interactive subcommands (`list`,
`inspect`, `edit`, `create`, each with `--json`) instead of the menu below.
```

## Menu actions

**List robots** -- shows every registered robot (class name, arm it
extends, file it's defined in).

**Inspect a robot** -- prints a robot's key properties (`default_base`,
`default_arms`, `default_gripper`, `init_qpos`, `init_torso_qpos`,
`base_xpos_offset`) exactly as they appear in source, so you can see what a
robot actually does without reading Python.

**Edit a robot's parameters** -- narrowly scoped to two safe, common edits:
- `default_gripper`, when it's a simple single-arm dict (offers a menu of
  robosuite's registered grippers plus the project's own `TwoFG7Gripper`).
- `init_qpos`, when it's a plain list of numbers (prompts for new
  comma-separated joint angles).

If a property is more complex than that (e.g. `base_xpos_offset`, which
varies per arena and can include lambdas), the tool tells you the exact
file and line to edit by hand instead of guessing.

**Preview a robot in 3D** -- opens the interactive MuJoCo viewer with just
the selected robot in robosuite's built-in `Lift` environment (no kitchen
assets needed), the same `robosuite.make(...)` settings `scene_loader` uses
day-to-day. Ctrl+C to stop.

**Create a new robot** -- prompts for a class name, then numbered menus
for:
- Arm base class (`UR5e`, `Panda`, `Sawyer`, `IIWA`, `XArm7`).
- Base: `OmronMobileBase` (wheeled) or none (fixed base, the arm's own
  default).
- Gripper: robosuite's registered grippers, the project's `TwoFG7Gripper`,
  or none (arm's own default).
- Optionally, a custom `init_qpos` (and `init_torso_qpos` if you picked the
  wheeled base).

Writes `emdb_simulator/core/<snake_case_name>_robot.py` and registers it in
{py:mod}`emdb_simulator.core.registered_robots` (imported once by
`scene_loader`, same pattern as {doc}`creating_tasks`).

```{note}
`base_xpos_offset` (how a robot is positioned relative to different arena
types) is arena-specific and needs real judgment, so the wizard doesn't try
to generate it. The generated file has a comment pointing at `UR5eOmron` as
a worked example -- if your new robot spawns in a bad position for a given
task, that's the property to add by hand.
```

## Try it

```bash
source env.sh
ros2 run emdb_simulator scene_loader --ros-args -p robot:=<YourRobotName>
```
