# Creating a New Task

`emdb_simulator` tasks are robosuite/robocasa environment classes (see
{doc}`../architecture`). Writing one by hand means subclassing `Kitchen`,
registering fixture references, describing object placement, and writing
success-condition code. `scripts/create_task.py` generates a working task
file for you from a template, so you don't need to know any of that.

## 1. Run the wizard

```bash
cd ros_packages/src/emdb_simulator
python3 scripts/create_task.py
```

No `env.sh`/ROS2 environment needed: the wizard only generates a text
file, it doesn't import robosuite.

```{tip}
Asking an LLM/coding agent to create a task for you? Point it at
{doc}`../llms`: the script also accepts flags (`--name`, `--template`,
`--json`, ...) for non-interactive, scriptable use instead of the prompts
below.
```

## 2. Answer the prompts

- **Task class name**: PascalCase, e.g. `PlaceMugInSink`.
- **Template**:
  - `lift`: pick up an object and raise it above a height threshold.
    Mirrors the project's own `KitchenLift`
    ({py:class}`emdb_simulator.core.kitchen_lift_task.KitchenLift`).
  - `place`: move an object from one fixture to another. Mirrors
    robocasa's own `PickPlaceCounterToCabinet`.
  - `custom`, advanced: a bare scaffold with `NotImplementedError` stubs
    instead of generated logic, for multi-object/multi-fixture ("composite")
    tasks the other two templates can't express. You (or an LLM/coding
    agent, see {doc}`../llms`) fill in the three stub methods by hand
    afterward.
- **Object group(s)**: comma-separated, e.g. `straw,peeler,sugar_cube`.
  Keep them small: the project's gripper (`TwoFG7Gripper`) only opens
  ~31mm, so most full-size kitchen items won't fit its jaws.
- **Fixture(s)**: picked from a numbered menu (island, counter, cabinet,
  sink, ...), not typed by hand, so you can't typo a fixture name. `lift`
  asks for one (where the object spawns); `place` asks for two (source and
  destination).
- `lift` also asks for the lift height in meters (default `0.10`).
- Both ask for an optional one-line description used as the episode's
  language instruction; leave it blank for a sensible auto-generated one.

The wizard shows a summary and asks for confirmation before writing
anything.

## 3. What gets written

- `emdb_simulator/core/<snake_case_name>_task.py`: the generated task
  class.
- One new import line appended to
  {py:mod}`emdb_simulator.core.registered_tasks`, which is what actually
  registers it with robosuite (imported once by `scene_loader`; see
  {doc}`../architecture`). You don't need to touch `scene_loader.py`.

## 4. Try it

```bash
source env.sh
ros2 run emdb_simulator scene_loader --ros-args -p task:=<YourTaskName>
```

`colcon build --symlink-install` (see {doc}`../getting_started`) means a
pure-Python change like this doesn't need a rebuild.

## Limitations

`lift` and `place` only offer success-condition patterns they can generate
correctly. If your task needs custom logic (e.g. a multi-step sequence, or
a success condition that isn't "lifted" or "moved to a fixture"), use
`--template custom` (see above) to get a correctly-registered scaffold, then
fill in `_setup_kitchen_references`/`_get_obj_cfgs`/`_check_success` by
hand: see `KitchenLift` ({doc}`../api/emdb_simulator`) for a worked
example, robocasa's own `kitchen_pick_place.py` for several `place`-family
variants (opening doors, partial containment checks, contact-based
checks), and robocasa's `environments/kitchen/composite/` directory for
real multi-object/multi-fixture tasks. This is the kind of thing an
LLM/coding agent is well-suited to do for you: see {doc}`../llms`.
