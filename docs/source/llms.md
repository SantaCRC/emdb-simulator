# LLM Reference: Creating Tasks & Robots

Machine-oriented reference for automating `emdb_simulator` task/robot
creation. For prose explanations see {doc}`howto/creating_tasks` and
{doc}`howto/managing_robots`. This page is deliberately terse and
table-heavy -- optimized for an LLM agent to read once and act on
correctly, not for narrative reading.

## Rules

1. **Never hand-write a task or robot `.py` file from scratch.** Always
   invoke the scripts below with `--json` to create and register the file,
   then read the result. The one sanctioned exception is filling in the
   body of a `--template custom` scaffold (see **Composite / custom
   tasks** below) -- even then, the *file creation and registration* still
   goes through the script; only the method bodies are hand-written.
2. **Never edit `registered_tasks.py` / `registered_robots.py` directly.**
   The scripts append to them. If you must inspect them, only read, don't
   write.
3. **Always run from the package root**: `ros_packages/src/emdb_simulator/`.
   Both scripts resolve all paths relative to their own file location, not
   the current working directory, so they work from anywhere, but examples
   below assume `cd ros_packages/src/emdb_simulator` first.
4. **Always pass `--json`** when calling non-interactively. Parse the
   single JSON line printed to stdout. `"ok": true` = success;
   `"ok": false` = failure, with an `"errors"` (create_task.py) or
   `"error"`/`"errors"` (robot_tool.py) field explaining exactly what was
   wrong -- do not guess a fix, use that message.
5. **`--dry-run` first if unsure.** Both `create_task.py` and
   `robot_tool.py create` accept `--dry-run`: writes nothing, returns the
   generated source in the JSON `"source"` field (or prints it plain
   without `--json`). Use it to sanity-check before committing to a name.
6. **Exit code matches `"ok"`**: `0` on success, `1` on failure. Check the
   exit code in addition to the JSON, in case stdout capture fails.
7. **Names must not already exist.** Both scripts refuse (with a clear
   error) to overwrite an existing `<name>_task.py` / `<name>_robot.py`.
   If creation fails for that reason, either pick a new name or delete the
   existing file and its registry line first.

## Creating a task -- `scripts/create_task.py`

```bash
python3 scripts/create_task.py --name <PascalCaseName> --template {lift,place,custom} [options] --json
```

| Flag | Required | Applies to | Valid values | Default |
|---|---|---|---|---|
| `--name` | yes | all | `^[A-Z][A-Za-z0-9]*$`, must not already exist as `emdb_simulator/core/<snake_case>_task.py` | -- |
| `--template` | yes | all | `lift`, `place`, `custom` | -- |
| `--obj-groups` | no | `lift`/`place` only | comma-separated robocasa object group names | `straw,peeler,sugar_cube,shrimp` (small objects -- the project's `TwoFG7Gripper` only opens ~31mm) |
| `--fixture` | no | `lift` only | one of the fixture tokens below | `ISLAND` |
| `--height` | no | `lift` only | float (meters) | `0.10` |
| `--source` | no | `place` only | one of the fixture tokens below | `COUNTER` |
| `--dest` | no | `place` only | one of the fixture tokens below | `CABINET` |
| `--lang` | no | all | any string (episode language instruction) | auto-generated from template/fixtures; for `custom`, a `"TODO: ..."` placeholder you should overwrite when filling in the scaffold |
| `--dry-run` | no | all | flag, no value | off |
| `--json` | no | all | flag, no value | off (prose output) |

**Fixture tokens** (`--fixture` / `--source` / `--dest`, `lift`/`place`
only): `ISLAND`, `COUNTER`, `DINING_COUNTER`, `CABINET`, `DRAWER`,
`SHELF`, `SINK`, `MICROWAVE`, `STOVE`, `FRIDGE`, `DISHWASHER`. Any other
value is rejected. If `--dest` is `CABINET`, `DRAWER`, `MICROWAVE`, or
`DISHWASHER`, the generated task automatically opens that fixture's door
in `_setup_scene`.

**Template semantics**:
- `lift`: spawn an object on `--fixture`, success = object rises more than
  `--height` meters above its start height.
- `place`: spawn an object on `--source`, success = object ends up inside
  `--dest` and the gripper has moved away from it.
- `custom`: freeform scaffold with correct imports/registration but
  `NotImplementedError` stubs for `_setup_kitchen_references`,
  `_get_obj_cfgs`, `_check_success` -- see
  **Composite / custom tasks** below. `--fixture`/`--source`/`--dest`/`--obj-groups` don't apply and are ignored.

**Success JSON** (`"ok": true`):
```json
{"ok": true, "class_name": "...", "module": "...", "file": "emdb_simulator/core/<module>.py",
 "registered_in": "emdb_simulator/core/registered_tasks.py",
 "run_command": "ros2 run emdb_simulator scene_loader --ros-args -p task:=<ClassName>",
 "template": "lift|place|custom", ...}
```
`--dry-run` success JSON: `{"ok": true, "dry_run": true, "file": "...", "source": "<full file text>", ...}` -- nothing written to disk.

**Failure JSON**: `{"ok": false, "errors": ["<reason>", ...]}`.

**Example**:
```bash
python3 scripts/create_task.py --name PlaceMugInSink --template place \
    --obj-groups mug --source COUNTER --dest SINK --json
```

## Composite / custom tasks

The `lift`/`place` templates only cover single-object, single-condition
tasks -- that's intentional, they're the safe default for the interactive
wizard's non-coder audience. An LLM agent can go further: robocasa ships
~60 real "composite" tasks (multi-object, multi-fixture, multi-stage) under
`misc/robocasa/robocasa/environments/kitchen/composite/<activity>/*.py` in
this repo (e.g.
`composite/adding_ice_to_beverages/make_ice_lemonade.py`,
`composite/adding_ice_to_beverages/place_ice_in_cup.py`) -- structurally
identical `Kitchen` subclasses to the `lift`/`place` templates, just with
more fixtures, more objects (including distractors), and richer
`_check_success` logic. **Read 1-2 of them before writing a composite
task** -- they are the ground truth for the API, not this page.

**Workflow**:
1. `python3 scripts/create_task.py --name <Name> --template custom --json`
   -- creates and registers a scaffold with `NotImplementedError` stubs
   (this still uses the script, so registration is handled correctly).
2. Read the generated file and 1-2 real composite examples above.
3. Edit the three stub methods directly (Read + Edit, not the wizard):
   - `_setup_kitchen_references`: call
     `self.<name> = self.register_fixture_ref("<name>", dict(id=FixtureType.<TOKEN>[, ref=self.<other_fixture>]))`
     once per fixture the task needs (any `FixtureType` member, not just
     the curated 11 the `lift`/`place` templates offer -- see
     `misc/robocasa/robocasa/models/fixtures/fixture.py` for the full
     enum), then set `self.init_robot_base_ref` to one of them.
   - `_get_obj_cfgs`: return a `list[dict]`, one per object. Common keys:
     `name`, `obj_groups` (or `exclude_obj_groups`), `graspable`,
     `object_scale`, `init_robot_here`, `fridgable`, and `placement=dict(`
     `fixture=<fixture ref>` or `object="<other obj's name>"` (stack on
     another spawned object, e.g. an ice cube "on" a bowl object), `
     sample_region_kwargs=dict(ref=<fixture ref>), size=(w, h), pos=(x, y),
     offset=(dx, dy))`.
   - `_check_success`: combine helpers from `robocasa.utils.object_utils`
     (imported as `OU` via the `from robocasa.environments.kitchen.kitchen
     import *` wildcard already in the scaffold) and `self.check_contact`.
     Ranked by how often robocasa's own composite tasks use them. Signatures
     are exactly as defined in `misc/robocasa/robocasa/utils/object_utils.py`
     (first param `env` is implicitly `self` when called from inside a task
     method, e.g. `OU.gripper_obj_far(self, "obj")`) -- verified against
     that file, not guessed:

     | Helper | Signature (minus `env`) | Use |
     |---|---|---|
     | `OU.gripper_obj_far` | `(obj_name="obj", th=0.25)` | gripper has moved away from an object (pair with an "in place" check so the episode doesn't end mid-grasp) |
     | `OU.check_obj_in_receptacle` | `(obj_name, receptacle_name, th=None)` | object is inside/on another spawned object (e.g. ice cube in a bowl); robocasa's own tasks usually pass `th=0.5` explicitly |
     | `OU.check_obj_fixture_contact` | `(obj_name, fixture_name)` | object is touching a fixture (e.g. pan on a stove burner) |
     | `OU.obj_inside_of` | `(obj_name, fixture_id, partial_check=False, th=0.05)` | object is inside a fixture's interior (cabinet, drawer, microwave, fridge) |
     | `OU.check_obj_any_counter_contact` | `(obj_name)` | object is touching any counter |
     | `OU.add_obj_liquid_site` | `(obj_name, liquid_rgba)` | (call in `_setup_scene`, not `_check_success`) adds a visual "liquid fill" site to a container object |
     | `OU.obj_fixture_bbox_min_dist` | `(obj_name, fixture)` | distance between an object and a fixture's bounding box |
     | `OU.check_obj_grasped` | `(obj_name, threshold=0.035)` | object is currently held by the gripper |
     | `OU.check_obj_upright` | `(obj_name, th=15)` | object hasn't tipped over |
     | `self.check_contact` | `(objA, objB)` (both are `self.objects["name"]`, not name strings) | direct contact between two spawned objects |

     Combine with plain Python `and`/`or`/`all(...)` -- see
     `place_ice_in_cup.py`'s `_check_success` for a real multi-object
     OR/AND combination.
4. Validate: `python3 -c "import ast; ast.parse(open('emdb_simulator/core/<module>.py').read())"`.
   This only catches syntax errors, not wrong robocasa API usage or a
   `NotImplementedError` you forgot to remove -- report to the user that
   real verification needs a sourced environment run (see **Verifying a
   created task/robot actually works** at the end of this page), don't
   claim the task works from a syntax check alone.

## Managing robots -- `scripts/robot_tool.py`

Five subcommands. `list`/`inspect`/`edit`/`create` need only Python 3 (no
`env.sh`). `preview` needs a sourced ROS2/robosuite environment and a
display -- it opens a blocking interactive viewer, so don't invoke it
expecting a text result; tell the user to run it themselves instead.

### `list` -- enumerate registered robots

```bash
python3 scripts/robot_tool.py list --json
```
```json
{"ok": true, "robots": [{"class_name": "UR5eOmron", "extends": ["UR5e"], "file": "emdb_simulator/core/robot_loader.py"}, ...]}
```

### `inspect` -- read a robot's properties exactly as written

```bash
python3 scripts/robot_tool.py inspect --robot <ClassName> --json
```
```json
{"ok": true, "class_name": "...", "extends": [...], "target_type": "WheeledRobot|FixedBaseRobot",
 "file": "...", "properties": {
   "default_base": "<source text or null>", "default_arms": "<source text or null>",
   "default_gripper": "<source text or null>", "init_qpos": "<source text or null>",
   "init_torso_qpos": "<source text or null>", "base_xpos_offset": "<source text or null>"}}
```
`properties` values are the **exact source text** of each property's
return expression (e.g. `"{\"right\": \"TwoFG7Gripper\"}"`), not parsed
Python objects -- read them as text, don't `eval` them.

### `edit` -- change `default_gripper` or `init_qpos` on an existing robot

```bash
python3 scripts/robot_tool.py edit --robot <ClassName> --param {default_gripper,init_qpos} --value <VALUE> --json
```

| `--param` | `--value` format | Valid values |
|---|---|---|
| `default_gripper` | a single gripper name | `PandaGripper`, `Robotiq85Gripper`, `Robotiq140Gripper`, `RobotiqThreeFingerGripper`, `TwoFG7Gripper` |
| `init_qpos` | comma-separated floats | any numbers, e.g. `"-0.47,-1.7,2.4,-2.3,-1.6,-2.0"` |

Only these two properties are editable, and only when they're already a
simple single-arm dict (`default_gripper`) or a flat `np.array([...])`
(`init_qpos`). Anything more complex (e.g. `base_xpos_offset`) is
**refused** with an error naming the exact file:line to edit by hand --
this is intentional, not a bug; don't try to work around it by editing the
file directly.

Success: `{"ok": true, "property": "...", "new": "<new source text>", "file": "..."}`.
Failure: `{"ok": false, "error": "<reason>"}`.

### `create` -- compose a new robot from an arm + base + gripper

```bash
python3 scripts/robot_tool.py create --name <PascalCaseName> --arm <ARM> --base {wheeled,fixed} --gripper <GRIPPER> [options] --json
```

| Flag | Required | Valid values | Default |
|---|---|---|---|
| `--name` | yes | `^[A-Z][A-Za-z0-9]*$`, must not already exist as `<snake_case>_robot.py` | -- |
| `--arm` | yes | `UR5e`, `Panda`, `Sawyer`, `IIWA`, `XArm7` | -- |
| `--base` | yes | `wheeled` (OmronMobileBase) or `fixed` (arm's own base) | -- |
| `--gripper` | yes | `PandaGripper`, `Robotiq85Gripper`, `Robotiq140Gripper`, `RobotiqThreeFingerGripper`, `TwoFG7Gripper`, `none` | -- |
| `--init-qpos` | no | comma-separated floats (joint angles, radians) | arm's own default (property omitted) |
| `--init-torso-qpos` | no | comma-separated floats; **only valid with `--base wheeled`** | arm's own default (property omitted) |
| `--dry-run` | no | flag | off |
| `--json` | no | flag | off |

`base_xpos_offset` is never auto-generated (arena-specific, needs manual
tuning) -- the generated file has a comment pointing at `UR5eOmron` in
`robot_loader.py` as a worked example. If a created robot spawns in a bad
position in a given task, that's the property to add by hand.

Success JSON:
```json
{"ok": true, "class_name": "...", "module": "...", "file": "emdb_simulator/core/<module>.py",
 "registered_in": "emdb_simulator/core/registered_robots.py",
 "run_command": "ros2 run emdb_simulator scene_loader --ros-args -p robot:=<ClassName>",
 "arm": "...", "base": "wheeled|fixed", "gripper": "...", "target_type": "WheeledRobot|FixedBaseRobot"}
```
Failure JSON: `{"ok": false, "errors": ["<reason>", ...]}`.

### `preview` -- open the 3D viewer (interactive, not agent-parseable)

```bash
python3 scripts/robot_tool.py preview --robot <ClassName>
```
Requires `source env.sh` and a display; blocks until Ctrl+C. No `--json`
mode -- this is a visual check for a human, not something an agent should
call expecting structured output. If asked to "show" a robot, tell the
user to run this command themselves rather than trying to invoke it and
capture output.

## Verifying a created task/robot actually works

Neither script can validate that generated robosuite/robocasa code
actually runs (that needs a real simulation with mujoco/a display). After
creating something, the deterministic checks available without a display
are:

```bash
python3 -c "import ast; ast.parse(open('emdb_simulator/core/<module>.py').read())"
```
(confirms syntax only). To confirm it actually loads and runs in
simulation, the `run_command` from the success JSON must be executed in a
sourced ROS2 environment with a display -- report that as a next step to
the user rather than claiming success prematurely.
