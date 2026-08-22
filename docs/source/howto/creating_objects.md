# Creating a New Object

RoboCasa objects (the things a task spawns and a robot picks up) are
registered into two module-level dicts, `OBJ_CATEGORIES` and `OBJ_GROUPS`,
that robocasa's own object sampler reads from. Most of robocasa's built-in
categories are full-size kitchen items -- an apple's narrowest side is
~54-75mm -- that don't fit `TwoFG7Gripper`'s ~31mm jaw opening (see
{doc}`../architecture`), so this project registers its own small test
objects instead of relying on stock ones.

Unlike {doc}`creating_tasks` and {doc}`managing_robots`, there's no wizard
script for this yet: you hand-author a module following the project's own
{py:mod}`emdb_simulator.core.cube_object`, which registers a 5x5x5cm "cube"
test object. This guide walks through that file as the worked example.

## 1. Get a MuJoCo object model

You need a MuJoCo XML model of the object (geometry, mass, joints) plus any
mesh files it references. `cube_object`'s asset lives at
`emdb_simulator/assets/objects/cube/Cube001/model.xml` -- a primitive box,
no meshes needed. For anything more complex than a primitive shape, export
or author a MuJoCo-compatible model the same way robocasa's own bundled
objects are structured (one folder per object, `model.xml` plus meshes next
to it).

## 2. Write the registration module

`emdb_simulator/core/cube_object.py` in full:

```python
import os

from ament_index_python.packages import get_package_share_directory

from robocasa.models.objects.kitchen_object_utils import ObjCat
from robocasa.models.objects.kitchen_objects import OBJ_CATEGORIES, OBJ_GROUPS

_ASSET_DIR = os.path.join(
    get_package_share_directory("emdb_simulator"), "assets", "objects", "cube"
)

OBJ_CATEGORIES["cube"] = {
    "objaverse": ObjCat(
        name="cube",
        reg_type="objaverse",
        types=("test_object",),
        graspable=True,
        washable=False,
        microwavable=False,
        cookable=False,
        fridgable=False,
        freezable=False,
        model_folders=[_ASSET_DIR],
    )
}
OBJ_GROUPS["cube"] = ["cube"]
```

Points that aren't obvious from reading it once:

- **Why register here instead of editing robocasa's `kitchen_objects.py`
  directly**: `misc/robocasa` is a git submodule and must stay unmodified.
  Importing this module mutates robocasa's `OBJ_CATEGORIES`/`OBJ_GROUPS`
  dicts at runtime instead, the same side-effect pattern
  {doc}`managing_robots` and {doc}`creating_tasks` use for robots and tasks.
- **`model_folders=[_ASSET_DIR]`**: `get_package_share_directory` resolves
  to the *installed* location (`install/emdb_simulator/share/...`), not the
  source tree, so this only works after a colcon build (step 3) and after
  `source env.sh` (step 4) -- `os.path.join`'s first argument is discarded
  once the second is absolute, which is why passing an absolute
  `_ASSET_DIR` here is safe.
- **`reg_type="objaverse"`**: this just has to match one of the folder
  layouts `ObjCat`/robocasa's sampler expects; it doesn't mean the object
  actually came from the Objaverse dataset.
- `graspable=True` and the various `*able=False` flags matter if you write
  a task that filters objects by property (robocasa's `place` tasks do);
  set them honestly for what the object supports.

## 3. Wire the asset into `setup.py`

Colcon only installs files listed in `data_files`. Add an entry for your
object's directory, next to `cube_object`'s:

```python
(
    os.path.join('share', package_name, 'assets', 'objects', 'cube', 'Cube001'),
    glob('assets/objects/cube/Cube001/*.xml')
),
```

## 4. Import the module so registration runs

`OBJ_CATEGORIES["cube"]` only gets populated when `cube_object.py` is
actually imported. Right now that happens once, directly:
`kitchen_lift_task.py` imports `cube_object` at module level, since it's
the only task that currently uses the "cube" group.

```{note}
Robots and tasks each get a dedicated registry module
({py:mod}`emdb_simulator.core.registered_robots`,
{py:mod}`emdb_simulator.core.registered_tasks`) that new entries are
appended to automatically. Objects don't have one yet -- import your new
module from whichever task(s) use it, the way `kitchen_lift_task.py` does.
```

## 5. Use it

Reference the object group's name (`"cube"` in the example) in a task's
`obj_groups`, either by editing `DEFAULT_OBJ_GROUPS` on an existing task or
picking it in {doc}`creating_tasks`'s wizard prompt for object groups.

## 6. Try it

```bash
colcon build --symlink-install --packages-select emdb_simulator
source env.sh
ros2 run emdb_simulator scene_loader --ros-args -p task:=<YourTaskName>
```

The asset copy in step 3 means you need a rebuild here, unlike
{doc}`creating_tasks`/{doc}`managing_robots`'s pure-Python generated files.

## Limitations

Check your object's narrowest dimension against the gripper's jaw opening
before spending time on a model that will just sit ungrasped in the open
jaws -- see `kitchen_lift_task.py`'s `DEFAULT_OBJ_GROUPS` comment for how
the project's own four default categories were verified. There's no
`object_tool.py` equivalent to {doc}`managing_robots`'s `robot_tool.py` yet;
inspecting an object's properties means reading `ObjCat`'s fields in source.
