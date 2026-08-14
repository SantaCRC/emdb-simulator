"""Registers a hand-authored "cube" object category with RoboCasa.

Importing this module has the side effect of registering the "cube" test
object into robocasa's global OBJ_CATEGORIES/OBJ_GROUPS dicts, so it must be
imported before anything samples the "cube" object group (see
kitchen_lift_task.DEFAULT_OBJ_GROUPS).

Most RoboCasa "graspable" categories are full-size kitchen items too big for
the TwoFG7 gripper's ~31mm jaw opening (see kitchen_lift_task.py). This cube
is a 5x5x5cm test object sized to fit, kept out of the misc/robocasa
submodule (which must stay unmodified) by registering it here instead of via
kitchen_objects.py's OBJ_CATEGORIES dict. ObjCat's model_folders accepts an
absolute path -- os.path.join discards its first argument when the second is
absolute -- so this points RoboCasa's asset scan straight at this package's
own assets/ directory.
"""
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
