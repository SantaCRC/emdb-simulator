"""Registers a "lift" task that still uses RoboCasa kitchen layouts/styles.

Importing this module has the side effect of registering KitchenLift with
robosuite's REGISTERED_ENVS (via robosuite's EnvMeta), so it must be
imported before robosuite.make() is called with this env name.
"""
from robocasa.environments.kitchen.kitchen import *


class KitchenLift(Kitchen):
    """Pick up a counter-top object and raise it above a height threshold.

    Mirrors robosuite's plain Lift task (success = object raised above a
    fixed margin over its starting height) but spawns the object on the
    RoboCasa kitchen island instead of Lift's bare table arena, so
    layout_ids/style_ids still apply. Only layouts with an island fixture
    can be used.
    """

    LIFT_HEIGHT = 0.10  # meters above starting height counted as "lifted"
    EXCLUDE_LAYOUTS = Kitchen.ISLAND_EXCLUDED_LAYOUTS

    def __init__(self, obj_groups="all", exclude_obj_groups=None, *args, **kwargs):
        self.obj_groups = obj_groups
        self.exclude_obj_groups = exclude_obj_groups
        self._obj_start_z = None
        super().__init__(*args, **kwargs)

    def _setup_kitchen_references(self):
        super()._setup_kitchen_references()
        self.island = self.register_fixture_ref("island", dict(id=FixtureType.ISLAND))
        self.init_robot_base_ref = self.island

    def get_ep_meta(self):
        ep_meta = super().get_ep_meta()
        obj_lang = self.get_obj_lang()
        ep_meta["lang"] = f"Pick up and lift the {obj_lang}."
        return ep_meta

    def _get_obj_cfgs(self):
        cfgs = []
        cfgs.append(
            dict(
                name="obj",
                obj_groups=self.obj_groups,
                exclude_obj_groups=self.exclude_obj_groups,
                graspable=True,
                init_robot_here=True,
                placement=dict(
                    fixture=self.island,
                    size=(0.40, 0.40),
                    pos=(0, -1.0),
                ),
            )
        )
        return cfgs

    def _reset_internal(self):
        super()._reset_internal()
        # objects have settled onto the counter by this point; that resting
        # height is the "not yet lifted" baseline _check_success compares against
        self._obj_start_z = self.sim.data.body_xpos[self.obj_body_id["obj"]][2]

    def _check_success(self):
        if self._obj_start_z is None:
            return False
        obj_z = self.sim.data.body_xpos[self.obj_body_id["obj"]][2]
        return (obj_z - self._obj_start_z) > self.LIFT_HEIGHT
