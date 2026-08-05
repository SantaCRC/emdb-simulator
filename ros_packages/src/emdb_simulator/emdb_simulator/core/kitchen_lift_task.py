"""Registers a "lift" task that still uses RoboCasa kitchen layouts/styles.

Importing this module has the side effect of registering KitchenLift with
robosuite's REGISTERED_ENVS (via robosuite's EnvMeta), so it must be
imported before robosuite.make() is called with this env name.
"""
from robocasa.environments.kitchen.kitchen import *
from robosuite.utils.mjcf_utils import array_to_string, find_elements, new_element


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

    # TwoFG7Gripper (see gripper_loader.py) is a real OnRobot 2FG7 small-parts
    # gripper: ~31mm max jaw opening. Most RoboCasa "graspable" categories are
    # full-size kitchen items (an apple's narrowest side is ~54-75mm) that the
    # jaw physically cannot close around, so the object just sits in the open
    # gap instead of being gripped. These four categories were verified by
    # measuring every instance's reg_bbox (restricted to Kitchen's default
    # obj_registries=("objaverse", "lightwheel") -- categories that only exist
    # under "aigen", e.g. chili_pepper, are never actually reachable and will
    # raise during sampling) and then confirmed with real end-to-end grasp
    # trials: every sampled instance's narrowest dimension stays under the
    # jaw's mechanical limit with margin, and all lift successfully.
    DEFAULT_OBJ_GROUPS = ["straw", "peeler", "sugar_cube", "shrimp"]

    def __init__(
        self,
        obj_groups=DEFAULT_OBJ_GROUPS,
        exclude_obj_groups=None,
        custom_cameras=None,
        *args,
        **kwargs,
    ):
        self.obj_groups = obj_groups
        self.exclude_obj_groups = exclude_obj_groups
        self._obj_start_z = None
        # List of resolved camera dicts from camera_config.load_custom_cameras():
        # {"name", "pos", "quat" (w,x,y,z), "camera_attribs"}.
        self._custom_cameras = custom_cameras or []
        super().__init__(*args, **kwargs)

    def _load_model(self, attempt_num=1):
        # Adding cameras must happen here (not in _setup_kitchen_references,
        # which runs after Kitchen has already merged self.mujoco_arena into
        # self.model -- anything appended to the arena's worldbody after that
        # merge is invisible to the compiled MuJoCo model). We instead append
        # directly onto self.model.worldbody, the same tree that gets handed
        # to MjSim.from_xml_string() once _load_model() returns.
        super()._load_model(attempt_num=attempt_num)
        self._add_custom_cameras()

    def _add_custom_cameras(self):
        for cam in self._custom_cameras:
            root = self.model.worldbody
            parent_body = cam.get("parent_body")
            if parent_body:
                root = find_elements(
                    root=self.model.worldbody,
                    tags="body",
                    attribs={"name": parent_body},
                    return_first=True,
                )
                if root is None:
                    print(
                        f"custom camera {cam['name']!r}: parent_body {parent_body!r} "
                        "not found in the loaded model, skipping"
                    )
                    continue

            existing = find_elements(
                root=root,
                tags="camera",
                attribs={"name": cam["name"]},
                return_first=True,
            )
            attribs = dict(cam.get("camera_attribs") or {})
            attribs["pos"] = array_to_string(cam["pos"])
            attribs["quat"] = array_to_string(cam["quat"])
            if existing is not None:
                print(
                    f"custom camera {cam['name']!r} overwrites an existing camera of the same name"
                )
                for k, v in attribs.items():
                    existing.set(k, v)
            else:
                root.append(new_element(tag="camera", name=cam["name"], **attribs))

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
