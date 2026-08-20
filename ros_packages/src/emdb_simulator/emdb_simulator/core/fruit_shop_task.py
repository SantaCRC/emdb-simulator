"""Registers FruitShop, a single-arm adaptation of the e-MDB Fruit Shop
experiment (see mdb_experiments/fruit_shop_experiment.yaml).

Importing this module has the side effect of registering FruitShop with
robosuite's REGISTERED_ENVS (via robosuite's EnvMeta), so it must be
imported before robosuite.make() is called with this env name.

Unlike KitchenLift/KitchenPlace, this task has no _check_success()-driven
mission of its own: success/reward for the real e-MDB experiment is computed
by emdb_policy's fruit_shop_bridge.py (a port of the reference
FruitShopSim's stage-gated reward logic, from the paper_experiment/src/
emdb_develop workspace), not by robosuite's env. This task only needs to
expose the physical scene (one fruit, one scale, a handful of fixed
robot-relative target zones) that the bridge's scripted motions act on.

Only one fruit is ever physically instantiated at a time -- the reference
FruitShopSim also only ever perceives/acts on the single closest fruit in
its internal inventory (see perceive_closest_fruit() in
fruit_shop_sim_discrete.py), so there's no need for multiple simultaneous
graspable fruit bodies here.
"""
import numpy as np

from robocasa.environments.kitchen.kitchen import *


class FruitShop(Kitchen):
    """Pick a fruit, test it on the scale, accept/discard it, and place it.

    A single UR5e+2FG7 arm plays every role the two-hand reference
    experiment splits across hands: pick_fruit, place_fruit, test_fruit,
    accept_fruit, discard_fruit, press_button, ask_nicely (change_hands has
    no single-arm equivalent and is dropped, matching the adapted
    single-arm experiment yaml).
    """

    # Provisional -- narrow this down with the empirical grasp trial
    # described in the implementation plan (temporarily point
    # KitchenLift.DEFAULT_OBJ_GROUPS at each candidate and watch
    # /emdb/simulator/sensor/obj/grasped over a few episodes) before
    # trusting this list. TwoFG7Gripper's jaw is small (~31mm nominal
    # opening; see kitchen_lift_task.py), and static bbox math has already
    # been shown unreliable as a predictor here (cube_object.py's own 5cm
    # test cube exceeds that figure yet grasps fine), so only a real sim
    # trial can confirm these.
    DEFAULT_OBJ_GROUPS = ["lime", "kiwi", "cherry", "strawberry", "raspberry"]

    # Fixed offsets (meters, world-frame axes) from the counter fixture's
    # own position -- never raw world constants, since RoboCasa kitchens are
    # procedurally laid out per episode/layout. Mirrors the reference sim's
    # fixed canonical zones (collection_area, weighing_area,
    # accepted_fruit_pos, rejected_fruit_pos, fruit_left/right_side_pos in
    # fruit_shop_sim_discrete.py), collapsed from that sim's polar
    # (distance, angle) convention into offsets convenient for a real
    # placement/fixture-relative frame. Tune by eye once a layout is picked.
    COLLECTION_OFFSET = np.array([-0.25, -0.15, 0.0])
    ACCEPTED_OFFSET = np.array([0.25, -0.35, 0.0])
    REJECTED_OFFSET = np.array([-0.25, -0.35, 0.0])
    PLACED_OFFSET = np.array([0.0, -0.15, 0.0])
    BUTTON_OFFSET = np.array([0.35, -0.05, 0.05])
    SCALE_OFFSET = np.array([0.0, -0.35, 0.0])

    # Read generically by scene_loader._augment_obs_with_control_frame and
    # published into obs_dict as "<name>" for each property listed here.
    OBS_ZONE_ATTRS = ("collection_pos", "accepted_pos", "rejected_pos", "placed_pos", "button_pos")

    def __init__(self, obj_groups=DEFAULT_OBJ_GROUPS, exclude_obj_groups=None, *args, **kwargs):
        self.obj_groups = obj_groups
        self.exclude_obj_groups = exclude_obj_groups
        super().__init__(*args, **kwargs)

    def _setup_kitchen_references(self):
        super()._setup_kitchen_references()
        self.counter = self.register_fixture_ref("counter", dict(id=FixtureType.COUNTER))
        self.init_robot_base_ref = self.counter

    @property
    def collection_pos(self):
        return np.asarray(self.counter.pos, dtype=np.float64) + self.COLLECTION_OFFSET

    @property
    def accepted_pos(self):
        return np.asarray(self.counter.pos, dtype=np.float64) + self.ACCEPTED_OFFSET

    @property
    def rejected_pos(self):
        return np.asarray(self.counter.pos, dtype=np.float64) + self.REJECTED_OFFSET

    @property
    def placed_pos(self):
        return np.asarray(self.counter.pos, dtype=np.float64) + self.PLACED_OFFSET

    @property
    def button_pos(self):
        return np.asarray(self.counter.pos, dtype=np.float64) + self.BUTTON_OFFSET

    def get_ep_meta(self):
        ep_meta = super().get_ep_meta()
        obj_lang = self.get_obj_lang("fruit")
        ep_meta["lang"] = (
            f"Pick up the {obj_lang}, test it on the scale, and accept or "
            "discard it."
        )
        return ep_meta

    def _get_obj_cfgs(self):
        cfgs = []
        cfgs.append(
            dict(
                name="fruit",
                obj_groups=self.obj_groups,
                exclude_obj_groups=self.exclude_obj_groups,
                graspable=True,
                init_robot_here=True,
                # No max_size cap: robocasa's sample_kitchen_object() retry
                # loop (misc/robocasa/robocasa/models/objects/
                # kitchen_object_utils.py:260-296) is unbounded -- if no
                # instance across DEFAULT_OBJ_GROUPS ever fits a cap, it
                # spins forever with no error. None of these categories'
                # measured reg_bbox sizes reliably clear 5cm on all 3 axes,
                # so a max_size here previously hung scene_loader at
                # startup. An oversized fruit just makes PickFruitMotion
                # fail to grasp (a visible _run_motion timeout), which is
                # the correct failure mode until the empirical grasp trial
                # above narrows DEFAULT_OBJ_GROUPS down for real.
                placement=dict(
                    fixture=self.counter,
                    size=(0.35, 0.25),
                    pos=(-1.0, -1.0),
                ),
            )
        )
        # digital_scale: a real, non-graspable RoboCasa prop (see
        # WeighIngredients in misc/robocasa/robocasa/environments/kitchen/
        # composite/measuring_ingredients/weigh_ingredients.py for the
        # precedent this mirrors) used as the physical "scale" surface.
        cfgs.append(
            dict(
                name="scale",
                obj_groups="digital_scale",
                placement=dict(
                    fixture=self.counter,
                    size=(0.3, 0.3),
                    pos=(1.0, -1.0),
                ),
            )
        )
        return cfgs

    def _check_success(self):
        # Success/reward for this experiment is computed by emdb_policy's
        # fruit_shop_bridge.py against /emdb/simulator/sensor/classify_fruit
        # and /emdb/simulator/sensor/place_fruit, not by this env -- there
        # is no single "done" condition to report here (the reference
        # experiment's classify_fruit_mission is terminal, but is satisfied
        # by the bridge's DriveExponential-fed reward, not by scene_loader's
        # generic env._check_success()/progress topic).
        return False
