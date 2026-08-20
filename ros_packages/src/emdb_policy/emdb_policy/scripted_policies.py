#!/usr/bin/env python3
"""Deterministic, hand-coded (non-learned) policies for KitchenLift/KitchenPlace.

Each policy is a small closed-loop state machine driven by proportional
control toward object/end-effector positions in obs_dict, exposing a bound
policy_fn(obs_dict, rng) -> action_vector compatible with policy_node.py's
PolicyRunner, plus on_episode_start() to reset state between episodes.

Action vectors are the flat 7-dim [dx, dy, dz, droll, dpitch, dyaw, gripper]
layout AgentBridge.step_vector() expects: gripper is an absolute open(<=0)/
closed(>0) command, and position/rotation are OSC_POSE deltas.

OSC_POSE's input_ref_frame is "base" (composite/basic.json): dx/dy/dz get
added to the controller's goal in its "origin" frame (a per-arm-controller
mount site, not the mobile base body -- see
SceneLoader._augment_obs_with_control_frame), so a world-frame position
error has to be rotated into that frame before being sent. scene_loader
publishes the site's full 3x3 world-frame rotation matrix as
obs_dict["robot0_origin_ori"] (flattened, row-major) -- its transpose is
the world-to-origin-frame rotation.

scene_loader also runs with mirror_actions=True (SceneLoader.__init__),
i.e. every /step_action goes through robosuite's
Device.input2action(mirror_actions=True), which negates dx/dy before OSC
ever sees them (a teleop convenience: the operator's left/right,
forward/back matches what they see facing the robot). Composed with the
origin_ori rotate-then-un-rotate round trip, that negation survives
unchanged for any pure yaw rotation (R @ diag(-1,-1,1) @ R^T ==
diag(-1,-1,1) for rotations about Z, since a 180 deg turn commutes with
any other Z rotation) -- so _to_base_frame has to counter it by negating
x/y right back, regardless of the robot's current orientation.
"""
import numpy as np

XY_APPROACH_THRESHOLD = 0.02  # meters
Z_DESCEND_THRESHOLD = 0.015  # meters
HOVER_HEIGHT = 0.10  # meters above the object to approach from
GRASP_HEIGHT_OFFSET = 0.02  # meters above obj_pos to descend to before closing
GRASP_SETTLE_STEPS = 15  # gripper closes at speed=0.2/step (gripper_loader.py)
LIFT_HEIGHT = 0.12  # meters to raise above the grasp height
PLACE_HOVER_HEIGHT = 0.15  # meters above dest_pos to transport at
PLACE_LOWER_OFFSET = 0.05  # meters above dest_pos to lower to before releasing
RELEASE_SETTLE_STEPS = 10
BUTTON_APPROACH_THRESHOLD = 0.02  # meters, 3D distance (not just XY)
ASK_NICELY_WAIT_STEPS = 30  # default idle duration for IdleMotion


class _FrameControlMixin:
    """World-frame-error -> base-frame-delta p-control, shared by every
    scripted state machine below (see module docstring for why the
    origin_ori un-rotate + mirror_actions negation is needed)."""

    def _to_base_frame(self, world_err, obs_dict):
        origin_ori = obs_dict["robot0_origin_ori"].reshape(3, 3)
        base_err = origin_ori.T @ world_err
        return base_err * np.array([-1.0, -1.0, 1.0])  # undo mirror_actions=True

    def _p_control(self, world_err, obs_dict, kp=4.0, max_delta=0.03):
        base_err = self._to_base_frame(np.asarray(world_err, dtype=np.float64), obs_dict)
        return np.clip(base_err * kp, -max_delta, max_delta)


class ScriptedPolicyBase(_FrameControlMixin):
    """Shared APPROACH -> DESCEND -> GRASP -> LIFT state machine."""

    # Overridden by subclasses that approach a differently-named object
    # (e.g. FruitShop's PickFruitMotion reads "fruit_pos" instead of "obj_pos").
    OBJ_POS_KEY = "obj_pos"

    STATE_APPROACH = "APPROACH"
    STATE_DESCEND = "DESCEND"
    STATE_GRASP = "GRASP"
    STATE_LIFT = "LIFT"
    STATE_DONE = "DONE"

    def on_episode_start(self):
        self.state = self.STATE_APPROACH
        self._counter = 0
        self._lift_start_z = None

    def policy_fn(self, obs_dict, rng):
        del rng  # deterministic policy
        return self._step(obs_dict)

    def _step(self, obs_dict):
        obj_pos = obs_dict[self.OBJ_POS_KEY]
        eef_pos = obs_dict["robot0_eef_pos"]
        action6 = np.zeros(6)
        gripper_cmd = -1.0

        if self.state == self.STATE_APPROACH:
            target = obj_pos + np.array([0, 0, HOVER_HEIGHT])
            err = target - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err[:2]) < XY_APPROACH_THRESHOLD and abs(err[2]) < 0.03:
                self.state = self.STATE_DESCEND

        elif self.state == self.STATE_DESCEND:
            target = obj_pos + np.array([0, 0, GRASP_HEIGHT_OFFSET])
            err = target - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err) < Z_DESCEND_THRESHOLD:
                self.state = self.STATE_GRASP
                self._counter = 0

        elif self.state == self.STATE_GRASP:
            gripper_cmd = 1.0
            target = obj_pos + np.array([0, 0, GRASP_HEIGHT_OFFSET])
            action6[:3] = self._p_control(target - eef_pos, obs_dict, max_delta=0.005)
            self._counter += 1
            if self._counter >= GRASP_SETTLE_STEPS:
                self.state = self.STATE_LIFT
                self._lift_start_z = eef_pos[2]

        elif self.state == self.STATE_LIFT:
            gripper_cmd = 1.0
            target_z = self._lift_start_z + LIFT_HEIGHT
            err = np.array([0.0, 0.0, target_z - eef_pos[2]])
            action6[:3] = self._p_control(err, obs_dict)
            if eef_pos[2] >= target_z - 0.01:
                self.state = self.STATE_DONE

        elif self.state == self.STATE_DONE:
            gripper_cmd = 1.0

        return np.concatenate([action6, [gripper_cmd]])


class PickAndLiftPolicy(ScriptedPolicyBase):
    """Go straight to the cube, close the gripper, lift it (KitchenLift)."""


class PlacePolicy(ScriptedPolicyBase):
    """Pick up the cube, then carry it to dest_pos and release it (KitchenPlace)."""

    STATE_TRANSPORT = "TRANSPORT"
    STATE_LOWER = "LOWER"
    STATE_RELEASE = "RELEASE"

    def _step(self, obs_dict):
        if self.state not in (self.STATE_TRANSPORT, self.STATE_LOWER, self.STATE_RELEASE):
            action = super()._step(obs_dict)
            if self.state == self.STATE_DONE:
                # Base class reached DONE (lifted); hand off to transport.
                self.state = self.STATE_TRANSPORT
            return action

        eef_pos = obs_dict["robot0_eef_pos"]
        dest_pos = obs_dict["dest_pos"]
        action6 = np.zeros(6)
        gripper_cmd = 1.0

        if self.state == self.STATE_TRANSPORT:
            target = dest_pos + np.array([0, 0, PLACE_HOVER_HEIGHT])
            err = target - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err[:2]) < XY_APPROACH_THRESHOLD and abs(err[2]) < 0.03:
                self.state = self.STATE_LOWER

        elif self.state == self.STATE_LOWER:
            target = dest_pos + np.array([0, 0, PLACE_LOWER_OFFSET])
            err = target - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err) < Z_DESCEND_THRESHOLD:
                self.state = self.STATE_RELEASE
                self._counter = 0

        elif self.state == self.STATE_RELEASE:
            gripper_cmd = -1.0
            self._counter += 1
            if self._counter >= RELEASE_SETTLE_STEPS:
                self.state = self.STATE_DONE

        return np.concatenate([action6, [gripper_cmd]])


# --- FruitShop motion primitives ---------------------------------------
#
# fruit_shop_bridge.py drives these one at a time, to completion, per
# executed_policy_service call -- unlike PickAndLiftPolicy/PlacePolicy
# (driven continuously across an episode by policy_node.py's PolicyRunner),
# each of these is instantiated fresh, run via a small step loop, and
# discarded. See fruit_shop_sim_discrete.py (paper_experiment/src/
# emdb_develop) for the reference policy semantics these stand in for.


class PickFruitMotion(ScriptedPolicyBase):
    """Approach, grasp and lift the fruit (pick_fruit)."""

    OBJ_POS_KEY = "fruit_pos"


class TransportReleaseMotion(_FrameControlMixin):
    """Carry an already-held object to TARGET_POS_KEY and release it.

    Standalone TRANSPORT -> LOWER -> RELEASE -> DONE state machine, factored
    out of PlacePolicy's tail so it can target different fixed zones
    (scale_pos, accepted_pos, rejected_pos, placed_pos) without re-grasping
    logic bundled in. Assumes the gripper is already closed around the
    object when on_episode_start() is called -- there is no physical
    teleport between zones, so accept_fruit/discard_fruit re-grasp from the
    scale via a fresh PickFruitMotion before chaining into this.
    """

    TARGET_POS_KEY = "dest_pos"

    STATE_TRANSPORT = "TRANSPORT"
    STATE_LOWER = "LOWER"
    STATE_RELEASE = "RELEASE"
    STATE_DONE = "DONE"

    def __init__(self, target_pos_key=None):
        if target_pos_key is not None:
            self.TARGET_POS_KEY = target_pos_key

    def on_episode_start(self):
        self.state = self.STATE_TRANSPORT
        self._counter = 0

    def policy_fn(self, obs_dict, rng):
        del rng
        return self._step(obs_dict)

    def _step(self, obs_dict):
        eef_pos = obs_dict["robot0_eef_pos"]
        target_pos = obs_dict[self.TARGET_POS_KEY]
        action6 = np.zeros(6)
        gripper_cmd = 1.0

        if self.state == self.STATE_TRANSPORT:
            target = target_pos + np.array([0, 0, PLACE_HOVER_HEIGHT])
            err = target - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err[:2]) < XY_APPROACH_THRESHOLD and abs(err[2]) < 0.03:
                self.state = self.STATE_LOWER

        elif self.state == self.STATE_LOWER:
            target = target_pos + np.array([0, 0, PLACE_LOWER_OFFSET])
            err = target - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err) < Z_DESCEND_THRESHOLD:
                self.state = self.STATE_RELEASE
                self._counter = 0

        elif self.state == self.STATE_RELEASE:
            gripper_cmd = -1.0
            self._counter += 1
            if self._counter >= RELEASE_SETTLE_STEPS:
                self.state = self.STATE_DONE

        elif self.state == self.STATE_DONE:
            gripper_cmd = -1.0

        return np.concatenate([action6, [gripper_cmd]])


class ApproachOnlyMotion(_FrameControlMixin):
    """Move the eef to TARGET_POS_KEY and hold; no grasp change.

    Used for press_button: the arm just has to reach a fixed point, nothing
    is picked up or released. gripper_closed reflects whatever the caller
    is currently holding (or not) so this doesn't accidentally open/close
    on an unrelated object mid-approach.
    """

    TARGET_POS_KEY = "button_pos"

    STATE_APPROACH = "APPROACH"
    STATE_DONE = "DONE"

    def __init__(self, target_pos_key=None):
        if target_pos_key is not None:
            self.TARGET_POS_KEY = target_pos_key

    def on_episode_start(self, gripper_closed=False):
        self.state = self.STATE_APPROACH
        self._gripper_cmd = 1.0 if gripper_closed else -1.0

    def policy_fn(self, obs_dict, rng):
        del rng
        return self._step(obs_dict)

    def _step(self, obs_dict):
        eef_pos = obs_dict["robot0_eef_pos"]
        target_pos = obs_dict[self.TARGET_POS_KEY]
        action6 = np.zeros(6)

        if self.state == self.STATE_APPROACH:
            err = target_pos - eef_pos
            action6[:3] = self._p_control(err, obs_dict)
            if np.linalg.norm(err) < BUTTON_APPROACH_THRESHOLD:
                self.state = self.STATE_DONE

        return np.concatenate([action6, [self._gripper_cmd]])


class IdleMotion:
    """Hold position and the current gripper state for wait_steps ticks.

    Physical stand-in for ask_nicely (no second arm to hand off to, so the
    single-arm adaptation scripts this as an idle wait) and for test_fruit's
    "observe the fruit" phase.
    """

    STATE_WAIT = "WAIT"
    STATE_DONE = "DONE"

    def __init__(self, wait_steps=ASK_NICELY_WAIT_STEPS):
        self.wait_steps = wait_steps

    def on_episode_start(self, gripper_closed=False):
        self.state = self.STATE_WAIT
        self._counter = 0
        self._gripper_cmd = 1.0 if gripper_closed else -1.0

    def policy_fn(self, obs_dict, rng):
        del obs_dict, rng
        if self.state == self.STATE_WAIT:
            self._counter += 1
            if self._counter >= self.wait_steps:
                self.state = self.STATE_DONE
        return np.concatenate([np.zeros(6), [self._gripper_cmd]])
