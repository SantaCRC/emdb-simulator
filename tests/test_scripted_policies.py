import numpy as np
import pytest

from emdb_policy import scripted_policies as sp


def _yaw(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _obs(eef, rot=np.eye(3), **extra):
    obs = {"robot0_eef_pos": np.array(eef, dtype=float),
           "robot0_origin_ori": rot.flatten()}
    obs.update({k: np.array(v, dtype=float) for k, v in extra.items()})
    return obs


# robosuite OSC_POSE (composite/basic.json): action +-1 -> +-0.05 m per step
OSC_POS_SCALE = 0.05


def _apply(obs, action):
    """Fake simulator: undo mirror_actions, rotate back to world, move eef."""
    rot = obs["robot0_origin_ori"].reshape(3, 3)
    delta = rot @ (action[:3] * [-1, -1, 1]) * OSC_POS_SCALE
    obs["robot0_eef_pos"] = obs["robot0_eef_pos"] + delta


def _run(policy, obs, max_steps=5000):
    states = []
    for _ in range(max_steps):
        action = policy.policy_fn(obs, rng=None)
        assert action.shape == (7,)
        _apply(obs, action)
        if not states or states[-1] != policy.state:
            states.append(policy.state)
        if policy.state == policy.STATE_DONE:
            return states
    pytest.fail(f"policy never finished, stuck in {policy.state}")


@pytest.mark.parametrize("yaw", [0, 90, -135])
def test_pick_and_lift_reaches_done_and_lifts(yaw):
    obj = [0.3, -0.2, 0.9]
    obs = _obs([0.0, 0.0, 1.2], rot=_yaw(yaw), obj_pos=obj)
    policy = sp.PickAndLiftPolicy()
    policy.on_episode_start()

    states = _run(policy, obs)

    assert states == ["APPROACH", "DESCEND", "GRASP", "LIFT", "DONE"]
    grasp_z = obj[2] + sp.GRASP_HEIGHT_OFFSET
    assert obs["robot0_eef_pos"][2] >= grasp_z + sp.LIFT_HEIGHT - 0.03
    np.testing.assert_allclose(obs["robot0_eef_pos"][:2], obj[:2], atol=0.03)


def test_gripper_open_until_grasp_then_closed():
    obs = _obs([0.0, 0.0, 1.2], obj_pos=[0.1, 0.1, 0.9])
    policy = sp.PickAndLiftPolicy()
    policy.on_episode_start()
    for _ in range(5000):
        state = policy.state
        action = policy.policy_fn(obs, None)
        expected = -1.0 if state in ("APPROACH", "DESCEND") else 1.0
        assert action[6] == expected
        _apply(obs, action)
        if policy.state == "DONE":
            break


def test_p_control_respects_max_delta():
    policy = sp.PickAndLiftPolicy()
    out = policy._p_control([10.0, -10.0, 10.0], _obs([0, 0, 0]), max_delta=0.03)
    np.testing.assert_allclose(np.abs(out), 0.03)


def test_place_policy_releases_at_destination():
    obs = _obs([0.0, 0.0, 1.2], obj_pos=[0.2, 0.0, 0.9], dest_pos=[-0.3, 0.3, 0.9])
    policy = sp.PlacePolicy()
    policy.on_episode_start()
    states = _run(policy, obs, max_steps=10000)
    assert states[-4:] == ["TRANSPORT", "LOWER", "RELEASE", "DONE"]
    np.testing.assert_allclose(obs["robot0_eef_pos"][:2], [-0.3, 0.3], atol=0.03)


def test_transport_release_uses_custom_target_key():
    obs = _obs([0.0, 0.0, 1.2], scale_pos=[0.4, 0.1, 0.9])
    motion = sp.TransportReleaseMotion(target_pos_key="scale_pos")
    motion.on_episode_start()
    _run(motion, obs)
    np.testing.assert_allclose(obs["robot0_eef_pos"][:2], [0.4, 0.1], atol=0.03)


def test_idle_motion_waits_exactly_wait_steps():
    idle = sp.IdleMotion(wait_steps=5)
    idle.on_episode_start(gripper_closed=True)
    for _ in range(4):
        np.testing.assert_array_equal(idle.policy_fn(None, None), [0] * 6 + [1.0])
        assert idle.state == "WAIT"
    idle.policy_fn(None, None)
    assert idle.state == "DONE"


def test_pick_object_motion_uses_obj_pos_key():
    obs = _obs([0.0, 0.0, 1.2], fruit1_pos=[0.2, 0.1, 0.9])
    motion = sp.PickObjectMotion(obj_pos_key="fruit1_pos")
    motion.on_episode_start()
    _run(motion, obs)
    np.testing.assert_allclose(obs["robot0_eef_pos"][:2], [0.2, 0.1], atol=0.03)


def test_obstacle_clear_z_rises_before_moving_sideways():
    start = np.array([0.0, 0.0, 0.9])
    obs = _obs(start, fruit1_pos=[0.3, 0.0, 0.9])
    motion = sp.PickObjectMotion(obj_pos_key="fruit1_pos", obstacle_clear_z=1.4)
    motion.on_episode_start()
    for _ in range(5000):
        if obs["robot0_eef_pos"][2] >= 1.4 - 0.02:
            break
        _apply(obs, motion.policy_fn(obs, None))
        # still below the obstacle: must not drift sideways into it
        if obs["robot0_eef_pos"][2] < 1.4 - 0.02:
            np.testing.assert_allclose(obs["robot0_eef_pos"][:2], start[:2], atol=1e-9)
    _run(motion, obs)


def _release_z(motion, obs):
    """Run until RELEASE starts; return the eef height the object is dropped at."""
    motion.on_episode_start()
    for _ in range(5000):
        _apply(obs, motion.policy_fn(obs, None))
        if motion.state == motion.STATE_RELEASE:
            return obs["robot0_eef_pos"][2]
    pytest.fail(f"never reached RELEASE, stuck in {motion.state}")


def test_release_clears_container_rim():
    obs = _obs([0.0, 0.0, 1.5], basket_pos=[0.3, 0.0, 0.9], basket_top_z=1.1)
    z = _release_z(sp.TransportReleaseMotion(target_pos_key="basket_pos"), obs)
    assert z == pytest.approx(1.1 + sp.CONTAINER_RIM_CLEARANCE, abs=sp.Z_DESCEND_THRESHOLD)


def test_lower_target_z_overrides_rim_clearance():
    obs = _obs([0.0, 0.0, 1.5], blender_pos=[0.3, 0.0, 0.9], blender_top_z=1.3)
    motion = sp.TransportReleaseMotion(target_pos_key="blender_pos", lower_target_z=1.0)
    assert _release_z(motion, obs) == pytest.approx(1.0, abs=sp.Z_DESCEND_THRESHOLD)


def test_transport_retreats_up_after_release():
    obs = _obs([0.0, 0.0, 1.2], dest_pos=[0.3, 0.0, 0.9])
    motion = sp.TransportReleaseMotion()
    release_z = _release_z(motion, obs)
    states = _run(motion, obs)
    assert states == ["RELEASE", "RETREAT", "DONE"]
    assert obs["robot0_eef_pos"][2] >= release_z + sp.RETREAT_HEIGHT - 0.01
