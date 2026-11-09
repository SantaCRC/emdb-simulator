import h5py
import numpy as np
import pytest

from emdb_policy.scripts import prepare_lift_demo_episodes as prep

BOUNDS = prep.DEFAULT_BOUNDS


def test_normalize():
    assert prep._normalize(0.0, -2.0, 2.0) == 0.5
    assert prep._normalize(2.0, -2.0, 2.0) == 1.0
    assert prep._normalize(3.0, 1.0, 1.0) == 3.0  # zero-width range passes through


def test_unscale_action_maps_bounds_to_unit_interval():
    native = np.array([
        [0.05, -0.05, 0.0, 0.5, -0.5, 0.0, 1.0],
        [1.0, -1.0, 0.0, 9.0, -9.0, 0.0, -5.0],  # out of range -> clipped
    ])
    out = prep.unscale_action(native, 0.05, 0.5, 1.0)
    np.testing.assert_allclose(out[0], [1, 0, 0.5, 1, 0, 0.5, 1])
    np.testing.assert_allclose(out[1], [1, 0, 0.5, 1, 0, 0.5, 0])


def test_unscale_action_ignores_extra_columns():
    native = np.zeros((3, 12))
    assert prep.unscale_action(native, 0.05, 0.5, 1.0).shape == (3, 7)


def test_normalize_obj_xyz():
    xyz = np.array([[-2.0, 0.0, 0.0], [2.0, 2.0, 2.0]])
    np.testing.assert_allclose(prep.normalize_obj_xyz(xyz, BOUNDS), [[0, 0.5, 0], [1, 1, 1]])


def _episode(tmp_path, n, mismatch=False):
    f = h5py.File(tmp_path / "demo.hdf5", "w")
    g = f.create_group("data/demo_0")
    g["actions"] = np.tile([0.05, 0, 0, 0, 0, 0, 1.0], (n, 1))
    g["mdb_obj_xyz"] = np.column_stack([np.zeros(n), np.zeros(n), np.linspace(0, 1, n)])
    g["mdb_grasped"] = np.arange(n - (1 if mismatch else 0)) > n // 2
    return f, g


def test_convert_episode_builds_shifted_transitions(tmp_path):
    f, g = _episode(tmp_path, n=5)
    with f:
        old_p, action, p, reward, done = prep.convert_episode(g, BOUNDS, 0.05, 0.5, 1.0)

    assert old_p.shape == p.shape == (4, 4) and action.shape == (4, 7)
    np.testing.assert_array_equal(old_p[1:], p[:-1])  # perception[t+1] == old_perception[t+1]
    np.testing.assert_allclose(action[0], [1, 0.5, 0.5, 0.5, 0.5, 0.5, 1])
    np.testing.assert_array_equal(reward, [0, 0, 0, 1])
    np.testing.assert_array_equal(done, [0, 0, 0, 1])
    assert old_p.dtype == np.float32


def test_convert_episode_length_mismatch_raises(tmp_path):
    f, g = _episode(tmp_path, n=5, mismatch=True)
    with f, pytest.raises(ValueError, match="length mismatch"):
        prep.convert_episode(g, BOUNDS, 0.05, 0.5, 1.0)


def test_convert_single_step_episode_has_no_transitions(tmp_path):
    f, g = _episode(tmp_path, n=1)
    with f:
        old_p, *_ = prep.convert_episode(g, BOUNDS, 0.05, 0.5, 1.0)
    assert old_p.shape[0] == 0
