from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from emdb_simulator.core.camera_config import _lookat_to_quat_wxyz, load_custom_cameras


def _cam_forward(quat_wxyz):
    w, x, y, z = quat_wxyz
    return Rotation.from_quat([x, y, z, w]).as_matrix() @ [0, 0, -1]  # MuJoCo looks down -Z


@pytest.mark.parametrize("pos, target", [
    ([1, 0, 1], [0, 0, 0]),
    ([0, 2, 0.5], [0, 0, 0.5]),
    ([0, 0, 3], [0, 0, 0]),  # straight down: degenerate "up" fallback
])
def test_lookat_points_camera_at_target(pos, target):
    q = _lookat_to_quat_wxyz(pos, target)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    expected = np.subtract(target, pos) / np.linalg.norm(np.subtract(target, pos))
    np.testing.assert_allclose(_cam_forward(q), expected, atol=1e-6)


def test_lookat_same_point_raises():
    with pytest.raises(ValueError):
        _lookat_to_quat_wxyz([1, 1, 1], [1, 1, 1])


def test_load_custom_cameras(tmp_path):
    f = tmp_path / "cams.yaml"
    f.write_text("""
cameras:
  - {name: top, pos: [0, 0, 3], quat: [1, 0, 0, 0], fovy: 60}
  - {name: side, pos: [1, 0, 1], lookat: [0, 0, 0], parent_body: mobilebase0_support}
  - {name: no_pos}
  - {name: short_pos, pos: [1, 2], quat: [1, 0, 0, 0]}
  - {name: no_orientation, pos: [1, 2, 3]}
  - {name: bad_quat, pos: [1, 2, 3], quat: [1, 0]}
  - {name: same_point, pos: [1, 1, 1], lookat: [1, 1, 1]}
""")
    logger = Mock()
    cams = load_custom_cameras(str(f), logger=logger)

    assert [c["name"] for c in cams] == ["top", "side"]
    assert cams[0] == {"name": "top", "pos": [0.0, 0.0, 3.0], "quat": [1.0, 0.0, 0.0, 0.0],
                       "camera_attribs": {"fovy": "60"}, "parent_body": None}
    assert cams[1]["parent_body"] == "mobilebase0_support"
    assert logger.warning.call_count == 5


def test_quat_wins_over_lookat(tmp_path):
    f = tmp_path / "cams.yaml"
    f.write_text("cameras: [{name: c, pos: [1, 0, 0], quat: [0, 1, 0, 0], lookat: [0, 0, 0]}]")
    assert load_custom_cameras(str(f))[0]["quat"] == [0.0, 1.0, 0.0, 0.0]


@pytest.mark.parametrize("content", [None, "", "cameras: [unclosed"])
def test_missing_empty_or_broken_file_returns_empty(tmp_path, content):
    path = tmp_path / "cams.yaml"
    if content is not None:
        path.write_text(content)
    assert load_custom_cameras(str(path)) == []
    assert load_custom_cameras("") == []
