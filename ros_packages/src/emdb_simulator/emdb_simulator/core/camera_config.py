#!/usr/bin/env python3
"""Load user-defined custom cameras from a small YAML file.

Each entry needs a `name`, a `pos` (3 floats), and either a raw MuJoCo
`quat` (w,x,y,z -- the native <camera quat="..."> XML convention, matching
robocasa's own CAM_CONFIGS values with zero conversion) or a friendlier
`lookat` point (position to aim at; we compute the quaternion for you).

An entry may also set `parent_body` (a MuJoCo body name, e.g.
"mobilebase0_support") to attach the camera to that body instead of the
world -- it then moves with the robot instead of staying world-fixed.
Whenever `parent_body` is set, `pos`/`lookat`/`up` are all interpreted in
that body's own local frame, not world coordinates (this is exactly how
robocasa's own body-parented default cameras, e.g. robot0_agentview_center,
are defined in CAM_CONFIGS). The look-at math itself doesn't care which
frame it's in -- it only needs `pos` and the target expressed consistently
in the same frame -- so `lookat` works for parented cameras too.

load_custom_cameras() never raises: a missing file, a YAML parse error, or
an invalid entry is logged and skipped (that entry, or the whole file)
rather than crashing node startup. The returned dicts are fully resolved
-- {"name", "pos", "quat" (w,x,y,z), "camera_attribs", "parent_body"} --
ready to hand straight to the XML-writing code in kitchen_lift_task.py.
"""
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def _lookat_to_quat_wxyz(pos, target, up=(0, 0, 1)):
    """Quaternion (w,x,y,z) for a camera at `pos` looking at `target`.

    MuJoCo cameras look down their own local -Z axis, with +Y "up" and +X
    "right" -- the standard OpenGL camera frame convention.
    """
    pos = np.asarray(pos, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(up, dtype=float)

    forward = target - pos
    norm = np.linalg.norm(forward)
    if norm < 1e-8:
        raise ValueError("camera pos and lookat point must differ")
    forward = forward / norm

    if np.linalg.norm(np.cross(forward, up)) < 1e-6:
        # forward (nearly) parallel to up: pick a different up to avoid a
        # degenerate cross product.
        up = np.array([0.0, 1.0, 0.0]) if abs(forward[2]) > 0.9 else np.array([0.0, 0.0, 1.0])

    right = np.cross(forward, up)
    right = right / np.linalg.norm(right)
    cam_up = np.cross(right, forward)

    # Columns = camera's local X, Y, Z axes expressed in world coordinates.
    rot = np.column_stack([right, cam_up, -forward])

    x, y, z, w = Rotation.from_matrix(rot).as_quat()
    return [float(w), float(x), float(y), float(z)]


def _resolve_entry(raw, logger=None):
    name = raw.get("name")
    pos = raw.get("pos")
    if not name or pos is None:
        if logger is not None:
            logger.warning(f"custom camera entry missing name/pos, skipping: {raw!r}")
        return None
    if len(pos) != 3:
        if logger is not None:
            logger.warning(f"custom camera {name!r}: pos must have 3 values, skipping")
        return None

    quat = raw.get("quat")
    lookat = raw.get("lookat")

    if quat is not None and lookat is not None:
        if logger is not None:
            logger.warning(
                f"custom camera {name!r}: both quat and lookat given, using quat"
            )
        lookat = None

    if quat is not None:
        if len(quat) != 4:
            if logger is not None:
                logger.warning(f"custom camera {name!r}: quat must have 4 values, skipping")
            return None
        resolved_quat = [float(v) for v in quat]
    elif lookat is not None:
        if len(lookat) != 3:
            if logger is not None:
                logger.warning(f"custom camera {name!r}: lookat must have 3 values, skipping")
            return None
        up = raw.get("up", (0, 0, 1))
        try:
            resolved_quat = _lookat_to_quat_wxyz(pos, lookat, up=up)
        except ValueError as exc:
            if logger is not None:
                logger.warning(f"custom camera {name!r}: {exc}, skipping")
            return None
    else:
        if logger is not None:
            logger.warning(
                f"custom camera {name!r}: needs either quat or lookat, skipping"
            )
        return None

    camera_attribs = {}
    if "fovy" in raw:
        camera_attribs["fovy"] = str(raw["fovy"])

    parent_body = raw.get("parent_body")

    return {
        "name": str(name),
        "pos": [float(v) for v in pos],
        "quat": resolved_quat,
        "camera_attribs": camera_attribs,
        "parent_body": str(parent_body) if parent_body else None,
    }


def load_custom_cameras(path, logger=None):
    """Read+validate a custom-cameras YAML file. Returns a list of
    resolved camera dicts (possibly empty). Never raises.
    """
    if not path:
        return []

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        if logger is not None:
            logger.error(f"Could not read custom_cameras_file {path!r}: {exc}")
        return []

    raw_cameras = data.get("cameras") or []
    cameras = []
    for raw in raw_cameras:
        resolved = _resolve_entry(raw, logger=logger)
        if resolved is not None:
            cameras.append(resolved)

    if logger is not None:
        logger.info(f"Loaded {len(cameras)} custom camera(s) from {path}")

    return cameras
