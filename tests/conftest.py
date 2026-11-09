"""Make the ROS 2 package sources importable without colcon/ROS.

These tests only cover modules that need numpy/scipy/yaml/h5py/imageio,
so they run in a plain Python venv (and in CI's fast `unit` job).
"""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "ros_packages" / "src"
for pkg in ("emdb_policy", "emdb_simulator"):
    sys.path.insert(0, str(SRC / pkg))

# camera_config imports robosuite's MJCF helpers at module level for
# CustomCamerasMixin; the functions tested here don't use them. Stub them
# when robosuite isn't installed so this job stays a light pip install.
try:
    import robosuite.utils.mjcf_utils  # noqa: F401
except ImportError:
    import types

    _mjcf = types.ModuleType("robosuite.utils.mjcf_utils")
    _mjcf.array_to_string = _mjcf.find_elements = _mjcf.new_element = None
    for name in ("robosuite", "robosuite.utils"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["robosuite.utils.mjcf_utils"] = _mjcf
