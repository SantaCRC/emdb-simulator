"""Registry of custom robot modules.

Each import below registers that robot class with robosuite as a side
effect (see the individual robot module's own docstring). scene_loader
imports this module once instead of importing every robot module directly.

New entries are appended by scripts/robot_tool.py -- keep one import per
line and don't remove the trailing marker comment.
"""
from emdb_simulator.core import robot_loader  # noqa: F401  registers UR5eOmron
# --- new robots are appended below this line by scripts/robot_tool.py ---
