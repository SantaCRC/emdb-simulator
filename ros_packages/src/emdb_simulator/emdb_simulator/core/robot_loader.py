"""Registers custom composite robots with robosuite.

Importing this module has the side effect of registering the robots below
with robosuite's REGISTERED_ROBOTS / ROBOT_CLASS_MAPPING, so it must be
imported before robosuite.make() is called with one of these robot names.
"""
import numpy as np

from robosuite.models.robots.manipulators.ur5e_robot import UR5e
from robosuite.robots import register_robot_class


@register_robot_class("WheeledRobot")
class UR5eOmron(UR5e):
    """UR5e arm mounted on an Omron mobile base, mirroring robosuite's PandaOmron."""

    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "UR5e"}

    @property
    def default_gripper(self):
        # Robotiq85Gripper (UR5e's own default) couples its underactuated
        # finger joints with only a weak spring tendon (stiffness=0.4, no
        # rigid <equality> constraint) in this vendored robosuite asset, so
        # the fingers don't reliably hold together under contact/grasp
        # forces. PandaGripper is fully actuated (no underactuated linkage)
        # and already proven reliable with PandaOmron in this project.
        return {"right": "PandaGripper"}

    @property
    def init_qpos(self):
        return np.array([-0.470, -1.735, 2.480, -2.275, -1.590, -1.991])

    @property
    def init_torso_qpos(self):
        return np.array([0.2])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.6, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }
