"""Registers the OnRobot 2FG7 gripper with robosuite.

Importing this module has the side effect of registering TwoFG7Gripper in
robosuite's GRIPPER_MAPPING, so it must be imported before robosuite.make()
is called with a robot whose default_gripper resolves to "TwoFG7Gripper"
(see robot_loader.UR5eOmron).
"""
import os

import numpy as np
from ament_index_python.packages import get_package_share_directory

from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

_XML_PATH = os.path.join(
    get_package_share_directory("emdb_simulator"), "assets", "grippers", "twofg7_gripper", "twofg7_gripper.xml"
)


@register_gripper
class TwoFG7Gripper(GripperModel):
    """
    OnRobot 2FG7 parallel gripper (two fingers, single actuated stroke).

    Structurally mirrors robosuite's PandaGripper (two mirrored slide joints
    driven by one action), which is what UR5eOmron used before switching to
    this gripper. Per-finger travel (19mm, 38mm total) comes from OnRobot's
    2FG7 datasheet; see twofg7_gripper.xml for how the CAD geometry was
    derived.

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(os.path.normpath(_XML_PATH), idn=idn)

    def format_action(self, action):
        """
        Maps continuous action into binary output
        -1 => open, 1 => closed

        Args:
            action (np.array): gripper-specific action

        Raises:
            AssertionError: [Invalid action dimension size]
        """
        assert len(action) == self.dof
        self.current_action = np.clip(
            self.current_action + np.array([-1.0, 1.0]) * self.speed * np.sign(action), -1.0, 1.0
        )
        return self.current_action

    @property
    def speed(self):
        return 0.2

    @property
    def dof(self):
        return 1

    @property
    def init_qpos(self):
        return np.array([-0.0095, 0.0095])

    @property
    def _important_geoms(self):
        return {
            "left_finger": ["finger1_collision", "finger1_pad_collision"],
            "right_finger": ["finger2_collision", "finger2_pad_collision"],
            "left_fingerpad": ["finger1_pad_collision"],
            "right_fingerpad": ["finger2_pad_collision"],
        }
