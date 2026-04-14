"""
QbHand2M Gripper for robosuite
Mano biónica underactuated con 2 sinergias (synergy + manipulation)
"""

import numpy as np
from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion
from robosuite.models.grippers import register_gripper
import importlib.resources as pkg_resources
import os

class QbHand2MGripperBase(GripperModel):

    def __init__(self, idn=0):
        super().__init__(
            fname=str(pkg_resources.files('robosuite_ros').joinpath(
                'models/softhandV2.xml'
            )),
            idn=idn,
        )

    @property
    def init_qpos(self):
        # robosuite resets all gripper joints, not only actuated joints.
        return np.zeros(len(self.joints), dtype=float)

    @property
    def _eef_name(self):          # ← AÑADIR ESTO
        return "eef"

    @property
    def _important_sites(self):
        return {
            "grip_site":     "grip_site",
            "grip_cylinder": "grip_site_cylinder",
            "eef":           "eef",
        }

    @property
    def _important_bodies(self):  # ← AÑADIR ESTO
        return {
            "base": "qbhand2m_base_link",
        }

    @property
    def _important_sensors(self):
        return {
            "force_ee":  "force_ee",
            "torque_ee": "torque_ee",
        }

    @property
    def _important_geoms(self):
        return {
            "left_finger":   [],
            "right_finger":  [],
            "bottom_finger": [],
        }

    @property
    def _important_actuators(self):
        return {
            "qbhand2m1_synergy_act":      "qbhand2m1_synergy_act",
            "qbhand2m1_manipulation_act": "qbhand2m1_manipulation_act",
        }

    @property
    def top_offset(self):
        return np.array([0, 0, 0])

    @property
    def _horizontal_radius(self):
        return 0.07

    @property
    def speed(self):
        return 0.2

    @property
    def dof(self):
        return 2

    def format_action(self, action):
        assert len(action) == self.dof
        synergy_cmd     = (action[0] + 1.0) / 2.0
        manipulation_cmd = action[1]
        return np.array([synergy_cmd, manipulation_cmd])


@register_gripper
class QbHand2MGripper(QbHand2MGripperBase):
    """
    qbHand2M - Mano biónica underactuated (2 sinergias).
    Versión lista para usar en robosuite con robot manipulador.
    
    Uso:
        gripper = gripper_factory("QbHand2MGripper")
    """

    @property
    def init_qpos(self):
        """Mano abierta por defecto."""
        return super().init_qpos

    def format_action(self, action):
        """
        Cierre completo al enviar action=[1, 0]:
          - action[0]=1  -> synergy=1.0 (mano cerrada)
          - action[1]=0  -> manipulation=0.0 (sin oposición)
        Apertura al enviar action=[-1, 0]:
          - action[0]=-1 -> synergy=0.0 (mano abierta)
        """
        return super().format_action(action)
