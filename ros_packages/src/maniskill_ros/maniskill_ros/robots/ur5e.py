from robot_descriptions import ur5e_mj_description
import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
@register_agent()
class UR5e(BaseAgent):
    uid = "ur5e"
    mjcf_path = ur5e_mj_description.MJCF_PATH