import sys
import os
import numpy as np
import robosuite as suite
from robosuite_ros.grippers.qbhand2m_gripper import QbHand2MGripper

def main():
    env = suite.make(
        env_name="Lift",
        robots="UR5e",
        has_renderer=True,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        gripper_types="QbHand2MGripper",
    )

    env.reset()

    for i in range(1000):
        action = np.random.randn(*env.action_spec[0].shape) * 0.1
        obs, reward, done, info = env.step(action)
        env.render()

    env.close()


if __name__ == '__main__':
    main()