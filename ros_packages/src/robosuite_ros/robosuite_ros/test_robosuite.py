import sys
import os
import numpy as np
import robosuite as suite


def main():
    env = suite.make(
        env_name="Lift",
        robots="Panda",
        has_renderer=True,
        has_offscreen_renderer=False,
        use_camera_obs=False,
    )

    env.reset()

    for i in range(1000):
        action = np.random.randn(*env.action_spec[0].shape) * 0.1
        obs, reward, done, info = env.step(action)
        env.render()

    env.close()


if __name__ == '__main__':
    main()