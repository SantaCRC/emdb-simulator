import numpy as np
import os
import robosuite as suite
from robosuite_ros.grippers.qbhand2m_gripper import QbHand2MGripper


def make_action(action_shape, grip_cmd, manipulation_cmd=0.0):
    """Builds a full robot action vector with explicit gripper commands."""
    action = np.zeros(action_shape, dtype=float)
    # In UR5e default composite controller, gripper channels are the last two dims.
    action[-2] = grip_cmd
    action[-1] = manipulation_cmd
    return action


def make_arm_grip_action(action_shape, arm_cmd, grip_cmd, manipulation_cmd=0.0):
    """Builds a full action vector for arm + gripper."""
    action = make_action(action_shape, grip_cmd=grip_cmd, manipulation_cmd=manipulation_cmd)
    arm_dims = action_shape[0] - 2
    action[:arm_dims] = np.asarray(arm_cmd, dtype=float)[:arm_dims]
    return action


def main():
    # Enable on-screen render only when explicitly requested.
    use_renderer = os.getenv("ROBOSUITE_RENDER", "0") == "1"

    env = suite.make(
        env_name="Lift",
        robots="UR5e",
        has_renderer=use_renderer,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        gripper_types="QbHand2MGripper",
    )

    try:
        env.reset()

        action_shape = env.action_spec[0].shape

        arm_dims = action_shape[0] - 2

        def step_arm_grip(grip_cmd, manipulation_cmd=0.0, n=1, phase=0.0):
            for i in range(n):
                t = phase + float(i)
                arm_cmd = np.zeros(arm_dims, dtype=float)
                # Oscillate the first 2 arm channels to make robot motion visible.
                if arm_dims > 0:
                    arm_cmd[0] = 0.08 * np.sin(0.03 * t)
                if arm_dims > 1:
                    arm_cmd[1] = 0.06 * np.cos(0.025 * t)
                action = make_arm_grip_action(
                    action_shape,
                    arm_cmd=arm_cmd,
                    grip_cmd=grip_cmd,
                    manipulation_cmd=manipulation_cmd,
                )
                env.step(action)

        step_count = 0

        # Open hand
        step_arm_grip(-1.0, 0.0, n=80, phase=step_count)
        step_count += 80

        # Close hand with smooth ramp to avoid dynamic spikes.
        for g in np.linspace(-1.0, 1.0, 220):
            step_arm_grip(float(g), 0.0, n=1, phase=step_count)
            step_count += 1

        # Hold close
        step_arm_grip(1.0, 0.0, n=60, phase=step_count)
        step_count += 60

        # Open hand again with smooth ramp.
        for g in np.linspace(1.0, -1.0, 260):
            step_arm_grip(float(g), 0.0, n=1, phase=step_count)
            step_count += 1

        print("test_robosuite completed")
    finally:
        env.close()


if __name__ == '__main__':
    main()