#!/usr/bin/env python3
"""Independent policy/RL node: subscribes to observations+reward, acts via /step_action.

Run alongside emdb_simulator's scene_loader with control_mode:=rl. Swap
`random_policy` below for a loaded RoboCasa/robomimic checkpoint or an RL
algorithm's `predict(obs) -> action` call -- this node has no dependency on
robosuite/robocasa itself, only on the ROS interface.
"""
import numpy as np
import rclpy

from emdb_policy.agent_bridge import AgentBridge


def random_policy(obs_dict, rng):
    """Placeholder policy. Replace with a real policy_fn(obs_dict, rng) -> action_vector."""
    del obs_dict
    action = rng.uniform(low=-0.05, high=0.05, size=6)
    gripper = rng.choice([-1.0, 1.0])
    return np.concatenate([action, [gripper]])


class PolicyRunner:
    def __init__(self, policy_fn=random_policy, max_episode_steps=200, num_episodes=5, seed=0):
        self.policy_fn = policy_fn
        self.max_episode_steps = max_episode_steps
        self.num_episodes = num_episodes
        self.rng = np.random.default_rng(seed)

        self.bridge = AgentBridge()
        self.bridge.start()
        self.bridge.wait_for_services()

    def run(self):
        for episode in range(self.num_episodes):
            obs = self.bridge.reset()
            episode_return = 0.0

            for t in range(self.max_episode_steps):
                action = self.policy_fn(obs, self.rng)
                obs, reward, terminated, truncated, info = self.bridge.step_vector(action)
                episode_return += reward

                self.bridge.get_logger().info(
                    f"episode={episode} step={t} reward={reward:.4f} "
                    f"success={info['success']} terminated={terminated}"
                )

                if terminated or truncated:
                    break

            self.bridge.get_logger().info(
                f"Episode {episode} finished: return={episode_return:.4f}"
            )

    def close(self):
        self.bridge.close()


def main(args=None):
    rclpy.init(args=args)
    runner = PolicyRunner()
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    finally:
        runner.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
