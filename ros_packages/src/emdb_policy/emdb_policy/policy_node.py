#!/usr/bin/env python3
"""Independent policy/RL node: subscribes to observations+reward, acts via /step_action.

Run alongside emdb_simulator's scene_loader with control_mode:=rl, task
matching --policy (KitchenLift for pick_and_lift, KitchenPlace for place):
    ros2 run emdb_simulator scene_loader --ros-args -p control_mode:=rl -p task:=KitchenLift
    ros2 run emdb_policy policy_node --policy pick_and_lift
    ros2 run emdb_simulator scene_loader --ros-args -p control_mode:=rl -p task:=KitchenPlace
    ros2 run emdb_policy policy_node --policy place

`--policy random` (default) keeps the original placeholder behavior. Swap in
a loaded RoboCasa/robomimic checkpoint or an RL algorithm's
`predict(obs) -> action` call the same way scripted_policies.py's classes do
-- this node has no dependency on robosuite/robocasa itself, only on the ROS
interface.
"""
import argparse

import numpy as np
import rclpy

from emdb_policy.agent_bridge import AgentBridge
from emdb_policy.scripted_policies import PickAndLiftPolicy, PlacePolicy


def random_policy(obs_dict, rng):
    """Placeholder policy. Replace with a real policy_fn(obs_dict, rng) -> action_vector."""
    del obs_dict
    action = rng.uniform(low=-0.05, high=0.05, size=6)
    gripper = rng.choice([-1.0, 1.0])
    return np.concatenate([action, [gripper]])


def build_policy(name):
    """Returns (policy_fn, on_episode_start) for --policy <name>."""
    if name == "random":
        return random_policy, None
    if name == "pick_and_lift":
        policy = PickAndLiftPolicy()
        return policy.policy_fn, policy.on_episode_start
    if name == "place":
        policy = PlacePolicy()
        return policy.policy_fn, policy.on_episode_start
    raise ValueError(f"Unknown --policy {name!r}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy", choices=["random", "pick_and_lift", "place"], default="random",
        help="pick_and_lift/place are deterministic, hand-coded policies from "
             "scripted_policies.py; the matching task (KitchenLift/KitchenPlace) "
             "must already be running in scene_loader, see the module docstring.",
    )
    parser.add_argument("--num-episodes", type=int, default=5)
    parser.add_argument("--max-episode-steps", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    return parser.parse_args()


class PolicyRunner:
    def __init__(self, policy_fn=random_policy, on_episode_start=None,
                 max_episode_steps=200, num_episodes=5, seed=0):
        self.policy_fn = policy_fn
        self.on_episode_start = on_episode_start
        self.max_episode_steps = max_episode_steps
        self.num_episodes = num_episodes
        self.rng = np.random.default_rng(seed)

        self.bridge = AgentBridge()
        self.bridge.start()
        self.bridge.wait_for_services()

    def run(self):
        for episode in range(self.num_episodes):
            obs = self.bridge.reset()
            if self.on_episode_start is not None:
                self.on_episode_start()
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


def main():
    args = parse_args()
    rclpy.init()
    policy_fn, on_episode_start = build_policy(args.policy)
    runner = PolicyRunner(
        policy_fn=policy_fn,
        on_episode_start=on_episode_start,
        max_episode_steps=args.max_episode_steps,
        num_episodes=args.num_episodes,
        seed=args.seed,
    )
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
