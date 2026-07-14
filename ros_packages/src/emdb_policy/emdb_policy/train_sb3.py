#!/usr/bin/env python3
"""Train a PPO policy against the emdb simulator's RL ROS interface.

Run the simulator separately first, with control_mode:=rl:
    ros2 run emdb_simulator test_scene_loader --ros-args -p control_mode:=rl
Then, to start fresh:
    ros2 run emdb_policy train_sb3 --timesteps 20000 --save-path /tmp/emdb_ppo
Or to continue training an existing checkpoint:
    ros2 run emdb_policy train_sb3 --timesteps 20000 --load-path /tmp/emdb_ppo --save-path /tmp/emdb_ppo
"""
import argparse
import os

import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from emdb_policy.gym_env import EmdbGymEnv


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timesteps", type=int, default=2000)
    parser.add_argument("--max-episode-steps", type=int, default=200)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--save-path", type=str, default="/tmp/emdb_ppo")
    parser.add_argument(
        "--load-path", type=str, default=None,
        help="Path to an existing PPO checkpoint (.zip) to continue training from, "
             "instead of initializing a fresh policy.",
    )
    parser.add_argument("--seed", type=int, default=0)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()

    env = EmdbGymEnv(max_episode_steps=args.max_episode_steps)
    env = Monitor(env)

    try:
        if args.load_path:
            load_path = args.load_path
            if not load_path.endswith(".zip") and not os.path.isfile(load_path):
                load_path = load_path + ".zip"
            print(f"Loading PPO checkpoint from {load_path}")
            model = PPO.load(load_path, env=env)
        else:
            print("Initializing a fresh PPO policy")
            model = PPO(
                "MlpPolicy",
                env,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                seed=args.seed,
                verbose=1,
            )

        model.learn(total_timesteps=args.timesteps, reset_num_timesteps=args.load_path is None)
        model.save(args.save_path)
        print(f"Saved PPO model to {args.save_path}.zip")
    finally:
        env.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
