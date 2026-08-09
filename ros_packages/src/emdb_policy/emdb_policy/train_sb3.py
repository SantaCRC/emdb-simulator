#!/usr/bin/env python3
"""Train a PPO or SAC policy against the emdb simulator's RL ROS interface.

Run the simulator separately first, with control_mode:=rl:
    ros2 run emdb_simulator scene_loader --ros-args -p control_mode:=rl
Then, to start fresh:
    ros2 run emdb_policy train_sb3 --timesteps 20000 --save-path /tmp/emdb_ppo
    ros2 run emdb_policy train_sb3 --algo sac --timesteps 20000 --save-path /tmp/emdb_sac
Or to continue training an existing checkpoint (same --algo it was saved with):
    ros2 run emdb_policy train_sb3 --timesteps 20000 --load-path /tmp/emdb_ppo --save-path /tmp/emdb_ppo
"""
import argparse
import os

import rclpy
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.monitor import Monitor

from emdb_policy.gym_env import EmdbGymEnv

ALGOS = {"ppo": PPO, "sac": SAC}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--algo", choices=list(ALGOS), default="ppo",
        help="ppo (on-policy, default) throws away each rollout after one epoch of "
             "updates, so on a sparse-reward task (KitchenLift's reward is 0 until the "
             "object is lifted, see kitchen.py's reward()) a rare success barely moves "
             "it. sac is off-policy with a replay buffer, so a single success keeps "
             "getting resampled/learned from -- usually a better fit here.",
    )
    parser.add_argument("--timesteps", type=int, default=2000)
    parser.add_argument("--max-episode-steps", type=int, default=200)
    parser.add_argument("--n-steps", type=int, default=256, help="PPO only.")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument(
        "--ent-coef", type=float, default=0.0,
        help="PPO only (SAC already auto-tunes its own entropy coefficient). SB3 "
             "default 0.0; raising this to explore more on a sparse reward is tempting, "
             "but in practice it also keeps the gripper's on/off dimension noisy for "
             "longer (see step_vector()'s toggle-on-sign-change logic in "
             "agent_bridge.py), so it can hurt more than it helps here -- try sac "
             "before pushing this up.",
    )
    parser.add_argument(
        "--sac-ent-coef", type=str, default="auto",
        help="SAC only. 'auto' (default) lets SB3 auto-tune it -- on this sparse-reward "
             "task that tends to decay toward ~0 (no success yet to justify exploring), "
             "collapsing exploration before ever finding one. Pass a fixed value (e.g. "
             "'0.1') to disable auto-tuning and keep exploration constant instead.",
    )
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

    # Log dir is derived from --save-path (no new flag): keeps monitor.csv
    # (per-episode reward/length/success, comparable to e-MDB's own
    # goodness.txt/trials.txt) and TensorBoard event files next to the
    # checkpoint, so a CESGA .out log points at one place for everything.
    log_dir = os.path.dirname(os.path.abspath(args.save_path))
    os.makedirs(log_dir, exist_ok=True)
    monitor_path = os.path.join(log_dir, "monitor.csv")

    env = EmdbGymEnv(max_episode_steps=args.max_episode_steps)
    env = Monitor(env, filename=monitor_path, info_keywords=("success",))

    algo_cls = ALGOS[args.algo]

    try:
        if args.load_path:
            load_path = args.load_path
            if not load_path.endswith(".zip") and not os.path.isfile(load_path):
                load_path = load_path + ".zip"
            print(f"Loading {args.algo.upper()} checkpoint from {load_path}")
            model = algo_cls.load(load_path, env=env)
            model.tensorboard_log = log_dir
        elif args.algo == "sac":
            print("Initializing a fresh SAC policy")
            model = SAC(
                "MlpPolicy",
                env,
                batch_size=args.batch_size,
                ent_coef=args.sac_ent_coef,
                seed=args.seed,
                verbose=1,
                tensorboard_log=log_dir,
            )
        else:
            print("Initializing a fresh PPO policy")
            model = PPO(
                "MlpPolicy",
                env,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                ent_coef=args.ent_coef,
                seed=args.seed,
                verbose=1,
                tensorboard_log=log_dir,
            )

        model.learn(total_timesteps=args.timesteps, reset_num_timesteps=args.load_path is None)
        model.save(args.save_path)
        print(f"Saved {args.algo.upper()} model to {args.save_path}.zip")
        print(f"Per-episode reward/length/success log: {monitor_path}")
        print(f"TensorBoard logs: {log_dir}")
    finally:
        env.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
