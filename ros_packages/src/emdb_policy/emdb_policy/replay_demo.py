#!/usr/bin/env python3
"""Replay a recorded RoboCasa human-demo episode through the ROS RL interface.

Reads a LeRobot-format episode (data/chunk-000/episode_XXXXXX.parquet) plus
its dataset's meta/modality.json, converts the action array from the LeRobot
export layout back to the sim's native env.step() ordering, and replays it
via /step_action_raw. Useful as an end-to-end sanity check that the ROS
interface reproduces real recorded behavior -- this is open-loop replay, so
success is not guaranteed since /reset_episode does not reproduce the exact
recorded scene layout (object positions), only the task/robot/layout/style.

Run the simulator separately first, with control_mode:=rl:
    ros2 run emdb_simulator test_scene_loader --ros-args -p control_mode:=rl
Then:
    ros2 run emdb_policy replay_demo --dataset <path>/lerobot --episode 0
"""
import argparse
import json
import os

import numpy as np
import pyarrow.parquet as pq
import rclpy

# Authoritative mapping from LeRobot action keys back to the raw HDF5/robosuite
# env.step() action vector layout -- see robocasa's own conversion script,
# robocasa/scripts/dataset_scripts/convert_hdf5_lerobot.py, which reorders
# raw HDF5 actions into this LeRobot schema via reorder_hdf5_action(). We
# invert that here rather than assuming the LeRobot column order is native.
from robocasa.utils.lerobot_utils import ACTION_KEY_ORDERING_HDF5

from emdb_policy.agent_bridge import AgentBridge


def load_episode_actions(dataset_root, episode_index):
    with open(os.path.join(dataset_root, "meta", "modality.json")) as f:
        modality = json.load(f)
    action_info = modality["action"]

    parquet_path = os.path.join(
        dataset_root, "data", "chunk-000", f"episode_{episode_index:06d}.parquet"
    )
    table = pq.read_table(parquet_path, columns=["action"])
    lerobot_actions = np.array(table.column("action").to_pylist(), dtype=np.float64)

    raw_actions = np.zeros_like(lerobot_actions)
    for key, info in action_info.items():
        lerobot_start, lerobot_end = info["start"], info["end"]
        raw_start, raw_end = ACTION_KEY_ORDERING_HDF5[key]
        raw_actions[:, raw_start:raw_end] = lerobot_actions[:, lerobot_start:lerobot_end]

    return raw_actions


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=str, required=True, help="Path to the lerobot dataset root")
    parser.add_argument("--episode", type=int, default=0)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    actions = load_episode_actions(args.dataset, args.episode)
    print(f"Loaded episode {args.episode}: {actions.shape[0]} steps, action_dim={actions.shape[1]}")

    rclpy.init()
    bridge = AgentBridge(node_name="emdb_replay_demo")
    bridge.start()
    bridge.wait_for_services()

    try:
        bridge.reset()
        total_reward = 0.0
        for t, action in enumerate(actions):
            _obs, reward, terminated, truncated, info = bridge.step_raw(action)
            total_reward += reward
            bridge.get_logger().info(
                f"step={t}/{len(actions)} reward={reward:.4f} "
                f"success={info['success']} terminated={terminated}"
            )
            if terminated or truncated:
                break
        bridge.get_logger().info(f"Replay finished: total_reward={total_reward:.4f}")
    finally:
        bridge.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
