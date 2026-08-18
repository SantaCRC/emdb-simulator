#!/usr/bin/env python3
"""Publish converted demo transitions onto the e-MDB architecture's episodes
topic, so PolicyLearned's existing episode_callback -> TraceBuffer.add_episode
pipeline (paper_experiment/src/emdb_develop/emdb_cognitive_nodes_gii/
cognitive_nodes/cognitive_nodes/policy.py:555-604) ingests them exactly as it
would ingest a real Reaction rollout step. No file in the architecture repo
is modified: this only publishes standard messages on a topic it already
subscribes to (Control.episodes_topic in lift_experiment.yaml).

Without this seeding, PolicyLearned.calculate_activation (policy.py:479-489)
returns 0.0 -- the main loop never selects it -- until
episodic_buffer.n_traces reaches min_traces (20 by default) from real online
successes, which on a sparse-reward task may never happen on its own.

Requires BOTH workspaces sourced: this one (for emdb_policy) and the
architecture's (for core/cognitive_nodes/core_interfaces, imported here only
as a read-only runtime dependency -- the same relationship mujoco_emdb_sim's
sim_bridge.py already has with this workspace's emdb_interfaces, in reverse).

Before trusting the label schema hardcoded below (PERCEPTION_LABELS /
ACTION_LABELS), verify it against a real message once:
    ros2 topic echo /main_loop/episodes --once
(with lift_launch.py + emdb_simulator.launch.py already running.)

Usage:
    python -m emdb_policy.publish_demo_episodes \\
        --demo-episodes /tmp/emdb_demos/demo_episodes.npz \\
        --episodes-topic /main_loop/episodes
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node

try:
    from core.container import Container
    from core_interfaces.msg import Container as ContainerMsg
    from cognitive_nodes.episode import Episode, episode_obj_to_msg
except ImportError as exc:
    raise ImportError(
        "publish_demo_episodes requires the e-MDB architecture workspace "
        "(core, cognitive_nodes, core_interfaces) to be importable alongside "
        "this one -- source both install/setup.bash files before running.\n"
        f"Original error: {exc}"
    ) from exc

# Perception container labels e-MDB's CognitiveProcess.read_perceptions
# produces by consolidating each Perception node's own container, prefixed
# by node name (cognitive_process.py:339-340): "obj" (x,y,z) + "obj_grasped"
# (data). Matches PolicyLearned._perception_dict_to_obs's obs_dim=4 layout.
PERCEPTION_LABELS = ["obj:x", "obj:y", "obj:z", "obj_grasped:data"]

# Matches lift_experiment.yaml's LTM.Globals.actuation_config
# (arm: [dx, dy, dz, droll, dpitch, dyaw, grasp]).
ACTION_LABELS = ["arm:dx", "arm:dy", "arm:dz", "arm:droll", "arm:dpitch", "arm:dyaw", "arm:grasp"]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo-episodes", required=True, help=".npz produced by prepare_lift_demo_episodes.py")
    parser.add_argument("--episodes-topic", default="/main_loop/episodes")
    parser.add_argument(
        "--reward-name", default="demo_success",
        help="Feature name for the rewards Container. episode_callback sums "
             "every reward key when target_reward is unset (the case in "
             "lift_experiment.yaml), so the exact name doesn't have to match "
             "any real Goal/Drive name.",
    )
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate; 0 disables throttling.")
    parser.add_argument("--max-episodes", type=int, default=None)
    args, _ = parser.parse_known_args()
    return args


def _build_container(name, labels, values, timestamp):
    container = Container(name, max_size=1, container_type=name, labels=labels)
    container.push(
        np.asarray(values, dtype=np.float64).reshape(1, -1),
        src_labels=labels,
        timestamps=[timestamp],
    )
    return container


def build_episode(old_perception, action, perception, reward, reward_name, timestamp):
    """Build an Episode matching one Reaction step, ready for episode_obj_to_msg.

    Uses Episode.update_reward (episode.py) rather than assigning
    episode.reward_list directly: update_reward is the helper that correctly
    produces the double-prefixed "rewards:<name>" label
    consolidate_containers/container_to_episode_obj expect on the wire.
    """
    episode = Episode()
    episode.old_perception = _build_container("old_perception", PERCEPTION_LABELS, old_perception, timestamp)
    episode.action = _build_container("action", ACTION_LABELS, action, timestamp)
    episode.perception = _build_container("perception", PERCEPTION_LABELS, perception, timestamp)
    episode.update_reward({reward_name: float(reward)}, timestamp)
    return episode


class DemoEpisodePublisher(Node):
    def __init__(self, topic):
        super().__init__("publish_demo_episodes")
        self.pub = self.create_publisher(ContainerMsg, topic, 10)


def main():
    args = parse_args()
    data = np.load(args.demo_episodes)

    episode_ids = data["episode_id"]
    unique_eps = np.unique(episode_ids)
    if args.max_episodes:
        unique_eps = unique_eps[: args.max_episodes]

    rclpy.init()
    node = DemoEpisodePublisher(args.episodes_topic)
    period = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0

    try:
        n_published = 0
        for ep_id in unique_eps:
            mask = episode_ids == ep_id
            steps = zip(
                data["old_perception"][mask], data["action"][mask],
                data["perception"][mask], data["reward"][mask],
            )
            for old_p, act, p, r in steps:
                episode = build_episode(old_p, act, p, r, args.reward_name, time.time())
                node.pub.publish(episode_obj_to_msg(episode))
                n_published += 1
                if period:
                    time.sleep(period)
            node.get_logger().info(f"Published demo episode {int(ep_id)} ({int(mask.sum())} steps)")
        node.get_logger().info(
            f"Done: published {n_published} demo transitions across {len(unique_eps)} episodes "
            f"on {args.episodes_topic}"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
