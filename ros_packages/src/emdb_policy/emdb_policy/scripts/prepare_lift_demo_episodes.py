#!/usr/bin/env python3
"""Convert a recorded demo.hdf5 into per-step transitions matching the e-MDB
architecture's KitchenLift experiment conventions (paper_experiment/src/
emdb_develop/emdb_experiments_gii/experiments/experiments/lift_experiment.yaml):
a 4D perception [obj_x, obj_y, obj_z, obj_grasped] normalized the same way as
mujoco_emdb_sim.perception.EmdbSimulatorPerception, and a 7-dim action in
[0,1] matching what PolicyLearned._action_vec_to_dict remaps a SAC [-1,1]
output into (and what therefore ends up recorded in a real Reaction episode).

Pure offline conversion: no simulator, no ROS, no robomimic/robocasa. Reads
mdb_obj_xyz/mdb_grasped directly from demo.hdf5 -- these are captured live
during teleoperation by scene_loader.py's _capture_demo_perception() and
merged in by _save_demos_cb(), so this script never has to reconstruct or
replay anything (earlier versions tried exactly that: an offline
robomimic-reconstructed env hit a chain of incompatibilities with this
project's custom robot/gripper/object, and live open-loop replay through
/step_action_raw couldn't reproduce the exact recorded object placement,
so its perception -- and, before demo.hdf5 stored a real success signal,
even its reward -- didn't reliably match the real recording).

reward/done: /save_demos (docs/source/howto/recording_demos.md,
scene_loader.py::_save_demos_cb) only ever includes episodes already marked
successful at recording time, so every episode in demo.hdf5 is a known
success -- reward=1.0/done=1.0 is assigned unconditionally to each episode's
last transition, not re-derived from anything.

Usage:
    ros2 run emdb_policy prepare_lift_demo_episodes \\
        --demo-hdf5 /tmp/emdb_demos/demo.hdf5 \\
        --out /tmp/emdb_demos/demo_episodes.npz
"""
import argparse

import h5py
import numpy as np

# Symmetric actuator bounds from lift_experiment.yaml's EmdbSimulator.Actuation
# block: dividing a native-scale action by these recovers the SAC [-1,1]
# value, because sim_bridge.denormalize_actuation's [0,1]->bounds map,
# composed with PolicyLearned._action_vec_to_dict's [-1,1]->[0,1] map,
# simplifies to native = bound * a for symmetric bounds [-bound, bound].
DEFAULT_POS_SCALE = 0.05
DEFAULT_ROT_SCALE = 0.5
DEFAULT_GRASP_SCALE = 1.0

# Perception normalization bounds from lift_experiment.yaml's
# Perception.obj.normalize_data block.
DEFAULT_BOUNDS = dict(x_min=-2.0, x_max=2.0, y_min=-2.0, y_max=2.0, z_min=0.0, z_max=2.0)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo-hdf5", required=True, help="Path to demo.hdf5 (with mdb_obj_xyz/mdb_grasped merged in)")
    parser.add_argument("--out", required=True, help="Output .npz path")
    parser.add_argument("--x-min", type=float, default=DEFAULT_BOUNDS["x_min"])
    parser.add_argument("--x-max", type=float, default=DEFAULT_BOUNDS["x_max"])
    parser.add_argument("--y-min", type=float, default=DEFAULT_BOUNDS["y_min"])
    parser.add_argument("--y-max", type=float, default=DEFAULT_BOUNDS["y_max"])
    parser.add_argument("--z-min", type=float, default=DEFAULT_BOUNDS["z_min"])
    parser.add_argument("--z-max", type=float, default=DEFAULT_BOUNDS["z_max"])
    parser.add_argument("--pos-scale", type=float, default=DEFAULT_POS_SCALE)
    parser.add_argument("--rot-scale", type=float, default=DEFAULT_ROT_SCALE)
    parser.add_argument("--grasp-scale", type=float, default=DEFAULT_GRASP_SCALE)
    parser.add_argument("--max-episodes", type=int, default=None)
    args, _ = parser.parse_known_args()
    return args


def _normalize(value, lo, hi):
    return (value - lo) / (hi - lo) if hi != lo else value


def unscale_action(action_native, pos_scale, rot_scale, grasp_scale):
    """Native OSC-scale [dx,dy,dz,droll,dpitch,dyaw,grasp] -> e-MDB [0,1] actuation.

    Inverse of sim_bridge.denormalize_actuation composed with
    PolicyLearned._action_vec_to_dict (both read-only references in the
    architecture repo, not modified here): native / bound -> SAC [-1,1],
    then (a+1)/2 -> the [0,1] convention actually stored in a real episode.
    """
    scales = np.array([pos_scale, pos_scale, pos_scale, rot_scale, rot_scale, rot_scale, grasp_scale])
    a = np.clip(np.asarray(action_native[:, :7], dtype=np.float64) / scales, -1.0, 1.0)
    return (a + 1.0) / 2.0


def normalize_obj_xyz(obj_xyz, bounds):
    x = _normalize(obj_xyz[:, 0], bounds["x_min"], bounds["x_max"])
    y = _normalize(obj_xyz[:, 1], bounds["y_min"], bounds["y_max"])
    z = _normalize(obj_xyz[:, 2], bounds["z_min"], bounds["z_max"])
    return np.stack([x, y, z], axis=1)


def convert_episode(ep_grp, bounds, pos_scale, rot_scale, grasp_scale):
    """Build (old_perception, action, perception, reward, done) for one
    demo_N group. n actions -> n perception samples -> n-1 transitions
    (perception[t] is the state actions[t] was taken from; there's no
    perception captured "after" the very last action).
    """
    actions_native = ep_grp["actions"][()]
    obj_xyz = ep_grp["mdb_obj_xyz"][()]
    grasped = ep_grp["mdb_grasped"][()]

    n = actions_native.shape[0]
    if obj_xyz.shape[0] != n or grasped.shape[0] != n:
        raise ValueError(
            f"length mismatch: actions={n}, mdb_obj_xyz={obj_xyz.shape[0]}, "
            f"mdb_grasped={grasped.shape[0]}"
        )

    perceptions = np.concatenate(
        [normalize_obj_xyz(obj_xyz, bounds), grasped.reshape(-1, 1)], axis=1
    ).astype(np.float32)
    actions_01 = unscale_action(actions_native, pos_scale, rot_scale, grasp_scale).astype(np.float32)

    # mdb_obj_xyz/mdb_grasped[t] is the state *after* actions[t] was applied
    # (scene_loader.py's _capture_demo_perception runs post-step, unlike
    # robocasa's own pre-step-inclusive `states` convention) -- so transition
    # i pairs (perceptions[i], actions[i+1]) -> perceptions[i+1], not
    # (perceptions[i], actions[i]) -> perceptions[i+1].
    old_perception = perceptions[:-1]
    perception = perceptions[1:]
    action = actions_01[1:]
    reward = np.zeros(n - 1, dtype=np.float32)
    done = np.zeros(n - 1, dtype=np.float32)
    if n > 1:
        # demo.hdf5 only ever contains episodes already verified successful
        # at recording time (see module docstring) -- reward/done are
        # therefore assigned unconditionally, not re-derived from anything.
        reward[-1] = 1.0
        done[-1] = 1.0
    return old_perception, action, perception, reward, done


def main():
    args = parse_args()
    bounds = dict(
        x_min=args.x_min, x_max=args.x_max,
        y_min=args.y_min, y_max=args.y_max,
        z_min=args.z_min, z_max=args.z_max,
    )

    all_old_p, all_action, all_p, all_reward, all_done, all_ep_id = [], [], [], [], [], []
    n_written = 0
    with h5py.File(args.demo_hdf5, "r") as f:
        demo_keys = list(f["data"].keys())
        if args.max_episodes:
            demo_keys = demo_keys[: args.max_episodes]

        for ep in demo_keys:
            ep_grp = f[f"data/{ep}"]
            if "mdb_obj_xyz" not in ep_grp or "mdb_grasped" not in ep_grp:
                print(
                    f"[{ep}] SKIPPED: no mdb_obj_xyz/mdb_grasped in this hdf5 -- "
                    "was it recorded before the scene_loader.py perception-capture change?"
                )
                continue

            old_p, action, p, reward, done = convert_episode(
                ep_grp, bounds, args.pos_scale, args.rot_scale, args.grasp_scale,
            )
            n = old_p.shape[0]
            if n == 0:
                print(f"[{ep}] SKIPPED: only one recorded step, no transitions to build")
                continue

            all_old_p.append(old_p)
            all_action.append(action)
            all_p.append(p)
            all_reward.append(reward)
            all_done.append(done)
            all_ep_id.append(np.full(n, n_written, dtype=np.int32))
            n_written += 1
            print(f"[{ep}] {n} transitions, final_reward={reward[-1]}")

    if not all_old_p:
        raise RuntimeError(f"No usable episodes found in {args.demo_hdf5}")

    np.savez(
        args.out,
        old_perception=np.concatenate(all_old_p),
        action=np.concatenate(all_action),
        perception=np.concatenate(all_p),
        reward=np.concatenate(all_reward),
        done=np.concatenate(all_done),
        episode_id=np.concatenate(all_ep_id),
    )
    total = sum(a.shape[0] for a in all_old_p)
    print(f"Wrote {total} transitions from {n_written}/{len(demo_keys)} episodes to {args.out}")


if __name__ == "__main__":
    main()
