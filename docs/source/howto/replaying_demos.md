# Replaying a Recorded Demo

`emdb_policy`'s `replay_demo` executable
({py:mod}`emdb_policy.replay_demo`) replays a recorded RoboCasa demo episode
through the ROS RL interface (`/step_action_raw`, via `AgentBridge.step_raw`)
as an end-to-end sanity check that the ROS interface reproduces real
recorded behavior.

It reads a **LeRobot-format** episode
(`data/chunk-000/episode_XXXXXX.parquet`) plus the dataset's
`meta/modality.json`, and converts the action columns from the LeRobot
export layout back to the sim's native `env.step()` ordering using RoboCasa's
`ACTION_KEY_ORDERING_HDF5` mapping (the inverse of the reordering RoboCasa's
own `convert_hdf5_lerobot.py` applies when exporting).

```{important}
This is **open-loop** replay: `/reset_episode` reproduces the recorded
task/robot/layout/style, but not the exact object placement from that
episode. Don't expect every replayed episode to report success — this tool
checks that actions are applied correctly, not that the task necessarily
completes.
```

## 1. Start the simulator in RL mode

```bash
source setup_tfm.sh
ros2 run emdb_simulator test_scene_loader --ros-args -p control_mode:=rl
```

## 2. Replay an episode

```bash
ros2 run emdb_policy replay_demo \
  --dataset /path/to/dataset/lerobot \
  --episode 0
```

| Flag | Default | Meaning |
|---|---|---|
| `--dataset` | *(required)* | Path to the LeRobot dataset root (containing `meta/` and `data/`). |
| `--episode` | `0` | Episode index to replay. |

The script prints per-step reward/success/terminated as it goes, and a
`total_reward` summary at the end.

## Producing a LeRobot dataset to replay

Record teleop demos with `collect_demos:=true` (see {doc}`recording_demos`)
to get a robomimic-format `demo.hdf5`, then convert it to LeRobot format with
RoboCasa's own `robocasa/scripts/dataset_scripts/convert_hdf5_lerobot.py`
(outside the scope of this ROS workspace — see the
[robocasa](https://github.com/SantaCRC/robocasa) submodule).
