# Recording Teleop Demonstrations

The simulator can record every teleop episode to disk and consolidate the
successful ones into a robomimic-compatible `demo.hdf5`, via robosuite's
`DataCollectionWrapper` and the `/save_demos` service
({py:meth}`emdb_simulator.core.scene_loader.SceneLoader._save_demos_cb`).

## 1. Launch with `collect_demos:=true`

```bash
source env.sh
ros2 run emdb_simulator scene_loader --ros-args \
  -p control_mode:=teleop \
  -p collect_demos:=true
```

Each episode is written to a fresh temp directory
(`/tmp/emdb_demo_raw_<timestamp>`); this is logged on startup. Then run
`keyboard_client` (or the `emdb_simulator.launch.py` launch file with
`teleop:=true`, edited to set `collect_demos:=true`) and teleoperate
normally. See
{doc}`teleoperation`.

```{note}
An episode only counts as recordable once you've actually reached a resettable
state. Success is determined per-episode from the env's own `state_*.npz`
checkpoints, not from anything you need to signal manually.
```

## 2. Save the recorded episodes

Once you've recorded one or more successful episodes (resetting the episode
with `q` starts a new one), call `/save_demos`:

```bash
ros2 service call /save_demos emdb_interfaces/srv/SaveDemos "{out_dir: '/tmp/emdb_demos'}"
```

Leaving `out_dir` empty uses the node's `demo_dir` parameter
(default `/tmp/emdb_demos`).

This call:

1. scans the raw episode directory for episodes whose recorded state marks
   them `successful`,
2. gathers only those into a single `demo.hdf5` via RoboCasa's
   `gather_demonstrations_as_hdf5`,
3. converts it in place to robomimic format via `convert_to_robomimic_format`.

The response's `hdf5_path` field points at the resulting file. Calling
`/save_demos` again appends any newly-successful episodes recorded since the
last call, as long as the node hasn't been restarted (the raw temp directory
is per-process).

```{warning}
`/save_demos` returns `success: false` if `collect_demos` was `false` at
launch, or if no episode has succeeded yet.
```

## Next

- {doc}`replaying_demos`: sanity-check the recording by replaying it through
  the ROS RL interface.
- {doc}`training_rl`: or train directly against the live simulator instead
  of from recorded demos.
