# Training an RL Policy (Stable-Baselines3)

`emdb_policy`'s `train_sb3` executable
({py:mod}`emdb_policy.train_sb3`) trains a Stable-Baselines3 PPO or SAC
policy against {py:class}`emdb_policy.gym_env.EmdbGymEnv`, a `gymnasium.Env`
wrapper around {py:class}`emdb_policy.agent_bridge.AgentBridge`. It has no
dependency on robosuite/robocasa/MuJoCo itself, only on the simulator's ROS
interface.

`KitchenLift`'s reward is sparse (0 until the object is lifted, see
`kitchen.py`'s `reward()`) -- PPO (on-policy) throws its rollout away after
one epoch of updates, so a rare success barely moves it; SAC (off-policy,
`--algo sac`) keeps it in a replay buffer and keeps learning from it. Worth
trying first if PPO isn't picking up any successes.

## 1. Start the simulator in RL mode

`/step_action`/`/step_action_raw` are only served when `control_mode:=rl`
(the periodic teleop render loop is disabled, so physics only advances when
called):

```bash
source env.sh
ros2 run emdb_simulator scene_loader --ros-args -p control_mode:=rl
```

Add `-p headless:=true` for actual training runs: without it, every step
also opens/updates an on-screen `mjviewer` window (nobody's watching it in
a headless run, but it's a real per-step cost -- see {doc}`run_simulator`).
Leave it off (default) if you want to watch the policy train live.

The default `task` is `PickPlaceCounterToCabinet`. For a baseline directly
comparable to the e-MDB experiment in
`mdb_experiments/lift_experiment.yaml`, add `-p task:=KitchenLift` (the
"pick up and raise an object above a height threshold" task; default
`layout_id:=12`/`style_id:=11` both work with it, unlike `layout_id:=11`
which has no kitchen island -- see {doc}`run_simulator`):

```bash
ros2 run emdb_simulator scene_loader --ros-args -p control_mode:=rl -p task:=KitchenLift
```

Video recording is a simulator-side feature (`emdb_simulator.core.video_recorder.VideoRecorder`,
driven by the episode-reset/step hooks) that works the same in `rl` mode as
in teleop -- `train_sb3`/`EmdbGymEnv` don't need to know about it. To record
during training, add:

```bash
ros2 run emdb_simulator scene_loader --ros-args -p control_mode:=rl \
  -p record_video:=true \
  -p record_video_dir:=/tmp/emdb_videos \
  -p record_video_episodes:=0-2 \
  -p record_video_keep_successes:=true
```

`record_video_episodes` (`"all"` or ranges like `"0-2,10-12"`) picks which
episode indices always get a video; `record_video_keep_successes:=true`
additionally keeps any out-of-range episode that succeeds, so a training run
also gives you a highlight reel of what the policy actually solved, without
recording (and storing) every single episode. Videos land in
`record_video_dir` as `run_<timestamp>/episode_<id>.mp4`.

## 2. Train

```bash
ros2 run emdb_policy train_sb3 \
  --timesteps 20000 \
  --save-path /tmp/emdb_ppo
```

Or with `ros2 launch emdb_policy policy_bridge.launch.py`, which starts the
simulator in `rl` mode alongside a policy node (edit the launch file /
run `train_sb3` separately if you want training rather than the default
random-action `policy_node`).

To resume from a checkpoint:

```bash
ros2 run emdb_policy train_sb3 \
  --timesteps 20000 \
  --load-path /tmp/emdb_ppo \
  --save-path /tmp/emdb_ppo
```

### CLI arguments

| Flag | Default | Meaning |
|---|---|---|
| `--algo` | `ppo` | `ppo` or `sac`. See the note above -- `sac` tends to make better use of rare sparse successes. |
| `--timesteps` | `2000` | Total training timesteps for this run. |
| `--max-episode-steps` | `200` | Episode truncation length inside `EmdbGymEnv`. |
| `--n-steps` | `256` | PPO only: rollout length per update. Ignored for `sac`. |
| `--batch-size` | `64` | Minibatch size (both algos). |
| `--ent-coef` | `0.0` | PPO only (`sac` auto-tunes its own). Raising this to explore more is tempting on a sparse reward, but it also keeps the gripper's on/off dimension noisy for longer (`step_vector()`'s toggle-on-sign-change in `agent_bridge.py`), which can hurt more than help -- try `sac` before pushing this up. |
| `--save-path` | `/tmp/emdb_ppo` | Where to save the resulting `.zip` checkpoint. |
| `--load-path` | *(none)* | Existing PPO `.zip` to continue training from, instead of a fresh policy. |
| `--seed` | `0` | PPO seed. |

```{note}
For actually learning the task (as opposed to a quick speed smoke test):
raise `--timesteps` well past the `20000`-ish range used above -- sparse-
reward continuous manipulation typically needs hundreds of thousands of
steps -- and keep `--max-episode-steps` short (the `200` default, not a
larger value tuned for fps): longer episodes mean fewer distinct
`env.reset()` attempts (and less exploration diversity) per timestep
budget, which matters more for learning than raw fps does.
```

## Logs

Alongside the `--save-path` checkpoint, `train_sb3` writes to that path's
directory:

- `monitor.csv`: one row per episode (`r` reward, `l` length, `t` wall-clock,
  `success`), via SB3's `Monitor`. Directly comparable to e-MDB's own
  per-trial logs (`goodness.txt`/`trials.txt`, see `LTM.Files` in
  `mdb_experiments/lift_experiment.yaml`).
- TensorBoard event files (`PPO_1/`, `PPO_2/`, ... one per run in that
  directory): `tensorboard --logdir <save-path's directory>`.
- Per-episode `.mp4`s under `record_video_dir`, if enabled on the simulator
  side (see step 1).

## Action / observation spaces

- **Action**: `Box(-1, 1, shape=(7,))`, `[dx, dy, dz, droll, dpitch, dyaw, gripper]`.
  Position/rotation components are scaled by `pos_scale`/`rot_scale`
  (defaults `0.05`/`0.5`) before being sent; `gripper` is an absolute
  open(`<=0`)/closed(`>0`) command that `AgentBridge.step_vector` converts
  into the sim's toggle convention.
- **Observation**: flattened RoboCasa `obs_dict`, shape discovered from a
  real `env.reset()` at `EmdbGymEnv` construction time (depends on the
  running `task`/`robot`, so don't change those on the simulator side mid
  training run).

## Next

- {doc}`replaying_demos`: validate the ROS interface against real recorded
  behavior before trusting RL training on it.
