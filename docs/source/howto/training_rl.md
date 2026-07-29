# Training an RL Policy (Stable-Baselines3)

`emdb_policy`'s `train_sb3` executable
({py:mod}`emdb_policy.train_sb3`) trains a PPO policy against
{py:class}`emdb_policy.gym_env.EmdbGymEnv`, a `gymnasium.Env` wrapper around
{py:class}`emdb_policy.agent_bridge.AgentBridge`. It has no dependency on
robosuite/robocasa/MuJoCo itself — only on the simulator's ROS interface.

## 1. Start the simulator in RL mode

`/step_action`/`/step_action_raw` are only served when `control_mode:=rl`
(the periodic teleop render loop is disabled, so physics only advances when
called):

```bash
source setup_tfm.sh
ros2 run emdb_simulator test_scene_loader --ros-args -p control_mode:=rl
```

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
| `--timesteps` | `2000` | Total PPO training timesteps for this run. |
| `--max-episode-steps` | `200` | Episode truncation length inside `EmdbGymEnv`. |
| `--n-steps` | `256` | PPO rollout length per update. |
| `--batch-size` | `64` | PPO minibatch size. |
| `--save-path` | `/tmp/emdb_ppo` | Where to save the resulting `.zip` checkpoint. |
| `--load-path` | *(none)* | Existing PPO `.zip` to continue training from, instead of a fresh policy. |
| `--seed` | `0` | PPO seed. |

## Action / observation spaces

- **Action**: `Box(-1, 1, shape=(7,))` — `[dx, dy, dz, droll, dpitch, dyaw, gripper]`.
  Position/rotation components are scaled by `pos_scale`/`rot_scale`
  (defaults `0.05`/`0.5`) before being sent; `gripper` is an absolute
  open(`<=0`)/closed(`>0`) command that `AgentBridge.step_vector` converts
  into the sim's toggle convention.
- **Observation**: flattened RoboCasa `obs_dict`, shape discovered from a
  real `env.reset()` at `EmdbGymEnv` construction time (depends on the
  running `task`/`robot`, so don't change those on the simulator side mid
  training run).

## Next

- {doc}`replaying_demos` — validate the ROS interface against real recorded
  behavior before trusting RL training on it.
