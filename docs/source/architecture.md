# Architecture

EMDB splits "the simulator" and "the agent" into two independent ROS 2
processes that only talk to each other over topics and services. Neither
`emdb_policy` node imports robosuite/robocasa/MuJoCo — everything
simulation-specific lives in `emdb_simulator`.

```text
┌─────────────────────────┐          topics: /observations, /reward,
│      emdb_simulator      │          /joint_states, /object_states
│  (scene_loader node)     │ ───────────────────────────────────────►
│                          │
│  robosuite + RoboCasa    │ ◄───────────────────────────────────────
│  MuJoCo env, one robot   │   services: /step_action, /step_action_raw,
└─────────────────────────┘   /reset_episode, /set_delta_action,
        ▲                     /reset_env, /save_demos
        │ keyboard input (teleop mode only)
┌─────────────────────────┐
│  keyboard_client node    │
│  (pynput -> deltas)      │
└─────────────────────────┘

┌─────────────────────────┐
│       emdb_policy         │  consumes only the emdb_interfaces API above
│  AgentBridge / gym_env /  │  (no robosuite/robocasa/mujoco imports)
│  policy_node / train_sb3  │
└─────────────────────────┘
```

## `emdb_interfaces`

An `ament_cmake` package holding only `.msg`/`.srv` definitions — the
contract between the other two packages. See {doc}`../interfaces/index` for
every field.

(emdb_simulator)=
## `emdb_simulator`

Owns the actual RoboCasa/robosuite `env`. The core node is
{py:class}`emdb_simulator.core.scene_loader.SceneLoader` (console script
`test_scene_loader`, ROS node name `robocasa_rollout_node`), which:

- builds the robosuite `env` from parameters (`task`, `robot`, `layout_id`,
  `style_id`, `renderer`, ...),
- runs in one of two mutually exclusive **`control_mode`**s:
  - `teleop` (default): a timer loop at `publish_rate` Hz reads the last
    keyboard-driven delta from `ROSKeyboardDevice`, steps the env, and
    republishes state every tick. `/step_action` and `/step_action_raw` are
    rejected in this mode.
  - `rl`: the periodic render loop is disabled entirely; physics only
    advances when `/step_action` or `/step_action_raw` is called, so an
    external agent has full control over the step cadence.
- publishes `/joint_states` (`sensor_msgs/JointState`), `/object_states`
  (`emdb_interfaces/ObjectStateArray`), `/observations`
  (`emdb_interfaces/Observation`, the flattened robosuite `obs_dict`), and
  `/reward` (`emdb_interfaces/StepInfo`) after every step;
- optionally records every teleop episode to disk when `collect_demos:=true`
  (via robosuite's `DataCollectionWrapper`), and consolidates the successful
  ones into a robomimic-format `demo.hdf5` on `/save_demos`.

Two other standalone nodes exist for manual testing:
{py:class}`emdb_simulator.core.keyboard_client.KeyboardDeltaClient` (console
script `test_keyboard_client`), which turns key presses into
`/set_delta_action` calls, and
{py:class}`emdb_simulator.core.position_server.PositionServer`, a minimal
example service/publisher pair unrelated to the RoboCasa env.

(emdb_policy)=
## `emdb_policy`

Everything here talks to `emdb_simulator` exclusively through the
`emdb_interfaces` topics/services, via
{py:class}`emdb_policy.agent_bridge.AgentBridge`:

- runs its own executor on a background thread so blocking `step()`/`reset()`
  calls can be made from synchronous code (e.g. an SB3 training loop);
- `reset()` and `step()` block until the *matching* `Observation`/`StepInfo`
  pair (same `episode_id`/`step_id`) has arrived on `/observations` and
  `/reward`, not just until the service call returns;
- exposes three step variants: `step()` (delta-command fields, same shape as
  teleop), `step_raw()` (a raw native `env.step()` action vector, for
  replaying recorded demos), and `step_vector()` (a flat 7-dim
  `[dx, dy, dz, droll, dpitch, dyaw, gripper]` action, matching a typical
  robomimic policy's output — gripper is absolute open/closed and gets
  translated into the sim's toggle convention internally).

Built on top of `AgentBridge`:

- {py:class}`emdb_policy.gym_env.EmdbGymEnv` — a `gymnasium.Env` (used by
  `train_sb3`), with a `Box(-1, 1)` action space over the 7-dim vector above
  and an observation space whose shape is discovered from a real `reset()`
  call at construction time (RoboCasa's `obs_dict` layout depends on the
  running task/robot).
- {py:mod}`emdb_policy.policy_node` — a standalone `PolicyRunner` that swaps
  in any `policy_fn(obs_dict, rng) -> action_vector` (a random policy by
  default) and drives episodes via `AgentBridge` directly, without gymnasium.
- {py:mod}`emdb_policy.train_sb3` — trains/continues a Stable-Baselines3 PPO
  policy against `EmdbGymEnv`.
- {py:mod}`emdb_policy.replay_demo` — replays a recorded LeRobot-format demo
  episode through `/step_action_raw` as an end-to-end sanity check.

See {doc}`howto/index` for runnable versions of each of these.
