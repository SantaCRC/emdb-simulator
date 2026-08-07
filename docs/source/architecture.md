# Architecture

EMDB splits "the simulator" and "the agent" into two independent ROS 2
processes that only talk to each other over topics and services. Neither
`emdb_policy` node imports robosuite/robocasa/MuJoCo. Everything
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

An `ament_cmake` package holding only `.msg`/`.srv` definitions: the
contract between the other two packages. See {doc}`../interfaces/index` for
every field.

(emdb_simulator)=
## `emdb_simulator`

Owns the actual RoboCasa/robosuite `env`. The core node is
{py:class}`emdb_simulator.core.scene_loader.SceneLoader` (console script
`scene_loader`, ROS node name `robocasa_rollout_node`), which:

- builds the robosuite `env` from parameters (`task`, `robot`, `layout_id`,
  `style_id`, `renderer`, ...). Importing `scene_loader` transitively
  imports {py:mod}`emdb_simulator.core.registered_robots` and
  {py:mod}`emdb_simulator.core.registered_tasks`, small registry modules
  that each import one module per custom robot/task (starting with
  `robot_loader`'s `UR5eOmron` and `kitchen_lift_task`'s `KitchenLift`),
  registering them with robosuite as a side effect; `robot_loader` in turn
  imports {py:mod}`emdb_simulator.core.gripper_loader`, which registers
  `TwoFG7Gripper` (`UR5eOmron`'s default gripper). New robots/tasks created
  via {doc}`howto/managing_robots`/{doc}`howto/creating_tasks` are appended
  to these registries automatically,
- runs in one of two mutually exclusive **`control_mode`**s:
  - `teleop` (default): a timer loop at `publish_rate` Hz reads the last
    keyboard-driven delta from `ROSKeyboardDevice`, steps the env, and
    republishes state every tick. `/step_action` and `/step_action_raw` are
    rejected in this mode.
  - `rl`: the periodic render loop is disabled entirely; physics only
    advances when `/step_action` or `/step_action_raw` is called, so an
    external agent has full control over the step cadence.
- publishes `/joint_states` (`sensor_msgs/JointState`), object poses
  (`emdb_interfaces/ObjectStateArray`), `/observations`
  (`emdb_interfaces/Observation`, the flattened robosuite `obs_dict`), and
  `/reward` (`emdb_interfaces/StepInfo`) after every step. Object poses are
  read generically from `self.env.objects` (works for any task, not just
  ones that name their object `"obj"`), and where they're published depends
  on the `perception_mode` parameter (see {doc}`howto/run_simulator`):
  `unified` (default) puts every object on a single `/object_states`;
  `grouped` splits them into one `/object_states/<fixture_name>` topic per
  fixture objects are placed on/in (resolved from each object's
  `_get_obj_cfgs` placement, following `placement["object"]` references
  transitively for objects placed relative to another object rather than a
  fixture directly); `split` gives each object its own
  `/object_states/<object_name>` topic; `mdb` targets the
  [e-MDB cognitive-architecture framework](https://docs.pillar-robots.eu/)'s
  perception convention (a fixed set of `{name, topic, message type}`
  publishers, one topic per named percept, no bundled arrays). It reuses
  `split`'s per-object mechanism but under `/emdb/simulator/sensor/...`
  instead of `/object_states/...`, adds a `std_msgs/Bool` on
  `/emdb/simulator/sensor/<object_name>/grasped` for every object whose cfg
  has `graspable=True` (via `robocasa.utils.object_utils.check_obj_grasped`),
  and publishes task success as a sparse `std_msgs/Float32` "perception" on
  `/emdb/simulator/sensor/progress` (`1.0`/`0.0`, reusing `_check_success()`)
  rather than only exposing it via `/reward`, since that framework models
  reward as just another named perception rather than a dedicated channel;
- optionally renders offscreen and saves one mp4 per episode when
  `record_video:=true` ({py:class}`emdb_simulator.core.video_recorder.VideoRecorder`,
  driven by the episode-reset and per-step hooks), or, with
  `preview_camera:=true`, short-circuits into a one-shot dry run that saves
  a still PNG per camera and exits without stepping an episode
  ({py:func}`emdb_simulator.core.video_recorder.save_camera_previews`).
  `custom_cameras_file` (`task=KitchenLift` only) loads extra MuJoCo cameras
  from a YAML file via
  {py:func}`emdb_simulator.core.camera_config.load_custom_cameras` and
  appends them to the compiled model in `KitchenLift._load_model`. Offscreen
  rendering (`has_offscreen_renderer`) is only enabled when one of these two
  features is. See {doc}`howto/recording_video`;
- optionally records every teleop episode to disk when `collect_demos:=true`
  (via robosuite's `DataCollectionWrapper`), and consolidates the successful
  ones into a robomimic-format `demo.hdf5` on `/save_demos`.

One other standalone node exists for manual testing:
{py:class}`emdb_simulator.core.keyboard_client.KeyboardDeltaClient` (console
script `keyboard_client`), which turns key presses into
`/set_delta_action` calls.

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
  robomimic policy's output; gripper is absolute open/closed and gets
  translated into the sim's toggle convention internally).

Built on top of `AgentBridge`:

- {py:class}`emdb_policy.gym_env.EmdbGymEnv`: a `gymnasium.Env` (used by
  `train_sb3`), with a `Box(-1, 1)` action space over the 7-dim vector above
  and an observation space whose shape is discovered from a real `reset()`
  call at construction time (RoboCasa's `obs_dict` layout depends on the
  running task/robot).
- {py:mod}`emdb_policy.policy_node`: a standalone `PolicyRunner` that swaps
  in any `policy_fn(obs_dict, rng) -> action_vector` (a random policy by
  default) and drives episodes via `AgentBridge` directly, without gymnasium.
- {py:mod}`emdb_policy.train_sb3`: trains/continues a Stable-Baselines3 PPO
  policy against `EmdbGymEnv`.
- {py:mod}`emdb_policy.replay_demo`: replays a recorded LeRobot-format demo
  episode through `/step_action_raw` as an end-to-end sanity check.

See {doc}`howto/index` for runnable versions of each of these.
