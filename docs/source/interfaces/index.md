# Interface Reference (`emdb_interfaces`)

`emdb_interfaces` is an `ament_cmake`/`rosidl` package: pure `.msg`/`.srv`
definitions, generated into Python/C++ bindings at build time, not
Python source Sphinx can introspect. This page documents each one by hand;
it should stay in sync with the source files under
`ros_packages/src/emdb_interfaces` (`msg/` and `srv/`).

## Messages

### `Observation`

Flattened robosuite/RoboCasa `obs_dict`, published on `/observations` every
time the sim steps (see {py:meth}`emdb_simulator.core.scene_loader.SceneLoader._publish_observation`).

| Field | Type | Notes |
|---|---|---|
| `header` | `std_msgs/Header` | |
| `episode_id` | `uint64` | Increments on every reset. |
| `step_id` | `uint64` | Increments on every step, reset to 0 on episode reset. |
| `entries` | `ObservationEntry[]` | One entry per numeric key in the RoboCasa `obs_dict`; non-numeric keys are dropped. |

`AgentBridge` matches `Observation`/`StepInfo` pairs by
`(episode_id, step_id)` before returning from `step()`/`reset()` — see
{py:func}`emdb_policy.agent_bridge.observation_to_dict` for the inverse
(rebuilding an `obs_dict` from this message).

### `ObservationEntry`

One key/value pair from an `obs_dict`.

| Field | Type | Notes |
|---|---|---|
| `key` | `string` | Original `obs_dict` key. |
| `data` | `float64[]` | Value, flattened. |
| `shape` | `uint32[]` | Original shape, to reshape `data` back with `np.reshape`. |

### `StepInfo`

Reward/done signal published on `/reward` in lockstep with `Observation`
(same `episode_id`/`step_id`).

| Field | Type | Notes |
|---|---|---|
| `header` | `std_msgs/Header` | |
| `episode_id` | `uint64` | |
| `step_id` | `uint64` | |
| `reward` | `float64` | |
| `terminated` | `bool` | Set from the env's own success check. |
| `truncated` | `bool` | Always `false` on the simulator side; `EmdbGymEnv` sets this itself once `max_episode_steps` is reached. |
| `success` | `bool` | |

### `ObjectState`

| Field | Type | Notes |
|---|---|---|
| `name` | `string` | MuJoCo joint name containing `"obj"` or `"distr"`. |
| `pose` | `geometry_msgs/Pose` | World-frame position + quaternion orientation. |

### `ObjectStateArray`

Published on `/object_states` every step.

| Field | Type | Notes |
|---|---|---|
| `header` | `std_msgs/Header` | |
| `objects` | `ObjectState[]` | One per free-joint object/distractor found in the MuJoCo model. |

## Services

### `SetDeltaAction`

Used by the keyboard teleop client (`/set_delta_action`); mutates the
active `ROSKeyboardDevice`'s internal state rather than stepping physics
directly (the timer loop in `teleop` mode consumes it next tick).

**Request**

| Field | Type | Notes |
|---|---|---|
| `dx, dy, dz` | `float64` | End-effector position delta (arm mode). |
| `droll, dpitch, dyaw` | `float64` | End-effector rotation delta (arm mode). |
| `base_dx, base_dy, base_dyaw` | `float64` | Base delta (base mode). |
| `grasp` | `int32` | Nonzero toggles the gripper. |
| `reset` | `int32` | Nonzero triggers an episode reset. |
| `toggle_base_mode` | `int32` | Nonzero switches between arm mode / base mode. |
| `next_arm` | `int32` | Nonzero switches the active arm (bimanual robots). |
| `next_robot` | `int32` | Nonzero switches the active robot (multi-robot scenes). |

**Response**

| Field | Type |
|---|---|
| `success` | `bool` |
| `message` | `string` |

### `StepAction`

Synchronous RL step call (`/step_action`). Only served when
`control_mode:=rl`; blocks (on the caller's side, via `AgentBridge`) until
the matching `Observation`/`StepInfo` has been published.

**Request** — same delta/base/grasp/`next_arm`/`next_robot` fields as
`SetDeltaAction`, minus `reset`/`toggle_base_mode` (episode reset goes
through `ResetEpisode` instead).

**Response**

| Field | Type |
|---|---|
| `success` | `bool` |
| `message` | `string` |
| `episode_id` | `uint64` |
| `step_id` | `uint64` |

### `StepActionRaw`

Raw robosuite action-vector passthrough (`/step_action_raw`) — bypasses the
teleop delta-device translation `StepAction` goes through. Used to replay
recorded demo actions that are already in the sim's native `env.step()`
action space (see {doc}`../howto/replaying_demos`).

**Request**

| Field | Type |
|---|---|
| `action` | `float64[]` |

**Response** — same as `StepAction`.

### `ResetEpisode`

Resets the episode (`/reset_episode`), optionally changing layout/style.

**Request**

| Field | Type | Notes |
|---|---|---|
| `layout_id` | `int32` | `-1` keeps the current layout. |
| `style_id` | `int32` | `-1` keeps the current style. |

**Response**

| Field | Type |
|---|---|
| `success` | `bool` |
| `message` | `string` |
| `episode_id` | `uint64` |

### `SaveDemos`

Consolidates recorded teleop episodes (`collect_demos:=true`) into a
robomimic/mimicgen-compatible `demo.hdf5` — see {doc}`../howto/recording_demos`.
Only successful episodes are kept.

**Request**

| Field | Type | Notes |
|---|---|---|
| `out_dir` | `string` | Empty uses the node's `demo_dir` parameter. |

**Response**

| Field | Type |
|---|---|
| `success` | `bool` |
| `message` | `string` |
| `hdf5_path` | `string` |

### `SetPosition`

Used by the standalone example `PositionServer` node (`/set_position`),
unrelated to the RoboCasa environment.

**Request**: `x, y, z` (`float64`). **Response**: `success` (`bool`),
`message` (`string`).

### `SetAction`

| Field | Type |
|---|---|
| `index` | `int32` |
| `value` | `float64` |

**Response**: `success` (`bool`), `message` (`string`).

```{note}
Not currently wired up to any node in this workspace — kept for reference /
future use.
```
