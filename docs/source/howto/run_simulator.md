# Running the Simulator Node

The core node is `emdb_simulator`'s `scene_loader` executable
({py:class}`emdb_simulator.core.scene_loader.SceneLoader`, ROS node name
`robocasa_rollout_node`). It can be launched directly with `ros2 run` or
through one of the two provided launch files.

## Directly with `ros2 run`

```bash
source env.sh
ros2 run emdb_simulator scene_loader --ros-args \
  -p task:=PickPlaceCounterToCabinet \
  -p robot:=UR5eOmron \
  -p layout_id:=12 \
  -p style_id:=11 \
  -p control_mode:=teleop
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `task` | `PickPlaceCounterToCabinet` | RoboCasa/robosuite environment name. Includes `emdb_simulator`'s own {py:class}`emdb_simulator.core.kitchen_lift_task.KitchenLift` ("pick up and raise an object above a height threshold on a kitchen island"), registered as a side effect of importing `kitchen_lift_task`. |
| `robot` | `UR5eOmron` | Robot model to load. `UR5eOmron` is {py:class}`emdb_simulator.core.robot_loader.UR5eOmron` (a UR5e arm on an Omron mobile base), registered as a side effect of importing `robot_loader`; its default gripper is `TwoFG7Gripper` ({py:class}`emdb_simulator.core.gripper_loader.TwoFG7Gripper`, an OnRobot 2FG7 parallel gripper), registered the same way by `gripper_loader`. |
| `layout_id` | `12` | Kitchen layout id (`-1` picks one at random). Layouts `1` to `10` are "test" layouts and include an `open_cabinet` fixture that some tasks' cabinet-geom lookups can't handle, so prefer `11` to `60` ("train" layouts) unless you specifically need a test layout. Layout `11` has no kitchen island, so it doesn't work with `KitchenLift` (which needs one); layout `12` does. |
| `style_id` | `11` | Kitchen visual style id (`-1` picks one at random). |
| `show_walls` | `false` | Disable the enclosing-wall render wrapper's transparency (walls fully visible). |
| `renderer` | `mjviewer` | robosuite/MuJoCo renderer backend. |
| `headless` | `false` | Don't create the on-screen `mjviewer` window at all (`has_renderer=False`) and skip the per-step `env.render()` call, regardless of `control_mode`. Unrelated to `record_video`/offscreen rendering, which is a separate render path and works the same either way. Turn on (`headless:=true`) for an `rl`-mode training run where nobody's watching the window. Leave off (default) to debug visually, in `teleop` or `rl`. |
| `publish_rate` | `20.0` | Hz for the teleop render/step loop and topic publishing. |
| `control_mode` | `teleop` | `teleop` (timer-driven, keyboard input) or `rl` (external `/step_action` calls only, see {doc}`training_rl`). |
| `collect_demos` | `false` | Record every teleop episode to a temp dir via robosuite's `DataCollectionWrapper`; see {doc}`recording_demos`. |
| `demo_dir` | `/tmp/emdb_demos` | Default output directory for `/save_demos` when the request doesn't specify `out_dir`. |
| `perception_mode` | `unified` | How object perceptions are published: `unified` (single `/object_states` topic with every object), `grouped` (one `/object_states/<fixture_name>` per fixture objects are placed on/in), `split` (one `/object_states/<object_name>` per object), or `mdb` (e-MDB-cognitive-architecture-compatible: per-object topics under `/emdb/simulator/sensor/...` plus a `.../grasped` fact per graspable object and a sparse `.../progress` signal). See {doc}`../architecture`. Fixed for the node's lifetime; restart to change it. |
| `record_video` | `false` | Enable per-episode offscreen mp4 recording. See {doc}`recording_video`. |
| `record_video_dir` | `/tmp/emdb_videos` | Parent directory for recorded episode videos and camera previews. |
| `record_video_episodes` | `all` | Which episodes to record: `all`, a single id (`5`), or comma-separated ids/ranges (`0-2,10-12`). |
| `record_video_camera` | `robot0_agentview_center` | Fixed MuJoCo camera name used for offscreen video recording. |
| `record_video_fps` | `-1.0` | Output video fps; `-1` = auto (`publish_rate / record_video_stride`). |
| `record_video_width` | `1280` | Recording frame width, in pixels. |
| `record_video_height` | `720` | Recording frame height, in pixels. |
| `record_video_stride` | `1` | Capture every Nth simulation step (`1` = every step). |
| `record_video_crf` | `18` | libx264 CRF quality (`0`=lossless, `18`=near-lossless, `23`=default, `51`=worst). |
| `record_video_keep_successes` | `false` | Also keep any episode where the task succeeds, even outside `record_video_episodes`. See {doc}`recording_video` for the cost tradeoff. |
| `preview_camera` | `false` | Debug dry-run: build the scene, save one PNG per camera under `record_video_dir`, log available camera names, then exit. No episode is stepped. Ignores `control_mode`/teleop. |
| `preview_camera_names` | `all` | Cameras to preview: `all` or a comma-separated list of camera names. |
| `custom_cameras_file` | `""` | Path to a YAML file defining extra cameras. Empty = none. Only applied for `task=KitchenLift`. See {doc}`recording_video`. |

## Via the provided launch files

```{list-table}
:header-rows: 1

* - Launch file
  - Package
  - Starts
* - `emdb_simulator.launch.py`
  - `emdb_simulator`
  - `scene_loader` (task `KitchenLift`, `control_mode:=rl` by default). Pass
    `teleop:=true` to switch `control_mode:=teleop` and also start
    `keyboard_client`.
* - `policy_bridge.launch.py`
  - `emdb_policy`
  - `scene_loader` (`control_mode:=rl`) + `emdb_policy`'s `policy_node`
```

```bash
ros2 launch emdb_simulator emdb_simulator.launch.py
# or, for manual teleoperation:
ros2 launch emdb_simulator emdb_simulator.launch.py teleop:=true
# or, for RL via the policy bridge:
ros2 launch emdb_policy policy_bridge.launch.py
```

## Published topics

| Topic | Type | Published |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | every step |
| `/object_states` | `emdb_interfaces/ObjectStateArray` | every step |
| `/observations` | `emdb_interfaces/Observation` | every step |
| `/reward` | `emdb_interfaces/StepInfo` | every step |

See {doc}`../interfaces/index` for the exact message fields.
