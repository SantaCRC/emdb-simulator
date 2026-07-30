# Running the Simulator Node

The core node is `emdb_simulator`'s `test_scene_loader` executable
({py:class}`emdb_simulator.core.scene_loader.SceneLoader`, ROS node name
`robocasa_rollout_node`). It can be launched directly with `ros2 run` or
through one of the two provided launch files.

## Directly with `ros2 run`

```bash
source env.sh
ros2 run emdb_simulator test_scene_loader --ros-args \
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
| `layout_id` | `12` | Kitchen layout id (`-1` picks one at random). Layouts `1`–`10` are "test" layouts and include an `open_cabinet` fixture that some tasks' cabinet-geom lookups can't handle — prefer `11`–`60` ("train" layouts) unless you specifically need a test layout. Layout `11` has no kitchen island, so it doesn't work with `KitchenLift` (which needs one); layout `12` does. |
| `style_id` | `11` | Kitchen visual style id (`-1` picks one at random). |
| `show_walls` | `false` | Disable the enclosing-wall render wrapper's transparency (walls fully visible). |
| `renderer` | `mjviewer` | robosuite/MuJoCo renderer backend. |
| `publish_rate` | `20.0` | Hz for the teleop render/step loop and topic publishing. |
| `control_mode` | `teleop` | `teleop` (timer-driven, keyboard input) or `rl` (external `/step_action` calls only — see {doc}`training_rl`). |
| `collect_demos` | `false` | Record every teleop episode to a temp dir via robosuite's `DataCollectionWrapper`; see {doc}`recording_demos`. |
| `demo_dir` | `/tmp/emdb_demos` | Default output directory for `/save_demos` when the request doesn't specify `out_dir`. |

## Via the provided launch files

```{list-table}
:header-rows: 1

* - Launch file
  - Package
  - Starts
* - `test_robocasa.launch.py`
  - `emdb_simulator`
  - `test_scene_loader` (`control_mode:=teleop`, task `KitchenLift`) + `test_keyboard_client`
* - `policy_bridge.launch.py`
  - `emdb_policy`
  - `test_scene_loader` (`control_mode:=rl`) + `emdb_policy`'s `policy_node`
```

```bash
ros2 launch emdb_simulator test_robocasa.launch.py
# or, for RL:
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
