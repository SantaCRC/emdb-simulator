# `emdb_simulator`

Owns the robosuite/RoboCasa MuJoCo environment and exposes it over the
`emdb_interfaces` ROS topics/services (see {doc}`../architecture`).

## `emdb_simulator.core.scene_loader`

Console script: `scene_loader`. See {doc}`../howto/run_simulator` for
launch/parameter details.

```{eval-rst}
.. automodule:: emdb_simulator.core.scene_loader
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.keyboard_client`

Console script: `keyboard_client`. See {doc}`../howto/teleoperation` for the
key bindings.

```{eval-rst}
.. automodule:: emdb_simulator.core.keyboard_client
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.ros_keyboard_device`

`robosuite.devices.Device` implementation used internally by `SceneLoader`
to translate `SetDeltaAction`/`StepAction` deltas into robosuite's
`input2action()` protocol.

```{eval-rst}
.. automodule:: emdb_simulator.core.ros_keyboard_device
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.registered_robots`

Registry module: imports one module per custom robot (starting with
`robot_loader`), registering each with robosuite as a side effect. Imported
by `scene_loader` instead of importing every robot module directly; new
entries are appended here by {doc}`../howto/managing_robots`'s
`scripts/robot_tool.py`.

```{eval-rst}
.. automodule:: emdb_simulator.core.registered_robots
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.robot_loader`

Registers `UR5eOmron` (a UR5e arm on an Omron mobile base) with robosuite as
an import side effect; imported by `registered_robots`. See {doc}`../architecture`.

```{eval-rst}
.. automodule:: emdb_simulator.core.robot_loader
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.gripper_loader`

Registers `TwoFG7Gripper` (an OnRobot 2FG7 parallel gripper, `UR5eOmron`'s
default gripper) with robosuite as an import side effect; imported by
`robot_loader`. See {doc}`../architecture`.

```{eval-rst}
.. automodule:: emdb_simulator.core.gripper_loader
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.registered_tasks`

Registry module: imports one module per custom task (starting with
`kitchen_lift_task`), registering each with robosuite as a side effect.
Imported by `scene_loader` instead of importing every task module directly;
new entries are appended here by {doc}`../howto/creating_tasks`'s
`scripts/create_task.py`.

```{eval-rst}
.. automodule:: emdb_simulator.core.registered_tasks
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.kitchen_lift_task`

Registers `KitchenLift`, a "lift" task (pick up a counter-top object and
raise it above a height threshold) that still uses RoboCasa kitchen
layouts/styles, with robosuite as an import side effect; imported by
`registered_tasks`. See {doc}`../howto/run_simulator` for the `task` parameter.

```{eval-rst}
.. automodule:: emdb_simulator.core.kitchen_lift_task
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.position_server`

Console script: `position_server`. Minimal example
publisher/service pair unrelated to the RoboCasa environment.

```{eval-rst}
.. automodule:: emdb_simulator.core.position_server
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.robocasa_node`

Console script: `robocasa_teleop_standalone`. Standalone RoboCasa teleoperation node
using robosuite/RoboCasa's own human-trajectory collection helper directly,
independent of the `SceneLoader`/ROS-interface path used elsewhere in this
package.

```{eval-rst}
.. automodule:: emdb_simulator.core.robocasa_node
   :members:
   :undoc-members:
   :show-inheritance:
```
