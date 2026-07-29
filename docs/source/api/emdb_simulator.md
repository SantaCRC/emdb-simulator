# `emdb_simulator`

Owns the robosuite/RoboCasa MuJoCo environment and exposes it over the
`emdb_interfaces` ROS topics/services (see {doc}`../architecture`).

## `emdb_simulator.core.scene_loader`

Console script: `test_scene_loader`. See {doc}`../howto/run_simulator` for
launch/parameter details.

```{eval-rst}
.. automodule:: emdb_simulator.core.scene_loader
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.keyboard_client`

Console scripts: `test_keyboard_client`, `test_keyboard_teleop`. See
{doc}`../howto/teleoperation` for the key bindings.

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

## `emdb_simulator.core.position_server`

Console script: `test_position_server`. Minimal example
publisher/service pair unrelated to the RoboCasa environment.

```{eval-rst}
.. automodule:: emdb_simulator.core.position_server
   :members:
   :undoc-members:
   :show-inheritance:
```

## `emdb_simulator.core.robocasa_node`

Console script: `test_robocasa`. Standalone RoboCasa teleoperation node
using robosuite/RoboCasa's own human-trajectory collection helper directly,
independent of the `SceneLoader`/ROS-interface path used elsewhere in this
package.

```{eval-rst}
.. automodule:: emdb_simulator.core.robocasa_node
   :members:
   :undoc-members:
   :show-inheritance:
```
