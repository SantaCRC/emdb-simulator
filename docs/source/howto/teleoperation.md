# Teleoperating the Robot with a Keyboard

This drives the simulator's `/set_delta_action` service by hand, using the
`keyboard_client` node
({py:class}`emdb_simulator.core.keyboard_client.KeyboardDeltaClient`).

## 1. Launch simulator + keyboard client

```bash
source env.sh
ros2 launch emdb_simulator emdb_simulator.launch.py teleop:=true
```

The `teleop:=true` argument starts `scene_loader` with `control_mode:=teleop`
and `keyboard_client` together (by default, without `teleop:=true`, the
launch file only starts `scene_loader` in `rl` mode and no keyboard client).
Make sure the terminal/window running the
keyboard client has keyboard focus. `pynput` listens globally, so any
window works, but only the process that has focus will feel responsive.

## 2. Controls

**Arm mode** (default):

| Keys | Action |
|---|---|
| Arrow keys | Move end-effector in X/Y |
| Shift+↓ / Shift+↑ | Move end-effector down / up (Z) |
| `e` / `r` | Roll +/- |
| `y` / `h` | Pitch +/- |
| `p` / `o` | Yaw +/- |
| Space (on release) | Toggle grasp |
| `q` (on release) | Reset episode |
| `s` (on release) | Switch active arm (bimanual robots) |
| `=` (on release) | Switch active robot (multi-robot scenes) |
| `b` (on release) | Toggle arm mode / base mode |

**Base mode** (after pressing `b`):

| Keys | Action |
|---|---|
| Arrow keys | Move base in X/Y |
| `o` / `p` | Rotate base +/- |

Each key press sends a `SetDeltaAction` request; see
{doc}`../interfaces/index` for the full field list (base deltas, `grasp`,
`reset`, `next_arm`, `next_robot`, `toggle_base_mode`).

## 3. Changing task/robot/layout

Pass parameters when launching, or edit `emdb_simulator.launch.py`'s
`parameters=[...]` block. See {doc}`run_simulator` for the full parameter
table. To change layout/style *without* restarting the node, call
`/reset_episode` with a specific `layout_id`/`style_id` (`-1` keeps the
current one):

```bash
ros2 service call /reset_episode emdb_interfaces/srv/ResetEpisode "{layout_id: 12, style_id: 3}"
```

## Next

- {doc}`recording_demos`: capture what you just did as training data.
