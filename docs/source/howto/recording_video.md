# Recording Video & Camera Setup

The simulator can render offscreen and save one mp4 per episode via
{py:class}`emdb_simulator.core.video_recorder.VideoRecorder`, and load extra
user-defined MuJoCo cameras via
{py:func}`emdb_simulator.core.camera_config.load_custom_cameras`. Both are
opt-in. `has_offscreen_renderer` is only turned on when `record_video` or
`preview_camera` is set, so normal teleop/RL runs pay no extra cost.

## 1. Preview cameras before recording

Before committing to a (possibly long) recorded rollout, check where each
camera actually points:

```bash
source env.sh
ros2 launch emdb_simulator emdb_simulator.launch.py \
  preview_camera:=true \
  preview_camera_names:=agentview,robot0_agentview_center
```

This builds the scene, saves one still PNG per requested camera under
`record_video_dir/camera_preview_<timestamp>/<camera_name>.png`, logs the
full list of available MuJoCo camera names, then **exits immediately**: no
episode is stepped and `control_mode`/`teleop` are ignored.

| Parameter | Default | Meaning |
|---|---|---|
| `preview_camera` | `false` | Enable the dry-run: build the scene, save previews, log camera names, exit. |
| `preview_camera_names` | `all` | `all` (every camera in the loaded model) or a comma-separated list of camera names. |

```{tip}
The node also logs every available camera name on startup regardless of
`preview_camera` (look for "Available MuJoCo cameras for
record_video_camera: [...]"). This is useful if you just want the list
without generating images.
```

## 2. Recording an episode run

```bash
source env.sh
ros2 launch emdb_simulator emdb_simulator.launch.py \
  record_video:=true \
  record_video_episodes:=0-2 \
  record_video_camera:=robot0_agentview_center
```

Each recorded episode is written to
`record_video_dir/run_<timestamp>/episode_NNNN.mp4` (one fresh `run_*`
subdirectory per launch), encoded with imageio's ffmpeg/libx264 plugin.

| Parameter | Default | Meaning |
|---|---|---|
| `record_video` | `false` | Enable per-episode offscreen mp4 recording. |
| `record_video_dir` | `/tmp/emdb_videos` | Parent directory for recorded videos (and camera previews). |
| `record_video_episodes` | `all` | Which episodes to record: `all`, a single id (`5`), or comma-separated ids/ranges (`0-2,10-12`). A malformed spec matches **no** episodes (logged as a warning) rather than silently recording everything and filling disk. |
| `record_video_camera` | `robot0_agentview_center` | Fixed MuJoCo camera name used for recording. |
| `record_video_fps` | `-1.0` | Output video fps; `-1` = auto (`publish_rate / record_video_stride`). |
| `record_video_width` | `1280` | Frame width, in pixels. |
| `record_video_height` | `720` | Frame height, in pixels. |
| `record_video_stride` | `1` | Capture every Nth simulation step (`1` = every step). |
| `record_video_crf` | `18` | libx264 CRF quality (`0`=lossless, `18`=near-lossless, `23`=default, `51`=worst). |
| `record_video_keep_successes` | `false` | Also record and keep any episode where the task succeeds, even if its index falls outside `record_video_episodes` (an in-range episode is always kept regardless of success). Costs more: *every* episode is recorded while this is on, and non-in-range/non-successful ones are deleted right after. |

```{note}
An episode outside `record_video_episodes` (and not kept via
`record_video_keep_successes`) is never written to disk in the first
place. `VideoRecorder` only opens a writer for episodes it intends to keep.
```

```{warning}
If the configured `record_video_camera` can't be rendered (bad name, or the
task doesn't expose it), the first failed frame logs an error with the list
of available cameras and disables recording for the rest of the run rather
than crashing the node.
```

## 3. Custom cameras

`custom_cameras_file` points at a YAML file defining extra MuJoCo cameras,
selectable via `record_video_camera`/`preview_camera_names` exactly like the
built-in ones (`agentview`, `robot0_agentview_center`, ...). The full field
schema is documented in
[`config/cameras/example_custom_cameras.yaml`](../../../ros_packages/src/emdb_simulator/config/cameras/example_custom_cameras.yaml),
which doubles as a copy-paste starting point: each entry needs a `name`, a
`pos`, and either a `lookat` point (a friendlier alternative: the
quaternion is computed for you) or a raw MuJoCo `quat`, plus optional
`fovy` and `parent_body`.

```bash
ros2 launch emdb_simulator emdb_simulator.launch.py \
  preview_camera:=true \
  custom_cameras_file:=$(pwd)/ros_packages/src/emdb_simulator/config/cameras/example_custom_cameras.yaml
```

```{important}
`custom_cameras_file` is only wired up for `task:=KitchenLift`. Other
Kitchen-family tasks don't accept the `custom_cameras` constructor kwarg
{py:class}`emdb_simulator.core.kitchen_lift_task.KitchenLift` adds it for.
```

Setting `parent_body` (a MuJoCo body name, e.g. `mobilebase0_support`)
attaches the camera to that body instead of the world, so it moves with the
robot instead of staying world-fixed. In that case `pos`/`lookat`/`up` are
all interpreted in the body's own local frame, not world coordinates. A bad
`parent_body` name is skipped with a warning rather than failing scene
load; a camera `name` that collides with an existing one (built-in or
another custom entry) simply overwrites its pose. Always `preview_camera`
after adding/editing an entry to confirm it frames what you expect.

## 4. Headless example: specific episodes + keep-successes + e-MDB perception

A worked example combining everything above for a non-interactive,
headless run: record episodes `0` through `4` explicitly, *also* keep any
later episode that succeeds, publish the e-MDB-compatible perception
topics (`perception_mode:=mdb`, see {doc}`../architecture`), and use the
custom rear camera from section 3 above.

```{important}
**Headless needs `xvfb-run`, not just EGL.** `MUJOCO_GL=egl` (set by
`env.sh`) already makes the actual frame rendering happen offscreen via
EGL, with no GPU display window and no `DISPLAY` required for that part.
But `scene_loader` unconditionally imports
`robocasa.wrappers.enclosing_wall_render_wrapper`, which imports
`pynput.keyboard` at module level regardless of `control_mode`, and
`pynput`'s X11 backend raises `ImportError: this platform is not
supported ... failed to acquire X connection` if there's no X server to
connect to at all, even though nothing ever listens for keys in `rl` mode.
Wrapping the launch in `xvfb-run` gives `pynput` a real (virtual) X
connection to import against, so the node starts cleanly on a machine
with no monitor attached. Confirmed by running this exact command with
`DISPLAY` unset: it fails without `xvfb-run`, and works with it.
```

```bash
source env.sh
xvfb-run -a ros2 launch emdb_simulator emdb_simulator.launch.py \
  perception_mode:=mdb \
  record_video:=true \
  record_video_episodes:=0-4 \
  record_video_keep_successes:=true \
  record_video_camera:=robot0_agentview_center_rear \
  custom_cameras_file:="$(pwd)/ros_packages/src/emdb_simulator/config/cameras/example_custom_cameras.yaml" \
  record_video_dir:=/tmp/emdb_videos
```

| Parameter/flag | Value here | Why |
|---|---|---|
| `xvfb-run -a` | n/a | Not a ROS param. It wraps the process in a virtual X server so `pynput`'s import succeeds with no real display attached (see above). `-a` picks a free virtual display number automatically. |
| `perception_mode` | `mdb` | The e-MDB-cognitive-architecture perception layout: per-object topics under `/emdb/simulator/sensor/...` (here, `/emdb/simulator/sensor/obj`, since `KitchenLift`'s object is named `"obj"`), a `.../grasped` `std_msgs/Bool` for it, and task success as `.../progress` (`std_msgs/Float32`). Reward is modeled as just another perception rather than a dedicated channel. See {doc}`../architecture`. |
| `record_video` | `true` | Turns on per-episode offscreen mp4 recording (and `has_offscreen_renderer`). |
| `record_video_episodes` | `0-4` | Explicitly records (and always keeps) episodes `0` through `4`, the "specific iterations" part of this example. Unquoted ranges/lists like this parse fine on the CLI; only a *single bare integer* (e.g. `0`) needs the `:="'0'"` string-literal trick to stop ROS inferring an int-typed parameter, since this one is a string param. |
| `record_video_keep_successes` | `true` | The "in case of success" part: any episode *outside* `0-4` is still recorded, but only kept on disk if `_check_success()` fires before it ends. Otherwise it's deleted right after. Episodes `0` through `4` are kept unconditionally either way. Costs more (every episode gets recorded while this is on), so only turn it on when you actually want to catch successes past the explicit range. |
| `record_video_camera` | `robot0_agentview_center_rear` | The custom rear-view camera from section 3, parented to `mobilebase0_support` so it follows the robot. |
| `custom_cameras_file` | `config/cameras/example_custom_cameras.yaml` | Required here: `robot0_agentview_center_rear` isn't a built-in camera, it only exists once this file is loaded (and only for `task:=KitchenLift`, this launch file's default task). |
| `record_video_dir` | `/tmp/emdb_videos` | Shown explicitly for clarity; it's already the default. Videos land at `record_video_dir/run_<timestamp>/episode_NNNN.mp4`. |
| `control_mode` (not passed) | stays `rl` | This launch file derives `control_mode` from the `teleop` argument (`teleop:=false` by default -> `rl`); there's no direct `control_mode:=` launch argument. `rl` is the right choice for a headless/scripted run since nothing needs a keyboard. Physics only advances on `/step_action`/`/step_action_raw` calls (see next). |

Since this stays in `rl` mode, something still has to call `/step_action`
to actually advance episodes: either `emdb_policy` (see
{doc}`training_rl`), or, for a quick smoke test, a manual loop:

```bash
for i in $(seq 1 10); do
  ros2 service call /step_action emdb_interfaces/srv/StepAction "{dz: 0.02}"
done
ros2 service call /reset_episode emdb_interfaces/srv/ResetEpisode "{layout_id: -1, style_id: -1}"
```

Stop the node with **Ctrl+C**/`SIGINT` when done. That's what flushes and
closes whichever episode's mp4 is still open.

```{note}
On this kind of headless/EGL setup, `SIGINT` can make the process report a
nonzero/segfault-looking exit *after* it has already finished shutting
down cleanly at the Python level, a native MuJoCo/EGL teardown quirk
during interpreter exit, not data loss. Verified with `ffprobe`: episode
files already being recorded at the time of `SIGINT` were complete and
playable regardless. If you automate this, don't treat a nonzero exit
code alone as proof the run failed. Check for the expected
`episode_NNNN.mp4` files instead.
```

## Next

- {doc}`run_simulator`: the full simulator parameter table.
- {doc}`recording_demos`, for the *other* kind of recording (teleop
  episodes saved as a robomimic `demo.hdf5` for imitation learning, not
  video).
