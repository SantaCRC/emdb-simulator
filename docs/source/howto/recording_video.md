# Recording Video & Camera Setup

The simulator can render offscreen and save one mp4 per episode via
{py:class}`emdb_simulator.core.video_recorder.VideoRecorder`, and load extra
user-defined MuJoCo cameras via
{py:func}`emdb_simulator.core.camera_config.load_custom_cameras`. Both are
opt-in — `has_offscreen_renderer` is only turned on when `record_video` or
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
full list of available MuJoCo camera names, then **exits immediately** — no
episode is stepped and `control_mode`/`teleop` are ignored.

| Parameter | Default | Meaning |
|---|---|---|
| `preview_camera` | `false` | Enable the dry-run: build the scene, save previews, log camera names, exit. |
| `preview_camera_names` | `all` | `all` (every camera in the loaded model) or a comma-separated list of camera names. |

```{tip}
The node also logs every available camera name on startup regardless of
`preview_camera` (look for "Available MuJoCo cameras for
record_video_camera: [...]" ) — useful if you just want the list without
generating images.
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
`record_video_keep_successes`) is never written to disk in the first place
— `VideoRecorder` only opens a writer for episodes it intends to keep.
```

```{warning}
If the configured `record_video_camera` can't be rendered (bad name, or the
task doesn't expose it), the first failed frame logs an error with the list
of available cameras and disables recording for the rest of the run rather
than crashing the node.
```

## 3. Custom cameras

`custom_cameras_file` points at a YAML file defining extra MuJoCo cameras —
selectable via `record_video_camera`/`preview_camera_names` exactly like the
built-in ones (`agentview`, `robot0_agentview_center`, ...). The full field
schema is documented in
[`config/cameras/example_custom_cameras.yaml`](../../../ros_packages/src/emdb_simulator/config/cameras/example_custom_cameras.yaml),
which doubles as a copy-paste starting point: each entry needs a `name`, a
`pos`, and either a `lookat` point (a friendlier alternative — the
quaternion is computed for you) or a raw MuJoCo `quat`, plus optional
`fovy` and `parent_body`.

```bash
ros2 launch emdb_simulator emdb_simulator.launch.py \
  preview_camera:=true \
  custom_cameras_file:=$(pwd)/ros_packages/src/emdb_simulator/config/cameras/example_custom_cameras.yaml
```

```{important}
`custom_cameras_file` is only wired up for `task:=KitchenLift` — other
Kitchen-family tasks don't accept the `custom_cameras` constructor kwarg
{py:class}`emdb_simulator.core.kitchen_lift_task.KitchenLift` adds it for.
```

Setting `parent_body` (a MuJoCo body name, e.g. `mobilebase0_support`)
attaches the camera to that body instead of the world, so it moves with the
robot instead of staying world-fixed — in that case `pos`/`lookat`/`up` are
all interpreted in the body's own local frame, not world coordinates. A bad
`parent_body` name is skipped with a warning rather than failing scene
load; a camera `name` that collides with an existing one (built-in or
another custom entry) simply overwrites its pose. Always `preview_camera`
after adding/editing an entry to confirm it frames what you expect.

## Next

- {doc}`run_simulator` — the full simulator parameter table.
- {doc}`recording_demos` — the *other* kind of recording: teleop episodes
  saved as a robomimic `demo.hdf5` for imitation learning, not video.
