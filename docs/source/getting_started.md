# Getting Started

## Repository layout

```text
emdb_simulator/
├── mdb_experiments/          # e-MDB experiment config templates (not yet wired to any code)
├── misc/                     # git submodules + helper scripts
│   ├── robosuite/            # SantaCRC/robosuite fork
│   ├── robocasa/             # SantaCRC/robocasa fork
│   ├── robosuite_models/     # SantaCRC/robosuite_models fork
│   ├── robomimic/            # robomimic (demo/imitation-learning formats)
│   ├── mimicgen/             # mimicgen (data generation)
│   └── scripts/
├── ros_packages/
│   └── src/
│       ├── emdb_interfaces/  # custom msg/srv definitions (ament_cmake)
│       ├── emdb_simulator/   # simulator node, scene/robot/gripper loaders (ament_python)
│       └── emdb_policy/      # RL/policy node, gym wrapper, training (ament_python)
├── docs/                     # this documentation (Sphinx)
├── setup.sh                  # one-time (idempotent) workspace setup
└── env.sh                    # source this in every new terminal
```

`emdb_interfaces` holds only message/service definitions and has no Python
code of its own; `emdb_simulator` and `emdb_policy` are plain `ament_python`
packages that import it.

## Prerequisites

- Ubuntu with **ROS 2** installed under `/opt/ros/<distro>`. `setup.sh`
  auto-detects whichever distro is present (e.g. Humble); set `ROS_DISTRO`
  yourself beforehand if more than one is installed.
- Python 3 with the `venv` module available.
- `rosdep` (`setup.sh` runs `sudo rosdep init` for you the first time, then
  `rosdep update`/`rosdep install`).

## Cloning with submodules

RoboCasa/robosuite/robosuite_models are git submodules (see
[`.gitmodules`](../../.gitmodules)):

```bash
git clone --recurse-submodules <repo-url> TFM
# or, if already cloned:
git submodule update --init --recursive
```

## Setting up the environment

Run `setup.sh` from the repo root. It is idempotent (safe to re-run; every
step checks whether it's already done before acting):

```bash
./setup.sh            # full setup
./setup.sh --docs     # also install docs/ build dependencies
```

It:

1. locates the installed ROS 2 distro under `/opt/ros` and sources its
   `setup.bash`,
2. initializes any empty git submodules under `misc/`,
3. creates a Python virtualenv at `.venv/` (override with
   `VENV_DIR=/path/to/venv ./setup.sh`) using `--system-site-packages`, so
   the apt-installed ROS 2 build toolchain (`colcon`, rosidl's `lark`
   parser, ...) stays importable, and installs `robosuite`, `robocasa`, and
   (optionally) `robosuite_models` into it in editable mode, plus
   `colcon-common-extensions` (with `empy` pinned to `3.3.4`, since newer
   EmPy breaks ROS 2 Humble's `rosidl_adapter`),
4. resolves ROS package dependencies via `rosdep install`,
5. builds the `ros_packages` workspace with `colcon build --symlink-install`.

Once set up, every shell that runs ROS 2 nodes from this workspace should
source `env.sh` rather than sourcing ROS 2 and the venv manually: ordering
matters (venv vs. ROS `setup.bash`), and a couple of environment variables
are required:

```bash
source env.sh
```

`env.sh`:

- sources ROS 2's `setup.bash`, then the `.venv` virtualenv (in that order,
  so the venv's Python doesn't shadow ROS 2's tools); override the venv
  path with `VENV_DIR=/path/to/venv source env.sh`,
- sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (unless already set in the
  shell),
- sets `MUJOCO_GL=egl` so MuJoCo renders headlessly via EGL instead of GLX
  (works around a `GL_INVALID_VALUE`/`0x501` error some GPU/driver
  combinations hit under GLFW; unset `MUJOCO_GL` before sourcing if your
  machine renders fine without it),
- sources `ros_packages/install/setup.bash` if the workspace has been built,
- enables colcon autocompletion if available on the machine,
- defines convenience aliases: `colcon_dev` (build with the venv's Python
  interpreter), `colcon_clean_build` (wipes `build/ install/ log/` and
  rebuilds), and `emdb_env` (a diagnostic that prints which `ros2`/`python`
  are active; useful for confirming you're not accidentally running the
  system Python).

## Building the ROS 2 workspace

`setup.sh` already builds the workspace once as its last step; to rebuild
after changing code:

```bash
source env.sh
cd ros_packages
colcon_dev             # alias for: python -m colcon build --symlink-install
source install/setup.bash
```

## Building this documentation

The docs are Sphinx + MyST (Markdown), with autodoc for the Python API
reference. From the venv set up above:

```bash
pip install -r docs/requirements.txt   # or: ./setup.sh --docs
cd docs
make html
```

Then open `docs/build/html/index.html` in a browser. Rebuild after editing
docstrings or any file under `docs/source/`.

```{note}
Autodoc mocks `rclpy` and the ROS message packages (`sensor_msgs`,
`geometry_msgs`, `std_msgs`, `std_srvs`, `emdb_interfaces`) so the docs build
without a sourced ROS 2 environment. `robosuite`, `robocasa`, `gymnasium`,
`stable_baselines3`, `pynput`, and `pyarrow` are imported for real, so they
must be installed in whichever environment runs `make html` (the same
`.venv` used to run the nodes works fine).
```

## Next steps

- {doc}`architecture`: how the packages fit together.
- {doc}`howto/index`: task-oriented guides (teleop, training, replay...).
