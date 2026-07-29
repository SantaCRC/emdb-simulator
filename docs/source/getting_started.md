# Getting Started

## Repository layout

```text
TFM/
├── misc/                     # git submodules + helper scripts
│   ├── robosuite/            # SantaCRC/robosuite fork
│   ├── robocasa/             # SantaCRC/robocasa fork
│   ├── robosuite_models/     # SantaCRC/robosuite_models fork
│   └── scripts/
├── ros_packages/
│   └── src/
│       ├── emdb_interfaces/  # custom msg/srv definitions (ament_cmake)
│       ├── emdb_simulator/   # simulator node, scene/world loaders (ament_python)
│       └── emdb_policy/      # RL/policy node, gym wrapper, training (ament_python)
├── docs/                     # this documentation (Sphinx)
├── setup_tfm.sh              # env setup for a headless/dev machine (Python 3.12 venv)
└── setup_desktop_casa.sh     # env setup for a desktop machine (Python 3.10 venv)
```

`emdb_interfaces` holds only message/service definitions and has no Python
code of its own; `emdb_simulator` and `emdb_policy` are plain `ament_python`
packages that import it.

## Prerequisites

- Ubuntu with **ROS 2 Humble** installed (`/opt/ros/humble`).
- A Python virtual environment for the RoboCasa/robosuite/MuJoCo/RL stack —
  the two setup scripts in the repo root assume it lives at `.tfm/` and use
  either Python 3.10 (`setup_desktop_casa.sh`) or Python 3.12
  (`setup_tfm.sh`), matching whichever machine they were written for.
- `colcon` (`sudo apt install python3-colcon-common-extensions`).

## Cloning with submodules

RoboCasa/robosuite/robosuite_models are git submodules (see
[`.gitmodules`](../../.gitmodules)):

```bash
git clone --recurse-submodules <repo-url> TFM
# or, if already cloned:
git submodule update --init --recursive
```

## Setting up the Python environment

Create the venv and install robosuite/robocasa (editable) plus the RL stack
(`gymnasium`, `stable-baselines3`, etc.) into it — see `misc/scripts` and each
submodule's own install instructions for the exact package list. Once the
venv exists, every shell that runs ROS 2 nodes from this workspace should
source one of the two provided environment scripts rather than sourcing ROS
and the venv manually, since ordering matters (venv vs. ROS `setup.bash`) and
a couple of MuJoCo/DDS environment variables are required:

```bash
source setup_tfm.sh            # headless dev machine, .tfm venv on Python 3.12
# or
source setup_desktop_casa.sh   # desktop machine, .tfm venv on Python 3.10
```

Both scripts:

- source the `.tfm` venv and `/opt/ros/humble/setup.bash` (in an order chosen
  to avoid the venv's Python shadowing ROS 2's tools),
- set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
- set `MUJOCO_GL=egl` so MuJoCo renders headlessly via EGL instead of GLX,
- source `ros_packages/install/setup.bash` if the workspace has been built,
- define a `colcon_dev` alias that builds with the venv's Python interpreter.

`setup_desktop_casa.sh` additionally defines `colcon_clean_build` (wipes
`build/ install/ log/` and rebuilds) and a `tfm_env` diagnostic alias that
prints which `ros2`/`python` are active and the installed `mujoco` version —
useful for confirming you're not accidentally running the system Python.

## Building the ROS 2 workspace

```bash
source setup_tfm.sh   # or setup_desktop_casa.sh
cd ros_packages
colcon_dev             # alias for: python -m colcon build --symlink-install
source install/setup.bash
```

## Building this documentation

The docs are Sphinx + MyST (Markdown), with autodoc for the Python API
reference. From the venv used above:

```bash
pip install -r docs/requirements.txt
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
must be installed in whichever environment runs `make html` (the same `.tfm`
venv used to run the nodes works fine).
```

## Next steps

- {doc}`architecture` — how the packages fit together.
- {doc}`howto/index` — task-oriented guides (teleop, training, replay...).
