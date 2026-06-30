#!/bin/bash

# 1. Venv primero (antes de ROS2)
source ~/Documents/data/TFM/.tfm/bin/activate

# 2. ROS2
source /opt/ros/humble/setup.bash

# 3. DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. MuJoCo headless con EGL (solución al error OpenGL 0x501)
export MUJOCO_GL=egl

# 5. Workspace
source ~/Documents/data/TFM/ros_packages/install/setup.bash

# 6. Colcon autocompletado
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# 7. PYTHONPATH al venv (opcional, pero ayuda si algo falla)
export PYTHONPATH=~/Documents/data/TFM/.tfm/lib/python3.12/site-packages:$PYTHONPATH

# 8. Alias de colcon
alias colcon_dev="python -m colcon build --symlink-install"
