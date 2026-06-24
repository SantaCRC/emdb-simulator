#!/bin/bash

# 1. ROS 2 primero
source /opt/ros/humble/setup.bash

# 2. Venv
source ~/Documents/TFM/.tfm/bin/activate

# 3. DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. MuJoCo headless
export MUJOCO_GL=egl

# 5. Forzar site-packages del venv correcto
export VENV_SITE_PACKAGES="$HOME/Documents/TFM/.tfm/lib/python3.10/site-packages"
export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"

# 6. Workspace ROS 2
if [ -f ~/Documents/TFM/ros_packages/install/setup.bash ]; then
    source ~/Documents/TFM/ros_packages/install/setup.bash
fi

# 7. Colcon autocompletado
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# 8. Alias de build usando el python del venv
alias colcon_dev="$HOME/Documents/TFM/.tfm/bin/python -m colcon build --symlink-install"

# 9. Alias útil para limpiar y recompilar
alias colcon_clean_build="cd ~/Documents/TFM/ros_packages && rm -rf build install log && $HOME/Documents/TFM/.tfm/bin/python -m colcon build --symlink-install"

# 10. Diagnóstico rápido
alias tfm_env='echo ROS=$(which ros2) && echo PY=$(which python) && python -c "import sys; print(sys.executable)" && python -c "import mujoco; print(mujoco.__version__)"'
