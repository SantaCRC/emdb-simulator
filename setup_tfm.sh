#!/bin/bash

# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 3. DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. Workspace
source ~/Documents/TFM/ros_packages/install/setup.bash

# 5. Colcon autocompletado
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# 6. VENV para RUNTIME
source  ~/Documents/TFM/.tfm/bin/activate

export PYTHONPATH=~/Documents/TFM/.tfm/lib/python3.10/site-packages:$PYTHONPATH
