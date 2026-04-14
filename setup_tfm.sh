#!/bin/bash

# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 3. DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. Workspace
source ros_packages/install/setup.bash

# 5. Colcon autocompletado
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# 6. VENV para RUNTIME
source  .tfm/bin/activate

export PYTHONPATH=.tfm/lib/python3.10/site-packages:$PYTHONPATH
