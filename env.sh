# Activates the emdb-simulator dev environment in the current shell.
# Source this (from bash) in every new terminal, don't execute it:
#   source env.sh
#
# Replaces the old machine-specific setup_tfm.sh / setup_desktop_casa.sh,
# which hardcoded a single computer's absolute path.
#
# Override the venv location (must match what setup.sh used) with:
#   VENV_DIR=/path/to/venv source env.sh
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${VENV_DIR:-$REPO_ROOT/.venv}"

# 1. ROS 2 first, so its PYTHONPATH additions are in place before the venv
#    (which does not have access to system site-packages) is activated.
if [ -z "${ROS_DISTRO:-}" ]; then
    for d in /opt/ros/*/; do
        if [ -f "$d/setup.bash" ]; then
            ROS_DISTRO="$(basename "$d")"
            break
        fi
    done
fi
if [ -n "${ROS_DISTRO:-}" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "env.sh: no ROS 2 installation found under /opt/ros" >&2
fi

# 2. Python virtualenv.
if [ -f "$VENV_DIR/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
else
    echo "env.sh: no venv at $VENV_DIR -- run ./setup.sh first" >&2
fi

# 3. DDS implementation (only if not already chosen by the user/shell).
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# 4. MuJoCo headless rendering via EGL, needed on machines without a
#    working GLFW/OpenGL context (works around a GL_INVALID_VALUE/0x501
#    error some GPU/driver combos hit). Unset MUJOCO_GL before sourcing
#    this file if your machine renders fine without it.
export MUJOCO_GL="${MUJOCO_GL:-egl}"

# 5. The workspace's own built packages.
if [ -f "$REPO_ROOT/ros_packages/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/ros_packages/install/setup.bash"
else
    echo "env.sh: workspace not built yet -- run ./setup.sh first" >&2
fi

# 6. Colcon autocompletion, if available on this machine.
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    # shellcheck disable=SC1091
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# 7. Convenience aliases, using this venv's python explicitly.
alias colcon_dev="$VENV_DIR/bin/python -m colcon build --symlink-install"
alias colcon_clean_build="cd $REPO_ROOT/ros_packages && rm -rf build install log && $VENV_DIR/bin/python -m colcon build --symlink-install"
alias emdb_env='echo ROS=$(which ros2) && echo PY=$(which python) && python -c "import sys; print(sys.executable)"'

unset REPO_ROOT VENV_DIR
