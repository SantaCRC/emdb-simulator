#!/bin/bash
# Shared entrypoint for docker/Dockerfile.cpu.jazzy and
# docker/Dockerfile.gpu.jazzy. Jazzy + Zenoh twin of docker/entrypoint.sh --
# see that file for the full rationale behind every step (ROS 2 activation,
# venv, the ros_packages staleness rebuild, the hand-rolled Xvfb start);
# comments here only call out what's different.
set -euo pipefail

REPO_ROOT=/opt/emdb

# 1. ROS 2.
for d in /opt/ros/*/; do
    if [ -f "$d/setup.bash" ]; then
        ROS_DISTRO="$(basename "$d")"
        break
    fi
done
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "entrypoint.jazzy.sh: no ROS 2 installation found under /opt/ros" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# 2. Python virtualenv.
set +u
# shellcheck disable=SC1091
source "$REPO_ROOT/.venv/bin/activate"
set -u

# 3. RMW implementation / MuJoCo rendering backend.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export MUJOCO_GL="${MUJOCO_GL:-osmesa}"

# 3b. Zenoh router. Unlike rmw_cyclonedds_cpp (automatic P2P discovery via
# UDP multicast), rmw_zenoh_cpp needs a router session for nodes to find
# each other -- without one, `ros2 node list`/topic discovery silently
# sees nothing even on the same host. Starting it here, once per
# container, mirrors how step 5 below already owns starting Xvfb: callers
# never have to remember a sidecar process themselves. Harmless to start
# even if RMW_IMPLEMENTATION was overridden away from Zenoh at `docker
# run`/`singularity exec --env` time -- an unused router just idles.
#
# Only start one if port 7447 isn't already taken: hpc/cesga jobs that run
# multiple containers on the same node share its network namespace (no
# --net isolation, see docs/source/howto/running_on_cesga.md's
# cesga-with-architecture section), so a second container's entrypoint
# would otherwise race to bind the same port and fail with "Address
# already in use" -- confirmed by hand on CESGA's login node. One router
# per node is correct and sufficient; every container's nodes just talk
# to whichever one got there first.
if ! (exec 3<>"/dev/tcp/127.0.0.1/7447") 2>/dev/null; then
    ros2 run rmw_zenoh_cpp rmw_zenohd &
    tries=50
    while [ "$tries" -gt 0 ] && ! (exec 3<>"/dev/tcp/127.0.0.1/7447") 2>/dev/null; do
        sleep 0.1
        tries=$((tries - 1))
    done
    if [ "$tries" -eq 0 ]; then
        echo "entrypoint.jazzy.sh: rmw_zenohd did not come up on 127.0.0.1:7447 after 5s" >&2
        exit 1
    fi
fi
exec 3>&- 2>/dev/null || true
# Zenoh router runs for the lifetime of whichever container started it;
# no separate teardown step needed here.

# 4. Rebuild ros_packages if it looks unbuilt or stale (same as
#    entrypoint.sh).
cd "$REPO_ROOT/ros_packages"
if [ ! -f install/setup.bash ] || [ -n "$(find src -newer install/setup.bash -type f 2>/dev/null)" ]; then
    echo "entrypoint.jazzy.sh: (re)building ros_packages workspace..." >&2
    python -m colcon build --symlink-install
fi
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
cd "$REPO_ROOT"

# 5. Headless X (same rationale as entrypoint.sh).
mkdir -p -m 1777 /tmp/.X11-unix
export DISPLAY="${DISPLAY:-:99}"
Xvfb "$DISPLAY" -screen 0 1280x1024x24 -nolisten tcp &
tries=50
while [ "$tries" -gt 0 ] && [ ! -e "/tmp/.X11-unix/X${DISPLAY#:}" ]; do
    sleep 0.1
    tries=$((tries - 1))
done
if [ ! -e "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
    echo "entrypoint.jazzy.sh: Xvfb did not come up on $DISPLAY after 5s" >&2
    exit 1
fi

if [ "$#" -eq 0 ]; then
    exec bash
else
    exec "$@"
fi
