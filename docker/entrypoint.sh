#!/bin/bash
# Shared entrypoint for docker/Dockerfile.cpu and docker/Dockerfile.gpu.
#
# Activates the workspace (ROS 2 + venv + colcon overlay, mirroring env.sh),
# rebuilds ros_packages if it looks stale (relevant when it's been
# bind-mounted live over the baked-in copy, e.g. on CESGA -- see
# docs/source/howto/running_on_cesga.md), and wraps whatever command was
# passed in a virtual X server so callers never have to remember that
# themselves (see the "Headless X" section below for why, and why that's
# a hand-rolled Xvfb start rather than the usual `xvfb-run` wrapper).
set -euo pipefail

REPO_ROOT=/opt/emdb

# 1. ROS 2. Baked to a single distro in these images, but auto-detect like
#    env.sh/setup.sh do rather than hardcoding "humble" here too.
for d in /opt/ros/*/; do
    if [ -f "$d/setup.bash" ]; then
        ROS_DISTRO="$(basename "$d")"
        break
    fi
done
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "entrypoint.sh: no ROS 2 installation found under /opt/ros" >&2
    exit 1
fi
# ROS 2's setup.bash (and colcon's install/setup.bash below) reference
# unset variables (e.g. AMENT_TRACE_SETUP_FILES, COLCON_TRACE), which is
# incompatible with `set -u` -- same issue setup.sh works around when
# sourcing ROS's setup.bash on bare metal.
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# 2. Python virtualenv.
set +u
# shellcheck disable=SC1091
source "$REPO_ROOT/.venv/bin/activate"
set -u

# 3. DDS implementation / MuJoCo rendering backend. Both already have a
#    Dockerfile-level ENV default (osmesa/egl per variant); ${VAR:-default}
#    here just means an operator can still override at `docker run`/
#    `singularity exec --env` time.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export MUJOCO_GL="${MUJOCO_GL:-osmesa}"

# 4. Rebuild ros_packages if it looks unbuilt or stale. This is what makes
#    "bind-mount a live ros_packages/ over the image, no rebuild step
#    needed before submitting a job" work (hpc/cesga/*.sbatch). The
#    staleness check is a cheap mtime comparison, not a full colcon
#    dependency-graph diff -- good enough to catch "edited a node, forgot
#    to rebuild", not a substitute for building manually once before
#    launching a large --array batch (rebuilding on every single container
#    invocation would be wasted work for a job that execs the same SIF
#    repeatedly).
cd "$REPO_ROOT/ros_packages"
if [ ! -f install/setup.bash ] || [ -n "$(find src -newer install/setup.bash -type f 2>/dev/null)" ]; then
    echo "entrypoint.sh: (re)building ros_packages workspace..." >&2
    python -m colcon build --symlink-install
fi
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
cd "$REPO_ROOT"

# 5. Headless X. scene_loader unconditionally imports robocasa's
#    enclosing_wall_render_wrapper -> pynput.keyboard at module level,
#    which raises ImportError with no X server at all, even in `rl` mode
#    where nothing listens for keys (see
#    docs/source/howto/recording_video.md, section 4). Every invocation is
#    wrapped in a virtual X server here so callers never have to pass
#    xvfb-run themselves -- unlike the bare-metal workflow documented there.
#
# Deliberately NOT using the `xvfb-run` wrapper script itself: it detects
# "Xvfb is ready" via a SIGUSR1 handshake between Xvfb and its parent
# shell, which hangs indefinitely when that parent shell is the
# container's PID 1 (exactly our case, since this script is exec'd all
# the way through from ENTRYPOINT) -- a well-known Docker/xvfb-run
# interaction. Starting Xvfb directly and polling for its socket file
# sidesteps that signal-delivery quirk entirely.
#
# mkdir /tmp/.X11-unix ourselves first: Xvfb refuses to create it itself
# when not running as root ("euid != 0, directory /tmp/.X11-unix will not
# be created" -- an intentional X11 safeguard against socket hijacking),
# which is exactly our case under unprivileged Singularity on CESGA (Docker
# locally runs as root, where this was never an issue). Creating it
# ourselves as the same non-root user first is fine -- the restriction is
# specifically about Xvfb creating it, not about using one that already
# exists with the right permissions.
mkdir -p -m 1777 /tmp/.X11-unix
export DISPLAY="${DISPLAY:-:99}"
Xvfb "$DISPLAY" -screen 0 1280x1024x24 -nolisten tcp &
tries=50
while [ "$tries" -gt 0 ] && [ ! -e "/tmp/.X11-unix/X${DISPLAY#:}" ]; do
    sleep 0.1
    tries=$((tries - 1))
done
if [ ! -e "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
    echo "entrypoint.sh: Xvfb did not come up on $DISPLAY after 5s" >&2
    exit 1
fi

if [ "$#" -eq 0 ]; then
    exec bash
else
    exec "$@"
fi
