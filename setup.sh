#!/bin/bash
# One-time (idempotent) setup for the emdb-simulator workspace.
# Safe to re-run: every step checks whether it's already done before acting.
#
# Usage:
#   ./setup.sh            # full setup
#   ./setup.sh --docs      # also install docs/ build dependencies
#
# Override the venv location (default: <repo>/.venv) with:
#   VENV_DIR=/path/to/venv ./setup.sh
set -euo pipefail

# ---------------------------------------------------------------------------
# 0. Locate ourselves. No hardcoded machine paths anywhere in this script.
# ---------------------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${VENV_DIR:-$REPO_ROOT/.venv}"
WITH_DOCS=0
for arg in "$@"; do
    case "$arg" in
        --docs) WITH_DOCS=1 ;;
        *) echo "Unknown argument: $arg" >&2; exit 1 ;;
    esac
done

log() { echo -e "\n==> $*"; }

cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# 1. ROS 2: auto-detect the installed distro instead of hardcoding one.
# ---------------------------------------------------------------------------
log "Locating ROS 2 installation..."
if [ -z "${ROS_DISTRO:-}" ]; then
    for d in /opt/ros/*/; do
        distro="$(basename "$d")"
        if [ -f "$d/setup.bash" ]; then
            ROS_DISTRO="$distro"
            break
        fi
    done
fi
if [ -z "${ROS_DISTRO:-}" ] || [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "No ROS 2 installation found under /opt/ros. Install ROS 2 first: https://docs.ros.org/en/rolling/Installation.html" >&2
    exit 1
fi
echo "Using ROS 2 distro: $ROS_DISTRO"
# ROS 2's setup.bash references unset variables (e.g. AMENT_TRACE_SETUP_FILES),
# which is incompatible with `set -u`; disable it just for the source.
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# ---------------------------------------------------------------------------
# 2. Git submodules (misc/robosuite, misc/robocasa, misc/robosuite_models).
#    Only initialize paths that are actually empty, so this doesn't clobber
#    a manually-checked-out working copy that isn't wired up as a proper
#    submodule worktree yet.
# ---------------------------------------------------------------------------
log "Checking git submodules..."
if [ -f .gitmodules ]; then
    while read -r path; do
        [ -z "$path" ] && continue
        if [ -d "$path" ] && [ -n "$(ls -A "$path" 2>/dev/null)" ]; then
            echo "  $path already populated, skipping"
        else
            echo "  initializing $path"
            git submodule update --init --recursive -- "$path"
        fi
    done < <(git config -f .gitmodules --get-regexp path | awk '{print $2}')
fi

# ---------------------------------------------------------------------------
# 3. Python virtualenv.
# ---------------------------------------------------------------------------
log "Setting up Python virtualenv at $VENV_DIR ..."
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip wheel setuptools

log "Installing robosuite (editable)..."
pip install -e "$REPO_ROOT/misc/robosuite"

log "Installing robocasa (editable)..."
pip install -e "$REPO_ROOT/misc/robocasa"

if [ -n "$(ls -A "$REPO_ROOT/misc/robosuite_models" 2>/dev/null)" ]; then
    log "Installing robosuite_models (editable, optional)..."
    pip install -e "$REPO_ROOT/misc/robosuite_models" || \
        echo "  robosuite_models install failed (optional dependency) -- continuing"
else
    echo "  misc/robosuite_models is empty, skipping (optional dependency)"
fi

if [ "$WITH_DOCS" -eq 1 ]; then
    log "Installing docs/ build dependencies..."
    pip install -r "$REPO_ROOT/docs/requirements.txt"
fi

# ---------------------------------------------------------------------------
# 4. ROS dependencies via rosdep.
# ---------------------------------------------------------------------------
log "Resolving ROS package dependencies via rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "  rosdep has not been initialized on this machine; running 'sudo rosdep init'"
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$REPO_ROOT/ros_packages/src" --ignore-src -r -y

# ---------------------------------------------------------------------------
# 5. Build the ROS 2 workspace.
# ---------------------------------------------------------------------------
log "Building ros_packages workspace..."
(
    cd "$REPO_ROOT/ros_packages"
    colcon build --symlink-install
)

log "Setup complete."
cat <<EOF

Repo root : $REPO_ROOT
Venv      : $VENV_DIR
ROS distro: $ROS_DISTRO

For every new terminal, run:
    source $REPO_ROOT/env.sh
EOF
