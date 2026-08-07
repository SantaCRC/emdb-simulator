#!/bin/bash
# Runs the simulator (this repo) alongside the separate, pre-existing e-MDB
# cognitive architecture container (santacrc/emdb_cesga_cpu/_gpu, built and
# published independently -- see docs/source/howto/running_on_cesga.md's
# note on this) on the SAME compute node, so their ROS 2 graphs can discover
# each other over the node's shared network namespace.
#
# SCOPE: this is a connectivity smoke test, not a full experiment run. It
# proves both containers' ROS 2 nodes/topics are mutually visible; it does
# NOT run an actual e-MDB experiment against this simulator, which also
# needs a bridge node translating cognitive_node_interfaces Action/Policy/
# WorldReset calls into this repo's /step_action, /reset_episode services
# (the role emdb_discrete_event_simulator_gii's bartender_sim_discrete.py
# plays for the discrete-event sim), plus new Perception/Policy/WorldModel
# classes on the architecture side for the "lift" domain. Both are explicit
# followups -- see mdb_experiments/lift_experiment.yaml's own TODO comments.
#
#   sbatch hpc/cesga/emdb_with_architecture.sbatch
#SBATCH --job-name=emdb_with_architecture
#SBATCH --output=%x_%A_%a.out
#SBATCH --error=%x_%A_%a.err
#SBATCH --time=00:15:00
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --partition=short
# ^ FinisTerraeIII's default partition (6h limit, ample idle capacity per
#   `sinfo`) -- plenty for this 15-minute connectivity check. Use `medium`
#   (3-day limit) or longer instead if you raise --time for a real run.

VARIANT=cpu
source "$SLURM_SUBMIT_DIR/hpc/cesga/_common.sh"

# The architecture's own image -- pre-existing, unrelated .sif naming/pull
# workflow (docs/source/howto/running_on_cesga.md's "Pulling the image"
# section covers pulling both this and the simulator's image).
ARCH_SIF="${ARCH_SIF:-$STORE/emdb_${VARIANT}.sif}"
if [ ! -f "$ARCH_SIF" ]; then
    echo "ERROR: $ARCH_SIF not found -- pull the e-MDB architecture image" \
         "first (singularity pull \$STORE/emdb_${VARIANT}.sif" \
         "docker://santacrc/emdb_cesga_${VARIANT})." >&2
    exit 1
fi

# Shared ROS 2 DDS domain so both containers' graphs are mutually visible.
# Both singularity exec calls below run with no --net/isolation flag, so
# they share this node's network namespace -- standard ROS 2 multicast
# discovery then works exactly as if they were two ordinary processes on
# one machine, with zero extra networking config. The per-array-task offset
# is defense in depth in case multiple (simulator, architecture) pairs ever
# land on nodes that can see each other's multicast traffic.
#
# `export`ed here, not passed via `singularity exec --env` -- that flag
# takes KEY=VALUE pairs only (no Docker-style pass-through-by-name), and is
# unnecessary anyway: singularity inherits the calling shell's exported
# environment into the container by default (same mechanism entrypoint.sh
# already relies on for its own RMW_IMPLEMENTATION/MUJOCO_GL fallbacks).
export ROS_DOMAIN_ID=$(( 10 + TASK_ID ))
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
    kill "${SIM_PID:-}" "${ARCH_PID:-}" 2>/dev/null
}
trap cleanup EXIT

# --- Simulator side (this repo's image) -------------------------------
singularity exec "${BIND_ARGS[@]}" \
    "$SIF" /opt/emdb/entrypoint.sh \
    ros2 launch emdb_simulator emdb_simulator.launch.py \
        perception_mode:=mdb &
SIM_PID=$!

# --- Architecture side --------------------------------------------------
# Matches the standalone architecture sbatch workflow's own pattern: source
# ROS, use an already-built workspace (install/setup.bash present -- no
# clone/colcon-build here, unlike that script's from-scratch fallback path),
# run `ros2 launch experiments bartender_rl_launch.py`. Override
# ARCH_PACKAGE/ARCH_LAUNCH_FILE for a different experiment, and
# ARCH_HOST_WORKSPACE if your built checkout lives somewhere other than the
# default (matches that script's own HOST_WORKSPACE default).
ARCH_HOST_WORKSPACE="${ARCH_HOST_WORKSPACE:-$HOME/emdb_develop}"
if [ ! -f "$ARCH_HOST_WORKSPACE/install/setup.bash" ]; then
    echo "ERROR: $ARCH_HOST_WORKSPACE/install/setup.bash not found -- build" \
         "the architecture workspace first (colcon build), or set" \
         "ARCH_HOST_WORKSPACE to point at one that's already built." >&2
    exit 1
fi
ARCH_PACKAGE="${ARCH_PACKAGE:-experiments}"
ARCH_LAUNCH_FILE="${ARCH_LAUNCH_FILE:-bartender_rl_launch.py}"
singularity exec \
    --bind "$ARCH_HOST_WORKSPACE:/workspace/emdb_develop:ro" \
    "$ARCH_SIF" bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /workspace/emdb_develop/install/setup.bash && \
     ros2 launch $ARCH_PACKAGE $ARCH_LAUNCH_FILE" &
ARCH_PID=$!

# Let both ROS graphs come up before checking.
sleep 20

# Plumbing check: confirm the simulator's nodes/topics are visible from
# inside the architecture container, proving DDS discovery works across the
# two singularity instances. Doesn't call/exercise anything -- visibility
# only, matching this script's "just plumbing" scope (see header comment).
singularity exec "$ARCH_SIF" \
    bash -c "source /opt/ros/humble/setup.bash && \
             ros2 node list; echo ---; ros2 topic list | grep /emdb/simulator" \
    > "$OUT_DIR/plumbing_check.log" 2>&1

echo "Plumbing check written to $OUT_DIR/plumbing_check.log"
wait "$SIM_PID" "$ARCH_PID"
