#!/bin/bash
# Runs the real Fruit Shop experiment end-to-end as a single SLURM job:
# launches the physics sim in the background, waits for it to be ready, then
# runs the e-MDB architecture side (commander + ltm + fruit_shop_bridge, via
# fruit_shop.launch.py) in the foreground until MainLoop finishes
# (kill_on_finish: True in mdb_experiments/fruit_shop_experiment.yaml), then
# tears the simulator down. Mirrors train_baseline_cpu.sbatch's
# background-sim/foreground-workload/trap-cleanup shape.
#
#   sbatch hpc/cesga/fruit_shop_experiment.sbatch
#
# Unlike hpc/cesga/emdb_with_architecture.sbatch, this does NOT use the
# separate e-MDB architecture container/image -- fruit_shop.launch.py
# launches emdb_policy's own bridge node, so the architecture side needs
# THIS repo's ros_packages install; only the separate emdb_develop
# workspace's *build output* (core/cognitive_node_interfaces/etc.) is
# needed, bind-mounted read-only into the simulator's own container and
# sourced alongside it -- exactly what fruit_shop_bridge.py's module
# docstring documents happening in one shared shell on bare metal.
#
# CPU only -- see train_baseline_cpu.sbatch's header for why (MuJoCo physics
# isn't GPU-accelerated here), and the architecture side has no GPU need of
# its own.
#
# Override on the command line, e.g.:
#   SEED=1 sbatch hpc/cesga/fruit_shop_experiment.sbatch
#SBATCH --job-name=fruit_shop_experiment
#SBATCH --output=%x_%A_%a.out
#SBATCH --error=%x_%A_%a.err
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --partition=medium
# ^ matches train_baseline_cpu.sbatch/emdb_simulator_cpu.sbatch -- a real
#   10000-iteration/20-trial MainLoop run is long. Drop --time and
#   --partition=short for a smoke test first (see the script's own header
#   recommendation to validate the architecture-side integration before
#   trusting a full run).

VARIANT=cpu
source "$SLURM_SUBMIT_DIR/hpc/cesga/_common.sh"

SEED="${SEED:-0}"

# The e-MDB architecture's *built workspace* (not its separate container --
# see header). Same default/checkout convention as
# emdb_with_architecture.sbatch's ARCH_HOST_WORKSPACE.
ARCH_HOST_WORKSPACE="${ARCH_HOST_WORKSPACE:-$HOME/emdb_develop}"
if [ ! -f "$ARCH_HOST_WORKSPACE/install/setup.bash" ]; then
    echo "ERROR: $ARCH_HOST_WORKSPACE/install/setup.bash not found -- build" \
         "the architecture workspace first (colcon build), or set" \
         "ARCH_HOST_WORKSPACE to point at one that's already built." >&2
    exit 1
fi

# Shared ROS 2 DDS domain so commander/ltm/bridge inside this one container
# invocation can see scene_loader running in the other. Same defense-in-depth
# offset emdb_with_architecture.sbatch uses.
export ROS_DOMAIN_ID=$(( 10 + TASK_ID ))
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Extra binds on top of _common.sh's BIND_ARGS (ros_packages + kitchen
# assets): the architecture's build output (read-only -- this job doesn't
# rebuild it), and mdb_experiments/ live (not baked into the image) so
# fruit_shop_experiment.yaml can be edited without an image rebuild, same
# rationale _common.sh already gives for bind-mounting ros_packages/.
EXTRA_BIND_ARGS=(
    --bind "$ARCH_HOST_WORKSPACE:/workspace/emdb_develop:ro"
    --bind "$HOST_WORKSPACE/mdb_experiments:/opt/emdb/mdb_experiments"
)

# Everything below runs inside the container via entrypoint.sh (ROS 2 +
# venv + colcon overlay for ros_packages + Xvfb already set up by the time
# this executes). $SIM_PID/$tries are inner-shell-only (escaped, resolved by
# the container's bash); the rest are this outer script's own variables,
# expanded here.
singularity exec "${BIND_ARGS[@]}" "${EXTRA_BIND_ARGS[@]}" "$SIF" \
    /opt/emdb/entrypoint.sh bash -c "
    set -euo pipefail

    ros2 run emdb_simulator scene_loader --ros-args \
        -p control_mode:=rl -p task:=FruitShop -p perception_mode:=mdb \
        -p layout_id:=-1 -p headless:=true \
        -p record_video:=true -p record_video_dir:='$OUT_DIR/videos' &
    SIM_PID=\$!
    trap 'kill \$SIM_PID 2>/dev/null || true' EXIT

    echo 'Waiting for /step_action to come up...'
    tries=300
    until ros2 service list 2>/dev/null | grep -q '^/step_action\$' || [ \"\$tries\" -le 0 ]; do
        sleep 1
        tries=\$((tries - 1))
    done
    if [ \"\$tries\" -le 0 ]; then
        echo 'ERROR: /step_action never came up after 300s' >&2
        exit 1
    fi

    # Architecture-side packages (core, cognitive_node_interfaces, ...) --
    # sourced on top of entrypoint.sh's ROS 2 + ros_packages install, same
    # 'both workspaces in one shell' shape fruit_shop_bridge.py documents.
    source /workspace/emdb_develop/install/setup.bash

    mkdir -p '$OUT_DIR'
    cd '$OUT_DIR'
    ros2 launch emdb_policy fruit_shop.launch.py \
        config_file:=/opt/emdb/mdb_experiments/fruit_shop_experiment.yaml \
        random_seed:='$SEED'
"

echo "Done -- results in $OUT_DIR (goodness.txt, trials.txt, pnodes_*.txt, videos/)"
