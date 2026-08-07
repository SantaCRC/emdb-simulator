#!/bin/bash
# CPU SLURM job for the emdb-simulator, submitted from a CESGA login node:
#   sbatch hpc/cesga/emdb_simulator_cpu.sbatch
#   sbatch --array=0-4 hpc/cesga/emdb_simulator_cpu.sbatch   # 5 parallel variations
#
# See docs/source/howto/running_on_cesga.md for the full setup this depends
# on (image pull, one-time kitchen-asset priming, HOST_WORKSPACE layout).
#SBATCH --job-name=emdb_simulator_cpu
#SBATCH --output=%x_%A_%a.out
#SBATCH --error=%x_%A_%a.err
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --partition=<CESGA_CPU_PARTITION>
# ^ PLACEHOLDER: fill in with an actual FinisTerraeIII CPU partition name
#   (check `sinfo`/`scontrol show partition` on the cluster, or CESGA's
#   FT3 user guide) before submitting -- this repo doesn't have that info.

VARIANT=cpu
source "$(dirname "$0")/_common.sh"

# `entrypoint.sh` is invoked explicitly rather than relying on Singularity
# to honor the image's Docker ENTRYPOINT (`singularity exec` does not
# reliably do so, unlike `docker run`/`singularity run`, and getting this
# wrong would silently drop the xvfb-run wrap -- see docker/entrypoint.sh
# and docs/source/howto/recording_video.md, section 4, for why that's
# needed even in headless `rl` mode).
#
# layout_id/style_id/record_video_dir vary per SLURM array task via
# $TASK_ID (set in _common.sh); adjust freely for your actual experiment.
singularity exec "${BIND_ARGS[@]}" "$SIF" /opt/emdb/entrypoint.sh \
    ros2 launch emdb_simulator emdb_simulator.launch.py \
        layout_id:="$((11 + TASK_ID % 50))" \
        style_id:="$((TASK_ID % 11))" \
        record_video:=true \
        record_video_dir:="$OUT_DIR/videos"
