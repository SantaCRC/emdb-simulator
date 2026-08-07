#!/bin/bash
# GPU SLURM job for the emdb-simulator, submitted from a CESGA login node:
#   sbatch hpc/cesga/emdb_simulator_gpu.sbatch
#   sbatch --array=0-4 hpc/cesga/emdb_simulator_gpu.sbatch   # 5 parallel variations
#
# See docs/source/howto/running_on_cesga.md for the full setup this depends
# on (image pull, one-time kitchen-asset priming, HOST_WORKSPACE layout).
#SBATCH --job-name=emdb_simulator_gpu
#SBATCH --output=%x_%A_%a.out
#SBATCH --error=%x_%A_%a.err
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --partition=<CESGA_GPU_PARTITION>
#SBATCH --gres=gpu:1
# ^ PLACEHOLDERS: fill in an actual FinisTerraeIII GPU partition name and
#   confirm the --gres=gpu:... syntax (check `sinfo`/`scontrol show
#   partition`, or CESGA's FT3 user guide) before submitting -- this repo
#   doesn't have that info.

VARIANT=gpu
source "$(dirname "$0")/_common.sh"

# --nv exposes the host's NVIDIA driver/EGL libs into the container (see
# docker/Dockerfile.gpu's header comment for why those aren't baked into
# the image itself). Everything else mirrors emdb_simulator_cpu.sbatch --
# see that file's comments for the entrypoint.sh/xvfb-run rationale.
singularity exec --nv "${BIND_ARGS[@]}" "$SIF" /opt/emdb/entrypoint.sh \
    ros2 launch emdb_simulator emdb_simulator.launch.py \
        layout_id:="$((11 + TASK_ID % 50))" \
        style_id:="$((TASK_ID % 11))" \
        record_video:=true \
        record_video_dir:="$OUT_DIR/videos"
