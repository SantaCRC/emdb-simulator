# Shared setup for hpc/cesga/emdb_simulator_{cpu,gpu}.sbatch. This is NOT an
# sbatch file itself (no #SBATCH pragmas -- those can only be parsed from
# the top of the file that's actually passed to `sbatch`, they can't live in
# a sourced file) -- it's sourced from within one, after `VARIANT` has been
# set to "cpu" or "gpu".
#
# See docs/source/howto/running_on_cesga.md for the full walkthrough
# (image pull, one-time asset priming, HOST_WORKSPACE layout, etc.).
set -euo pipefail

if [ -z "${VARIANT:-}" ]; then
    echo "_common.sh: VARIANT must be set to 'cpu' or 'gpu' before sourcing this file" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Singularity cache/tmp on $LUSTRE (fast NVMe, large) instead of $HOME
# (10GB quota, too small for image pulls/builds). NOTE: $LUSTRE's default
# 200,000-file quota can be tight for this image's extraction (FUSE mount
# isn't available on this cluster, so `singularity exec` falls back to
# fully extracting the image on every invocation) -- see
# docs/source/howto/running_on_cesga.md, "Pulling the image", if you hit
# EDQUOT/"disk quota exceeded" errors.
# ---------------------------------------------------------------------------
export SINGULARITY_CACHEDIR="${SINGULARITY_CACHEDIR:-$LUSTRE/singularity-cache}"
export SINGULARITY_TMPDIR="${SINGULARITY_TMPDIR:-$LUSTRE/singularity-tmp}"
mkdir -p "$SINGULARITY_CACHEDIR" "$SINGULARITY_TMPDIR"

module load singularity

# ---------------------------------------------------------------------------
# The simulator's .sif image, pulled once per
# docs/source/howto/running_on_cesga.md. Named emdb_simulator_*.sif to keep
# it distinct from the separate, pre-existing e-MDB architecture image
# (emdb_cpu.sif/emdb_gpu.sif, from santacrc/emdb_cesga_cpu/_gpu -- not built
# by this repo, see hpc/cesga/emdb_with_architecture.sbatch).
# ---------------------------------------------------------------------------
SIF="${SIF:-$STORE/emdb_simulator_${VARIANT}.sif}"
if [ ! -f "$SIF" ]; then
    echo "ERROR: $SIF not found -- pull it first (see" \
         "docs/source/howto/running_on_cesga.md, 'Pulling the image')." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Kitchen assets, downloaded ONCE (not per job/array task -- concurrent
# downloads into the same directory would race/corrupt partial extracts)
# onto fast NVMe $LUSTRE storage and bind-mounted into every job. Bound as
# the WHOLE assets/ directory, not just the downloaded subdirectories -- see
# docs/source/howto/running_on_cesga.md, "One-time kitchen-asset priming"
# for why (a `.sif` is read-only outside bind mounts, and the download
# script itself needs to write into this parent, not just its leaf dirs) and
# for the priming command that seeds this directory with the image's other,
# non-downloaded baked-in files before first use.
# ---------------------------------------------------------------------------
ASSETS_DIR="${ASSETS_DIR:-$LUSTRE/emdb_assets}"
if [ ! -d "$ASSETS_DIR/textures" ]; then
    echo "ERROR: $ASSETS_DIR looks empty -- run the one-time asset-priming" \
         "step from docs/source/howto/running_on_cesga.md first." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Live ros_packages/ workspace, bind-mounted over the image's own baked-in
# copy so code changes don't require an image rebuild. Deliberately scoped
# to ros_packages/ only -- NEVER bind-mount misc/robosuite or misc/robocasa
# here, that would shadow the editable-installed package (and its assets
# mount point) with whatever partial copy exists under $HOME's 10GB quota.
#
# Defaults to $HOME/emdb_simulator_develop, NOT $HOME/emdb_develop -- the
# latter is already used by the separate e-MDB architecture's own existing
# CESGA workflow for its own checkout (see
# hpc/cesga/emdb_with_architecture.sbatch's ARCH_HOST_WORKSPACE).
# ---------------------------------------------------------------------------
HOST_WORKSPACE="${HOST_WORKSPACE:-$HOME/emdb_simulator_develop}"
if [ ! -d "$HOST_WORKSPACE/ros_packages" ]; then
    echo "ERROR: $HOST_WORKSPACE/ros_packages not found -- clone/rsync the" \
         "repo there first (see docs/source/howto/running_on_cesga.md," \
         "'Getting code onto the cluster')." >&2
    exit 1
fi

BIND_ARGS=(
    --bind "$ASSETS_DIR:/opt/emdb/misc/robocasa/robocasa/models/assets"
    --bind "$HOST_WORKSPACE/ros_packages:/opt/emdb/ros_packages"
)

# ---------------------------------------------------------------------------
# Per-array-task output directory. TASK_ID defaults to 0 for a plain
# (non-array) `sbatch` submission.
# ---------------------------------------------------------------------------
TASK_ID="${SLURM_ARRAY_TASK_ID:-0}"
OUT_DIR="$STORE/emdb_runs/${SLURM_JOB_NAME:-emdb}_${SLURM_JOB_ID:-manual}/task_${TASK_ID}"
mkdir -p "$OUT_DIR"
