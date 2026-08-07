# Running on CESGA (FinisTerraeIII)

This guide covers running the simulator on [CESGA](https://www.cesga.es/)'s
FinisTerraeIII HPC cluster via Docker/Singularity images and SLURM, instead
of the bare-metal `setup.sh`/`env.sh` workflow described elsewhere in these
docs. It assumes you already have an active CESGA account and can SSH in
(`ft3.cesga.es`); see CESGA's own
[FT3 user guide](https://cesga-docs.gitlab.io/ft3-user-guide/first_steps.html)
for account/connection issues (e.g. the "Corrupted MAC on input" error some
Windows SSH clients hit).

```{important}
Never run experiments on a login node -- they're for quick checks only.
Everything that actually simulates goes through SLURM (`sbatch`), as
described below.
```

## 1. Storage tiers

| Directory | Use | Speed | Space limit | Files limit | Backup | Snapshot |
|---|---|---|---|---|---|---|
| `$HOME` | Source code | Low | 10 GB | 100,000 | Yes (daily, 2 months) | Yes (7 days) |
| `$STORE` | Final results | Low | 500 GB | 300,000 | No | Yes (3 days) |
| `$LUSTRE` | Simulation execution (fast NVMe) | High | 1 TB | 200,000 | No | No |

To recover a deleted file: `cp -p .snapshot/daily.<date>/path/file $HOME/destination`.

Job-scoped scratch (all wiped when the job ends):

| Variable | Storage | Notes |
|---|---|---|
| `$LUSTRE_SCRATCH` | Shared Lustre (NVMe) | Shared between the job's nodes |
| `$LOCAL_SCRATCH` / `$TMPDIR` | Node-local disk | Private per node |
| `$TMPSHM` | Node RAM (tmpfs) | Counts against the job's memory limit; max 50% of node RAM |

## 2. Pulling the container image

```{important}
This repo publishes the **simulator's** image only
(`santacrc/emdb_simulator_cpu`/`_gpu`). The **e-MDB cognitive architecture**
(from the separate `ws_bartender` workspace) has its own, already-existing
image, `santacrc/emdb_cesga_cpu`/`_gpu` — built and published independently,
not by this repo. Running a full experiment needs both; see
{ref}`cesga-with-architecture` below. Don't confuse the two `.sif` filenames
this section produces (`emdb_simulator_cpu.sif`) with the architecture's
(`emdb_cpu.sif`, no `simulator` in the name).
```

CESGA uses Singularity, so the Docker image published by
[`.github/workflows/docker.yml`](../../../.github/workflows/docker.yml)
(`santacrc/emdb_simulator_cpu` / `santacrc/emdb_simulator_gpu` on Docker Hub)
needs to be pulled and converted to `.sif` once. Point Singularity's
cache/tmp at `$LUSTRE` rather than `$HOME` (10GB is far too small for image
pulls):

```bash
mkdir -p "$LUSTRE/singularity-cache" "$LUSTRE/singularity-tmp"
export SINGULARITY_CACHEDIR="$LUSTRE/singularity-cache"
export SINGULARITY_TMPDIR="$LUSTRE/singularity-tmp"
```

Add those two `export` lines to `.bashrc` so they persist across sessions,
or re-export them every login.

```bash
module load singularity
singularity pull --force $STORE/emdb_simulator_cpu.sif docker://santacrc/emdb_simulator_cpu   # CPU
# singularity pull --force $STORE/emdb_simulator_gpu.sif docker://santacrc/emdb_simulator_gpu # GPU
```

Verify it:

```bash
ls -lh $STORE/emdb_simulator_cpu.sif
singularity exec $STORE/emdb_simulator_cpu.sif python --version
```

Optionally symlink into `$HOME` for convenience:

```bash
ln -s $STORE/emdb_simulator_cpu.sif $HOME/
# ln -s $STORE/emdb_simulator_gpu.sif $HOME/
```

## 3. One-time kitchen-asset priming

RoboCasa's ~10GB of kitchen assets (textures, fixtures, objects) are
**not** baked into the image -- see the
[Dockerfiles'](../../../docker/Dockerfile.cpu) header comments for why. They're
downloaded once to fast `$LUSTRE` NVMe storage and bind-mounted read-only
into every job, so many parallel `--array` tasks share a single copy
instead of each holding or re-downloading their own.

```bash
module load singularity
mkdir -p "$LUSTRE/emdb_assets"
singularity exec \
  --bind "$LUSTRE/emdb_assets:/opt/emdb/misc/robocasa/robocasa/models/assets" \
  "$STORE/emdb_simulator_cpu.sif" bash -c \
  "yes | python -m robocasa.scripts.download_kitchen_assets --type all"
```

```{warning}
Run this **exactly once**, from the login node (it needs internet access,
which compute nodes don't have). Running it again concurrently from
multiple jobs would race and corrupt partial extracts. `hpc/cesga/_common.sh`
checks `$LUSTRE/emdb_assets/textures` exists before letting a job proceed,
and fails fast with a clear error if priming hasn't happened yet.
```

## 4. Getting code onto the cluster

Clone or `rsync` the repository to `$HOME/emdb_develop` (the default
`HOST_WORKSPACE` the sbatch scripts expect):

```bash
git clone --recurse-submodules <repo-url> $HOME/emdb_develop
```

```{important}
Only `ros_packages/` is meant to be live-edited here and bind-mounted into
jobs. **Never** bind-mount `misc/robosuite` or `misc/robocasa` over the
image -- those come from the baked-in editable install and mounting a
fresh/partial copy from `$HOME` would shadow it, silently breaking the
simulator (and losing the asset mount point from step 3, since the assets
live inside `misc/robocasa`'s own tree). Edit `ros_packages/src` freely;
[`docker/entrypoint.sh`](../../../docker/entrypoint.sh) rebuilds it
automatically if it looks newer than the last build, the first time a job
execs the container.
```

## 5. Submitting jobs

The SLURM job scripts live in `hpc/cesga/` in this repo
([`emdb_simulator_cpu.sbatch`](../../../hpc/cesga/emdb_simulator_cpu.sbatch),
[`emdb_simulator_gpu.sbatch`](../../../hpc/cesga/emdb_simulator_gpu.sbatch),
and a shared [`_common.sh`](../../../hpc/cesga/_common.sh) they both
source):

```bash
cd $HOME/emdb_develop
sbatch hpc/cesga/emdb_simulator_cpu.sbatch                  # single run
sbatch --array=0-4 hpc/cesga/emdb_simulator_cpu.sbatch      # 5 parallel variations
sbatch hpc/cesga/emdb_simulator_gpu.sbatch                  # GPU variant
```

Both scripts default `HOST_WORKSPACE` to `$HOME/emdb_develop` and the
`.sif` path to `$STORE/emdb_simulator_<variant>.sif`; override either on
the command line if you used different locations:

```bash
HOST_WORKSPACE=$HOME/some_other_checkout sbatch hpc/cesga/emdb_simulator_cpu.sbatch
```

```{important}
Both `.sbatch` files ship with **placeholder** `#SBATCH --partition=...`
values (and `--gres=gpu:1` for the GPU one) -- fill these in with
FinisTerraeIII's actual partition names before submitting (check `sinfo` /
`scontrol show partition`, or CESGA's FT3 user guide). This repo doesn't
have that information baked in.
```

Each SLURM array task's `$SLURM_ARRAY_TASK_ID` varies `layout_id`/
`style_id` and picks a separate output directory under
`$STORE/emdb_runs/<job-name>_<job-id>/task_<id>/` -- edit the `ros2 launch`
invocation at the bottom of either `.sbatch` file to change what actually
runs (see {doc}`run_simulator` for the full parameter table).

```{note}
`--time=48:00:00` in the header is the max job runtime; adjust to taste. A
job's virtual-X/EGL teardown on `SIGINT` can report a nonzero exit code
even after shutting down cleanly at the Python level (see
{doc}`recording_video`, section 4's note) -- don't treat that alone as
proof a run failed.
```

(cesga-with-architecture)=
## 6. Running alongside the e-MDB architecture

A full experiment needs the simulator running together with the actual
**e-MDB cognitive architecture** (from the separate `ws_bartender`
workspace — `emdb_core`, `emdb_cognitive_nodes_gii`,
`emdb_cognitive_processes_gii`, `emdb_discrete_event_simulator_gii`,
`emdb_experiments_gii`), published as its own, already-existing image,
`santacrc/emdb_cesga_cpu`/`_gpu` (unrelated to and unbuilt by this repo).

Pull it the same way as step 2 above, just with the architecture's own
image name and `.sif` filename (no `simulator` in either):

```bash
singularity pull --force $STORE/emdb_cpu.sif docker://santacrc/emdb_cesga_cpu
```

[`hpc/cesga/emdb_with_architecture.sbatch`](../../../hpc/cesga/emdb_with_architecture.sbatch)
launches both containers **on the same compute node** within one SLURM job:

```bash
sbatch hpc/cesga/emdb_with_architecture.sbatch
```

```{important}
Both containers run with no `--net`/network-isolation flags on their
`singularity exec` calls, so they share the node's network namespace —
this is what makes standard ROS 2 DDS multicast discovery work with zero
extra configuration, given a shared `ROS_DOMAIN_ID` (the script sets one).
This only works because both land on the **same** node; it deliberately
doesn't attempt to support the two containers running on different nodes,
which would need a real cross-node discovery mechanism (e.g. a CycloneDDS
static peer list) that hasn't been built or tested here.
```

```{note}
The script's architecture-side launch command defaults to `ros2 launch
experiments bartender_launch.py` (`ARCH_PACKAGE`/`ARCH_LAUNCH_FILE`
variables near the top, per `ws_bartender`'s
`emdb_experiments_gii/experiments/launch/bartender_launch.py`) — the
current standalone invocation for `santacrc/emdb_cesga_cpu`/`_gpu`.
Override both on the `sbatch` command line for a different experiment.
```

```{note}
**Scope**: this script is a connectivity smoke test, not a full experiment.
It proves both containers' ROS 2 graphs are mutually visible (writing
`ros2 node list`/`ros2 topic list` output to
`$STORE/emdb_runs/.../plumbing_check.log`) — it does not run an actual
e-MDB experiment against this simulator. That additionally needs a bridge
node translating the architecture's `cognitive_node_interfaces`
`Action`/`Policy`/`WorldReset` service calls into this repo's existing
`/step_action`/`/reset_episode` services (the role
`emdb_discrete_event_simulator_gii`'s `bartender_sim_discrete.py` plays for
the discrete-event simulator), plus new `Perception`/`Policy`/`WorldModel`
Python classes on the architecture side for the "lift" domain. Both are
explicit followups — see `mdb_experiments/lift_experiment.yaml`'s own TODO
comments for the current state of that gap.
```

## 7. Downloading results

Results land under `$STORE/emdb_runs/...` (see step 5). Pull them back to
your local machine over SSH, e.g.:

```bash
rsync -avz ft3.cesga.es:'$STORE/emdb_runs/' ./emdb_runs/
```

## Next

- {doc}`run_simulator`: the full `scene_loader` parameter table used inside
  the `ros2 launch` calls above.
- {doc}`recording_video`: background on why headless runs need `xvfb-run`
  at all (handled automatically by the image's entrypoint here, but useful
  context if you're debugging outside a container).
- {doc}`training_rl`: run `train_sb3` inside a job instead of a fixed
  `ros2 launch` command, for actual RL training on CESGA.
- {ref}`cesga-with-architecture`: running the simulator together with the
  separate e-MDB cognitive architecture container.
