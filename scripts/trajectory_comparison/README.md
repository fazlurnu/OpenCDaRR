# Trajectory-level comparison (ours vs BlueSky/CDaRR_git)

Records and plots the **ownship** trajectory of a single **no-noise** conflict pair in both
OpenCDaRR and the reference (`CDaRR_git`, which runs BlueSky), for a chosen crossing angle. Used
to validate that the two produce the same deterministic maneuver, and to inspect where they differ.
Backs [`vault/observations/trajectory-level-comparison.md`](../../vault/observations/trajectory-level-comparison.md).

Scenario (fixed in the scripts): ownship north at 10.2889 m/s, `dcpa = 0`, `rpz = 50 m`,
`lookahead = 120 s`, `tlos = 180 s`, `margin = 1.05`, 250 s, ownship init hdg 0.

## Run

```bash
# 1. our side (project venv)
python scripts/trajectory_comparison/run_ours.py 2
python scripts/trajectory_comparison/run_ours.py 90

# 2. reference side — needs the `cdarr` conda env (BlueSky) and the CDaRR_git repo
CDARR_GIT=/path/to/CDaRR_git \
    /path/to/envs/cdarr/bin/python scripts/trajectory_comparison/run_reference.py 2
CDARR_GIT=/path/to/CDaRR_git \
    /path/to/envs/cdarr/bin/python scripts/trajectory_comparison/run_reference.py 90

# 3. plot -> vault/observations/img/trajectory-<dpsi>deg.png
python scripts/trajectory_comparison/plot.py 2
python scripts/trajectory_comparison/plot.py 90
```

`run_*` write intermediate `.npz` into `_out/` (git-ignored). `CDARR_GIT` defaults to
`~/Projects/CDaRR_git`. The reference script `chdir`s into that repo (its env reads
`envs/pairwise_params.json` relative to cwd).
