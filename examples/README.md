# Examples

Runnable Jupyter notebooks for OpenCDaRR. Each one uses only the public API, so it works from a
plain install — nothing here depends on `scripts/`.

## Setup

OpenCDaRR is not published on PyPI — install it from this clone. From the repository root
(one level above this folder):

```bash
pip install -e ".[examples]"   # editable core + matplotlib + a Jupyter kernel
jupyter lab                     # or: jupyter notebook
```

`-e` (editable) means your source edits take effect without reinstalling; the `examples` extra adds
matplotlib and ipykernel, and pulls in the `parallel` extra (joblib) that the rare-event notebooks
need. Installing it this way registers `opencdarr` in your environment, so the notebooks import it
from any directory. Then open a notebook and run it top to bottom.

## Notebooks

- [`01_pairwise_conflict.ipynb`](01_pairwise_conflict.ipynb) — one multirotor-vs-fixed-wing
  crossing: run it clean, run it with GNSS noise, plot the ground tracks and separation history,
  and estimate the loss-of-separation rate with a small Monte-Carlo sweep. The runs stop once the
  aircraft reach their waypoints (`stop_within`).
- [`01_pairwise_conflict_extended.ipynb`](01_pairwise_conflict_extended.ipynb) — a guided tour of
  every simulation parameter (geometry, protected zone, lookahead, resolver/margin, recovery, CNS
  noise, wind, timestep, termination): each is changed one at a time from a baseline, with a plot
  of the effect.
- [`02_build_your_own_separation_manager.ipynb`](02_build_your_own_separation_manager.ipynb) — write
  your own conflict **detection**, **resolution**, and **recovery** by subclassing one base class
  each, and swap them into `run_fleet` one at a time (defaults `StateBased` / `MVP` / `PastCPA`);
  ends by combining resolution + recovery into a single object.
- [`03_build_your_own_performance.ipynb`](03_build_your_own_performance.ipynb) — write a new
  airframe as a `Performance` value: the horizontal envelope (speed, acceleration, turn authority)
  that `Kinematics.step` reads. Builds a heavy-lift drone, a racing quad and a large fixed-wing, and
  shows why a mismatched envelope is rejected rather than silently flown.

## Handbook notebooks

The notebooks in [`handbook/`](handbook/) are the runnable source behind the pages on
[opencdarr.github.io](https://opencdarr.github.io). They are narrower than the numbered examples
above — one module per notebook — and several of them generate the figures the site publishes.

- [`handbook/a_first_run.ipynb`](handbook/a_first_run.ipynb) — the shortest path from an empty
  notebook to a Monte-Carlo result: spawn two aircraft, run without resolution, add resolution, add
  CNS uncertainty, sweep, add a waypoint, add wind.
- [`handbook/build-your-own-distilled.ipynb`](handbook/build-your-own-distilled.ipynb) — the same
  arc, with your own kinematics, performance and resolver substituted at each step.
- [`handbook/kinematics_multirotor.ipynb`](handbook/kinematics_multirotor.ipynb) — the `Multirotor`
  holonomic point mass under the DJI M600 envelope, driven through each `MotionCommand` setpoint
  (`target_velocity`, `target_body_velocity`, `target_position`) and its edge cases.
- [`handbook/kinematics_fixedwing.ipynb`](handbook/kinematics_fixedwing.ipynb) — the `FixedWing`
  coordinated-turn point mass: flying a velocity command it cannot fly directly, turning by
  banking at a finite roll rate, and following a path.
- [`handbook/autopilot.ipynb`](handbook/autopilot.ipynb) — why one `WaypointAutopilot` serves both
  airframes: it emits a position setpoint, and each airframe interprets it through its own physics.
  Also covers L1 leg tracking.
- [`handbook/rare_event_ips.ipynb`](handbook/rare_event_ips.ipynb) — running the interacting
  particle system (IPS) estimator in `opencdarr.ips` on a loss of separation too rare for plain
  Monte Carlo, parallelised across replications and particles with `opencdarr.parallel`.
- [`handbook/rare_event_ips_illustrated.ipynb`](handbook/rare_event_ips_illustrated.ipynb) — the
  same estimate with the loop opened up: the particles, the resampling step, the splitting
  genealogy, the importance function, and how survival fractions become the final probability.

## How the figures are made

`run_fleet(..., record=True)` returns the usual `FleetOutcome` with its `frames` populated — the
full states log. Plotting is a separate tool:

- `opencdarr.viz.plot_pairwise(run, rpz=...)` — one run as ground tracks + separation vs time.
- `opencdarr.viz.plot_pairwise_montecarlo(runs, ...)` — a whole recorded sweep overlaid, one faint
  line per run per aircraft, with a `P(LoS) / IPR / CPA` header.
- `opencdarr.viz.extract_tracks(run)` — the same data as plain arrays, if you would rather draw it
  yourself.

For runs that are expensive to reproduce, `opencdarr.cache` persists a recorded run to disk (in
`.opencdarr_cache/`) keyed on its parameters + seed + a fingerprint of the source, so an unchanged
run is loaded instead of recomputed. The last section of the pairwise notebook shows the pattern.
