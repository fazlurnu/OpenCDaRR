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

- [`handbook/tutorial_your_first_experiment.ipynb`](handbook/tutorial_your_first_experiment.ipynb) —
  **start here if you are new.** A plain-English tutorial that assumes no prior knowledge: one
  encounter, why one encounter proves nothing, many encounters and the interval around the answer,
  then a `run_experiment` sweep. Ends on the lesson the plots make hardest to miss — the median
  closest approach stays flat at ~125 m while P(LoS) climbs 137-fold, so the typical case says
  nothing about the failures.
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
- [`handbook/navigation.ipynb`](handbook/navigation.ipynb) — the **N** of CNS: where measurement
  accuracy lives, the four built-in error shapes and the containment guarantee they share,
  declaring an accuracy other than the truth, and receiver degradation that persists across ticks.
  Ends with your own error distribution and your own `NavEffect`.
- [`handbook/communication.ipynb`](handbook/communication.ipynb) — the **C** of CNS: broadcast
  cadence, phase and jitter; reception probability and the update interval it produces; latency
  shapes; and the two seams for extending the channel.
- [`handbook/circle_scenario.ipynb`](handbook/circle_scenario.ipynb) — `N` multirotors spawned on a
  ring, each flying to the point diametrically opposite, so every aircraft conflicts with every
  other at the same instant. Starts at four drones with the count as a parameter, sweeps it to 24,
  and adds the standard CNS stack (`GnssNavigation` + `Comm` + `LastKnown`) over twenty seeds.
  Includes a timestep-refinement check that separates one real breakdown (N = 24) from one that is
  only an artifact of `dt = 1.0` (N = 20).
- [`handbook/traffic_density.ipynb`](handbook/traffic_density.ipynb) — building *traffic* rather
  than a hand-placed encounter: the entry-bearing rule from Groot, Ellerbroek & Hoekstra (2024)
  Fig. 4 (left), where `arcsin` makes the miss distance from the centre uniform across the diameter
  so the traffic comes out homogeneous. Verifies that claim against the naive perimeter rule, keeps
  the paper's two concentric areas (spawn vs. measurement, at the same 1.35/1.62 ratio), then sweeps
  density from 5 to 25 aircraft/km² with perfect information and with the standard CNS stack
  (`MVP` + `FTR`, 10 m / 1 m/s). With perfect information the worst pair sits on the protected zone
  at every density — `FTR` resumes as soon as reverting is clear, so the fleet rides the boundary
  and traffic density barely registers. Uncertainty of this size mostly *adds* margin through
  over-avoidance, and only turns over at the top of the range.
- [`handbook/mixed_fleet.ipynb`](handbook/mixed_fleet.ipynb) — a multirotor against a small
  fixed-wing in one encounter: `Airframe` bundles each aircraft's envelope with its integrator, and
  `Methods(airframes=[...])` declares one per aircraft. Runs the same declaration under `MC` and
  `IPS` and tabulates P(LoS) for both. Shows the two guards that catch the usual mistakes — a
  mismatched envelope/integrator pair, and an aircraft spawned below its own stall speed.
- [`handbook/resolver_comparison.ipynb`](handbook/resolver_comparison.ipynb) — MVP against VO as
  the crossing gets shallow and the position fix gets worse, declared as one `run_experiment` sweep
  over sixteen conditions. Builds up from a single encounter to the full grid, and reports two
  metrics rather than one: P(LoS) for how often separation was lost, and the median achieved
  minimum separation for how much room was left when it was not.
- [`handbook/probftr_angular_grid.ipynb`](handbook/probftr_angular_grid.ipynb) — the quadrature
  inside `ProbabilisticFTR`, and whether its uniform grid of `ktheta` angles is fine enough. Plots
  where the shipped grid puts its samples against where the integrand actually has mass, then
  compares a grid centred on the mean velocity direction on accuracy and on cost. Verifies its own
  reference against a Monte Carlo estimate that uses no quadrature at all. Research notebook rather
  than a tutorial, and it reaches into module internals on purpose.
- [`handbook/rare_event_ips.ipynb`](handbook/rare_event_ips.ipynb) — running the interacting
  particle system (IPS) estimator in `opencdarr.ips` on a loss of separation too rare for plain
  Monte Carlo, parallelised across replications and particles with `opencdarr.parallel`.
- [`handbook/rare_event_ips_illustrated.ipynb`](handbook/rare_event_ips_illustrated.ipynb) — the
  same estimate with the loop opened up: the particles, the resampling step, the splitting
  genealogy, the importance function, and how survival fractions become the final probability.
- [`handbook/ring_mc_vs_ips.ipynb`](handbook/ring_mc_vs_ips.ipynb) — the same probability measured
  twice. Two, three and four drones on the ring of `circle_scenario`, flown with `MVP` +
  `ProbabilisticFTR` under a 10 m / 1 m/s fix, estimated first by counting losses over 2000
  Monte-Carlo encounters and then by the splitting estimator in `opencdarr.ips`. Counting reads zero
  for the pair and rests on one and two events for the larger fleets; IPS returns an interval in all
  three, and the intervals overlap wherever Monte Carlo has anything to say. Also shows where a
  shell ladder comes from — the Monte-Carlo run's own minimum-separation record — and why its run-in
  has to be fine through the wall MVP's margin builds at `rpz × 1.05`.

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
