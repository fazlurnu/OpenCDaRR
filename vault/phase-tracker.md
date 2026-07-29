# Phase tracker

Where each phase stands. Two reading notes, because the plan files and this tracker looked like
they disagreed:

- **Unchecked boxes in a "done" plan are not unfinished work.** Every phase plan opens with a
  *Not in this phase* section whose items are deliberately left unticked — they record what was
  ruled out, and most were picked up by a later phase. The one genuinely deferred deliverable is
  `tests/test_ipr_anchor.py` (phase 2), which was never written: the anchoring pass it belonged to
  was superseded by the analytical validation in ADR 0002 and the trajectory comparison in ADR
  0005. Where a parent bullet is unticked but all its children are checked, that is a formatting
  artifact, not a status.
- **Phase numbering has gaps.** 6c was removed on purpose; phase 7 was absorbed into
  `future-features/`. Both are noted below.

| # | Plan | Status |
|---|---|---|
| 0 | [[phase-0-plan]] | done |
| 1 | [[phase-1-plan]] | done |
| 2 | [[phase-2-plan]] | done |
| 3 | [[phase-3-plan]] | done |
| 4 | [[phase-4-plan]] | done |
| 5 | [[phase-5-plan]] | done |
| 6 | [[phase-6-plan]] | done |
| 7 | [[phase-7-plan]] | not started — moved to [[pilot-response-delay]] |
| 8 | [[phase-8-plan]] | done — the plan file was never updated to match |
| 9 | [[phase-9-plan]] | not started |
| 10 | [[phase-10-plan]] | superseded by [[site-plan]] |

## Detail

**0–3 — scaffolding, `step_dynamics`, the pairwise tracer bullet, CNS uncertainty.** Done. Note
that `step_dynamics` itself did not survive: phase 4 replaced it with the `opencdarr/dynamics/`
package (ADR 0010).

**4 — PX4-aligned vehicle dynamics + the Mission / Autopilot / Separation split.** Rewritten
mid-flight into the PX4-offboard framing, then completed in five rungs:
[[phase-4a-motioncommand-layer-split]] ✅, [[phase-4b-multirotor-dynamics]] ✅,
[[phase-4c-fixedwing-dynamics]] ✅, [[phase-4d-mission-autopilot]] ✅,
[[phase-4e-mixed-fleet-daa]] ✅. All 43 boxes across the five sub-plans are checked.

**5 — Wind.** Rewritten to reconcile with phase 4, then 5a ✅, 5b ✅, 5c ✅, 5d ✅ — steady,
uniform wind on both airframes (ADR 0016). The four unticked boxes are the test list, and all four
tests exist (`test_wind.py`, `test_kinematics_wind.py`, `test_state.py`, `test_loop.py`).

**6 — Multi-aircraft (n > 2).** 6a ✅ (MVP-sum / VO-union + `FleetMemory`), 6b ✅ (`run_fleet`),
the MVP `_BIAS_EPS` head-on fix ✅, 6d ✅ (scenario builders + combined demo/observation), 6e ✅
(fleet-density IPR sweep — MVP/VO degrade with N; VO appears more brittle, but that reading is
provisional: the VO implementation is ours), 6f ✅ (lossy comm/surveillance threaded through
`run_fleet` — N=2 lossy gate bit-for-bit, asymmetric-perception test, [[fleet-lossy-ipr]]; plus
broadcast phase/jitter transmit-timing knobs). **There is no 6c:** it was removed during the
phase. Coordination stays implicit, and an explicit priority model went to
[[priority-coordination]]. The two unticked boxes are again the test list, and both tests exist.

**7 — Pilot response model.** Never started. The plan file holds one line; the idea now lives in
[[pilot-response-delay]] with a target version.

**8 — Rare-event collision risk via the Blom–Bakker IPS.** **Shipped.** The plan file was left as
an early draft and is the one place in the vault that still understates what exists. What was
actually built: `opencdarr/ips.py` (fixed-effort splitting, ADR 0017) and `opencdarr/parallel.py`
(scheduling across particles and replications, ADR 0018); the look-ahead conflict coordinate as the
default importance function ([[lookahead-conflict-coordinate]], [[important-unified-coordinate]]);
correctness and efficiency gates ([[ips-gate1-correctness]], [[ips-gate2-efficiency]]) via
`scripts/ips_validate.py`; parallel scaling ([[ips-parallel-scaling]]); the splitting tree
([[ips-splitting-tree]]); the validation ladder ([[rare-event-validation-ladder]]); and the
resolved gap in [[important-ips-gap]]. Two notebooks demonstrate it —
`examples/handbook/rare_event_ips.ipynb` and `rare_event_ips_illustrated.ipynb`.

**9 — Logging and metrics.** Not started. The plan file is three lines of raw notes. The concrete
want is item 2 in [[TODO]]: total delta-velocity, extra flight time, extra distance, path
deviation, and time spent resolving. `AircraftState` already carries `flight_time` and
`distance_flown`, so the state side is partly there.

**10 — The public site.** Superseded. [[site-plan]] covers the same ground in far more detail and
is the file to read; the phase-10 notes are kept only as the original sketch.
