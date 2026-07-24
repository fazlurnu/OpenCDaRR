# Phase 4a — `MotionCommand` (offboard currency) + the layer split

Parent: [[phase-4-plan]]. **Behaviour-preserving.** Nothing observable changes — pure re-homing,
gated by a **bit-for-bit** MVP/VO IPR regression. Finish what is already in the working tree, give the
guidance and separation layers homes, rewire the loop, then start adding capability (4b+).

## Status of the in-flight work (already in the tree)

- `opencdarr/dynamics/base.py` — `MotionCommand` exists (`target_velocity`/`target_position`/
  `target_heading`/`target_speed`/`target_altitude`/`target_vertical_speed`), with `from_track_speed`
  / `from_velocity` / `gs` / `trk` / `v_east` / `v_north`, plus the `Command` alias. ✅ seed done.
- `cr/mvp.py`, `cr/vo.py`, `cr/base.py` — return `MotionCommand(target_velocity=...)`. ✅
- `dubins.py` / `holonomic.py` — consume `MotionCommand`. ✅ (these classes are deleted in 4b/4c.)
- `tests/test_motion_command.py` — new. Extend per below.

## Checklist — DONE (2026-07-24)

- [x] **`MotionCommand`** — kept as seeded (`target_velocity` + `target_position` +
  `target_heading`/`target_speed`/`target_altitude`/`target_vertical_speed`). **Channel-resolver helper
  deferred to 4b** (no consumer in 4a; adding it now is speculative — the `state.py` rule). The
  fixed-wing field rename (`target_course`/`target_airspeed_direction`/`target_airspeed`, D1) lands in
  4c with its model.
- [x] **`opencdarr/autopilot/base.py`** — `Autopilot` ABC: `step(state, perf) -> MotionCommand`.
  (`mission` is *not* a step arg in 4a — Mission doesn't exist until 4d; a guidance autopilot will
  carry its mission as construction config. No speculative param.)
- [x] **`opencdarr/autopilot/cruise.py`** — `CruiseAutopilot(heading, speed)` returns a constant
  `MotionCommand.from_track_speed(...)`, **independent of the (noisy) state** — the frozen-nominal
  property the bit-for-bit gate rests on.
- [x] **`opencdarr/separation.py`** — `SeparationManager.step(state, perceived_traffic, nominal, memory,
  rpz, t_lookahead, detector, resolver, recovery) -> (MotionCommand, PairMemory)`. `loop._decide` ported
  verbatim; **stateless object**, `PairMemory` (now homed here) threaded in/out (ADR 0011 §5). (`dt` was
  dropped from the signature — `_decide` never used it; added back only if a consumer needs it.)
- [x] **`opencdarr/loop.py`** — rewired `run_encounter` to the layered flow: per aircraft per tick,
  `nominal = autopilot.step(self_fix, perf)` → `final, memory = separation.step(...)` → hold `final` →
  integrate `dynamics.step`. Per-aircraft **autopilots** threaded internally. **External signature
  unchanged** (shared `dynamics`/`perf`); per-aircraft *external* threading (ADR 0011 §7) **deferred to
  4e** (mixed-fleet) where it's actually needed. `PairMemory`/`_decide`/`_INACTIVE` keep byte-compatible
  shims in `loop.py` for the scripts/tests that import them directly.
- [x] **ADR 0011** — marked **accepted** (4a gate green); Update section records D1/D2/D3 and the two
  scoping deferrals.

## Gate — GREEN

- [x] `test_motion_command.py` — round-trip + fail-fast-on-missing-velocity (channel-resolver test
  moves to 4b with the helper).
- [x] `test_autopilot_cruise.py` — constant command, independent of noisy state; no evolving instance
  state.
- [x] `test_separation.py` — byte-identical to `loop._decide` on shared inputs; manager holds no state
  (memory threaded in/out).
- [x] `test_loop.py` — **bit-for-bit**: exact `min_sep` anchors (stronger than the aggregate IPR) for
  noiseless MVP/VO (`109.5894691711749` / `110.03070025405336`) and seeded-noisy MVP/VO
  (`127.15549192351872` / `127.1339570429545`). Full suite green; my files add zero ruff/mypy errors.
