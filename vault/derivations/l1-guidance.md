# Derivation — L1 path-following guidance (fixed-wing lateral, 2D, no wind)

The guidance law the :class:`~opencdarr.dynamics.FixedWing` tracker uses to turn a **leg line** (the
previous → current waypoint) into a commanded ground course, so the aircraft flies *along* the line
(nulling cross-track error) rather than cutting to the endpoint. Standard L1 guidance (Park, Deyst &
How, 2004), taken for the *effect* — a reference for how a fixed-wing follows a path — not ported.

- Implemented by: [`opencdarr/dynamics/fixedwing.py`](../../opencdarr/dynamics/fixedwing.py)
  (`_guidance_course`)
- Consumed by: `FixedWing.step` (produces `target_course`, then the existing coordinated-turn core
  chases it under the bank / roll limits — [ADR 0013](../decisions/0013-fixedwing-coordinated-turn.md))
- Emitted by: [`WaypointAutopilot`](../../opencdarr/autopilot/waypoint.py) as
  `target_position` (B) + `target_leg_start` (A); guidance placement in the airframe is
  [ADR 0014](../decisions/0014-mission-position-guidance.md)

## Setup

Work in a local ENU frame centred on the aircraft (origin = ownship position `P`), so a waypoint's
`(lat, lon)` becomes `(east, north)` metres via `geo.qdrdist` (bearing/range). The leg is the segment
from `A` to `B`; `L1` is a fixed lookahead distance (`_L1_DISTANCE`, the capture-vs-tracking knob).

## 1. Foot of the perpendicular (cross-track)

Unit vector along the leg, `u = (B − A)/‖B − A‖`. The closest point on the leg line to the aircraft
(the foot) is

$$ F = A + \big((P - A)\cdot u\big)\,u \;=\; A + (-A\cdot u)\,u \qquad (P = \text{origin}) $$

and the **cross-track distance** is `d = ‖P − F‖ = ‖F‖` (the aircraft is at the origin).

## 2. The L1 reference point

Advance along the leg from the foot by the lookahead component that keeps the reference point on the
L1 circle (radius `L1`) around the aircraft:

$$ R = F + \sqrt{\max(0,\; L_1^2 - d^2)}\;\, u $$

- On track (`d = 0`): `R` is `L1` ahead along the line — steer straight down the leg.
- Off track (`0 < d < L1`): `R` is the forward intersection of the L1 circle with the line — the
  heading toward it curves the aircraft *onto* the line and then along it.
- Far off track (`d ≥ L1`): the square root is zero, `R = F` — steer straight at the line (maximum
  cross-track correction).

## 3. Commanded course

$$ \chi_{\text{cmd}} = \operatorname{atan2}(R_E,\; R_R)\ \ [\text{deg, aviation}] $$

the bearing from the aircraft (origin) to `R`. `FixedWing.step` then treats `χ_cmd` as its heading
target (course = heading at zero wind) and turns toward it under the bank / roll-rate limits.

**Bare goto (`A = None`)** or a degenerate leg ⇒ pure-pursuit: `χ_cmd = atan2(B_E, B_R)`, steering
straight at the target.

The classic lateral-acceleration form `a_cmd = 2 V² / L1 · sin η` (with `η` the angle between the
velocity and the line to `R`) is equivalent — commanding that acceleration produces exactly the turn
toward `R`; here we express it as a course setpoint so the coordinated-turn core (which already
converts a heading error to bank) does the work.

## Check (validated by `tests/test_fixedwing_guidance.py`)

- **Cross-track nulling:** from 100 m off a due-north leg, the aircraft captures the line
  (cross-track → 0) and tracks along it, from either side — a smooth capture, no oscillation.
- **Plan tracking:** a 3-waypoint plan is flown leg-by-leg in order.
- **Pursuit fallback:** with no leg start, the commanded course points straight at the target.

## Deferred

- **Wind / NPFG.** L1 is airspeed-/wind-sensitive; PX4 replaced it with NPFG (which decouples capture
  from track and is wind-robust). This pass is `w = 0`, where NPFG ≈ a well-tuned L1; the wind-robust
  form lands with [[phase-5-plan|Phase 5]] (wind), alongside the crab angle `ψ − χ`.
- **L1 on a circle** (loiter) — Phase 4d.5 uses a simpler tangent orbit; a circular-path L1 is a
  refinement.
