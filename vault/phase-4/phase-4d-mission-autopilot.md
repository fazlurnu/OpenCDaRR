# Phase 4d — Mission + Autopilot ladder + Mission↔Offboard resume

Parent: [[phase-4-plan]]. Turns the cruise stand-in (4a) into real guidance for **both** airframes, and
implements the user's core flow: **fly the mission → DAA interrupts (offboard) → resume the mission
from the same leg.** Build the ladder one rung at a time; each must fly on multirotor **and** fixed-wing
(same `Mission`, different `Autopilot` implementation / natural command channel).

## Checklist

- [ ] **`opencdarr/mission.py`** — `Mission` value: `goto: tuple | None`, `flight_plan:
  list[Waypoint] | None`. Frozen intent; does not control the aircraft. Frame: WGS84 lat/lon (matches
  `AircraftState`), converted via `geo`/`kinematics` in the autopilot. (A local-NED convenience mirrors
  PX4 global-vs-local; keep the state truth in lat/lon.)
- [ ] **`goto` guidance** — `autopilot/goto_multirotor.py` and `autopilot/goto_fixedwing.py`:
  - multirotor: bearing/range → `MotionCommand(target_velocity = unit(bearing)·cruise_speed)`, slowing
    to hover near the point.
  - fixed-wing: an L1 / NPFG-style law → `MotionCommand(target_course=…, target_airspeed=…)`; cannot
    stop, so arrival = within capture radius, then loiter.
- [ ] **`Waypoint` + `flight_plan` sequencing** — advance to the next leg on an arrival test (capture
  radius); emit the leg's command. **The active-leg index is clonable value state** (on the aircraft
  state or a threaded autopilot-memory value), **never on the autopilot object** — same invariant as
  `PairMemory` (parent §2). This is the one place hidden state can creep back; the test guards it.
- [ ] **Loiter after arrival** — multirotor: hover (`target_velocity=0`); fixed-wing: min-radius orbit
  (`target_course` swept — it cannot stop).
- [ ] **Mission ↔ Offboard resume (D-mission).** Decide and implement the mode surfacing:
  - the `SeparationManager` overriding the nominal **is** the Mission→Offboard switch; releasing on
    recovery **is** the return to Mission. On release the autopilot resumes from the **persisted leg
    index** — so an encounter that interrupts mid-plan continues, not restarts.
  - Recommendation: keep it implicit (nominal-vs-final command selection) unless a scenario needs a
    literal `FlightMode` enum; if surfaced, the enum is clonable state, and the SeparationManager still
    owns no mode state (it reads/returns, never stores).

## Gate

- [ ] `test_autopilot_goto.py` — multirotor reaches a goto point and settles (direct path); fixed-wing
  reaches it on a feasible curved path and enters loiter.
- [ ] `test_autopilot_waypoints.py` — a 3-waypoint plan flown in order on **both** airframes; **leg index
  clones correctly** (an IPS-style clone mid-plan continues from the same leg).
- [ ] `test_mission_resume.py` — DAA interrupts a mission mid-leg, overrides, recovers, and the aircraft
  **resumes the same leg** (not leg 0); the resume index survives a clone taken during the override.
- [ ] `test_autopilot_loiter.py` — multirotor holds position (hover); fixed-wing orbits at min radius.
