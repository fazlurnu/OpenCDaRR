# Phase 4d — Mission + Autopilot ladder + Mission↔Offboard resume

Parent: [[phase-4-plan]]. Turns the cruise stand-in (4a) into real guidance for **both** airframes, and
implements the user's core flow: **fly the mission → DAA interrupts (offboard) → resume the mission
from the same leg.** Build the ladder one rung at a time; each must fly on multirotor **and** fixed-wing
(same `Mission`, different `Autopilot` implementation / natural command channel).

## DONE (2026-07-24) — [[0014-mission-position-guidance|ADR 0014]]

Built the **position-setpoint** design (Q1) with **L1** for the fixed-wing (Q2), as approved.

- [x] **`opencdarr/mission.py`** — `Mission(goto | flight_plan of Waypoint)`, frozen intent, WGS84
  lat/lon.
- [x] **Position setpoints tracked in the dynamics** (not vehicle-specific goto autopilots). A single
  vehicle-neutral **`WaypointAutopilot`** emits `MotionCommand(target_position, target_leg_start,
  target_airspeed)`; the airframe tracks it. `Multirotor` position tracker = stopping-distance law
  `√(2·ax·range)` + hover-capture deadband. `FixedWing` = **L1** (leg line → course →
  coordinated-turn core; pursuit when no leg), `vault/derivations/l1-guidance.md`.
- [x] **Waypoint sequencing** — advances `leg_index` on a capture radius (flies through intermediate
  waypoints). **Leg index is threaded `GuidanceMemory`** (ADR ABC change `step(state, memory, perf) ->
  (cmd, memory)`), clonable — never on the autopilot object.
- [x] **Loiter** — `MotionCommand.target_loiter_radius` at the final waypoint; MC hovers, FW flies a
  min-radius orbit (single capture-and-hold law).
- [x] **Mission ↔ Offboard resume** — **automatic**, no `FlightMode` machine: the autopilot re-plans
  each tick, so releasing the `SeparationManager` override resumes the mission from the persisted leg
  (FW L1 re-intercepts the leg line). Demonstrated + tested on the multirotor.

## Gate — GREEN

- [x] `test_autopilot_goto.py` — multirotor reaches a goto point and hovers (direct path, no overshoot).
- [x] `test_autopilot_waypoints.py` — 3-waypoint plan in order; **leg index clones correctly** mid-plan.
- [x] `test_fixedwing_guidance.py` — L1 nulls a 100 m cross-track offset from either side; flies a plan.
- [x] `test_autopilot_loiter.py` — MC hovers; FW holds a clean min-radius orbit (one full loop).
- [x] `test_mission_resume.py` — DAA interrupts leg 1, recovers, **resumes the same leg** to the final
  waypoint; leg index + pair memory survive a clone taken during the override.
- [x] Full suite green; my files add zero avoidable ruff/mypy errors.

**Deferred to 4e:** the fixed-wing DAA interrupt (velocity→course projection — MVP/VO emit a velocity a
fixed-wing can't fly; prototyped in `scripts/l1_reintercept_demo.py`). NPFG/wind → Phase 5.
