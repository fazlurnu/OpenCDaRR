"""Aircraft flight-envelope limits — plain data, one instance per airframe.

Kept separate from ``dynamics.py`` on purpose: the *integrator* (how an aircraft moves)
should not be tangled with the *limits* (how fast and how tightly this particular airframe
can move). A new airframe is then a new :class:`Performance` instance, not an edit to the
step function — the airframe is a value the dynamics reads, not code it hard-codes
(``design_brief.md``: the interface is the contribution surface).

Constants are *read* from the BlueSky fork at ``~/Projects/bluesky`` and re-stated here; the
limiter logic that consumes them is re-derived in ``dynamics.py``, not imported
(``lesson-learnt.md``: don't port). See ``vault/derivations/step-dynamics-m600.md``.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Performance:
    """The horizontal flight-envelope limits of one airframe.

    Attributes
    ----------
    v_max:
        Maximum ground speed, metres per second.
    v_min:
        Minimum ground speed, metres per second. Negative for an airframe that can fly
        backward (the M600 envelope allows it); forward-flight scenarios simply command
        positive speeds, and the dynamics clamps a command into ``[v_min, v_max]``.
    ax:
        Maximum acceleration, metres per second squared. Bounds how fast the ground speed
        may change per step (the speed analogue of a roll-rate limit for turn rate), so a speed
        command is approached via a ramp, not a jump.
    yaw_rate_max:
        Maximum yaw rate, degrees per second — the limit at which
        :class:`~opencdarr.dynamics.Multirotor` converges its *nose heading* (``yaw``) toward a
        commanded ``target_yaw``, **independent** of the translation channel (ADR 0012). A
        multirotor-only limit: a fixed-wing's facing follows from its bank (``phi_max`` /
        ``roll_rate_max``) instead, so this is unused there. Defaults to ``0.0`` for airframes that
        declare no independent-yaw capability.
    phi_max:
        Maximum bank (roll) angle ``φ_max``, degrees — the fixed-wing turn authority
        (:class:`~opencdarr.dynamics.FixedWing`, ADR 0013). The coordinated-turn yaw rate is
        ``g·tan φ / V_TAS``, so a bank cap is a *speed-dependent* turn-rate/radius cap
        (``R = V²/(g·tan φ)``), unlike the multirotor's fixed ``yaw_rate_max``. Bounded further in
        a turn by the stall-in-turn limit (load factor ``n = 1/cos φ``). Defaults to ``0.0`` for
        airframes with no banking (a multirotor).
    roll_rate_max:
        Maximum roll rate ``p_max``, degrees per second — how fast ``bank`` (``φ``) may change per
        step. This is what makes ``bank`` part of the state (finite-roll, ADR 0013). Defaults to
        ``0.0``.
    """

    v_max: float
    v_min: float
    ax: float
    yaw_rate_max: float = 0.0
    phi_max: float = 0.0
    roll_rate_max: float = 0.0


# DJI Matrice 600. Sources in the BlueSky fork:
#   v_max, v_min     -> bluesky/resources/performance/OpenAP/rotor/aircraft.json (M600 envelop)
#   ax               -> 5.0 m/s^2. Set by the user (override). For reference, the value MEASURED
#                       from BlueSky's running perf model is 3.5 m/s^2: traf.perf.axmax reads 2.0
#                       right after cre() (a placeholder), but the OpenAP rotor model resets it to
#                       a constant 3.5 once the aircraft is moving (accel probe: 10.3->18 and 18->6
#                       both ramp at 3.5). 5.0 is a more aggressive multirotor acceleration.
#   yaw_rate_max -> 90.0 deg/s. NOT a BlueSky value (BlueSky's point-mass rotor model couples
#                   heading to track, so it has no independent yaw rate). A spec-level figure for
#                   the M600's yaw authority, used only by the Multirotor model's decoupled yaw
#                   channel (ADR 0012); it never affects the coupled-heading / turn-rate path.
M600 = Performance(
    v_max=18.0,
    v_min=-18.0,
    ax=5.0,
    yaw_rate_max=90.0,
)


# A small fixed-wing UAV, from the example airframe in Reyner & Liem, *Energy-Efficient Trochoidal
# Path Planning...* (Drones 2026, 10, 426; ``vault/papers/drones-wind.pdf``). NOT a BlueSky
# airframe: the fixed-wing model is re-derived from the paper (ADR 0013), analytically validated
# (ADR 0002), never ported. Sources:
#   v_max         -> V_TAS = 17 m/s is the paper's example true airspeed (§3.1.4, Figs 4/6); small
#                    fixed-wings cruise modestly (< 80 km/h, §2.1). Envelope top set to 25 m/s.
#   v_min         -> stall speed. The paper notes V_IMD ~= 1.3 V_s (Swatton, §2.2); near best
#                    endurance this puts the level stall ~= 12 m/s. ASSUMED (not in the paper).
#   phi_max       -> operational bank ~44 deg, the stall-margin-limited value the paper uses for
#                    its mission cases (§3.1.4; its 60 deg is an aggressive illustration).
#   roll_rate_max -> 60 deg/s, the "typical UAV" roll rate the paper models (§3.1.4, Fig 6/7).
#   ax            -> 2.0 m/s^2 airspeed accel. ASSUMED (fixed-wing airspeed changes slowly via
#                    thrust/TECS); not in the paper.
SMALL_FIXEDWING = Performance(
    v_max=25.0,
    v_min=12.0,
    ax=2.0,
    phi_max=44.0,
    roll_rate_max=60.0,
)
