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
    max_tr:
        Maximum turn rate, degrees per second.
    max_dtr2:
        Maximum rate of change of the turn rate ("turn acceleration"), degrees per second
        squared. This is what makes the instantaneous turn rate part of the aircraft state.
    v_max:
        Maximum ground speed, metres per second.
    v_min:
        Minimum ground speed, metres per second. Negative for an airframe that can fly
        backward (the M600 envelope allows it); forward-flight scenarios simply command
        positive speeds, and the dynamics clamps a command into ``[v_min, v_max]``.
    ax:
        Maximum acceleration, metres per second squared. Bounds how fast the ground speed
        may change per step (the speed analogue of ``max_dtr2`` for turn rate), so a speed
        command is approached via a ramp, not a jump.
    """

    max_tr: float
    max_dtr2: float
    v_max: float
    v_min: float
    ax: float


# DJI Matrice 600. Sources in the BlueSky fork:
#   max_tr, max_dtr2 -> bluesky/traffic/traffic.py:288-289 (the M600 turn-rate limiter)
#   v_max, v_min     -> bluesky/resources/performance/OpenAP/rotor/aircraft.json (M600 envelop)
#   ax               -> ASSUMED 5.0 m/s^2, not sourced from BlueSky. Revisit against the perf
#                       model (or a datasheet) when speed-change fidelity matters (Step 3, CR).
M600 = Performance(
    max_tr=15.0,
    max_dtr2=10.0,
    v_max=18.0,
    v_min=-18.0,
    ax=5.0,
)
