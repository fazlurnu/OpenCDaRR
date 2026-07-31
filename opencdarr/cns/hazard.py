"""The exponential failure law, and the constant-draw-count discipline that goes with it.

Shared by every CNS effect that models a subsystem failing and recovering — a
:class:`~opencdarr.cns.base.LinkGate` on the channel side
(:class:`~opencdarr.cns.communication.RadioHealth`), a
:class:`~opencdarr.cns.base.NavEffect` on the navigation side. Neither the law nor the draw
discipline is specific to radios, and duplicating them would also duplicate the docstring below,
which is where the constant-draw-count rule is actually written down (ADR 0021's consequences).
"""

from __future__ import annotations

import math

import numpy as np

_SECONDS_PER_HOUR = 3600.0


def hazard(rate: float, elapsed: float) -> float:
    """Probability of at least one event in ``elapsed`` **seconds** at constant ``rate`` [1/h].

    ``1 - exp(-rate * elapsed / 3600)``, the exponential (memoryless) failure law — so the
    parameter is a **rate** and the mean time to the event is ``1 / rate`` hours *whatever the
    broadcast cadence*. Quoting a probability per broadcast instead would tie that mean to the
    interval, and a cadence sweep would then be moving two things at once.

    The rate is per **hour** because that is the unit reliability is quoted in: a mean time between
    failures of 28 hours is readable where ``1e-5`` per second is not. ``elapsed`` stays in
    seconds, the simulation's own unit, and the conversion happens here. Written with ``expm1``
    because ``1 - exp(-x)`` loses every significant digit for the small ``x`` a rare failure uses.
    """
    if rate <= 0.0 or elapsed <= 0.0:
        return 0.0
    return -math.expm1(-rate * elapsed / _SECONDS_PER_HOUR)


def toggle(
    down: set[str], aid: str, p_fail: float, p_recover: float, rng: np.random.Generator
) -> None:
    """One draw for one subsystem of one aircraft: fail if it is up, recover if it is down.

    Exactly one draw either way, and it is made whatever the current health *and whatever the
    rates* — including when both are zero. That is what keeps the stream position a function of the
    roster and the tick count rather than of the failure history, so sweeping a rate moves the
    outages without moving the draws underneath them (ADR 0006 §6, and the same discipline
    ``scenario.sample_pairwise`` applies to its pinned slots).
    """
    if aid in down:
        if float(rng.random()) < p_recover:
            down.discard(aid)
    elif float(rng.random()) < p_fail:
        down.add(aid)
