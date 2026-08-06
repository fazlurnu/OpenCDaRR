"""The premise of the per-aircraft normalisation (ADR 0022), measured on the simulator.

``p_los_run`` is the fraction of runs with a loss (today's ``p_los``). ``p_los_ac`` is
``mean(A) / N`` (Blom & Bakker 2015). These are the raw ingredients — ``los`` and ``A`` per run
— that the estimator will aggregate; the estimator does not exist yet, so the property is checked
here at the fleet level.

At ``N = 2`` the two are one number, run for run: with a single pair ``A`` is 0 or 2, so ``A / 2``
is exactly ``1{los}``. Past two aircraft they separate — a run can lose separation (``los`` True)
while only some aircraft are involved (``A < N``). Per-run counts that run whole; per-aircraft
counts the fraction. That gap is what makes the per-run rate saturate in dense traffic while the
per-aircraft rate does not.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from opencdarr.autopilot import CruiseAutopilot
from opencdarr.cd import StateBased
from opencdarr.fleet import Agent, FleetOutcome, run_fleet
from opencdarr.performance import M600
from opencdarr.scenario import sample_pairwise

_RPZ, _DCPA_MAX, _TLOS, _SPEED = 50.0, 100.0, 60.0, 10.0
_RUN = dict(rpz=_RPZ, t_lookahead=60.0, dt=1.0, detector=StateBased(),
            resolver=None, recovery=None, t_max=200.0, done_timeout=10.0)


def _pair(seed: int, dlon: float = 0.0) -> list[Agent]:
    """One random pairwise encounter as two straight-flying agents, optionally shifted east."""
    own, intr = sample_pairwise(np.random.default_rng(seed), speed=_SPEED, dcpa_max=_DCPA_MAX,
                                tlos=_TLOS, rpz=_RPZ)
    if dlon:
        own, intr = replace(own, lon=own.lon + dlon), replace(intr, lon=intr.lon + dlon)
    return [Agent(s, M600, autopilot=CruiseAutopilot(s.trk, s.gs)) for s in (own, intr)]


def _fly(agents: list[Agent]) -> FleetOutcome:
    return run_fleet(agents, **_RUN)  # type: ignore[arg-type]


def test_n2_per_aircraft_equals_per_run_run_for_run() -> None:
    """N = 2: ``A`` is 0 or 2, so ``A / 2 == 1{los}`` every run — the two metrics are one number.

    This identity is what keeps every pairwise result bit-identical under the rewrite.
    """
    n = 60
    los = ac = 0
    for i in range(n):
        o = _fly(_pair(i))
        assert o.n_los_aircraft == (2 if o.los else 0)   # the identity, run by run
        assert o.n_los_pairs == int(o.los)               # one pair: K is 0 or 1
        los += o.los
        ac += o.n_los_aircraft
    assert ac / (2 * n) == los / n                        # p_los_ac == p_los_run, exactly


def test_n4_per_aircraft_falls_below_per_run() -> None:
    """N = 4 as two independent, far-apart pairs: each loss involves 2 of 4 aircraft, so a run's
    per-aircraft weight is ``A / 4`` while per-run counts it whole.

    The gap is measured *paired*, on the same runs, so it is robust at small n: a run where exactly
    one pair loses adds ``1 - 2/4 = 0.5`` to ``p_los_run - p_los_ac`` and 0 otherwise, so the
    difference is strictly positive and clear of sampling noise at this density.
    """
    n = 80
    los = ac = 0
    for i in range(n):
        # second pair ~34 km east: the two never interact
        o = _fly(_pair(2 * i) + _pair(2 * i + 1, dlon=0.5))
        assert o.n_los_aircraft == 2 * o.n_los_pairs     # disjoint pairs: 2 aircraft each
        assert o.los == (o.n_los_pairs >= 1)
        los += o.los
        ac += o.n_los_aircraft
    p_run, p_ac = los / n, ac / (4 * n)
    assert p_run - p_ac > 0.1   # per-run saturates above per-aircraft — the metric's point
    assert 0.3 < p_ac < 0.7     # p_ac tracks the single-pair rate, non-degenerate
