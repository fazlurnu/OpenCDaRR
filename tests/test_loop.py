"""Functional tests for the encounter runner.

The load-bearing check is the contrast: with resolution the encounter clears (no LoS); with
resolution disabled the same encounter loses separation — proving detect/resolve/recover and
the loop are doing real work.
"""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.dynamics import Command, Dynamics
from opencdarr.loop import run_encounter
from opencdarr.performance import M600, Performance
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0
_LOOKAHEAD = 120.0
_DT = 1.0


def _encounter() -> tuple[AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=90.0, rpz=_RPZ)
    return own, intr


def test_unresolved_encounter_loses_separation() -> None:
    """Baseline: no resolver -> the conflict becomes a loss of separation."""
    own, intr = _encounter()
    outcome = run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT, detector=StateBased()
    )
    assert outcome.conflict is True
    assert outcome.los is True
    assert outcome.min_sep < _RPZ


def test_resolved_encounter_keeps_separation() -> None:
    """With MVP + Past-CPA the same conflict is cleared with no loss of separation."""
    own, intr = _encounter()
    outcome = run_encounter(
        own,
        intr,
        perf=M600,
        rpz=_RPZ,
        t_lookahead=_LOOKAHEAD,
        dt=_DT,
        detector=StateBased(),
        resolver=MVP(margin=1.1),
        recovery=PastCPA(),
    )
    assert outcome.conflict is True
    assert outcome.los is False
    assert outcome.min_sep >= _RPZ


def _run(own: AircraftState, intr: AircraftState) -> object:
    return run_encounter(
        own,
        intr,
        perf=M600,
        rpz=_RPZ,
        t_lookahead=_LOOKAHEAD,
        dt=_DT,
        detector=StateBased(),
        resolver=MVP(margin=1.1),
        recovery=PastCPA(),
    )


def test_encounter_is_deterministic() -> None:
    """No RNG in Step 2: identical inputs -> identical outcome."""
    own, intr = _encounter()
    assert _run(own, intr) == _run(own, intr)


class _FrozenDynamics(Dynamics):
    """Test double: aircraft never move, whatever the command. Proves `dynamics=` is what
    actually drives the encounter, not a hardcoded call inside `run_encounter` (ADR 0007)."""

    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        return state


def test_dynamics_is_pluggable() -> None:
    """A custom Dynamics passed as `dynamics=` replaces the default, not just decorates it."""
    own, intr = _encounter()
    outcome = run_encounter(
        own,
        intr,
        perf=M600,
        dynamics=_FrozenDynamics(),
        rpz=_RPZ,
        t_lookahead=_LOOKAHEAD,
        dt=_DT,
        detector=StateBased(),
    )
    # frozen: the pair never converges, so no loss of separation despite no resolver -
    # with the default PointMassDynamics this same setup loses separation (see the
    # unresolved-encounter test above), so this result is only possible if our Dynamics ran.
    assert outcome.los is False
