"""Functional tests for the encounter runner.

The load-bearing check is the contrast: with resolution the encounter clears (no LoS); with
resolution disabled the same encounter loses separation — proving detect/resolve/recover and
the loop are doing real work.
"""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.cr import MVP, VO
from opencdarr.crr import PastCPA
from opencdarr.dynamics import Command, Dynamics
from opencdarr.loop import run_encounter
from opencdarr.performance import M600, Performance
from opencdarr.rng import generator, root_seed_sequence, spawn
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
    # with the default Multirotor this same setup loses separation (see the
    # unresolved-encounter test above), so this result is only possible if our Dynamics ran.
    assert outcome.los is False


# --- Deterministic loop regression: the layered flow (CruiseAutopilot + SeparationManager) on the
# default dynamics gives an exact, reproducible ``min_sep`` per seed — a strictly stronger check
# than the aggregate IPR. Re-anchored on ``Multirotor`` in Phase 4c (the new default after Dubins
# was deleted, ADR 0013): the noiseless gentle-maneuver anchors are unchanged from the
# coupled-heading model (the turn-rate limit never bound there), the noisy ones moved (Multirotor
# sidesteps cleanly where that turn-rate limit used to bind). The MVP/VO IPR on the *fixed-wing*
# airframe re-anchors in Phase 4e (it needs the velocity->course/airspeed projection).
_ANCHOR_NOISELESS_MVP = 109.5894691711749
_ANCHOR_NOISELESS_VO = 110.03070025405336
_ANCHOR_NOISY_MVP = 267.74240154935825
_ANCHOR_NOISY_VO = 261.9742439798254


def test_bit_for_bit_noiseless_mvp() -> None:
    """Deterministic (no-noise) MVP encounter reproduces the pre-refactor min_sep exactly."""
    own, intr = _encounter()
    out = run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=MVP(margin=1.1), recovery=PastCPA(),
    )
    assert out.min_sep == _ANCHOR_NOISELESS_MVP


def test_bit_for_bit_noiseless_vo() -> None:
    """Deterministic (no-noise) VO encounter reproduces the pre-refactor min_sep exactly."""
    own, intr = _encounter()
    out = run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=VO(margin=1.1), recovery=PastCPA(),
    )
    assert out.min_sep == _ANCHOR_NOISELESS_VO


def _noisy_encounter(resolver: MVP | VO) -> float:
    """One seeded, GPS-noisy encounter through the full self-fix path (exercises CruiseAutopilot's
    state-independence and the SeparationManager under noise). Seed 0, single substream."""
    seq = list(spawn(root_seed_sequence(0), 1))[0]
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889, pos_ci95=10.0, vel_ci95=1.0
    )
    intr = create_conflict(own, intr_id="INT", dpsi=45.0, dcpa=0.0, tlos=180.0, rpz=_RPZ, side=1)
    return run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.2,
        detector=StateBased(), resolver=resolver, recovery=PastCPA(bouncing_guard=True),
        navigation=GnssNavigation(), rng=generator(seq),
    ).min_sep


def test_bit_for_bit_noisy_mvp() -> None:
    """Seeded GPS-noisy MVP encounter reproduces the pre-refactor min_sep exactly (noisy path)."""
    assert _noisy_encounter(MVP(margin=1.05)) == _ANCHOR_NOISY_MVP


def test_bit_for_bit_noisy_vo() -> None:
    """Seeded GPS-noisy VO encounter reproduces the pre-refactor min_sep exactly (noisy path)."""
    assert _noisy_encounter(VO(margin=1.05)) == _ANCHOR_NOISY_VO
