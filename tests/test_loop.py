"""Functional tests for the encounter runner.

The load-bearing check is the contrast: with resolution the encounter clears (no LoS); with
resolution disabled the same encounter loses separation — proving detect/resolve/recover and
the loop are doing real work.
"""

from __future__ import annotations

import pytest

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.cr import MVP, VO
from opencdarr.crr import PastCPA
from opencdarr.kinematics import Command, FixedWing, Kinematics, MotionCommand, Multirotor
from opencdarr.loop import run_encounter
from opencdarr.performance import M600, SMALL_FIXEDWING, Performance
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField

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


class _FrozenKinematics(Kinematics):
    """Test double: aircraft never move, whatever the command. Proves `kinematics=` is what
    actually drives the encounter, not a hardcoded call inside `run_encounter` (ADR 0007)."""

    def step(
        self,
        state: AircraftState,
        command: Command,
        perf: Performance,
        dt: float,
        wind: WindField = NO_WIND,
    ) -> AircraftState:
        return state


def test_kinematics_is_pluggable() -> None:
    """A custom Kinematics passed as `kinematics=` replaces the default, not just decorates it."""
    own, intr = _encounter()
    outcome = run_encounter(
        own,
        intr,
        perf=M600,
        kinematics=_FrozenKinematics(),
        rpz=_RPZ,
        t_lookahead=_LOOKAHEAD,
        dt=_DT,
        detector=StateBased(),
    )
    # frozen: the pair never converges, so no loss of separation despite no resolver -
    # with the default Multirotor this same setup loses separation (see the
    # unresolved-encounter test above), so this result is only possible if our Kinematics ran.
    assert outcome.los is False


# --- Deterministic loop regression: the layered flow (CruiseAutopilot + SeparationManager) on the
# default kinematics gives an exact, reproducible ``min_sep`` per seed — a strictly stronger check
# than the aggregate IPR. Re-anchored on ``Multirotor`` in Phase 4c (the new default after Dubins
# was deleted, ADR 0013): the noiseless gentle-maneuver anchors are unchanged from the
# coupled-heading model (the turn-rate limit never bound there), the noisy ones moved (Multirotor
# sidesteps cleanly where that turn-rate limit used to bind). The MVP/VO IPR on the *fixed-wing*
# airframe re-anchors in Phase 4e (it needs the velocity->course/airspeed projection).
#
# Re-anchored again for the segment-minimum measurement (``relative.segment_min_range``): min_sep
# is now the minimum over each *step* rather than at its endpoints, so every anchor moved **down**,
# by 1.8e-5 m to 0.20 m here. The direction is guaranteed, not observed — a segment minimum can
# never exceed the minimum of its own endpoints — and ``test_segment_minimum_never_exceeds_the_
# sampled_comb`` pins that as a property so a future change cannot move one of these *up*
# unnoticed. The trajectories themselves are untouched: nothing in the decision path reads min_sep.
#
# Compared with pytest.approx(rel=1e-8), not ==: trig calls compounded over many steps land on a
# different last bit depending on the platform's libm (e.g. macOS vs glibc), even with identical
# code and seed. The tolerance is tight enough to still catch a real modelling regression.
_ANCHOR_NOISELESS_MVP = 109.29398339330471
_ANCHOR_NOISELESS_VO = 109.82844921479813
_ANCHOR_NOISY_MVP = 267.74238306504367
_ANCHOR_NOISY_VO = 261.9739565914773


def test_bit_for_bit_noiseless_mvp() -> None:
    """Deterministic (no-noise) MVP encounter reproduces the pre-refactor min_sep exactly."""
    own, intr = _encounter()
    out = run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=MVP(margin=1.1), recovery=PastCPA(),
    )
    assert out.min_sep == pytest.approx(_ANCHOR_NOISELESS_MVP, rel=1e-8)


def test_bit_for_bit_noiseless_vo() -> None:
    """Deterministic (no-noise) VO encounter reproduces the pre-refactor min_sep exactly."""
    own, intr = _encounter()
    out = run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=VO(margin=1.1), recovery=PastCPA(),
    )
    assert out.min_sep == pytest.approx(_ANCHOR_NOISELESS_VO, rel=1e-8)


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
    assert _noisy_encounter(MVP(margin=1.05)) == pytest.approx(_ANCHOR_NOISY_MVP, rel=1e-8)


def test_bit_for_bit_noisy_vo() -> None:
    """Seeded GPS-noisy VO encounter reproduces the pre-refactor min_sep exactly (noisy path)."""
    assert _noisy_encounter(VO(margin=1.05)) == pytest.approx(_ANCHOR_NOISY_VO, rel=1e-8)


# --- 5a wind plumbing: passing wind=NO_WIND explicitly is byte-identical to omitting it, for both
# airframes (the second half of the 5a gate — wind is inert until a non-zero field is supplied).
def test_no_wind_step_is_identical_to_omitting_wind() -> None:
    """``step(..., wind=NO_WIND) == step(...)`` for the multirotor and the fixed-wing."""
    mr_state = AircraftState(id="M", lat=52.0, lon=4.0, trk=30.0, gs=10.0, yaw=45.0)
    mr_cmd = MotionCommand.from_track_speed(90.0, 12.0)
    assert Multirotor().step(mr_state, mr_cmd, M600, _DT) == Multirotor().step(
        mr_state, mr_cmd, M600, _DT, NO_WIND
    )
    fw_state = AircraftState(id="F", lat=52.0, lon=4.0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    fw_cmd = MotionCommand(target_course=20.0, target_airspeed=18.0)
    assert FixedWing().step(fw_state, fw_cmd, SMALL_FIXEDWING, 0.1) == FixedWing().step(
        fw_state, fw_cmd, SMALL_FIXEDWING, 0.1, NO_WIND
    )


def test_run_encounter_no_wind_matches_default() -> None:
    """Threading ``wind=NO_WIND`` through the loop reproduces the default-run outcome exactly."""
    own, intr = _encounter()
    kw = dict(
        perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT, detector=StateBased(),
        resolver=MVP(margin=1.1), recovery=PastCPA(),
    )
    assert run_encounter(own, intr, wind=NO_WIND, **kw) == run_encounter(own, intr, **kw)
