"""Functional tests for the plain-MC loss-of-separation estimator."""

from __future__ import annotations

import dataclasses

import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.estimate.montecarlo import (
    MonteCarloEstimate,
    combine_p_los,
    estimate_p_los,
)
from opencdarr.kinematics import Kinematics, MotionCommand
from opencdarr.kinematics.base import odometry_update
from opencdarr.performance import M600, Performance
from opencdarr.rng import children, root_seed_sequence
from opencdarr.scenario import pairwise
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField


def _config(seed: int = 1, n: int = 200) -> Config:
    return Config(
        seed=seed,
        n_encounters=n,
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 60.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def _noisy_config(seed: int = 1, n: int = 200) -> Config:
    """``_config`` at a GNSS noise level where LoS is common but far from certain.

    Picked deliberately mid-range (22/200 LoS): a golden anchor at 0/200 or 200/200 would agree
    with almost any broken refactor, so it would not be an anchor at all.
    """
    cfg = _config(seed, n)
    return dataclasses.replace(
        cfg, scenario=dataclasses.replace(cfg.scenario, pos_ci95=60.0, vel_ci95=6.0)
    )


def test_ipr_is_reproducible() -> None:
    cfg = _config()
    r1 = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA())
    r2 = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA())
    assert r1 == r2


def test_every_sampled_encounter_is_detected_in_this_config() -> None:
    """``detection_rate == 1`` here — but that is a property of *this config*, not of the sampler.

    Every sampled encounter is a genuine conflict by construction (``create_conflict``, and the
    ``dcpa_max <= rpz`` config check). Whether the *detector* also flags it depends on the
    lookahead: with ``tlos=60 < t_lookahead=120`` the conflict is inside the horizon from ``t =
    0``, so all of them are. Spawn outside the horizon and this legitimately drops below 1 — which
    is exactly why ``n_conflict`` is a diagnostic and not the denominator of ``p_los_run``.
    """
    cfg = _config()
    assert cfg.scenario.tlos < cfg.conflict.t_lookahead  # the precondition doing the work here
    result = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA())
    assert result.n_conflict == cfg.n_encounters
    assert result.detection_rate == 1.0
    assert result.n_encounters == cfg.n_encounters


def test_chunked_run_pools_back_to_the_serial_estimate() -> None:
    """Slicing the encounter fan-out and pooling the counts reproduces the whole run exactly.

    This is what makes a parallel MC anchor reproducible: chunks address slices of the *one* seed
    tree the serial run walks. Rooting each chunk at ``seed + i`` instead would give a different
    tree altogether — the pattern this seam exists to replace.

    Equality covers the per-encounter ``min_seps`` too, so this pins the pooled record
    element-by-element and in order — a stronger statement than the counts agreeing, which they
    would also do if the chunks were concatenated backwards.
    """
    cfg = _config(n=120)
    whole = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA())
    for jobs in (1, 3, 7):  # 7 divides 120 unevenly, so the bounds are ragged
        bounds = [(120 * i // jobs, 120 * (i + 1) // jobs) for i in range(jobs)]
        pooled = combine_p_los([
            estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA(),
                         seqs=children(root_seed_sequence(cfg.seed), lo, hi))
            for lo, hi in bounds if hi > lo
        ])
        assert pooled == whole


def test_combine_p_los_recomputes_the_ratio_from_pooled_counts() -> None:
    """The rates are ratios, so chunks pool by counts — not by averaging their per-chunk ratios."""
    a = MonteCarloEstimate(min_seps=(10.0, 60.0), n_los=1, n_conflict=2,
                           sum_k=1, sum_a=2, sum_n=4)
    b = MonteCarloEstimate(min_seps=(70.0,) * 98, n_los=0, n_conflict=98,
                           sum_k=0, sum_a=0, sum_n=196)
    pooled = combine_p_los([a, b])
    assert pooled.n_encounters == 100
    assert pooled.n_los == 1
    assert pooled.p_los_run == 0.01  # not (0.5 + 0.0) / 2 = 0.25, the per-chunk average


def test_resolution_cuts_p_los_far_below_baseline() -> None:
    cfg = _config()
    resolved = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA())
    baseline = estimate_p_los(pairwise(M600), cfg, StateBased(), None, None)
    assert resolved.p_los_run < 0.1   # good CDR prevents nearly all LoS
    assert baseline.p_los_run > 0.8   # no resolution -> nearly all conflicts become LoS
    assert resolved.p_los_run < baseline.p_los_run


def test_golden_ipr_at_midrange_noise() -> None:
    """**Golden anchor.** Exact counts for a fixed seed, config and noise level.

    Every other test here pins a *relation* (reproducible, pooled, above baseline), all of which a
    refactor can satisfy while silently changing the numbers. This pins the numbers themselves, so
    that moving the estimator onto a different runner — as Phase-v1 did, moving plain MC from the
    old pairwise runner to ``run_fleet`` at n=2 — has to prove it changed nothing. Update these
    values only alongside a deliberate, recorded modelling change.
    """
    result = estimate_p_los(
        pairwise(M600), _noisy_config(), StateBased(), MVP(1.05),
        PastCPA(bouncing_guard=False), GnssNavigation(),
    )
    assert (result.n_los, result.n_conflict) == (22, 200)
    assert result.p_los_run == 0.11  # 22/200 — was `ipr == 0.89` before the per-aircraft rename
    assert result.p_los_ac == result.p_los_run == result.mean_k  # N=2: the three coincide
    # rel=1e-8, not ==: trig calls accumulated over many steps land on a different last bit
    # depending on the platform's libm (e.g. macOS vs glibc), even with identical code and seed.
    assert result.median_min_sep == pytest.approx(126.45469556207351, rel=1e-8)


def test_min_seps_is_the_record_p_los_was_thresholded_from() -> None:
    """``n_los`` is recoverable from ``min_seps`` — the two are one measurement, not two.

    ``MonteCarloEstimate`` stores a per-encounter separation and a LoS count, which would be the
    same fact written down twice if they could ever disagree. They cannot, and this is why:
    ``FleetEnv.advance`` accumulates ``min_sep`` and ``los`` from the *same* per-step segment
    minimum (``los = state.los or cur < rpz``, ``min_sep = min(state.min_sep, cur)``), so
    ``los`` is exactly ``min_sep < rpz``. Asserting it here is what licenses reading a median, a
    quantile or ``P(min_sep <= d)`` off the record and trusting it to be the same population
    ``p_los`` describes.

    Checked at two noise levels because the equality is uninteresting when nothing breaches: at
    ``pos_ci95=0`` both sides are 0, which any broken implementation satisfies.
    """
    for cfg in (_config(), _noisy_config()):
        result = estimate_p_los(
            pairwise(M600), cfg, StateBased(), MVP(1.05),
            PastCPA(bouncing_guard=False), GnssNavigation(),
        )
        assert len(result.min_seps) == cfg.n_encounters == result.n_encounters
        assert result.n_los == sum(1 for s in result.min_seps if s < cfg.conflict.rpz)
    assert result.n_los > 0  # the noisy pass actually exercised the branch it is checking


def test_median_min_sep_separates_resolvers_p_los_cannot() -> None:
    """The reason to keep the record: ``p_los`` saturates at 0, the median keeps reporting.

    Three MVP margins on a geometry all three clear completely. ``p_los`` is 0 for every one of
    them, so on that metric alone the three are indistinguishable and a wider resolution zone looks
    free. The median says what it actually bought: 76 m of room at ``margin=1.05``, 165 m at
    ``2.0``.

    The ordering is asserted, not just the inequality, because it is *predicted* — MVP steers to
    make the trajectory tangent to a zone of radius ``margin * rpz``, so a larger margin must leave
    more separation. An inequality that merely happened to hold at this seed would be a threshold
    tuned to the run rather than a property of the resolver.
    """
    cfg = _config()
    results = [estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(m), PastCPA())
               for m in (1.05, 1.5, 2.0)]
    assert all(r.p_los_run == 0.0 for r in results)  # premise: p_los_run can't tell them apart
    medians = [r.median_min_sep for r in results]
    assert medians == sorted(medians)
    assert medians[0] > cfg.conflict.rpz  # a median inside the PZ would contradict p_los_run == 0


def test_denominator_does_not_move_with_the_resolver() -> None:
    """**The 4a regression.** ``p_los`` divides by the encounter count, so CDR cannot move it.

    Spawned outside the detection horizon (``tlos = 1.5 x t_lookahead``, the published spawn rule),
    a working resolver grows the predicted miss distance past ``rpz`` before the horizon catches
    the conflict — and ``StateBased`` then reports *no conflict* for the rest of the run. The old
    denominator was that count, so a resolver deleted its own successes from it: measured, it fell
    from 300/300 with no resolver to 178/300 with MVP.

    Now the denominator is ``n_encounters``, identical in both runs, and the detection shortfall
    shows up where it belongs — as ``detection_rate``, a diagnostic.
    """
    cfg = dataclasses.replace(
        _noisy_config(n=120),
        scenario=dataclasses.replace(_noisy_config().scenario, tlos=180.0, pos_ci95=10.0,
                                     vel_ci95=1.0),
    )
    assert cfg.scenario.tlos > cfg.conflict.t_lookahead  # spawned outside the horizon

    resolved = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA(),
                              GnssNavigation())
    unresolved = estimate_p_los(pairwise(M600), cfg, StateBased(), None, None, GnssNavigation())

    # the denominator is the same in both, whatever CDR did
    assert resolved.n_encounters == unresolved.n_encounters == cfg.n_encounters
    # ... while detection genuinely differs, which is the diagnostic's job to report
    assert unresolved.detection_rate == 1.0
    assert resolved.detection_rate < 1.0
    # and the safety comparison is still the right way round, on a denominator neither run chose
    assert resolved.p_los_run < unresolved.p_los_run



class _Ballistic(Kinematics):
    """A kinematics that ignores every command and coasts on the current track.

    Deliberately incapable of avoidance: with it fitted, no resolver can prevent a loss of
    separation, whatever it commands. That makes it a *detector* for whether ``kinematics=``
    actually reaches the encounter — see :func:`test_kinematics_reaches_the_mc_path`.
    """

    def step(
        self,
        state: AircraftState,
        command: MotionCommand,
        perf: Performance,
        dt: float,
        wind: WindField = NO_WIND,
    ) -> AircraftState:
        lat, lon = geo.forward(state.lat, state.lon, state.trk, state.gs * dt)
        return dataclasses.replace(
            state, lat=lat, lon=lon, **odometry_update(state, state.gs, dt)
        )


def test_kinematics_reaches_the_mc_path() -> None:
    """A contributed ``Kinematics`` must actually fly the MC encounters, not be silently dropped.

    Plain MC's old pairwise runner did not forward ``kinematics``, so a custom airframe
    was ignored here while IPS (which builds its own ``FleetEnv``) honoured it — the same model
    giving different answers on the two backends, with nothing in either result to show why.
    Fitting an airframe that *cannot* manoeuvre is the cheapest way to prove the wiring.

    The sharp form of the claim is an **exact** one: a resolver whose commands are thrown away by
    the airframe must give bit-for-bit what flying with *no resolver at all* gives, because both
    hold the initial cruise. So ``ballistic + MVP == multirotor + None``, while the multirotor
    actually fitted with MVP resolves every one of the same encounters. If ``kinematics=`` were
    dropped again, the first equality is what breaks.
    """
    cfg = _config()
    fitted = estimate_p_los(
        pairwise(M600, kinematics=_Ballistic()), cfg, StateBased(), MVP(1.05), PastCPA(),
    )
    unresolved = estimate_p_los(pairwise(M600), cfg, StateBased(), None, None)
    resolved = estimate_p_los(pairwise(M600), cfg, StateBased(), MVP(1.05), PastCPA())

    assert fitted == unresolved  # the airframe discarded every resolution command
    assert resolved.n_los == 0  # the same conflicts, resolved, on the default airframe
    assert fitted.p_los_run > resolved.p_los_run
