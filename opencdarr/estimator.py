"""Plain Monte Carlo loss-of-separation estimator.

Samples ``config.n_encounters`` independent pairwise encounters and aggregates
``P(LoS) = n_los/n_encounters`` (equivalently ``IPR = 1 - P(LoS)``; see :class:`IPRResult` on why
the denominator is the encounter count and not the detected-conflict count). Each encounter gets
its own RNG substream spawned from the run seed (ADR 0001), so the estimate is reproducible and
order-independent — which is what lets a caller hand slices of the encounter fan-out to different
processes (``seqs=``) and pool the counts with :func:`combine_ipr` for exactly the serial answer.
Pure: no I/O.

**One environment, both estimators.** Each encounter runs through :func:`opencdarr.fleet.run_fleet`
at ``n = 2`` — the same ``build_env`` / ``advance`` / ``is_terminal`` interface the rare-event
estimator drives (:mod:`opencdarr.ips`) — rather than calling :func:`opencdarr.loop.run_encounter`
directly. The two runners are equal at ``n = 2`` by construction, pinned across the whole sampled
crossing-angle support (``tests/test_fleet.py``), so this changes no number. What it buys is that a
model handed to *this* estimator is the same model IPS would run. Before it, ``kinematics`` never
reached the encounter at all: plain MC silently used the default multirotor while IPS honoured
whatever the caller built into its ``FleetEnv``, so a contributed airframe appeared to work under
one backend and be ignored under the other — with nothing in either result to show it.
"""

from __future__ import annotations

import math
import statistics
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import CommunicationModel, NavigationModel, SurveillanceModel
from opencdarr.cns.broadcast import schedule_for
from opencdarr.config import Config
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.fleet import Agent, run_fleet
from opencdarr.kinematics import Kinematics
from opencdarr.performance import Performance
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import Draw, sample_pairwise
from opencdarr.wind import NO_WIND, WindField


def wilson_interval(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """A 95% Wilson score interval for ``k`` successes in ``n`` Bernoulli trials.

    Preferred over the textbook normal approximation ``p̂ ± z√(p̂(1-p̂)/n)`` because it stays
    inside ``[0, 1]`` and keeps sensible coverage when ``p̂`` is near 0 — which is the regime every
    interesting safety number lives in. At ``k = 0`` it still returns a positive upper bound, i.e.
    "no events observed" becomes a real bound rather than the false certainty of ``(0, 0)``.

    Valid here because ``n`` is the *encounter count*, fixed by the experiment design, so this is a
    genuine binomial and not a ratio with a random denominator (see :class:`IPRResult`).
    """
    if n <= 0:
        return (float("nan"), float("nan"))
    p_hat = k / n
    denom = 1.0 + z * z / n
    centre = (p_hat + z * z / (2 * n)) / denom
    half = z / denom * math.sqrt(p_hat * (1.0 - p_hat) / n + z * z / (4 * n * n))
    return (max(0.0, centre - half), min(1.0, centre + half))


@dataclass(frozen=True)
class IPRResult:
    """Loss-of-separation counts over a set of encounters, and the rates derived from them.

    **The denominator is ``n_encounters``.** One encounter — one simulation run from one seed — is
    the unit of the experiment: its count is *chosen* by the caller rather than discovered by the
    run, draws are independent by seed construction (ADR 0001), and it is the same unit the
    rare-event estimator samples (one particle = one initial condition), which is what makes
    :mod:`opencdarr.ips` and this module estimate *literally* the same quantity.

    This replaces an earlier ``1 - n_los/n_conflict``, which divided by "encounters where the
    detector fired on the true states". That was contaminated by the very thing under test:
    :class:`~opencdarr.cd.StateBased` reports no conflict once predicted ``dcpa >= rpz``, so a
    resolver that built separation early **erased its own successes from the denominator** —
    measured at ``tlos=180 / lookahead=120``, ``n_conflict`` fell from 300/300 with no resolver to
    178/300 with MVP. The rule the fix follows: *a denominator must be fixed by the experiment
    design, never discovered from the run.* Anything whose count depends on behaviour is a
    numerator or a distribution — never a divisor.

    ``n_conflict`` is therefore kept as a **diagnostic only** (see :attr:`detection_rate`), never
    as a divisor. Because the scenario layer constructs every sampled encounter to be a genuine
    conflict (:func:`~opencdarr.scenario.create_conflict`, guarded by the ``dcpa_max <= rpz`` check
    in :mod:`opencdarr.config`), ``IPR = 1 - P(LoS)`` exactly — they are one quantity under two
    names, not two quantities.

    **The record is** :attr:`min_seps`, **one achieved minimum separation per encounter**, in
    fan-out order; the encounter count is its length rather than a second stored number, so the two
    cannot disagree about how many encounters there were. Keeping it turns the binary LoS outcome
    back into the continuous quantity it was thresholded from: ``P(LoS)`` is a single point on the
    CDF of these values, and any *other* point — ``P(min_sep <= 25)``, a median, a quantile — is
    now a read rather than another simulation. This is the scalar half of the encounter record
    designed in ``vault/run-experiment-todo.md`` item 4b; the event lists (engagements, LoS
    episodes) it also specifies are still to come, and are what would cost real storage. One float
    per encounter does not: 10 000 encounters is 80 kB.
    """

    min_seps: tuple[float, ...]  # achieved minimum separation [m], one per encounter
    n_los: int
    n_conflict: int  # diagnostic: how many were *detected* as conflicts, not the denominator

    @property
    def n_encounters(self) -> int:
        """The denominator: how many encounters were run. Derived from :attr:`min_seps`."""
        return len(self.min_seps)

    @property
    def p_los(self) -> float:
        """P(loss of separation) — the fraction of encounters in which separation was lost."""
        return self.n_los / self.n_encounters if self.n_encounters else float("nan")

    @property
    def median_min_sep(self) -> float:
        """Median achieved minimum separation over all encounters [m], ``nan`` when there are none.

        Named for what it measures. It is *not* ``dcpa``, which everywhere else in this package is
        the **predicted** miss distance a detector computes from a straight-line extrapolation;
        this is the miss distance the encounter actually flew, after resolution, measured over each
        step rather than at its endpoints (:func:`~opencdarr.fleet._segment_min_sep`).

        A median rather than a mean because the distribution is bounded below by zero and skewed:
        a resolver that clears almost everything but folds a few encounters onto the protected zone
        has a mean dragged by the tail, which is precisely the part :attr:`p_los` already reports.
        The two answer different questions — "how often did it fail" and "how much room did it
        leave when it did not" — and a resolver can win one while losing the other.
        """
        return statistics.median(self.min_seps) if self.min_seps else float("nan")

    @property
    def ipr(self) -> float:
        """The intrusion-prevention rate, ``1 - P(LoS)`` — the papers' reported metric.

        Derived, not stored, so it cannot drift out of step with the counts (the same reason
        :class:`~opencdarr.state.AircraftState` does not store velocity components).
        """
        return 1.0 - self.p_los

    @property
    def ci95(self) -> tuple[float, float]:
        """95% Wilson interval for :attr:`p_los`. Subtract from 1 (and swap) for one on the IPR."""
        return wilson_interval(self.n_los, self.n_encounters)

    @property
    def detection_rate(self) -> float:
        """Fraction of encounters the detector flagged on the true states — a *diagnostic*.

        Below 1 means some constructed conflicts were never predicted: either they were spawned
        outside the lookahead horizon (``tlos > t_lookahead``) or resolution grew the predicted
        miss distance past ``rpz`` before the horizon caught them. Informative about detection and
        about early manoeuvring; deliberately **not** the denominator of :attr:`p_los`.
        """
        return self.n_conflict / self.n_encounters if self.n_encounters else float("nan")


def combine_ipr(results: Sequence[IPRResult]) -> IPRResult:
    """Pool chunked runs into the result a single serial run over the same encounters would give.

    The rates are ratios, so they are recomputed from the pooled counts rather than averaged —
    averaging per-chunk ratios would weight a chunk of few encounters as heavily as one of many.
    Summing counts is exact here because every chunk is a disjoint slice of the same encounter
    fan-out (see :func:`estimate_ipr`'s ``seqs``).

    The per-encounter records concatenate in argument order, which reproduces the serial run's
    order when the chunks are passed in the order their slices were taken — the ``children(root,
    lo, hi)`` convention :func:`estimate_ipr` documents. Order does not affect
    :attr:`~IPRResult.median_min_sep` or any other aggregate, so a caller who pools out of order
    still gets the right numbers; it only affects which encounter is which if they index in.
    """
    return IPRResult(
        min_seps=tuple(s for r in results for s in r.min_seps),
        n_los=sum(r.n_los for r in results),
        n_conflict=sum(r.n_conflict for r in results),
    )


def estimate_ipr(
    config: Config,
    perf: Performance,
    detector: ConflictDetector,
    resolver: ConflictResolver | None,
    recovery: RecoveryCriterion | None,
    navigation: NavigationModel | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    *,
    kinematics: Kinematics | None = None,
    wind: WindField = NO_WIND,
    share_intent: bool = False,
    dpsi: float | Draw | None = None,
    dcpa: float | Draw | None = None,
    side: int | Draw | None = None,
    gs_intr: float | Draw | None = None,
    seqs: Sequence[np.random.SeedSequence] | None = None,
) -> IPRResult:
    """Run the plain-MC estimate over ``config.n_encounters`` sampled encounters.

    ``kinematics`` is the airframe both aircraft fly (``None`` = the fleet default
    :class:`~opencdarr.kinematics.Multirotor`, ADR 0007); ``wind`` and ``share_intent`` are the
    other two per-run settings the fleet environment takes. All three are keyword-only additions
    that were previously reachable through IPS but *not* through this estimator — see the module
    docstring for why that asymmetry mattered.

    ``dpsi`` / ``dcpa`` / ``side`` / ``gs_intr`` pin or re-distribute one geometry parameter of the
    sampled encounter, passed straight through to :func:`~opencdarr.scenario.sample_pairwise` (a
    constant pins it, a callable draws it, ``None`` keeps the built-in draw). ``dpsi=90.0`` is the
    single-crossing response-curve case; the rest of the encounter distribution is untouched,
    because a pinned slot still consumes its own draw.

    ``seqs`` overrides which per-encounter substreams to run, defaulting to the whole fan-out
    ``spawn(root_seed_sequence(config.seed), config.n_encounters)``. It exists so a caller can run
    *contiguous slices* of that same fan-out in parallel — ``children(root, lo, hi)``,
    :mod:`opencdarr.rng` — and pool them with :func:`combine_ipr` for a result bit-identical to the
    serial run. That is the reproducible way to chunk; offsetting the seed per chunk (``seed + i``)
    is not, because those trees can correlate and their union is not the serial run's tree at all.
    """
    min_seps: list[float] = []
    n_conflict = 0
    n_los = 0
    encounters = (
        spawn(root_seed_sequence(config.seed), config.n_encounters) if seqs is None else seqs
    )
    for seq in encounters:
        # always 4 substreams (geometry, navigation, communication, broadcast), regardless of which
        # CNS layers or transmit-timing options are enabled — the stream tree stays
        # config-invariant (ADR 0006 §6). The broadcast child was added last precisely so it could
        # be: a SeedSequence's i-th child depends only on i and its parent, so spawning four leaves
        # the first three bit-identical to the three-child tree every published number came from.
        geom_seq, nav_seq, comm_seq, bc_seq = spawn(seq, 4)
        geom_rng = generator(geom_seq)
        own, intr = sample_pairwise(
            geom_rng,
            speed=config.scenario.speed,
            dcpa_max=config.scenario.dcpa_max,
            tlos=config.scenario.tlos,
            rpz=config.conflict.rpz,
            pos_ci95=config.scenario.pos_ci95,
            vel_ci95=config.scenario.vel_ci95,
            pos_ci95_declared=config.scenario.pos_ci95_declared,
            vel_ci95_declared=config.scenario.vel_ci95_declared,
            dpsi=dpsi,
            dcpa=dcpa,
            side=side,
            gs_intr=gs_intr,
        )
        # the transmit timing, built the same way IPS builds it (:func:`schedule_for`). The phase
        # draws from ``geom_rng`` — which the geometry has finished with — so switching it on
        # appends draws instead of shifting the ones already there.
        schedule = schedule_for(
            2,
            config.simulation.broadcast_interval,
            geom_rng,
            jitter=config.simulation.broadcast_jitter,
            random_phase=config.simulation.broadcast_random_phase,
        )
        outcome = run_fleet(
            [Agent(own, perf, kinematics=kinematics), Agent(intr, perf, kinematics=kinematics)],
            rpz=config.conflict.rpz,
            t_lookahead=config.conflict.t_lookahead,
            dt=config.simulation.dt,
            detector=detector,
            resolver=resolver,
            recovery=recovery,
            navigation=navigation,
            rng=generator(nav_seq),
            communication=communication,
            surveillance=surveillance,
            comm_rng=generator(comm_seq),
            t_max=config.simulation.t_max,
            done_timeout=config.simulation.done_timeout,
            schedule=schedule,
            broadcast_rng=generator(bc_seq),
            wind=wind,
            share_intent=share_intent,
        )
        # counted unconditionally, and independently of each other: a lost separation is a lost
        # separation whether or not the detector ever flagged that encounter. (The old code nested
        # the LoS count inside the conflict count, so an undetected breach was silently dropped
        # from the numerator as well as the denominator.)
        min_seps.append(outcome.min_sep)
        n_conflict += int(outcome.conflict)
        n_los += int(outcome.los)

    return IPRResult(min_seps=tuple(min_seps), n_los=n_los, n_conflict=n_conflict)
