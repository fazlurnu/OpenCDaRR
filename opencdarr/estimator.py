"""Plain Monte Carlo loss-of-separation estimator.

Samples ``config.n_encounters`` independent encounters — pairwise or a whole fleet, whichever the
:data:`EncounterBuilder` builds — and aggregates
``P(LoS) = n_los/n_encounters`` (see :class:`MonteCarloEstimate` on why
the denominator is the encounter count and not the detected-conflict count). Each encounter gets
its own RNG substream spawned from the run seed (ADR 0001), so the estimate is reproducible and
order-independent — which is what lets a caller hand slices of the encounter fan-out to different
processes (``seqs=``) and pool the counts with :func:`combine_p_los` for exactly the serial answer.
Pure: no I/O.

**One environment, both estimators.** Each encounter runs through :func:`opencdarr.fleet.run_fleet`
at ``n = 2`` — the same ``build_env`` / ``advance`` / ``is_terminal`` interface the rare-event
estimator drives (:mod:`opencdarr.ips`) — rather than through a dedicated pairwise loop. The move
changed no number: the reduction was pinned bit-for-bit across the whole sampled crossing-angle
support before the pairwise runner was deleted, and the n = 2 anchors in ``tests/test_fleet.py``
carry it now. What it buys is that a
model handed to *this* estimator is the same model IPS would run. Before it, ``kinematics`` never
reached the encounter at all: plain MC silently used the default multirotor while IPS honoured
whatever the caller built into its ``FleetEnv``, so a contributed airframe appeared to work under
one backend and be ignored under the other — with nothing in either result to show it.
"""

from __future__ import annotations

import statistics
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import CommunicationModel, NavigationModel, SurveillanceModel
from opencdarr.cns.broadcast import schedule_for
from opencdarr.config import Config
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.fleet import Agent, Airframe, run_fleet
from opencdarr.kinematics import Kinematics
from opencdarr.measurement import MeasurementArea
from opencdarr.performance import Performance
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import Draw, sample_pairwise
from opencdarr.wind import NO_WIND, WindField


@dataclass(frozen=True)
class MonteCarloEstimate:
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
    n_los: int  # runs with a loss (K >= 1) — the per-run numerator
    n_conflict: int  # diagnostic: how many were *detected* as conflicts, not the denominator
    sum_k: int  # Σ K over runs — losing pairs summed (the E[K] numerator)
    sum_a: int  # Σ A over runs — aircraft-involved summed (the per-aircraft numerator)
    sum_n: int  # Σ N over runs — the aircraft that flew (the per-aircraft denominator)

    @property
    def n_encounters(self) -> int:
        """The denominator: how many encounters were run. Derived from :attr:`min_seps`."""
        return len(self.min_seps)

    @property
    def p_los_run(self) -> float:
        """P(LoS) per run — the fraction of encounters in which any pair lost separation.

        The quantity the old ``p_los`` reported, renamed: it is one of three normalisations now
        (see :attr:`p_los_ac`, :attr:`mean_k`). At N = 2 the three coincide.
        """
        return self.n_los / self.n_encounters if self.n_encounters else float("nan")

    @property
    def p_los_ac(self) -> float:
        """P(LoS) per aircraft (Blom & Bakker 2015) — the headline, and literally the aircraft
        that lost separation over the aircraft that flew, ``Σ A / Σ N``.

        Summing the denominator rather than assuming ``n_encounters · N`` costs nothing at a fixed
        fleet size and keeps the ratio meaningful when the builder draws a different number of
        aircraft per encounter. Unlike :attr:`p_los_run` it does not saturate as the fleet grows;
        at N = 2 the two are equal exactly.
        """
        return self.sum_a / self.sum_n if self.sum_n else float("nan")

    @property
    def mean_k(self) -> float:
        """E[K] — the mean losing pairs per run, ``Σ K / n_encounters``. A frequency, not a
        probability (unbounded above by 1); at N = 2 it equals :attr:`p_los_run`.
        """
        return self.sum_k / self.n_encounters if self.n_encounters else float("nan")

    @property
    def median_min_sep(self) -> float:
        """Median achieved minimum separation over all encounters [m], ``nan`` when there are none.

        Named for what it measures. It is *not* ``dcpa``, which everywhere else in this package is
        the **predicted** miss distance a detector computes from a straight-line extrapolation;
        this is the miss distance the encounter actually flew, after resolution, measured over each
        step rather than at its endpoints (:func:`~opencdarr.relative.segment_min_sep`).

        A median rather than a mean because the distribution is bounded below by zero and skewed:
        a resolver that clears almost everything but folds a few encounters onto the protected zone
        has a mean dragged by the tail, which is precisely the part :attr:`p_los` already reports.
        The two answer different questions — "how often did it fail" and "how much room did it
        leave when it did not" — and a resolver can win one while losing the other.
        """
        return statistics.median(self.min_seps) if self.min_seps else float("nan")

    @property
    def detection_rate(self) -> float:
        """Fraction of encounters the detector flagged on the true states — a *diagnostic*.

        Below 1 means some constructed conflicts were never predicted: either they were spawned
        outside the lookahead horizon (``tlos > t_lookahead``) or resolution grew the predicted
        miss distance past ``rpz`` before the horizon caught them. Informative about detection and
        about early manoeuvring; deliberately **not** the denominator of :attr:`p_los`.
        """
        return self.n_conflict / self.n_encounters if self.n_encounters else float("nan")


def combine_p_los(results: Sequence[MonteCarloEstimate]) -> MonteCarloEstimate:
    """Pool chunked runs into the result a single serial run over the same encounters would give.

    The rates are ratios, so they are recomputed from the pooled counts rather than averaged —
    averaging per-chunk ratios would weight a chunk of few encounters as heavily as one of many.
    Summing counts is exact here because every chunk is a disjoint slice of the same encounter
    fan-out (see :func:`estimate_p_los`'s ``seqs``).

    The per-encounter records concatenate in argument order, which reproduces the serial run's
    order when the chunks are passed in the order their slices were taken — the ``children(root,
    lo, hi)`` convention :func:`estimate_p_los` documents. Order does not affect
    :attr:`~MonteCarloEstimate.median_min_sep` or any other aggregate, so a caller who pools out
    of order gets the right numbers; it only affects which encounter is which if they index in.
    """
    return MonteCarloEstimate(
        min_seps=tuple(s for r in results for s in r.min_seps),
        n_los=sum(r.n_los for r in results),
        n_conflict=sum(r.n_conflict for r in results),
        sum_k=sum(r.sum_k for r in results),
        sum_a=sum(r.sum_a for r in results),
        sum_n=sum(r.sum_n for r in results),
    )


EncounterBuilder = Callable[[np.random.Generator, Config], list[Agent]]
"""Build one encounter's fleet from its own geometry stream and the run's config.

The estimator's encounter model, and the only thing that decides **N**: whatever list of
:class:`~opencdarr.fleet.Agent` comes back is flown as-is, so a builder returning two agents is a
pairwise study and one returning eight is a fleet study, through the same estimator. The IPS side
takes the same shape (``build_initial``), which is what lets one campaign drive both backends.

Draw every random choice from the generator handed in — it is this encounter's own substream, so a
builder that draws from anywhere else breaks reproducibility (ADR 0001).
"""


def pairwise(
    perf: Performance,
    *,
    kinematics: Kinematics | None = None,
    airframes: Sequence[Airframe] | None = None,
    dpsi: float | Draw | None = None,
    dcpa: float | Draw | None = None,
    side: int | Draw | None = None,
    gs_intr: float | Draw | None = None,
) -> EncounterBuilder:
    """The two-aircraft encounter builder: one ownship, one intruder crossing it.

    The standard :data:`EncounterBuilder` — :func:`~opencdarr.scenario.sample_pairwise` drawn from
    the encounter's own stream, wrapped as a pair of agents. Everything specific to a *pairwise*
    encounter lives here rather than on :func:`estimate_p_los`, which is what leaves that function
    with nothing to say about N.

    ``kinematics`` is the airframe both aircraft fly (``None`` = the fleet default
    :class:`~opencdarr.kinematics.Multirotor`, ADR 0007).

    ``airframes`` is the **mixed-fleet** spelling: one :class:`~opencdarr.fleet.Airframe` per
    aircraft (ownship first), overriding ``perf`` and ``kinematics`` entirely. Give the two
    aircraft different envelopes and the sampler must give each a speed its own airframe can fly —
    ``speed`` for the ownship, ``gs_intr`` for the intruder — or :class:`~opencdarr.fleet.Agent`
    refuses the encounter.

    ``dpsi`` / ``dcpa`` / ``side`` / ``gs_intr`` pin or re-distribute one geometry parameter (a
    constant pins it, a callable draws it, ``None`` keeps the built-in draw). ``dpsi=90.0`` is the
    single-crossing response-curve case; the rest of the encounter distribution is untouched,
    because a pinned slot still consumes its own draw.
    """
    def build(rng: np.random.Generator, config: Config) -> list[Agent]:
        own, intr = sample_pairwise(
            rng,
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
        if airframes is None:
            return [Agent(own, perf, kinematics=kinematics),
                    Agent(intr, perf, kinematics=kinematics)]
        return [af.agent(ac) for af, ac in zip(airframes, (own, intr), strict=True)]

    return build


def estimate_p_los(
    build: EncounterBuilder,
    config: Config,
    detector: ConflictDetector,
    resolver: ConflictResolver | None,
    recovery: RecoveryCriterion | None,
    navigation: NavigationModel | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    *,
    wind: WindField = NO_WIND,
    share_intent: bool = False,
    area: MeasurementArea | None = None,
    seqs: Sequence[np.random.SeedSequence] | None = None,
) -> MonteCarloEstimate:
    """Run the plain-MC estimate over ``config.n_encounters`` sampled encounters.

    ``build`` is the encounter model (:data:`EncounterBuilder`) and the only thing that sets the
    fleet size: :func:`pairwise` for the two-aircraft study, any other builder for a fleet. This
    function itself is N-agnostic — it flies whatever comes back and normalises by the aircraft
    that flew — so the same estimator serves a crossing pair and an eight-aircraft ring.

    ``wind`` and ``share_intent`` are the per-run settings the fleet environment takes; everything
    about *which* aircraft fly, and how they are equipped, belongs to ``build``. ``area`` restricts
    where a result counts (:mod:`opencdarr.measurement`); ``None`` measures everywhere.

    ``seqs`` overrides which per-encounter substreams to run, defaulting to the whole fan-out
    ``spawn(root_seed_sequence(config.seed), config.n_encounters)``. It exists so a caller can run
    *contiguous slices* of that same fan-out in parallel — ``children(root, lo, hi)``,
    :mod:`opencdarr.rng` — and pool them with :func:`combine_p_los` for a result bit-identical to
    the serial run. That is the reproducible way to chunk; offsetting the seed per chunk
    (``seed + i``) is not, because those trees can correlate and their union is not the serial
    run's tree at all.
    """
    min_seps: list[float] = []
    n_conflict = 0
    n_los = 0
    sum_k = 0
    sum_a = 0
    sum_n = 0
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
        agents = build(geom_rng, config)
        # the transmit timing, built the same way IPS builds it (:func:`schedule_for`). The phase
        # draws from ``geom_rng`` — which the builder has finished with — so switching it on
        # appends draws instead of shifting the ones already there.
        schedule = schedule_for(
            len(agents),
            config.simulation.broadcast_interval,
            geom_rng,
            jitter=config.simulation.broadcast_jitter,
            random_phase=config.simulation.broadcast_random_phase,
        )
        outcome = run_fleet(
            agents,
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
            area=area,
        )
        # counted unconditionally, and independently of each other: a lost separation is a lost
        # separation whether or not the detector ever flagged that encounter. (The old code nested
        # the LoS count inside the conflict count, so an undetected breach was silently dropped
        # from the numerator as well as the denominator.)
        min_seps.append(outcome.min_sep)
        n_conflict += int(outcome.conflict)
        n_los += int(outcome.los)
        sum_k += outcome.n_los_pairs
        sum_a += outcome.n_los_aircraft
        sum_n += len(agents)

    return MonteCarloEstimate(
        min_seps=tuple(min_seps), n_los=n_los, n_conflict=n_conflict,
        sum_k=sum_k, sum_a=sum_a, sum_n=sum_n,
    )
