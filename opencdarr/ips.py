"""Rare-event estimator — interacting particle system (IPS), Blom et al. 2007. Phase 8 / ADR 0017.

Fixed-effort multilevel splitting over the fleet estimator interface (``advance / level /
is_terminal``, :mod:`opencdarr.fleet`). Where plain Monte Carlo (:func:`opencdarr.estimator.
estimate_ipr`) starves in the rare regime — a single 10⁴-encounter run can read *zero* events
([[rare-event-validation-ladder]]) — IPS concentrates effort on the trajectories heading toward the
rare set and returns the small probability with a usable confidence interval.

**The method (ADR 0017).** Nest the rare event in shrinking shells on the **running-minimum**
separation ``FleetState.min_sep`` (monotone, so crossings are one-way): a decreasing sequence
``d_1 … d_m``, with ``d_m`` the rare boundary (``rpz`` for loss of separation). Keep a fixed ``N``
particles; at each shell, evolve every particle with ``env.advance`` until it either crosses
(reaches ``min_sep <= d_k``, a *survivor*) or goes ``is_terminal`` first (*dropped*), then resample
the survivors with replacement back to ``N``. The estimate is the product of survival fractions
``prod_k (S_k / N)`` — no per-particle weights. A clone is a shared (immutable)
:class:`~opencdarr.fleet.FleetState` plus a **freshly spawned** per-particle-per-level
:class:`~opencdarr.fleet.FleetStreams`, so two clones of one survivor diverge (ADR 0001).

**Terminal is unchanged** — ``env.is_terminal`` (cleared / ``t_max``) is the drop condition; no
absorbing past-CPA kill (ADR 0017 §3). **The initial cloud samples geometry; splitting acts on the
forward CNS noise** (§4), so IPS estimates the same probability MC does — which is what makes the
validation meaningful. Scenario-agnostic: the caller supplies ``build_initial`` (one sampled
particle from a seed) and the shell sequence; everything else rides the interface.

Confidence interval by **independent replications** (§5): within one run the particles interact
(shared ancestors), so a valid CI comes from ``R`` independent IPS runs, reported in log space.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace

import numpy as np

from opencdarr.fleet import CnsStreams, FleetEnv, FleetState, FleetStreams, los_counts
from opencdarr.rng import children, generator, root_seed_sequence, spawn

# A particle's initial-state factory: sample one geometry from a seed → its env + world state.
# (The forward-evolution RNG is NOT drawn here; it is spawned per level inside :func:`ips_once`.)
BuildInitial = Callable[[np.random.SeedSequence], "Particle"]


@dataclass(frozen=True)
class Particle:
    """One IPS particle: its fixed rules (``env``) and its current world (``state``).

    ``env`` is per-particle because a homogeneous fleet still bakes each aircraft's cruise into its
    autopilot (:func:`opencdarr.fleet.build_env`), so particles from different sampled geometries
    carry different envs — but they share their heavy references (detector, resolver, …). ``state``
    is deeply immutable, so cloning a survivor is sharing this value, not copying it.
    """

    env: FleetEnv
    state: FleetState


@dataclass(frozen=True)
class IPSResult:
    """One IPS replication: the estimate and the per-level survival fractions behind it."""

    prob: float  # P̂ = Π_k survival_k (0.0 if a level collapsed)
    levels: tuple[float, ...]  # the shell distances d_1 … d_m [m]
    survival: tuple[float, ...]  # S_k / N per crossed level
    n_particles: int
    collapsed_at: int | None  # index of the level where S_k = 0, or None
    # --- tail measurements, only when ips_once(..., tail=True) --------------------------------
    tail_los_pairs: tuple[int, ...] = ()  # K per survivor, flown on to termination
    tail_los_aircraft: tuple[int, ...] = ()  # A per survivor
    n_aircraft: int = 0  # N, the fleet size
    n_lineages: int = 0  # distinct survivors behind the cloud — the effective sample size

    @property
    def mean_los_pairs(self) -> float:
        """``E[K | reached the rare set]``, from the continued survivors."""
        return (sum(self.tail_los_pairs) / len(self.tail_los_pairs)
                if self.tail_los_pairs else float("nan"))

    @property
    def expected_los_pairs(self) -> float:
        """``E[K] = P̂ · E[K | rare set]`` — the mean number of losing pairs per encounter."""
        return self.prob * self.mean_los_pairs

    @property
    def p_ac(self) -> float:
        """``P_ac = P̂ · E[A | rare set] / N`` — the per-aircraft probability (Blom & Bakker).

        The rare set is the same event for all three metrics, because a run has a loss of
        separation exactly when ``min_sep`` crosses the last shell. So the splitting estimate of
        ``P_run`` carries over, and the survivors supply the conditional means. Precision here is
        set by :attr:`n_lineages`, **not** by ``n_particles`` — resampling fills the cloud with
        clones of a few survivors, and a clone adds no independent information about ``A``.
        """
        if not self.tail_los_aircraft or not self.n_aircraft:
            return float("nan")
        mean_a = sum(self.tail_los_aircraft) / len(self.tail_los_aircraft)
        return self.prob * mean_a / self.n_aircraft


@dataclass(frozen=True)
class RareEventEstimate:
    """The replicated estimate: mean probability with a log-space CI from independent IPS runs."""

    prob: float  # mean of the per-replication P̂ (the unbiased point estimate)
    ci: tuple[float, float]  # 95% CI (log-space when all reps > 0, else the min/max span)
    reps: tuple[IPSResult, ...]  # every replication, for inspection
    n_collapsed: int  # replications that hit an empty level (P̂ = 0)


def _streams(seq: np.random.SeedSequence) -> FleetStreams:
    """A particle's forward-evolution RNG: nav + comm + broadcast substreams from one seed.

    Always three substreams regardless of which CNS layers are active, so the stream tree stays
    config-invariant (ADR 0006 §6); unused generators are simply never drawn from.
    """
    nav_seq, comm_seq, bc_seq = children(seq, 0, 3)
    return FleetStreams(
        cns=CnsStreams(nav=generator(nav_seq), comm=generator(comm_seq)),
        broadcast=generator(bc_seq),
    )


def _evolve_to_shell(particle: Particle, target: float, streams: FleetStreams) -> FleetState:
    """Advance until the running-min crosses ``target`` (survivor) or the encounter ends (dropped).

    Because ``min_sep`` is monotone non-increasing, ``state.min_sep <= target`` means the particle
    reached shell ``target``; if it goes ``is_terminal`` first it never will (this leg). An already
    crossed state (overshoot from a prior level) returns immediately.
    """
    env, state = particle.env, particle.state
    while state.min_sep > target and not env.is_terminal(state):
        state = env.advance(state, streams)
    return state


def _evolve_to_end(particle: Particle, streams: FleetStreams) -> FleetState:
    """Fly a particle from wherever it is to ``is_terminal`` — the **tail leg**.

    :func:`_evolve_to_shell` stops a particle the instant it crosses, so at the last shell it is
    frozen at its *first* loss of separation. K and A need the rest of the encounter: a second
    loss, or a third aircraft drawn in, happens after that instant. This leg supplies it.
    """
    env, state = particle.env, particle.state
    while not env.is_terminal(state):
        state = env.advance(state, streams)
    return state


def evolve_shard(
    particles: Sequence[Particle],
    target: float,
    seeds: Sequence[np.random.SeedSequence],
) -> list[Particle]:
    """Evolve each particle to ``target`` on its own freshly spawned stream — one level's *map*.

    Factored out of :func:`ips_once` so a parallel driver can run **contiguous slices** of the same
    map in worker processes (:mod:`opencdarr.parallel`). Particle ``i`` reads only its own
    ``particles[i]`` and ``seeds[i]``, so any partition of the range recomposes to the identical
    list — which is what makes the parallel estimate bit-identical rather than merely equivalent.
    """
    return [
        Particle(env=p.env, state=_evolve_to_shell(p, target, _streams(s)))
        for p, s in zip(particles, seeds, strict=True)
    ]


def resample_level(
    evolved: Sequence[Particle],
    target: float,
    n_particles: int,
    seq: np.random.SeedSequence,
) -> tuple[float, list[Particle]]:
    """One level's *barrier*: the survival fraction and the resampled cloud.

    Survivors are those that reached the shell; they are drawn with replacement back up to
    ``n_particles``. An empty returned cloud means the level collapsed (ADR 0017 §2) — the caller
    decides what to record. Independence between clones comes from the next level's fresh
    per-particle streams, not from this draw.
    """
    survivors = [p for p in evolved if p.state.min_sep <= target]
    fraction = len(survivors) / n_particles
    if not survivors:
        return fraction, []
    idx = generator(seq).integers(0, len(survivors), size=n_particles)
    return fraction, [survivors[i] for i in idx]


def ips_once(
    build_initial: BuildInitial,
    levels: Sequence[float],
    n_particles: int,
    seq: np.random.SeedSequence,
    *,
    tail: bool = False,
) -> IPSResult:
    """One fixed-effort multilevel-splitting run: ``P̂ = Π_k S_k/N`` over the shells ``levels``.

    ``levels`` is the decreasing shell sequence ``d_1 > … > d_m`` (``d_0 = ∞`` is implicit — every
    particle starts above the first shell). ``build_initial`` makes one particle from a seed;
    geometry is sampled there, forward noise is spawned here per particle per level (so resampled
    clones of one survivor diverge). Returns ``prob = 0`` with ``collapsed_at`` set if some level
    has no survivors — a signal the shells are spaced too aggressively (ADR 0017 §2), not a real 0.

    ``seq`` is read, never consumed: the whole tree is addressed by index
    (:func:`~opencdarr.rng.children`), so this is a pure function of its arguments and the caller's
    sequence comes back untouched. That matters because ``SeedSequence.spawn`` is *stateful* —
    spawning from ``seq`` here would mean a second call on the same object walked a different tree
    and quietly returned a different answer, a difference nothing in the result would reveal.

    ``tail=True`` adds a final leg that flies every survivor on to termination, which is what
    :attr:`IPSResult.p_ac` and :attr:`IPSResult.expected_los_pairs` need. It costs roughly
    ``n_particles`` full encounter tails on top of the splitting, so it is off by default; ``prob``
    is unaffected either way. The tail draws from child index 2, so the splitting itself stays
    bit-identical to a ``tail=False`` run of the same seed.
    """
    init_seq, evolve_seq, tail_seq = children(seq, 0, 3)
    particles = [build_initial(s) for s in children(init_seq, 0, n_particles)]
    level_seqs = children(evolve_seq, 0, len(levels))

    survival: list[float] = []
    for k, target in enumerate(levels):
        # fresh forward streams per particle this level (+ one resampling stream)
        sub = children(level_seqs[k], 0, n_particles + 1)
        evolved = evolve_shard(particles, target, sub[:n_particles])
        fraction, particles = resample_level(evolved, target, n_particles, sub[n_particles])
        survival.append(fraction)
        if not particles:
            return IPSResult(prob=0.0, levels=tuple(levels), survival=tuple(survival),
                             n_particles=n_particles, collapsed_at=k)

    result = IPSResult(prob=float(np.prod(survival)), levels=tuple(levels),
                       survival=tuple(survival), n_particles=n_particles, collapsed_at=None)
    if not tail:
        return result

    # clones share their state object, so identity counts the distinct survivors behind the cloud
    lineages = len({id(p.state) for p in particles})
    ends = [
        _evolve_to_end(p, _streams(s))
        for p, s in zip(particles, children(tail_seq, 0, n_particles), strict=True)
    ]
    counts = [los_counts(len(e.states), e.los_pairs) for e in ends]
    return replace(
        result,
        tail_los_pairs=tuple(k for k, _ in counts),
        tail_los_aircraft=tuple(a for _, a in counts),
        n_aircraft=len(ends[0].states) if ends else 0,
        n_lineages=lineages,
    )


def _log_ci(probs: list[float], z: float = 1.96) -> tuple[float, float]:
    """A 95% CI for the mean rare-event probability. Log-space (the product estimator is
    right-skewed) when every replication is positive; otherwise the raw min/max span, since a
    collapsed replication (P̂ = 0) has no logarithm and signals under-resolved shells."""
    positive = [p for p in probs if p > 0.0]
    if len(positive) < 2 or len(positive) != len(probs):
        return (min(probs), max(probs))
    logs = np.log(positive)
    se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
    centre = float(np.mean(logs))
    return (math.exp(centre - z * se), math.exp(centre + z * se))


def replication_seeds(seed: int, reps: int) -> tuple[np.random.SeedSequence, ...]:
    """The ``reps`` independent seed subtrees for the replications (ADR 0001). Exposed so a caller
    can run :func:`ips_once` in parallel over them and still :func:`combine_replications` the same
    way :func:`estimate_rare_prob` does — the reproducibility is identical either way."""
    return tuple(spawn(root_seed_sequence(seed), reps))


def combine_replications(results: Sequence[IPSResult]) -> RareEventEstimate:
    """Aggregate independent :func:`ips_once` results into the point estimate + CI (ADR 0017 §5):
    mean of the per-replication ``P̂`` (each is unbiased) with a log-space CI across replications.
    Collapsed replications (an empty level ⇒ ``P̂ = 0``) are counted, not hidden."""
    probs = [r.prob for r in results]
    return RareEventEstimate(
        prob=float(np.mean(probs)),
        ci=_log_ci(probs),
        reps=tuple(results),
        n_collapsed=sum(1 for r in results if r.collapsed_at is not None),
    )


def estimate_rare_prob(
    build_initial: BuildInitial,
    levels: Sequence[float],
    *,
    n_particles: int,
    reps: int,
    seed: int,
    tail: bool = False,
) -> RareEventEstimate:
    """Estimate the rare-event probability with a CI from ``reps`` independent IPS replications.

    Each replication is an :func:`ips_once` on an independent seed subtree (ADR 0001); results are
    combined by :func:`combine_replications`. Serial — a caller wanting parallel replications runs
    :func:`ips_once` over :func:`replication_seeds` itself (e.g. joblib) and calls
    :func:`combine_replications`, for the identical result.

    ``tail=True`` is passed through to every replication, so each carries the per-aircraft and
    per-pair measurements as well as ``prob``. It does not change ``prob``.
    """
    results = [ips_once(build_initial, levels, n_particles, s, tail=tail)
               for s in replication_seeds(seed, reps)]
    return combine_replications(results)


def ladder_from_record(
    min_seps: Sequence[float],
    rpz: float,
    *,
    halving: float = 0.5,
    min_count: int = 30,
    step: float = 0.5,
) -> list[float]:
    """Build a shell ladder from a pilot run's own minimum-separation record.

    ADR 0017 accepts fixed shells with hand-tuned spacing and defers adaptive levels; this is the
    middle ground that has done the work in practice. Shells are placed where the *measured*
    distribution says a fraction ``halving`` of the survivors will reach the next one, for as long
    as the pilot can still resolve a quantile (at least ``min_count`` encounters below it). Below
    that floor there is no data left, and the ladder falls back to a uniform ``step`` — which is
    the region through a resolver's own margin, where the descent has to be slow anyway.

    Spacing by the distribution rather than by a geometric descent in distance matters because the
    distribution moves: a ladder that works for one fleet size collapses at another.
    """
    ms = np.asarray([m for m in min_seps if math.isfinite(m)], dtype=float)
    if ms.size == 0:
        raise ValueError("no finite minimum separations to build a ladder from")
    shells: list[float] = []
    p, floor = halving, min_count / ms.size
    while p >= floor:
        d = float(np.percentile(ms, p * 100))
        if d > rpz and (not shells or d < shells[-1] - step):
            shells.append(d)
        p *= halving
    d = (shells[-1] if shells else float(ms.max())) - step
    while d > rpz:
        shells.append(d)
        d -= step
    shells.append(rpz)
    return [round(x, 2) for x in shells]
