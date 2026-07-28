"""Rare-event estimator — Blom–Bakker interacting particle system (IPS), Phase 8 / ADR 0017.

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
(shared ancestors), so the honest CI comes from ``R`` independent IPS runs, reported in log space.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr.fleet import CnsStreams, FleetEnv, FleetState, FleetStreams
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
    """
    init_seq, evolve_seq = children(seq, 0, 2)
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

    return IPSResult(prob=float(np.prod(survival)), levels=tuple(levels),
                     survival=tuple(survival), n_particles=n_particles, collapsed_at=None)


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
) -> RareEventEstimate:
    """Estimate the rare-event probability with a CI from ``reps`` independent IPS replications.

    Each replication is an :func:`ips_once` on an independent seed subtree (ADR 0001); results are
    combined by :func:`combine_replications`. Serial — a caller wanting parallel replications runs
    :func:`ips_once` over :func:`replication_seeds` itself (e.g. joblib) and calls
    :func:`combine_replications`, for the identical result.
    """
    results = [ips_once(build_initial, levels, n_particles, s)
               for s in replication_seeds(seed, reps)]
    return combine_replications(results)
