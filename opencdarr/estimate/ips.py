"""Rare-event estimator — interacting particle system (IPS), Blom et al. 2007. Phase 8 / ADR 0017.

Fixed-effort multilevel splitting over the fleet estimator interface (``advance / level /
is_terminal``, :mod:`opencdarr.fleet`). Where plain Monte Carlo
(:func:`opencdarr.estimate.montecarlo.estimate_p_los`) starves in the rare regime — a single
10⁴-encounter run can read *zero* events
([[rare-event-validation-ladder]]) — IPS concentrates effort on the trajectories heading toward the
rare set and returns the small probability at a usable cost.

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

**Replications** (§5): within one run the particles interact through resampling (shared
ancestors), so a single run's spread understates the real one. ``R`` independent runs are averaged
instead, and their spread is what a reader judges the estimate's stability by. No interval is
reported — agreement with the Monte-Carlo anchor is judged on the **ratio** of the two estimates (a
factor of two, or five at 1e-4 and below where the anchor itself rests on few events), because at
these probabilities an interval invited a precision the shell spacing does not support (ADR 0022).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr.fleet import CnsStreams, FleetEnv, FleetState, FleetStreams
from opencdarr.relative import pairwise_min_sep
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

    prob: float  # P̂ = Π_k survival_k (0.0 if a level collapsed) — the per-run probability
    levels: tuple[float, ...]  # the shell distances d_1 … d_m [m]
    survival: tuple[float, ...]  # S_k / N per crossed level
    n_particles: int
    collapsed_at: int | None  # index of the level where S_k = 0, or None
    # --- the tail leg (None when it was not run, or the ladder collapsed) ---
    tail_k: float | None = None  # mean K over the survivors flown to termination
    tail_a: float | None = None  # mean A over the same
    n_lineages: int | None = None  # distinct survivors behind that cloud — the tail's real ESS
    n_aircraft: int = 0  # N, read off a particle (0 when there were none)


@dataclass(frozen=True)
class RareEventEstimate:
    """The replicated estimate: the three loss metrics, averaged over independent IPS runs."""

    p_los_run: float  # mean of the per-replication P̂ — P(LoS) per run
    reps: tuple[IPSResult, ...]  # every replication, for inspection
    n_collapsed: int  # replications that hit an empty level (P̂ = 0)
    p_los_ac: float = float("nan")  # P(LoS) per aircraft — needs the tail leg
    mean_k: float = float("nan")  # E[K] — needs the tail leg
    n_lineages: int = 0  # distinct lineages summed over replications (the tail's ESS)

    @property
    def prob(self) -> float:
        """The per-run probability, under its historical name. See :attr:`p_los_run`."""
        return self.p_los_run


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


def level(state: FleetState) -> float:
    """The importance function IPS splits on: the fleet's **current** minimum pairwise separation
    [m], smaller = closer to the rare event (ADR 0004's starting point; a Phase-8 ADR may refine
    it for simultaneous multi-aircraft conflict). A pure read of ``state``, independent of N."""
    return pairwise_min_sep(state.states)


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


def _evolve_to_terminal(particle: Particle, streams: FleetStreams) -> FleetState:
    """Fly a survivor **past** its first breach, to the end of the encounter.

    The tail leg. :func:`_evolve_to_shell` stops the instant the running minimum crosses a shell,
    so at the rare boundary every survivor has exactly one losing pair by construction — K and A
    are not *measured* there, they are an artefact of where the ladder stopped. Continuing to
    ``is_terminal`` on fresh streams lets the rest of the encounter happen, which is what makes
    ``E[A | rare set]`` an observation rather than an assumption of 2.
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
    map in worker processes (:mod:`opencdarr.estimate.parallel`). Particle ``i`` reads only its own
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
) -> tuple[float, list[Particle], int]:
    """One level's *barrier*: the survival fraction, the resampled cloud, and its lineage count.

    Survivors are those that reached the shell; they are drawn with replacement back up to
    ``n_particles``. An empty returned cloud means the level collapsed (ADR 0017 §2) — the caller
    decides what to record. Independence between clones comes from the next level's fresh
    per-particle streams, not from this draw.

    The third value is how many **distinct** survivors the draw actually took — the cloud's
    effective sample size, which is what any conditional mean read off it is really based on
    (``n_particles`` counts clones, not information). Taken from the draw itself rather than by
    de-duplicating the returned particles: clones *share* one immutable state object, so counting
    by identity gives the right answer in-process and the wrong one after a worker pickles the
    cloud, turning one shared object into several equal copies.
    """
    survivors = [p for p in evolved if p.state.min_sep <= target]
    fraction = len(survivors) / n_particles
    if not survivors:
        return fraction, [], 0
    idx = generator(seq).integers(0, len(survivors), size=n_particles)
    return fraction, [survivors[i] for i in idx], len(set(idx.tolist()))


def ips_once(
    build_initial: BuildInitial,
    levels: Sequence[float],
    n_particles: int,
    seq: np.random.SeedSequence,
    *,
    tail: bool = True,
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

    ``tail`` runs the continuation leg (:func:`_evolve_to_terminal`) on the final cloud, which is
    the only way to observe K and A — the ladder stops each survivor at its *first* breach, so
    without it the per-aircraft number would assume A = 2 and undercount at N > 2. It reads a
    **third** child of ``seq``: a ``SeedSequence`` child depends only on its index and its parent,
    so asking for three where there were two leaves the init and evolve subtrees bit-identical and
    the splitting is the same whether or not the tail runs.
    """
    init_seq, evolve_seq, tail_seq = children(seq, 0, 3)
    particles = [build_initial(s) for s in children(init_seq, 0, n_particles)]
    level_seqs = children(evolve_seq, 0, len(levels))
    n_aircraft = len(particles[0].state.states) if particles else 0

    survival: list[float] = []
    lineages = 0
    for k, target in enumerate(levels):
        # fresh forward streams per particle this level (+ one resampling stream)
        sub = children(level_seqs[k], 0, n_particles + 1)
        evolved = evolve_shard(particles, target, sub[:n_particles])
        fraction, particles, lineages = resample_level(
            evolved, target, n_particles, sub[n_particles]
        )
        survival.append(fraction)
        if not particles:
            return IPSResult(prob=0.0, levels=tuple(levels), survival=tuple(survival),
                             n_particles=n_particles, collapsed_at=k, n_aircraft=n_aircraft)

    tail_k = tail_a = None
    if tail:
        finals = [
            _evolve_to_terminal(p, _streams(s))
            for p, s in zip(particles, children(tail_seq, 0, n_particles), strict=True)
        ]
        tail_k = float(np.mean([f.n_los_pairs for f in finals]))
        tail_a = float(np.mean([f.n_los_aircraft for f in finals]))

    return IPSResult(prob=float(np.prod(survival)), levels=tuple(levels),
                     survival=tuple(survival), n_particles=n_particles, collapsed_at=None,
                     tail_k=tail_k, tail_a=tail_a,
                     n_lineages=lineages if tail else None, n_aircraft=n_aircraft)


def replication_seeds(seed: int, reps: int) -> tuple[np.random.SeedSequence, ...]:
    """The ``reps`` independent seed subtrees for the replications (ADR 0001). Exposed so a caller
    can run :func:`ips_once` in parallel over them and still :func:`combine_replications` the same
    way :func:`estimate_rare_prob` does — the reproducibility is identical either way."""
    return tuple(spawn(root_seed_sequence(seed), reps))


def combine_replications(results: Sequence[IPSResult]) -> RareEventEstimate:
    """Aggregate independent :func:`ips_once` results into the point estimate + CI (ADR 0017 §5):
    mean of the per-replication ``P̂`` (each is unbiased) with a log-space CI across replications.
    Collapsed replications (an empty level ⇒ ``P̂ = 0``) are counted, not hidden.

    The per-aircraft rate and E[K] are each replication's *own* estimate averaged, not a ratio of
    two averages: a replication measures ``P̂`` and its own ``E[· | rare set]`` from one cloud, so
    combining them per replication keeps each term unbiased and lets a collapsed replication
    contribute the zero it actually found. They are ``nan`` when the tail did not run — an absent
    measurement, which is not the same statement as zero.
    """
    probs = [r.prob for r in results]
    tails = [r for r in results if r.tail_a is not None or r.collapsed_at is not None]
    per_ac = [
        0.0 if r.collapsed_at is not None else r.prob * (r.tail_a or 0.0) / (r.n_aircraft or 1)
        for r in tails
    ]
    per_k = [
        0.0 if r.collapsed_at is not None else r.prob * (r.tail_k or 0.0)
        for r in tails
    ]
    ran_tail = any(r.tail_a is not None for r in results)
    return RareEventEstimate(
        p_los_run=float(np.mean(probs)),
        reps=tuple(results),
        n_collapsed=sum(1 for r in results if r.collapsed_at is not None),
        p_los_ac=float(np.mean(per_ac)) if ran_tail else float("nan"),
        mean_k=float(np.mean(per_k)) if ran_tail else float("nan"),
        n_lineages=sum(r.n_lineages or 0 for r in results),
    )


def estimate_rare_prob(
    build_initial: BuildInitial,
    levels: Sequence[float],
    *,
    n_particles: int,
    reps: int,
    seed: int,
    tail: bool = True,
) -> RareEventEstimate:
    """Estimate the rare-event probability with a CI from ``reps`` independent IPS replications.

    Each replication is an :func:`ips_once` on an independent seed subtree (ADR 0001); results are
    combined by :func:`combine_replications`. Serial — a caller wanting parallel replications runs
    :func:`ips_once` over :func:`replication_seeds` itself (e.g. joblib) and calls
    :func:`combine_replications`, for the identical result.

    ``tail`` (default on) adds the continuation leg that makes ``p_los_ac`` and ``mean_k``
    measurable; switching it off leaves them ``nan`` and changes no other number.
    """
    results = [ips_once(build_initial, levels, n_particles, s, tail=tail)
               for s in replication_seeds(seed, reps)]
    return combine_replications(results)
