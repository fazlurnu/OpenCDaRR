"""Reproducible random number generation.

Implements ADR 0001 (``vault/decisions/0001-rng-per-particle-spawn.md``): every
stochastic component gets its *own* generator, derived reproducibly from a single
integer seed, with no global or shared RNG anywhere.

Stream-layout contract
----------------------
A run has one integer ``seed``. From it we build a root ``SeedSequence``. Independent
substreams are created with :func:`spawn`, which delegates to
``numpy.random.SeedSequence.spawn`` — the children it returns are statistically
independent by construction (unlike ``seed + k`` offsets, which can correlate).

A ``SeedSequence`` is used for exactly one of two roles, never both:

- **internal node** — call :func:`spawn` on it to create child sequences (e.g. a run
  spawns one substream per component; an IPS particle spawns one per clone);
- **leaf** — call :func:`generator` on it to obtain the ``Generator`` a function draws
  from.

Keeping the two roles separate makes the assignment of streams to components an
explicit, documented tree — which is what lets an experiment's provenance record
*exactly* how its randomness was wired.

:func:`child` / :func:`children` address that same tree by index: a child is fixed by
its parent and its position, so a parallel worker can rebuild only the slice it needs
rather than receiving the whole fan-out. They produce exactly what :func:`spawn` would.

They also leave the parent **untouched**, which :func:`spawn` does not — it is stateful,
handing out children from ``n_children_spawned``, so fanning out twice from one object
continues the numbering instead of repeating it. A routine that spawns from its seed
argument therefore quietly returns a different answer the second time it is called on
that object. Routines meant to be reproducible from their arguments alone (the
estimators in :mod:`opencdarr.estimate.ips` and :mod:`opencdarr.estimate.parallel`) address by
index for
exactly that reason, and are pure functions of the sequence they are handed.

Every function that needs randomness should take a ``numpy.random.Generator`` as an
explicit argument; this module is the only place a generator is created.
"""

from __future__ import annotations

import numpy as np


def root_seed_sequence(seed: int) -> np.random.SeedSequence:
    """Return the root ``SeedSequence`` for a run, derived from a single integer seed.

    This is the sole entry point of randomness for a run: everything else is spawned
    from the returned sequence, so the whole stream tree is fixed by ``seed`` alone.
    """
    if seed < 0:
        raise ValueError(f"seed must be non-negative, got {seed}")
    return np.random.SeedSequence(seed)


def spawn(parent: np.random.SeedSequence, n: int) -> list[np.random.SeedSequence]:
    """Spawn ``n`` statistically independent child sequences from ``parent``.

    Use this for internal nodes of the stream tree — one child per component, or one
    child per IPS particle clone. The children are independent of each other and of
    every other stream spawned from a different parent.
    """
    if n < 0:
        raise ValueError(f"n must be non-negative, got {n}")
    return list(parent.spawn(n))


def child(parent: np.random.SeedSequence, i: int) -> np.random.SeedSequence:
    """Return the child of ``parent`` at absolute index ``i``.

    A child is fully determined by its parent and its position, so reconstructing one directly
    lets a worker regenerate just the slice of a fan-out it was handed, instead of the caller
    materialising all ``n`` children and shipping them across a process boundary — which matters
    when ``n`` is a two-million-encounter Monte Carlo. Same tree, same numbers: a cheaper spelling
    of :func:`spawn`, not a second stream layout (ADR 0001).

    .. warning::
       ``SeedSequence.spawn`` is **stateful** — it hands out children starting from the parent's
       ``n_children_spawned``, so spawning twice from one sequence continues rather than restarts.
       ``child`` indexes *absolutely*. The two therefore agree — ``child(p, i) == spawn(p, n)[i]``
       for every ``n > i`` — exactly when ``p`` has not been spawned from yet, which the module
       contract above already requires: a sequence is an internal node fanned out **once**, or a
       leaf. Mixing :func:`spawn` and :func:`child` on one parent re-uses indices, so pick one.
    """
    if i < 0:
        raise ValueError(f"i must be non-negative, got {i}")
    return np.random.SeedSequence(
        entropy=parent.entropy,
        spawn_key=tuple(parent.spawn_key) + (i,),
        pool_size=parent.pool_size,
    )


def children(
    parent: np.random.SeedSequence, start: int, stop: int
) -> list[np.random.SeedSequence]:
    """The children of ``parent`` with indices ``start`` … ``stop - 1`` — one chunk's worth.

    ``children(parent, 0, n)`` equals ``spawn(parent, n)``; a half-open slice of it is the
    substream set for one chunk of a parallel run. Building the slice directly is what keeps a
    worker from re-spawning all ``n`` siblings just to reach its own few.
    """
    if start < 0 or stop < start:
        raise ValueError(f"require 0 <= start <= stop, got {start=}, {stop=}")
    return [child(parent, i) for i in range(start, stop)]


def generator(seq: np.random.SeedSequence) -> np.random.Generator:
    """Return the ``Generator`` (PCG64) for a leaf sequence.

    Call this only on a sequence you will *not* also spawn from, so that a stream is
    either an internal node or a leaf, never both (see the module contract).
    """
    return np.random.default_rng(seq)
