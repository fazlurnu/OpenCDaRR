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


def generator(seq: np.random.SeedSequence) -> np.random.Generator:
    """Return the ``Generator`` (PCG64) for a leaf sequence.

    Call this only on a sequence you will *not* also spawn from, so that a stream is
    either an internal node or a leaf, never both (see the module contract).
    """
    return np.random.default_rng(seq)
