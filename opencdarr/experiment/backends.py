"""The backends — plain Monte Carlo (``MC``) and rare-event splitting (``IPS``) as declarations."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class MC:
    """Plain Monte Carlo: ``n_encounters`` independent encounters per condition.

    Carries exactly its own estimator's parameters, so an illegal pairing (particles on MC, an
    encounter count on IPS) is unrepresentable rather than validated.
    """

    n_encounters: int

    def __post_init__(self) -> None:
        if self.n_encounters <= 0:
            raise ValueError(f"n_encounters must be > 0, got {self.n_encounters}")


@dataclass(frozen=True)
class IPS:
    """Rare-event interacting particle system: fixed-effort multilevel splitting (ADR 0017).

    ``shells`` is the decreasing sequence of running-minimum separations to split on, ending at the
    rare boundary (``rpz`` for loss of separation). They are **explicit and per-experiment**, not
    derived: ADR 0017 accepts fixed shells with hand-tuned spacing and defers adaptive levels, and
    a ladder spaced too aggressively collapses (reported as ``n_collapsed``, never as a real zero).

    ``reps`` is structural, not a convenience: particles within one run interact through
    resampling, so a single run's spread understates the real one — only independent replications
    are independent (ADR 0017 §5).

    ``tail`` (default on) flies the final cloud past its first breach to the end of the encounter,
    which is the only way this backend can report ``p_los_ac`` and ``mean_k``: the ladder stops
    each survivor the instant it crosses, so K is 1 and A is 2 there by construction. Switching it
    off leaves those two ``nan`` and changes no other number.
    """

    shells: tuple[float, ...]
    n_particles: int
    reps: int
    tail: bool = True

    def __init__(self, shells: Sequence[float], n_particles: int, reps: int,
                 tail: bool = True) -> None:
        ladder = tuple(float(s) for s in shells)
        if not ladder:
            raise ValueError("IPS needs at least one shell")
        if any(b >= a for a, b in zip(ladder, ladder[1:], strict=False)):
            raise ValueError(f"shells must be strictly decreasing, got {ladder}")
        if n_particles <= 0 or reps <= 0:
            raise ValueError(f"require n_particles > 0 and reps > 0, got {n_particles}, {reps}")
        object.__setattr__(self, "shells", ladder)
        object.__setattr__(self, "n_particles", n_particles)
        object.__setattr__(self, "reps", reps)
        object.__setattr__(self, "tail", tail)


Backend = MC | IPS
