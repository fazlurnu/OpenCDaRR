"""Broadcast timing — when each aircraft transmits (the transmit side of the datalink).

Separate from the Comm *channel* (reception + latency, :mod:`opencdarr.cns.communication`): a
:class:`BroadcastSchedule` owns the transmit cadence — the nominal ``interval``, an optional
per-aircraft ``phase`` offset, and optional per-transmission ``jitter`` — and the per-aircraft
broadcast clock the fleet loop threads. This mirrors the ADS-B reception model's split of the
transmission timing (period + jitter) from the reception probability (the channel). The clock is a
plain ``list[float]`` so it clones with the rest of the run state (ADR 0001). See
``vault/observations/broadcast-phase-offset.md`` and ``broadcast-jitter.md``.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


def random_broadcast_phase(
    n: int, interval: float, rng: np.random.Generator
) -> list[float]:
    """Draw an independent initial broadcast offset in ``[0, interval)`` per aircraft.

    The realistic unsynchronised-transmitter model: aircraft spawn at different times, so each runs
    the *same* interval at a *random phase*, not a shared ``t=0`` tick — the aligned default is the
    pessimally-correlated case where every aircraft's update peaks together (real ADS-B dithers its
    slot to avoid exactly that). Seed-reproducible and clone-safe when ``rng`` is a spawned
    substream (ADR 0001). Pass the result as ``BroadcastSchedule(phase=...)``. See
    ``vault/observations/broadcast-phase-offset.md``.
    """
    return [float(x) for x in rng.uniform(0.0, interval, n)]


@dataclass(frozen=True)
class BroadcastSchedule:
    """When each aircraft transmits: nominal ``interval``, per-aircraft ``phase``, per-gap jitter.

    ``interval`` is the broadcast period (the ADS-L/ASAS decision rate, 1 Hz in the reference).
    ``phase`` offsets each aircraft's clock — ``None`` aligns them all at ``t = 0`` (one shared
    cadence, the pessimally-correlated case and the reduction to a single-clock pair); pass one
    offset per aircraft (e.g. from :func:`random_broadcast_phase`) to model unsynchronised
    transmitters. ``jitter`` (seconds, ``0`` = fixed) dithers *each* gap by ``U(-jitter, +jitter)``
    — the per-transmission slot randomisation real ADS-B uses; it needs an rng and must be
    ``< interval`` so gaps stay positive.
    """

    interval: float = 1.0
    phase: Sequence[float] | None = None
    jitter: float = 0.0

    def __post_init__(self) -> None:
        if self.interval <= 0.0:
            raise ValueError(f"interval must be > 0, got {self.interval}")
        if self.jitter < 0.0:
            raise ValueError(f"jitter must be >= 0, got {self.jitter}")
        if self.jitter >= self.interval:
            raise ValueError(
                f"jitter must be < interval ({self.interval}), got {self.jitter}"
            )

    def initial(self, n: int) -> list[float]:
        """The per-aircraft broadcast clock at ``t = 0``: the phase offsets, or aligned at 0."""
        if self.phase is None:
            return [0.0] * n
        if len(self.phase) != n:
            raise ValueError(f"phase needs n={n} entries, got {len(self.phase)}")
        if any(p < 0.0 for p in self.phase):
            raise ValueError(f"phase entries must be >= 0, got {list(self.phase)}")
        return [float(p) for p in self.phase]

    def due(self, clock: Sequence[float], t: float, eps: float = 1e-9) -> list[int]:
        """The indices whose broadcast clock is due at time ``t`` (in aircraft order)."""
        return [i for i in range(len(clock)) if t + eps >= clock[i]]

    def advance(self, clock_i: float, rng: np.random.Generator | None) -> float:
        """The next broadcast time after ``clock_i`` — a fixed ``interval``, or one dithered by
        ``jitter`` (drawn from ``rng``, which must be supplied when ``jitter > 0``)."""
        step = self.interval
        if self.jitter > 0.0:
            if rng is None:
                raise ValueError(
                    "jitter requires a broadcast rng (its own substream, ADR 0006 §6)"
                )
            step += float(rng.uniform(-self.jitter, self.jitter))
        return clock_i + step
