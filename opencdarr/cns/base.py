"""CNS interfaces (Phase 3a: navigation).

The C-N-S layers are pluggable, like cd/cr/crr. Phase 3a introduces the **N** (navigation)
piece: how an aircraft measures its own state to broadcast. Communication (reception + latency)
and the held-message surveillance state arrive in 3b.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from opencdarr.state import AircraftState


@dataclass(frozen=True)
class Message:
    """A broadcast: an aircraft's own (noisy) self-measurement, timestamped for delivery."""

    source: str
    state: AircraftState  # the measured self-state (noisy)
    t_meas: float  # when it was measured [s]


class NoiseDistribution(Protocol):
    """A 2D position-error distribution: ``(rng, ci95, trk_deg) -> (east, north)`` error [m]."""

    def __call__(
        self, rng: np.random.Generator, ci95: float, trk_deg: float
    ) -> tuple[float, float]: ...


class NavigationModel(ABC):
    """How an aircraft measures its own state to broadcast — the contribution surface."""

    @abstractmethod
    def measure(self, true: AircraftState, t: float, rng: np.random.Generator) -> Message:
        """Return the aircraft's (noisy) self-measurement as a broadcastable :class:`Message`."""
