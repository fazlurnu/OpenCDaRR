"""Surveillance — what a receiver believes about a source (the S of CNS, Phase 3b).

Implements :class:`~opencdarr.cns.base.SurveillanceModel`. Design decision (hold-as-is, not
dead-reckoning) is recorded in ``vault/decisions/0006-communication-model-design.md`` §2; visual
validation in ``vault/observations/surveillance-hold-as-is.md``.
"""

from __future__ import annotations

from opencdarr.cns.base import CommState, SurveillanceModel
from opencdarr.state import AircraftState


class LastKnown(SurveillanceModel):
    """Hold-as-is: the receiver's belief is exactly its last delivered message, unchanged.

    No extrapolation — a stale message is used as-is, not dead-reckoned forward by its age
    (ADR 0006 §2: it is the honest representation of what the receiver actually has, and
    extrapolation assumes the source kept flying straight, which is wrong exactly when the source
    just started maneuvering).
    """

    def perceived(
        self, state: CommState, receiver: str, source: str, t_now: float
    ) -> AircraftState | None:
        held = state.held.get((receiver, source))
        return None if held is None else held.state


def age(state: CommState, receiver: str, source: str, t_now: float) -> float | None:
    """How stale ``receiver``'s current belief about ``source`` is, or ``None`` if it has none.

    Instrumentation only: hold-as-is means this never changes what CDR perceives (see
    :class:`LastKnown`), only how old the perceived state is. Every :class:`SurveillanceModel`
    would report the same age regardless of its hold-vs-extrapolate policy, since it is a
    property of ``t_meas``, not of what the model does with the state — hence a free function,
    not part of the ABC.
    """
    held = state.held.get((receiver, source))
    return None if held is None else t_now - held.t_meas
