"""What can be declared — the parameter vocabulary and the ``Fixed`` / ``Sweep`` axes."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

# Every key below is wired end to end. The list is deliberately short: it is what the estimators
# actually thread today, not an aspiration. An unknown key fails immediately with this list in the
# message, which is a better contributor experience than a silently ignored keyword.

_SCENARIO_FIELDS = frozenset(
    {"speed", "dcpa_max", "tlos", "pos_ci95", "vel_ci95",
     "pos_ci95_declared", "vel_ci95_declared"}
)
_CONFLICT_FIELDS = frozenset({"rpz", "t_lookahead"})
_SIMULATION_FIELDS = frozenset(
    {"dt", "t_max", "done_timeout", "broadcast_interval", "broadcast_jitter",
     "broadcast_random_phase", "stop_within"}
)
_GEOMETRY_SLOTS = frozenset({"dpsi", "dcpa", "side", "gs_intr"})

# Every field of `Methods`, so declaring one as an axis overrides the bundle per condition.
# `wind` is not a pluggable model but it is a per-run input the bundle carries, so it is swept the
# same way — `wind=Sweep([NO_WIND, WindField.from_met(270, 8)])`.
_COMPONENTS = frozenset(
    {"detector", "resolver", "recovery", "navigation", "communication", "surveillance",
     "kinematics", "perf", "wind", "airframes", "scenario"}
)
_KNOWN_KEYS = (
    _SCENARIO_FIELDS | _CONFLICT_FIELDS | _SIMULATION_FIELDS | _GEOMETRY_SLOTS | _COMPONENTS
)


@dataclass(frozen=True)
class Fixed:
    """One parameter held at ``value`` for every condition."""

    value: Any


@dataclass(frozen=True)
class Sweep:
    """One parameter varied across ``values`` — an output axis, one condition per level.

    ``name`` labels the axis in the results table, defaulting to the declaration key. ``build``
    maps a level onto the value the run actually needs, which is what lets a *component* parameter
    be swept over a scalar: the table reads as numbers while the run receives objects.

        Sweep([1.05, 1.2], build=lambda m: MVP(margin=m), name="margin")

    Without ``build`` the levels are used as-is, so a categorical axis is simply the objects
    themselves (``Sweep([MVP(1.05), VO(1.05)])``) — readable in the table only if they have useful
    ``repr``s, which is the reason ``build`` exists.
    """

    values: tuple[Any, ...]
    name: str | None = None
    build: Callable[[Any], Any] | None = None

    def __init__(
        self,
        values: Sequence[Any],
        name: str | None = None,
        build: Callable[[Any], Any] | None = None,
    ) -> None:
        levels = tuple(values)
        if not levels:
            raise ValueError("a Sweep needs at least one level; got an empty sequence")
        object.__setattr__(self, "values", levels)
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "build", build)

    def resolve(self, level: Any) -> Any:
        """The value a run receives for ``level`` — through ``build`` when one was given."""
        return level if self.build is None else self.build(level)


Axis = Fixed | Sweep
