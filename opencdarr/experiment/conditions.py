"""Conditions — the cross-product of a declaration, one per cell."""

from __future__ import annotations

import itertools
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from opencdarr.experiment.declaration import _KNOWN_KEYS, Axis, Fixed, Sweep


@dataclass(frozen=True)
class Condition:
    """One cell of the experiment: the swept levels labelling it, and the values the run gets."""

    levels: tuple[tuple[str, Any], ...]  # (column name, level) per swept axis, declaration order
    values: tuple[tuple[str, Any], ...]  # key -> resolved value, for every declared parameter

    @property
    def label(self) -> dict[str, Any]:
        """The swept levels as a dict — the identifying columns of this row."""
        return dict(self.levels)

    def get(self, key: str, default: Any = None) -> Any:
        return dict(self.values).get(key, default)


def expand(independent_vars: Mapping[str, Axis]) -> tuple[Condition, ...]:
    """The cross-product of the declared :class:`Sweep` axes, in declaration order.

    Every parameter appears in each condition's ``values``; only the swept ones appear in
    ``levels``, so an all-:class:`Fixed` declaration is the single-cell case, not a separate path.
    """
    unknown = sorted(set(independent_vars) - _KNOWN_KEYS)
    if unknown:
        raise ValueError(
            f"unknown parameter(s) {unknown}. Declarable: {sorted(_KNOWN_KEYS)}"
        )
    swept = [(key, axis) for key, axis in independent_vars.items() if isinstance(axis, Sweep)]
    fixed = [(key, axis.value) for key, axis in independent_vars.items()
             if isinstance(axis, Fixed)]

    conditions: list[Condition] = []
    for combo in itertools.product(*(axis.values for _, axis in swept)):
        levels = tuple(
            (axis.name or key, level) for (key, axis), level in zip(swept, combo, strict=True)
        )
        resolved = tuple(
            (key, axis.resolve(level)) for (key, axis), level in zip(swept, combo, strict=True)
        )
        conditions.append(Condition(levels=levels, values=tuple(fixed) + resolved))
    return tuple(conditions)
