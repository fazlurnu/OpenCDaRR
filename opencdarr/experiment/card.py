"""Provenance — the run card, one committable Markdown record of what ran."""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml

from opencdarr import cache
from opencdarr.config import Config
from opencdarr.experiment.declaration import Axis, Fixed
from opencdarr.experiment.identity import CacheIdentityError, identity
from opencdarr.experiment.methods import Methods
from opencdarr.experiment.results import ExperimentResult


def _describe_axes(independent_vars: Mapping[str, Axis]) -> str:
    """The declaration as readable lines: each parameter, its role, and its levels."""
    lines = []
    for key, axis in independent_vars.items():
        if isinstance(axis, Fixed):
            lines.append(f"- `{key}`: Fixed({axis.value!r})")
        else:
            label = f" as `{axis.name}`" if axis.name else ""
            built = " via build()" if axis.build is not None else ""
            lines.append(f"- `{key}`: Sweep({list(axis.values)!r}){label}{built}")
    return "\n".join(lines)


def _write_card(
    result: ExperimentResult,
    independent_vars: Mapping[str, Axis],
    methods: Methods,
    base_config: Config,
    card_dir: Path,
) -> Path:
    """Write one Markdown card describing the experiment, and return its path.

    Generalises :func:`opencdarr.experiment.run_one_experiment`'s card from one run to a whole
    sweep: the declaration with each parameter's role, the component identities, the backend, the
    seed, the code fingerprint, and the results table. Component identities are the same strings
    the cache keys on, so a card and a cache entry cannot disagree about what was run — and
    identity is best-effort here (a card should still be written for an unkeyable component), so a
    failure is recorded as such rather than aborting the write.
    """
    card_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S_%fZ")
    path = card_dir / f"{stamp}_seed{result.seed}.md"

    def described(value: Any) -> str:
        try:
            return identity(value)
        except CacheIdentityError as exc:
            return f"(no stable identity: {exc.args[0].split(chr(46))[0]})"

    config_yaml = yaml.safe_dump(dataclasses.asdict(base_config), sort_keys=False)
    rows = result.records()
    columns = list(rows[0]) if rows else []
    table = "\n".join(
        ["| " + " | ".join(columns) + " |", "|" + "---|" * len(columns)]
        + ["| " + " | ".join(_format(row[c]) for c in columns) + " |" for row in rows]
    )
    path.write_text(
        f"# Experiment {stamp}\n\n"
        f"- backend: `{result.backend}`\n"
        f"- seed: {result.seed}\n"
        f"- conditions: {len(result.conditions)}\n"
        f"- swept axes: {list(result.axes)}\n"
        f"- code_hash: {cache.code_fingerprint()}\n\n"
        f"## Declaration\n\n{_describe_axes(independent_vars)}\n\n"
        f"## Methods\n\n"
        + "\n".join(
            f"- {f.name}: `{described(getattr(methods, f.name))}`"
            for f in dataclasses.fields(methods)
        )
        + f"\n\n## Base config\n\n```yaml\n{config_yaml}```\n"
        f"\n## Results\n\n{table}\n"
    )
    return path


def _format(value: Any) -> str:
    return f"{value:.6g}" if isinstance(value, float) else str(value)
