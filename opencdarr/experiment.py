"""The experiment entry point: ``config + seed -> IPR``, with a provenance card.

``run_one_experiment`` is the single top-level a newcomer can read straight through: resolve
the CDR components named in the config, run the plain-MC estimate, and write one provenance
card. Effects (the file write) live only here; the estimator stays pure. A code-hash stamp is
deferred (recorded as such on the card for now).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path

import yaml

from opencdarr.cd import StateBased
from opencdarr.cd.base import ConflictDetector
from opencdarr.config import Config
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.estimator import IPRResult, estimate_ipr
from opencdarr.performance import M600, Performance

# Proto-registry: name -> component. A full registry (config-selectable plugins) is deferred
# to the first outside contribution (design_brief.md).
_PERF = {"M600": M600}


def _make_perf(name: str) -> Performance:
    try:
        return _PERF[name]
    except KeyError:
        raise ValueError(f"unknown aircraft_type {name!r}") from None


def _make_detector(name: str) -> ConflictDetector:
    if name == "statebased":
        return StateBased()
    raise ValueError(f"unknown detector {name!r}")


def _make_resolver(name: str | None, margin: float) -> ConflictResolver | None:
    if name is None:
        return None
    if name == "mvp":
        return MVP(margin=margin)
    if name == "vo":
        return VO(margin=margin)
    raise ValueError(f"unknown resolver {name!r}")


def _make_recovery(name: str | None, bouncing_guard: bool) -> RecoveryCriterion | None:
    if name is None:
        return None
    if name == "pastcpa":
        return PastCPA(bouncing_guard=bouncing_guard)
    raise ValueError(f"unknown recovery {name!r}")


@dataclass(frozen=True)
class ExperimentResult:
    ipr: IPRResult
    card_path: Path | None


def run_one_experiment(
    config: Config, *, card_dir: Path | None = Path("vault/experiments")
) -> ExperimentResult:
    """Run one experiment end to end; write a provenance card unless ``card_dir`` is None."""
    perf = _make_perf(config.scenario.aircraft_type)
    detector = _make_detector(config.methods.detection)
    resolver = _make_resolver(config.methods.resolution, config.methods.margin)
    recovery = _make_recovery(config.methods.recovery, config.methods.bouncing_guard)

    result = estimate_ipr(config, perf, detector, resolver, recovery)

    card_path = _write_card(config, result, card_dir) if card_dir is not None else None
    return ExperimentResult(ipr=result, card_path=card_path)


def _write_card(config: Config, result: IPRResult, card_dir: Path) -> Path:
    card_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S_%fZ")
    path = card_dir / f"{stamp}_seed{config.seed}.md"
    config_yaml = yaml.safe_dump(asdict(config), sort_keys=False)
    path.write_text(
        f"# Experiment {stamp}\n\n"
        f"- seed: {config.seed}\n"
        f"- n_encounters: {config.n_encounters}\n"
        f"- IPR: {result.ipr:.6f}  ({result.n_los}/{result.n_conflict} LoS)\n"
        f"- code_hash: (deferred)\n\n"
        f"## Config\n\n```yaml\n{config_yaml}```\n"
    )
    return path
