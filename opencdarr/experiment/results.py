"""Results — the per-condition table and the metrics read off each cell."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from opencdarr.estimate.montecarlo import MonteCarloEstimate
from opencdarr.experiment.backends import IPS, Backend
from opencdarr.experiment.conditions import Condition


@dataclass(frozen=True)
class ExperimentResult:
    """One row per condition, plus the raw estimator result behind each.

    The columns adapt to the backend, because the two estimators do not report the same things: MC
    gives counts on a design-fixed denominator and a median achieved separation, IPS gives a
    replicated probability plus the per-shell survival and a collapse count. Forcing them into one
    schema would mean inventing a value for whichever half is absent.
    """

    backend: Backend
    seed: int
    conditions: tuple[Condition, ...]
    results: tuple[Any, ...]  # MonteCarloEstimate or RareEventEstimate, one per condition
    axes: tuple[str, ...]  # the swept column names, declaration order
    card_path: Path | None = None  # the provenance card, when one was written

    def records(self) -> list[dict[str, Any]]:
        """One dict per condition: its swept levels, then the backend's metrics."""
        return [
            {**condition.label, **_metrics(result)}
            for condition, result in zip(self.conditions, self.results, strict=True)
        ]

    def frame(self) -> Any:
        """:meth:`records` as a ``pandas.DataFrame``.

        ``pandas`` is imported here rather than at module scope: it is an optional extra (like
        ``matplotlib`` for :mod:`opencdarr.viz` and ``joblib`` for
        :mod:`opencdarr.estimate.parallel`), so a
        plain install stays numpy + pyyaml and :meth:`records` works without it.
        """
        try:
            import pandas as pd  # type: ignore[import-untyped]  # optional extra, no stubs
        except ModuleNotFoundError as exc:  # pragma: no cover - depends on the install
            raise ModuleNotFoundError(
                "frame() needs pandas (an optional extra: pip install 'opencdarr[examples]'). "
                "records() returns the same rows as plain dicts with no extra dependency."
            ) from exc
        return pd.DataFrame(self.records())

    def plot(self, metric: str = "p_los_run", *, ax: Any = None, log: bool | None = None) -> Any:
        """The response curve: the first swept axis on x, the rest as one line each.

        Layout comes from the axis roles, so nothing has to be restated: the first :class:`Sweep`
        becomes x and any further ones become the series. ``log`` defaults to a log y-axis for
        :class:`IPS` (a rare-event probability spans decades) and linear for :class:`MC`.

        Returns the :class:`~matplotlib.figure.Figure`, as :func:`opencdarr.viz.plot_pairwise`
        does, so a caller can save it or keep tweaking. Deliberately plain — no grid, no figure
        title — on the house convention that a figure carries axes and a legend, and the prose
        carries the rest.
        """
        import matplotlib.pyplot as plt

        if not self.axes:
            raise ValueError(
                "nothing to plot against: every parameter is Fixed, so there is one condition. "
                "Declare at least one Sweep, or read the single row from records()."
            )
        rows = self.records()
        if metric not in rows[0]:
            raise KeyError(f"no metric {metric!r} on this backend; have {sorted(rows[0])}")

        x_axis, *series_axes = self.axes
        fig = plt.figure(figsize=(6.4, 4.0)) if ax is None else ax.get_figure()
        ax = fig.add_subplot(111) if ax is None else ax

        # one line per combination of the remaining swept axes, in first-seen order
        groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
        for row in rows:
            groups.setdefault(tuple(row[a] for a in series_axes), []).append(row)

        for key, group in groups.items():
            group = sorted(group, key=lambda r: r[x_axis])
            xs = [r[x_axis] for r in group]
            label = ", ".join(f"{a}={k}" for a, k in zip(series_axes, key, strict=True))
            ax.plot(xs, [r[metric] for r in group], marker="o", ms=3, lw=1.6,
                              label=label or None)

        ax.set_xlabel(x_axis)
        ax.set_ylabel(metric)
        if log if log is not None else isinstance(self.backend, IPS):
            ax.set_yscale("log")
        if series_axes:
            ax.legend(frameon=False, loc="best")
        return fig

    def _repr_html_(self) -> str:
        """Show the results table when a notebook displays this object.

        Display only — the return value of :func:`run_experiment` stays an
        :class:`ExperimentResult`, because the table is a *reduction*: :meth:`cell` reaches the raw
        estimator result behind a row, and with it the per-encounter record every other metric is
        computed from. Handing back a bare frame would make ``pandas`` a hard dependency of the
        entry point and put ``min_seps`` out of reach.

        Falls back to plain text when ``pandas`` is absent, so displaying a result never raises on
        a core install.
        """
        head = (f"<p><code>{self.backend}</code> &middot; seed {self.seed} &middot; "
                f"{len(self.conditions)} condition(s)"
                + (f" &middot; axes {list(self.axes)}" if self.axes else "")
                + "</p>")
        try:
            return head + self.frame()._repr_html_()
        except ModuleNotFoundError:
            rows = "\n".join(str(r) for r in self.records())
            return head + f"<pre>{rows}</pre>"

    def cell(self, **levels: Any) -> Any:
        """The raw estimator result for the condition matching ``levels`` (exactly one must match).

        Raw on purpose: it is the material a metric nobody has written yet would be computed from,
        so it is handed back rather than reduced.
        """
        matches = [
            result for condition, result in zip(self.conditions, self.results, strict=True)
            if all(condition.label.get(k) == v for k, v in levels.items())
        ]
        if len(matches) != 1:
            raise KeyError(
                f"{levels} matches {len(matches)} conditions, expected exactly 1. "
                f"Swept axes are {list(self.axes)}."
            )
        return matches[0]

    def __len__(self) -> int:
        return len(self.conditions)


def _metrics(result: Any) -> dict[str, Any]:
    """The reported columns for one cell, per backend.

    ``median_min_sep`` is an MC column only, and deliberately so: it is an expectation over the
    *whole* encounter population, which is exactly what a splitting estimator cannot give. IPS
    discards the particles that fail to reach a shell and clones the survivors, so its cloud is a
    sample of the rare set, not of the population — a median over it would be the median given
    near-LoS, silently mislabelled. (Conditional-on-the-rare-set quantities *are* available from
    IPS; item 8 of the build order is where they belong.)
    """
    if isinstance(result, MonteCarloEstimate):
        return {
            "p_los_ac": result.p_los_ac,
            "p_los_run": result.p_los_run,
            "mean_k": result.mean_k,
            "median_min_sep": result.median_min_sep,
            "n_los": result.n_los,
            "n_encounters": result.n_encounters,
            "detection_rate": result.detection_rate,
        }
    return {
        # the ladder gives the per-run probability natively; the other two come from the tail leg
        "p_los_ac": result.p_los_ac,
        "p_los_run": result.p_los_run,
        "mean_k": result.mean_k,
        "n_lineages": result.n_lineages,
        "n_collapsed": result.n_collapsed,
        "reps": len(result.reps),
    }
