"""Plot a recorded fleet run — the separate viewer for :func:`opencdarr.fleet.run_fleet`'s log.

``run_fleet(..., record=True)`` returns raw data: a :class:`~opencdarr.fleet.FleetOutcome` whose
``frames`` is the states log (every :class:`~opencdarr.fleet.FleetState`). This module is the
*separate tool* that turns that log into a picture, so the simulator never imports a plotting
library and the core package stays numpy-only. ``matplotlib`` is imported lazily inside the drawing
functions; install it with the ``examples`` extra (``pip install "opencdarr[examples]"``).

Two steps, kept apart so the numbers are reusable without a figure:

1. :func:`extract_tracks` — turn the states log into plain arrays (ground tracks in a local
   east/north frame, the separation history, and when anyone is resolving).
2. :func:`plot_pairwise` — draw those arrays as a two-panel figure (ground tracks + separation vs
   time). Pairwise-focused, as its name says.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from opencdarr import geo
from opencdarr.fleet import FleetOutcome, FleetState, StatesLog
from opencdarr.relative import pairwise_min_sep

if TYPE_CHECKING:  # matplotlib is an optional (``examples``) dependency, imported lazily below.
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure

LatLon = tuple[float, float]

Run = FleetOutcome | StatesLog | Sequence[FleetState]


def _frames(run: Run) -> tuple[FleetState, ...]:
    """Accept a recorded :class:`FleetOutcome`, a :class:`StatesLog`, or a raw frame sequence."""
    if isinstance(run, FleetOutcome):
        if run.frames is None:
            raise ValueError("outcome has no states log — call run_fleet(..., record=True)")
        return run.frames.frames
    frames = run.frames if isinstance(run, StatesLog) else tuple(run)
    if not frames:
        raise ValueError("empty states log — call run_fleet(..., record=True) first")
    return frames


def _enu(origin: LatLon, lat: float, lon: float) -> tuple[float, float]:
    """(lat, lon) as (east, north) metres from ``origin`` — the frame the tracks are drawn in."""
    qdr, dist = geo.qdrdist(origin[0], origin[1], lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


@dataclass(frozen=True)
class Tracks:
    """A recorded run reduced to plottable arrays, in one local east/north frame."""

    ids: tuple[str, ...]  # aircraft ids, in track order
    times: NDArray[np.float64]  # (T,) seconds since the run start
    tracks: tuple[NDArray[np.float64], ...]  # per aircraft, (T, 2) east/north [m] from ``origin``
    separation: NDArray[np.float64]  # (T,) instantaneous min pairwise separation [m]
    resolving: NDArray[np.bool_]  # (T,) bool — was any aircraft avoiding this tick?
    origin: LatLon  # the (lat, lon) the east/north frame is centred on


def extract_tracks(run: Run) -> Tracks:
    """Reduce a states log to arrays: ground tracks, separation history, resolving flags.

    The east/north frame is centred on the first aircraft's initial position, so a pairwise
    encounter reads as two legs on a plane in metres. Everything here is recomputed from the true
    ``FleetState.states`` — no simulator internals, just geometry over the log.
    """
    frames = _frames(run)
    n = len(frames[0].states)
    ids = tuple(ac.id for ac in frames[0].states)
    origin: LatLon = (frames[0].states[0].lat, frames[0].states[0].lon)

    tracks = tuple(
        np.array([_enu(origin, f.states[k].lat, f.states[k].lon) for f in frames], dtype=float)
        for k in range(n)
    )
    times = np.array([f.t for f in frames], dtype=float)
    separation = np.array([pairwise_min_sep(f.states) for f in frames], dtype=float)
    resolving = np.array([any(m.resolving for m in f.mems) for f in frames], dtype=bool)
    return Tracks(ids=ids, times=times, tracks=tracks, separation=separation,
                  resolving=resolving, origin=origin)


def plot_pairwise(
    run: Run,
    *,
    rpz: float | None = None,
    title: str | None = None,
    axes: tuple[Axes, Axes] | None = None,
) -> Figure:
    """Draw a recorded run as ground tracks (left) and separation vs time (right).

    Pass ``rpz`` to mark the protected zone on the separation panel (and shade any loss of
    separation). ``axes`` lets a notebook place the two panels into an existing figure; omit it to
    get a fresh one. Returns the :class:`~matplotlib.figure.Figure` for saving or further tweaking.
    """
    import matplotlib.pyplot as plt

    t = extract_tracks(run)
    if axes is None:
        fig, (ax_xy, ax_sep) = plt.subplots(1, 2, figsize=(11, 4.5))
    else:
        ax_xy, ax_sep = axes
        fig = ax_xy.get_figure()

    # Left: ground tracks, start (o) to end (x), in metres.
    for ac_id, track in zip(t.ids, t.tracks, strict=True):
        ax_xy.plot(track[:, 0], track[:, 1], lw=1.6, label=ac_id)
        ax_xy.plot(track[0, 0], track[0, 1], marker="o", ms=6, color=ax_xy.lines[-1].get_color())
        ax_xy.plot(track[-1, 0], track[-1, 1], marker="x", ms=7, color=ax_xy.lines[-1].get_color())
    ax_xy.set_xlabel("east [m]")
    ax_xy.set_ylabel("north [m]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(frameon=False, loc="best")

    # Right: instantaneous separation, the protected zone, and where avoidance was active.
    ax_sep.plot(t.times, t.separation, lw=1.6, color="tab:blue")
    sep_top = float(t.separation.max()) * 1.05
    if t.resolving.any():  # shade first, so the separation line stays on top
        ax_sep.fill_between(t.times, 0, sep_top, where=t.resolving, color="tab:orange",
                            alpha=0.12, label="resolving", step="mid")
    if rpz is not None:
        ax_sep.axhline(rpz, ls="--", lw=1.0, color="tab:red", label=f"rpz = {rpz:g} m")
        below = t.separation < rpz
        if below.any():
            ax_sep.fill_between(t.times, 0, t.separation, where=below, color="tab:red", alpha=0.15)
    ax_sep.set_xlabel("time [s]")
    ax_sep.set_ylabel("separation [m]")
    ax_sep.set_ylim(0, sep_top)
    if rpz is not None or t.resolving.any():
        ax_sep.legend(frameon=False, loc="best")

    if title is not None:
        fig.suptitle(title)
    fig.tight_layout()
    return fig


# Per-aircraft colours, indexed by track order — consistent across every overlaid run.
_AC_COLORS = ("tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown")


def _run_outcome(run: Run) -> tuple[bool, bool, float]:
    """(conflict, los, min_sep) for one run — from its :class:`FleetOutcome` or its last frame."""
    if isinstance(run, FleetOutcome):
        return run.conflict, run.los, run.min_sep
    last = _frames(run)[-1]
    return last.conflict, last.los, last.min_sep


def _montecarlo_stats(runs: Sequence[Run]) -> str:
    """The header line for a sweep: ``P(LoS/run)`` and the median minimum separation."""
    outcomes = [_run_outcome(r) for r in runs]
    n = len(outcomes)
    n_los = sum(los for _, los, _ in outcomes)
    p_los = n_los / n if n else float("nan")
    med_min_sep = float(np.median([m for _, _, m in outcomes])) if n else float("nan")
    return f"P(LoS/run)={p_los:.2f}  median min sep={med_min_sep:.0f} m"


def plot_pairwise_montecarlo(
    runs: Sequence[Run],
    *,
    title: str | None = None,
    equal_aspect: bool = False,
    alpha: float | None = None,
    ax: Axes | None = None,
) -> Figure:
    """Overlay the ground tracks of a whole Monte-Carlo sweep, one faint line per run per aircraft.

    Each element of ``runs`` is a **recorded** run (``run_fleet(..., record=True)``, or its raw
    states log). Every aircraft keeps one colour across all runs, drawn at low opacity so the
    density of trajectories — where the fleet usually goes, and how far the tails spread — shows
    through. The title carries the sweep's summary: ``P(LoS/run)`` and the median minimum
    separation.

    By default the east axis is not held to the same scale as north (``equal_aspect=False``), so a
    narrow lateral spread is stretched to fill the width — the axis is labelled *(exaggerated)* to
    say so. ``alpha`` overrides the automatic per-line opacity (which fades as the sweep grows).
    Returns the :class:`~matplotlib.figure.Figure`.
    """
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    if not runs:
        raise ValueError("no runs to plot — pass a non-empty sequence of recorded runs")
    tracks = [extract_tracks(r) for r in runs]
    line_alpha = alpha if alpha is not None else float(np.clip(10.0 / len(tracks), 0.02, 0.4))

    if ax is None:
        fig, ax = plt.subplots(figsize=(6.5, 6.5))
    else:
        parent = ax.get_figure()
        assert parent is not None  # an Axes passed in always belongs to a figure
        fig = parent

    ids = tracks[0].ids
    for t in tracks:  # every run, every aircraft, same colour per aircraft
        for k, track in enumerate(t.tracks):
            ax.plot(track[:, 0], track[:, 1], lw=0.6, alpha=line_alpha,
                    color=_AC_COLORS[k % len(_AC_COLORS)])
    # Full-opacity proxy handles so the faint overlay still gets a readable legend.
    handles = [Line2D([], [], color=_AC_COLORS[k % len(_AC_COLORS)], lw=1.6, label=ac_id)
               for k, ac_id in enumerate(ids)]
    ax.legend(handles=handles, frameon=False, loc="best")

    ax.set_xlabel("east [m]" if equal_aspect else "east [m]  (exaggerated)")
    ax.set_ylabel("north [m]")
    if equal_aspect:
        ax.set_aspect("equal", adjustable="datalim")

    header = _montecarlo_stats(runs)
    ax.set_title(f"{title}\n{header}" if title else header)
    fig.tight_layout()
    return fig
