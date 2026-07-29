"""The README's overview figure: one crossing encounter, four times, each layer harder than before.

Four panels on one row, drawn from the real `run_fleet` loop (the same code the
`examples/handbook/a_first_run.ipynb` walkthrough runs, panel by panel):

  1. ``perfect`` — one deterministic crossing: MVP resolves, Past-CPA recovers, nothing is noisy.
  2. ``uncertainty`` — the same encounter under GNSS noise and a lossy, laggy datalink, N runs.
  3. ``waypoint`` — as above, but the multirotor now flies a mission leg instead of cruising.
  4. ``wind`` — as above, in a steady 10 m/s wind (drawn as a faint background vector field).

Every panel shares one set of axis limits, so the spread you see between them is the spread the
model actually produces. Writes ``docs/img/overview.png``.

    PYTHONPATH=. python scripts/readme_overview.py [N_RUNS]
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo, rng  # noqa: E402
from opencdarr.autopilot import CruiseAutopilot, WaypointAutopilot  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import Comm, GnssNavigation, lognormal_latency  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import FixedWing, Multirotor  # noqa: E402
from opencdarr.fleet import Agent, FleetOutcome, run_fleet  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import M600, SMALL_FIXEDWING  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.viz import extract_tracks, plot_pairwise_montecarlo  # noqa: E402
from opencdarr.wind import NO_WIND, WindField  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "docs" / "img" / "overview.png"
RPZ, LOOKAHEAD = 50.0, 20.0
POS_CI95, VEL_CI95 = 15.0, 1.5  # GNSS 95% radial accuracy, position [m] / velocity [m/s]
SEED = 42
WIND = WindField.from_met(coming_from_deg=30.0, speed=10.0)
WINDCOL = "#f2a6a6"
PAD = 80.0  # m of breathing room around the widest track, for the shared axis limits


def _agents(*, waypoint: bool, noisy: bool) -> list[Agent]:
    """The crossing: a multirotor heading north, a fixed-wing spawned straight into conflict.

    ``waypoint`` swaps the multirotor's cruise for a mission leg 75 s ahead; ``noisy`` stamps the
    CI95 uncertainties that switch the navigation model on.
    """
    copter = AircraftState(id="COPTER", lat=52.0, lon=4.0, trk=0.0, gs=18.0, yaw=0.0)
    plane = create_conflict(copter, intr_id="PLANE", dpsi=90.0, dcpa=0.0,
                            tlos=30.0, rpz=RPZ, gs_intr=15.0, side=1)
    if waypoint:
        goto = geo.forward(copter.lat, copter.lon, copter.trk, copter.gs * 75.0)
        copter_ap: CruiseAutopilot | WaypointAutopilot = WaypointAutopilot(Mission(goto=goto))
    else:
        copter_ap = CruiseAutopilot(copter.trk, copter.gs)
    agents = [Agent(copter, M600, Multirotor(), copter_ap),
              Agent(plane, SMALL_FIXEDWING, FixedWing(), CruiseAutopilot(plane.trk, plane.gs))]
    if not noisy:
        return agents
    return [replace(a, state=replace(a.state, pos_ci95=POS_CI95, vel_ci95=VEL_CI95))
            for a in agents]


# A directed link: COPTER's broadcasts reach PLANE 90% of the time, PLANE's reach COPTER 60% —
# asymmetric, and every delivery arrives late by a heavy-tailed draw.
COMM = Comm(reception_prob={("COPTER", "PLANE"): 0.9, ("PLANE", "COPTER"): 0.6},
            latency=lognormal_latency(median=0.5, sigma=0.4))


def run_perfect() -> FleetOutcome:
    """One run, perfect information: no navigation noise, no datalink, no wind."""
    return run_fleet(
        _agents(waypoint=False, noisy=False),
        rpz=RPZ, t_lookahead=LOOKAHEAD, dt=0.1,
        detector=StateBased(), resolver=MVP(), recovery=PastCPA(bouncing_guard=True),
        done_timeout=10.0, record=True,
    )


def run_sweep(n_runs: int, *, waypoint: bool, wind: WindField | None = None) -> list[FleetOutcome]:
    """``n_runs`` noise realisations of the same encounter, one reproducible seed stream each.

    One root seed spawns a per-run pair of substreams — navigation and communication — so any run
    can be replayed on its own (ADR 0006).
    """
    agents = _agents(waypoint=waypoint, noisy=True)
    seeds = (rng.spawn(sub, 2) for sub in rng.spawn(rng.root_seed_sequence(SEED), n_runs))
    return [
        run_fleet(
            agents, rpz=RPZ, t_lookahead=LOOKAHEAD, dt=0.2,
            wind=wind if wind is not None else NO_WIND,
            detector=StateBased(), resolver=MVP(), recovery=PastCPA(bouncing_guard=True),
            navigation=GnssNavigation(), rng=rng.generator(nav_seq),
            communication=COMM, comm_rng=rng.generator(comm_seq),
            stop_within=50.0 if waypoint else 100.0, done_timeout=60.0, record=True,
        )
        for nav_seq, comm_seq in seeds
    ]


def _wind_field(ax: plt.Axes, wind: WindField, xlim: tuple[float, float],
                ylim: tuple[float, float], n: int = 7) -> None:
    """A faint background velocity-vector field over the panel; the arrows point downwind."""
    xx, yy = np.meshgrid(np.linspace(*xlim, n), np.linspace(*ylim, n))
    w_east, w_north = wind.components()
    length = (xlim[1] - xlim[0]) / (n - 1) * 0.6
    u = np.full_like(xx, w_east / wind.speed * length)
    v = np.full_like(yy, w_north / wind.speed * length)
    ax.quiver(xx, yy, u, v, color=WINDCOL, alpha=0.35, scale=1.0, scale_units="xy",
              angles="xy", width=0.010, zorder=0)


def _limits(panels: list[list[FleetOutcome]]) -> tuple[tuple[float, float], tuple[float, float]]:
    """One (xlim, ylim) that contains every track of every panel, padded — so the four panels are
    read against the same ruler and a wider spread *looks* wider."""
    points = np.vstack([t.tracks[k]
                        for runs in panels for run in runs
                        for t in [extract_tracks(run)] for k in range(len(t.tracks))])
    lo, hi = points.min(axis=0) - PAD, points.max(axis=0) + PAD
    return (float(lo[0]), float(hi[0])), (float(lo[1]), float(hi[1]))


def draw(panels: list[tuple[str, list[FleetOutcome]]], out: Path) -> None:
    """The four panels on one row, sharing colours, axis limits, and a single legend."""
    xlim, ylim = _limits([runs for _, runs in panels])
    fig, axes = plt.subplots(1, 4, figsize=(17, 5.0))
    for ax, (title, runs) in zip(axes, panels, strict=True):
        plot_pairwise_montecarlo(runs, ax=ax, title=title,
                                 alpha=1.0 if len(runs) == 1 else None)
        if len(runs) == 1:  # a sweep's P(LoS)/IPR header says nothing about a single run
            ax.set_title(f"{title}\nCPA = {runs[0].min_sep:.0f} m")
        ax.set_xlabel("east [m]")  # the helper warns of an exaggerated axis; ours is true to scale
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
    _wind_field(axes[-1], WIND, xlim, ylim)
    axes[-1].set_xlim(*xlim)  # quiver's own autoscale can nudge the limits — pin them back
    axes[-1].set_ylim(*ylim)
    for ax in axes[1:]:  # one legend for the row; the colours are the same in every panel
        ax.get_legend().remove()
    for ax in axes[1:]:
        ax.set_ylabel("")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"wrote {out}")


def main() -> None:
    n_runs = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    panels = [
        ("perfect information", [run_perfect()]),
        (f"+ GNSS & datalink noise ({n_runs} runs)", run_sweep(n_runs, waypoint=False)),
        ("+ waypoint", run_sweep(n_runs, waypoint=True)),
        ("+ wind (10 m/s)", run_sweep(n_runs, waypoint=True, wind=WIND)),
    ]
    draw(panels, OUT)


if __name__ == "__main__":
    main()
