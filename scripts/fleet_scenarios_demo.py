"""Phase-6d demo: the four fleet scenarios side by side — cooperative avoidance and its one limit.

The scenario builders (:mod:`opencdarr.scenario`, Phase 6d) each place a fleet that, unresolved,
collides. Every aircraft flies a :class:`~opencdarr.autopilot.WaypointAutopilot` goto mission and
runs its own detect → resolve → recover against all the others (cooperative MVP + Past-CPA) — the
fleet loop of :func:`~opencdarr.fleet.run_fleet`, reimplemented here so the per-tick ground tracks
and separation history can be captured (``run_fleet`` returns only the scalar outcome).

Three of the four **clear** (min pairwise sep ≥ rpz): the two-aircraft swap, the eight-aircraft
opposite-start swap, and the 5° near-parallel crossing. The **converging ring** (all eight to the
*same* centre point) is the headline stress case — the goal itself is incompatible with separation
(eight aircraft cannot occupy one point), so the DAA can only *mitigate* it: min-sep lifts from
~0 m to well below rpz but stably held. A genuine, documented limit — the symmetric superconflict.

One combined figure, one row per scenario: left = cooperative ground tracks (▲ start, ★ waypoint);
right = minimum pairwise separation, cooperative vs unresolved, against the rpz line.

Writes ``vault/observations/img/fleet-scenarios.png``.

    PYTHONPATH=. python scripts/fleet_scenarios_demo.py
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr import scenario as sc  # noqa: E402
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import Multirotor  # noqa: E402
from opencdarr.kinematics import relative_enu  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.separation import INACTIVE, FleetMemory, SeparationManager  # noqa: E402
from opencdarr.state import AircraftState, DesiredVelocity  # noqa: E402

LAT0, LON0, DT, BCAST = 52.0, 4.0, 0.5, 1.0
RPZ, LOOKAHEAD, SPEED, MARGIN, CAPTURE = 50.0, 30.0, 10.0, 1.1, 60.0
T_MAX, DONE_TIMEOUT = 400.0, 10.0
_MR = Multirotor()


class _Sim:
    """One scenario's captured run: ground tracks, min-pairwise-sep history, and the outcome."""

    def __init__(self, tracks: list[list[tuple[float, float]]], min_sep: list[float]) -> None:
        self.tracks = tracks
        self.min_sep = min_sep

    @property
    def worst(self) -> float:
        return min(self.min_sep)


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _pairwise_min_sep(states: list[AircraftState]) -> float:
    return min(
        geo.qdrdist(states[a].lat, states[a].lon, states[b].lat, states[b].lon)[1]
        for a in range(len(states)) for b in range(a + 1, len(states))
    )


def _all_clear(states: list[AircraftState], mems: list[FleetMemory]) -> bool:
    """The ``run_fleet`` termination test: no one resolving, every pair past-CPA and separated."""
    if any(m.resolving for m in mems):
        return False
    n = len(states)
    for i in range(n):
        for j in range(i + 1, n):
            rel = relative_enu(states[i], states[j])
            diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0
            if not (diverging and rel.dist >= RPZ):
                return False
    return True


def simulate(fleet: sc.FleetScenario, resolver: ConflictResolver | None) -> _Sim:
    """Mirror ``run_fleet`` (multirotor ⇒ identity adapter) while capturing tracks + min-sep."""
    n = len(fleet)
    aps = [WaypointAutopilot(Mission(goto=target), cruise_airspeed=SPEED, capture_radius=CAPTURE)
           for _, target in fleet]
    states = [replace(s, desired=DesiredVelocity.from_track_speed(s.trk, s.gs)) for s, _ in fleet]
    gms = [GuidanceMemory() for _ in range(n)]
    mems: list[FleetMemory] = [INACTIVE for _ in range(n)]
    sep = SeparationManager()
    det = StateBased()
    rec = PastCPA(bouncing_guard=True) if resolver else None
    cmds = [aps[i].step(states[i], gms[i], M600)[0] for i in range(n)]

    tracks: list[list[tuple[float, float]]] = [[] for _ in range(n)]
    min_sep: list[float] = []
    t, next_bcast, done_timer = 0.0, 0.0, 0.0

    while t < T_MAX:
        for i in range(n):
            tracks[i].append(_enu(states[i].lat, states[i].lon))
        min_sep.append(_pairwise_min_sep(states))
        if t + 1e-9 >= next_bcast:
            for i in range(n):
                nom, gms[i] = aps[i].step(states[i], gms[i], M600)
                perceived = [replace(states[j], desired=None) for j in range(n) if j != i]
                cmds[i], mems[i] = sep.step(states[i], perceived, nom, mems[i], RPZ, LOOKAHEAD,
                                            det, resolver, rec, None)
            next_bcast += BCAST
        states = [_MR.step(states[i], cmds[i], M600, DT) for i in range(n)]
        t += DT
        done_timer = done_timer + DT if _all_clear(states, mems) else 0.0
        if done_timer >= DONE_TIMEOUT:
            break
    return _Sim(tracks, min_sep)


# (fleet, title) for each scenario — the builders are pure; converging_ring is the headline case.
_SCENARIOS: tuple[tuple[sc.FleetScenario, str], ...] = (
    (sc.swap_pair(), "1 — two-aircraft swap (head-on)"),
    (sc.swap_ring(8), "2 — eight-aircraft swap (opposite starts)"),
    (sc.converging_ring(8), "3 — eight-aircraft converging ring (superconflict)"),
    (sc.near_parallel(), "4 — near-parallel 5° crossing"),
)


def _plot_tracks(ax: Axes, fleet: sc.FleetScenario, sim: _Sim) -> None:
    n = len(fleet)
    cmap = plt.get_cmap("hsv")
    for k in range(n):
        col = cmap(k / n)
        xs, ys = [p[0] for p in sim.tracks[k]], [p[1] for p in sim.tracks[k]]
        ax.plot(xs, ys, color=col, lw=1.8)
        ax.scatter([xs[0]], [ys[0]], color=col, marker="^", s=42, zorder=5)
        tgt = _enu(*fleet[k][1])
        ax.scatter([tgt[0]], [tgt[1]], color=col, marker="*", s=120, zorder=5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.grid(True, alpha=0.3)


def _plot_sep(ax: Axes, coop: _Sim, base: _Sim) -> None:
    ax.plot([i * DT for i in range(len(base.min_sep))], base.min_sep,
            color="tab:gray", lw=1.6, ls=":", label="unresolved")
    ax.plot([i * DT for i in range(len(coop.min_sep))], coop.min_sep,
            color="tab:green", lw=2.2, label="cooperative")
    ax.axhline(RPZ, color="tab:red", ls="--", lw=1.2, label=f"rpz = {RPZ:.0f} m")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("min pairwise sep [m]")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")


def main() -> None:
    fig, axes = plt.subplots(len(_SCENARIOS), 2, figsize=(12.5, 4.0 * len(_SCENARIOS)))
    for row, (fleet, title) in enumerate(_SCENARIOS):
        coop = simulate(fleet, MVP(margin=MARGIN))
        base = simulate(fleet, None)
        cleared = coop.worst >= RPZ
        verdict = "clear" if cleared else "mitigated (< rpz)"
        print(f"{title}: cooperative min pairwise sep = {coop.worst:.1f} m "
              f"(unresolved {base.worst:.1f} m) -> {verdict}")
        _plot_tracks(axes[row][0], fleet, coop)
        axes[row][0].set_title(f"Scenario {title}", fontsize=10)
        _plot_sep(axes[row][1], coop, base)
        tag = "clears rpz" if cleared else "mitigated, cannot reach rpz"
        axes[row][1].set_title(f"min pairwise separation — {coop.worst:.1f} m ({tag})",
                               fontsize=10)

    fig.suptitle(
        "Phase 6d: the four fleet scenarios — cooperative MVP + Past-CPA (▲ start, ★ waypoint)\n"
        "Three clear (min-sep ≥ rpz); the converging ring (all → one centre) only mitigates",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/fleet-scenarios.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
