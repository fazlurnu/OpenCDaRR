"""Phase-6 demo: the ±60° three-aircraft conflict — all three avoid *and reach their waypoints*.

The geometry from ``test_multi_intruder._two_symmetric`` (an ownship heading north, two intruders
crossing from ±60°, all aimed at one point), but now each aircraft has a **destination waypoint**
2.5 km ahead along its original track, flown with a :class:`WaypointAutopilot` (Phase 4d). Every
aircraft runs its own detect → resolve → recover (cooperative MVP + Past-CPA) through the fleet,
so all three manoeuvre around the shared conflict and then **resume navigation to their waypoints**
— "avoid, then continue to the destination", the behaviour a route-flying fleet has.

Two things this fixes over a naive cruise-only version, matching BlueSky's ASAS:
  * aircraft have **routes** and resume to them (BlueSky's ``pastcpa`` directs a recovered aircraft
    to its next waypoint) — without a destination, a cruise-only aircraft flies off after avoiding;
  * a **sensible look-ahead** — a very long horizon makes a symmetric conflict over-react and
    *livelock* (two aircraft chase each other away and never recover); ~30 s here resolves cleanly.

Writes ``vault/observations/img/three-aircraft-cooperative.png``.

    PYTHONPATH=. python scripts/three_aircraft_demo.py
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP, VO  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import Multirotor  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager  # noqa: E402
from opencdarr.state import AircraftState, DesiredVelocity  # noqa: E402

LAT0, LON0, DT, BCAST, T_MAX = 52.0, 4.0, 0.2, 1.0, 360.0
RPZ, LOOKAHEAD, SPEED, MARGIN = 50.0, 30.0, 10.0, 1.05
WP_DIST, CAPTURE = 2500.0, 60.0
_MR = Multirotor()
_COLORS = ("tab:orange", "tab:blue", "tab:green")
_LABELS = ("OWN (north)", "I1 (+60°)", "I2 (−60°)")


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _fleet() -> list[AircraftState]:
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=SPEED)
    i1 = create_conflict(own, intr_id="I1", dpsi=60.0, dcpa=0.0, tlos=50.0, rpz=RPZ, side=1)
    i2 = create_conflict(own, intr_id="I2", dpsi=300.0, dcpa=0.0, tlos=50.0, rpz=RPZ, side=-1)
    return [own, i1, i2]


def simulate(resolver: ConflictResolver) -> tuple[list[list[tuple[float, float]]],
                                                  list[list[float]], list[float],
                                                  list[tuple[float, float]]]:
    """Cooperative capture with per-aircraft goto missions (mirrors the fleet loop)."""
    fleet = _fleet()
    wps = [geo.forward(s.lat, s.lon, s.trk, WP_DIST) for s in fleet]
    wps_enu = [_enu(w[0], w[1]) for w in wps]
    states = [replace(s, desired=DesiredVelocity.from_track_speed(s.trk, s.gs)) for s in fleet]
    aps = [WaypointAutopilot(Mission(goto=(wps[i][0], wps[i][1])), cruise_airspeed=SPEED,
                             capture_radius=CAPTURE) for i in range(3)]
    gms = [GuidanceMemory() for _ in range(3)]
    mems = [INACTIVE for _ in range(3)]
    sep = SeparationManager()
    det, res, rec = StateBased(), resolver, PastCPA(bouncing_guard=True)
    cmds = [aps[i].step(states[i], gms[i], M600)[0] for i in range(3)]

    tracks: list[list[tuple[float, float]]] = [[] for _ in range(3)]
    pair_sep: list[list[float]] = [[], [], []]
    times: list[float] = []
    t, nb = 0.0, 0.0

    def dwp(i: int) -> float:
        return geo.qdrdist(states[i].lat, states[i].lon, wps[i][0], wps[i][1])[1]

    while t < T_MAX:
        for i in range(3):
            tracks[i].append(_enu(states[i].lat, states[i].lon))
        for k, (i, j) in enumerate(((0, 1), (0, 2), (1, 2))):
            pair_sep[k].append(geo.qdrdist(states[i].lat, states[i].lon,
                                           states[j].lat, states[j].lon)[1])
        times.append(t)
        if t + 1e-9 >= nb:
            for i in range(3):
                nom, gms[i] = aps[i].step(states[i], gms[i], M600)
                perceived = [states[j] for j in range(3) if j != i]
                cmds[i], mems[i] = sep.step(states[i], perceived, nom, mems[i], RPZ, LOOKAHEAD,
                                            det, res, rec, None)
            nb += BCAST
        states = [_MR.step(states[i], cmds[i], M600, DT) for i in range(3)]
        t += DT
        if all(dwp(i) < CAPTURE for i in range(3)):
            break
    return tracks, pair_sep, times, wps_enu


def plot(coop: list[list[tuple[float, float]]], coop_sep: list[list[float]], times: list[float],
         wps: list[tuple[float, float]], name: str, out: Path) -> None:
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 6.4))

    for k in range(3):
        a1.plot([p[0] for p in coop[k]], [p[1] for p in coop[k]], color=_COLORS[k], lw=2.4,
                label=_LABELS[k])
        a1.scatter([coop[k][0][0]], [coop[k][0][1]], color=_COLORS[k], marker="^", s=60, zorder=5)
        a1.scatter([wps[k][0]], [wps[k][1]], color=_COLORS[k], marker="*", s=140, zorder=5)
    a1.scatter([0], [0], color="k", marker="x", s=55, zorder=6, label="conflict point")
    a1.set_aspect("equal", adjustable="datalim")
    a1.set_xlabel("East [m]")
    a1.set_ylabel("North [m]")
    a1.set_title("All three avoid, then reach their waypoints (▲ start, ★ waypoint)")
    a1.grid(True, alpha=0.3)
    a1.legend(fontsize=8, loc="best")

    pair_labels = ("OWN–I1", "OWN–I2", "I1–I2")
    pair_cols = ("tab:purple", "tab:brown", "tab:cyan")
    for k in range(3):
        a2.plot(times, coop_sep[k], color=pair_cols[k], lw=2.0, label=pair_labels[k])
    a2.axhline(RPZ, color="tab:red", ls="--", lw=1.2, label=f"rpz = {RPZ:.0f} m")
    a2.set_ylim(0, 400)
    a2.set_xlabel("t [s]")
    a2.set_ylabel("pairwise separation [m]")
    a2.set_title("The three pairwise separations — all clear rpz")
    a2.grid(True, alpha=0.3)
    a2.legend(fontsize=8)

    fig.suptitle(
        f"Phase 6: the ±60° three-aircraft conflict — all three avoid and continue to their "
        f"waypoints (cooperative {name} + Past-CPA, WaypointAutopilot resume)",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    for name, resolver, suffix in (("MVP", MVP(margin=MARGIN), ""),
                                   ("VO", VO(margin=MARGIN), "-vo")):
        coop, coop_sep, times, wps = simulate(resolver)
        fleet_min = min(min(s) for s in coop_sep)
        print(f"{name}: min pairwise sep = {fleet_min:.1f} m (rpz {RPZ:.0f}) -> "
              f"{'clear' if fleet_min >= RPZ else 'LOSS'}; ran to t = {times[-1]:.0f} s")
        out = root / f"vault/observations/img/three-aircraft-cooperative{suffix}.png"
        plot(coop, coop_sep, times, wps, name, out)


if __name__ == "__main__":
    main()
