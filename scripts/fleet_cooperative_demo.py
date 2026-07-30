"""Phase-6b demo: an 8-aircraft cooperative fleet — every aircraft avoids and reaches its waypoint.

Eight aircraft sit uniformly on a ring; each flies to the **diametrically-opposite** start
(Phase-6 scenario 2), so all eight converge through the centre — a superconflict that, unresolved,
collides. Each has a goto mission flown by a :class:`WaypointAutopilot` (Phase 4d), and every plane
runs its own detect → resolve → recover through the fleet loop (cooperative MVP + Past-CPA), so the
fleet bows around the centre and every aircraft **resumes navigation to its waypoint**.

Left: ground tracks (dotted = unresolved straight lines that collide at the centre; solid = the
cooperative tracks reaching their opposite-side waypoints ★). Right: minimum pairwise separation,
resolved vs unresolved.

Writes ``vault/observations/img/fleet-cooperative-ring.png``.

    PYTHONPATH=. python scripts/fleet_cooperative_demo.py
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
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.kinematics import Multirotor  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager  # noqa: E402
from opencdarr.state import AircraftState, DesiredVelocity  # noqa: E402

LAT0, LON0, DT, BCAST, T_MAX = 52.0, 4.0, 0.2, 1.0, 300.0
RPZ, LOOKAHEAD, N, RADIUS, SPEED, MARGIN = 50.0, 30.0, 8, 1500.0, 10.0, 1.1
CAPTURE = 60.0


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _starts_and_waypoints() -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    starts = [geo.forward(LAT0, LON0, 360.0 * k / N, RADIUS) for k in range(N)]
    waypoints = [starts[(k + N // 2) % N] for k in range(N)]  # diametrically-opposite start
    return starts, waypoints


def simulate(
    resolver: ConflictResolver | None,
) -> tuple[list[list[tuple[float, float]]], list[float]]:
    """Cooperative capture with per-aircraft goto missions (mirrors the fleet loop)."""
    starts, wps = _starts_and_waypoints()
    states = []
    for k in range(N):
        trk = geo.qdrdist(starts[k][0], starts[k][1], wps[k][0], wps[k][1])[0]
        s = AircraftState(id=f"A{k}", lat=starts[k][0], lon=starts[k][1], trk=trk, gs=SPEED)
        states.append(replace(s, desired=DesiredVelocity.from_track_speed(trk, SPEED)))
    aps = [WaypointAutopilot(Mission(goto=(wps[k][0], wps[k][1])), cruise_airspeed=SPEED,
                             capture_radius=CAPTURE) for k in range(N)]
    gms = [GuidanceMemory() for _ in range(N)]
    mems = [INACTIVE for _ in range(N)]
    sep = SeparationManager()
    det, rec = StateBased(), (PastCPA(bouncing_guard=True) if resolver else None)
    cmds = [aps[k].step(states[k], gms[k], M600)[0] for k in range(N)]

    tracks: list[list[tuple[float, float]]] = [[] for _ in range(N)]
    min_sep: list[float] = []
    t, nb = 0.0, 0.0

    def dwp(k: int) -> float:
        return geo.qdrdist(states[k].lat, states[k].lon, wps[k][0], wps[k][1])[1]

    while t < T_MAX:
        for k in range(N):
            tracks[k].append(_enu(states[k].lat, states[k].lon))
        min_sep.append(min(
            geo.qdrdist(states[a].lat, states[a].lon, states[b].lat, states[b].lon)[1]
            for a in range(N) for b in range(a + 1, N)
        ))
        if t + 1e-9 >= nb:
            for k in range(N):
                nom, gms[k] = aps[k].step(states[k], gms[k], M600)
                perceived = [states[j] for j in range(N) if j != k]
                cmds[k], mems[k] = sep.step(states[k], perceived, nom, mems[k], RPZ, LOOKAHEAD,
                                            det, resolver, rec, None)
            nb += BCAST
        states = [Multirotor().step(states[k], cmds[k], M600, DT) for k in range(N)]
        t += DT
        if resolver is not None and all(dwp(k) < CAPTURE for k in range(N)):
            break
    return tracks, min_sep


def plot(coop: list[list[tuple[float, float]]], coop_sep: list[float],
         base_sep: list[float], out: Path) -> None:
    _, wps = _starts_and_waypoints()
    wps_enu = [_enu(w[0], w[1]) for w in wps]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 6.4))

    cmap = plt.get_cmap("hsv")
    for k in range(N):
        col = cmap(k / N)
        a1.plot([p[0] for p in coop[k]], [p[1] for p in coop[k]], color=col, lw=2.0)
        a1.scatter([coop[k][0][0]], [coop[k][0][1]], color=col, marker="^", s=45, zorder=5)
        a1.scatter([wps_enu[k][0]], [wps_enu[k][1]], color=col, marker="*", s=120, zorder=5)
    a1.scatter([0], [0], color="k", marker="x", s=55, zorder=6)
    a1.set_aspect("equal", adjustable="box")
    a1.set_xlim(-RADIUS * 1.15, RADIUS * 1.15)
    a1.set_ylim(-RADIUS * 1.15, RADIUS * 1.15)
    a1.set_xlabel("East [m]")
    a1.set_ylabel("North [m]")
    a1.set_title(f"{N} aircraft each cross to the opposite start (▲ start, ★ waypoint)")
    a1.grid(True, alpha=0.3)

    tb = [i * DT for i in range(len(base_sep))]
    tc = [i * DT for i in range(len(coop_sep))]
    a2.plot(tb, base_sep, color="tab:gray", lw=1.8, ls=":", label="unresolved (collides)")
    a2.plot(tc, coop_sep, color="tab:green", lw=2.2, label="cooperative (all avoid)")
    a2.axhline(RPZ, color="tab:red", ls="--", lw=1.2, label=f"rpz = {RPZ:.0f} m")
    a2.set_xlabel("t [s]")
    a2.set_ylabel("min pairwise separation [m]")
    a2.set_title("Minimum separation across all pairs")
    a2.grid(True, alpha=0.3)
    a2.legend(fontsize=9)

    fig.suptitle(
        "Phase 6b: an 8-aircraft cooperative fleet — every aircraft resolves against all the "
        "others and reaches its waypoint (run_fleet, MVP + Past-CPA)",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    coop, coop_sep = simulate(MVP(margin=MARGIN))
    _, base_sep = simulate(None)
    verdict = "clear" if min(coop_sep) >= RPZ else "LOSS"
    print(f"cooperative min pairwise sep = {min(coop_sep):.1f} m (rpz {RPZ:.0f}) -> {verdict}; "
          f"reached at t = {len(coop_sep) * DT:.0f} s")
    print(f"unresolved  min pairwise sep = {min(base_sep):.1f} m")
    img = "vault/observations/img/fleet-cooperative-ring.png"
    out = Path(__file__).resolve().parents[1] / img
    plot(coop, coop_sep, base_sep, out)


if __name__ == "__main__":
    main()
