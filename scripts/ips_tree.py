"""Visualise the IPS splitting tree for one encounter geometry, with a loss-of-separation lineage.

A diagnostic / handbook figure for the rare-event estimator (ADR 0017). It re-runs the fixed-effort
multilevel splitting of :func:`opencdarr.ips.ips_once` for a single geometry — mirroring its seed
spawning exactly, so this *is* the real IPS run — but records each particle's separation-vs-time
path, the two aircraft ground tracks, and the resample parentage (which the estimator itself does
not keep). Then it traces one particle that reached ``rpz`` (a genuine LoS) back through its
parent-clones to ``t = 0`` and draws that lineage bold over the faint full tree, in two panels:

  (left)  separation vs time — the genealogy: a survivor crossing a shell clones (fresh noise); a
          killed particle bottoms out above the next shell (x at its closest approach);
  (right) the two aircraft ground tracks for every particle — the physical avoidance.

The highlighted path shows what splitting buys: a valid LoS trajectory assembled by reusing a
shared promising ancestor and only re-rolling the hard descent — not one simulated 10^5 times.

Examples
--------
    python scripts/ips_tree.py                                        # 90 deg, pos 10, margin 1.0
    python scripts/ips_tree.py --dpsi 90 --pos 10 --margin 1.0 -N 200
    python scripts/ips_tree.py --pos 20 --lookahead 30 -N 100 --out /tmp/tree.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import CnsStreams, GnssNavigation  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.fleet import Agent, FleetStreams, build_env  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.relative import pairwise_min_sep  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

SPEED, DT, T_MAX, DONE = 10.2889, 0.5, 250.0, 10.0  # 20 kts; fixed sim params
LAT0, LON0 = 52.0, 4.0
DEFAULT_LEVELS = [150.0, 100.0, 75.0, 66.0, 61.0, 58.0, 56.0, 54.0, 52.0, 51.0, 50.0]


def enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _streams(seq: np.random.SeedSequence) -> FleetStreams:
    """Mirror of :func:`opencdarr.ips._streams` — three substreams (nav, comm, broadcast)."""
    nav, comm, bc = seq.spawn(3)
    return FleetStreams(cns=CnsStreams(nav=generator(nav), comm=generator(comm)),
                        broadcast=generator(bc))


def build_pair(a: argparse.Namespace):
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=SPEED,
                        pos_ci95=a.pos, vel_ci95=a.pos / 10.0)
    intr = create_conflict(own, intr_id="INT", dpsi=a.dpsi, dcpa=0.0, tlos=a.tlos,
                           rpz=a.rpz, side=1)
    agents = [Agent(own, M600), Agent(intr, M600)]
    env = build_env(agents, rpz=a.rpz, t_lookahead=a.lookahead, dt=DT, detector=StateBased(),
                    resolver=MVP(margin=a.margin), recovery=PastCPA(bouncing_guard=True),
                    navigation=GnssNavigation(), t_max=T_MAX, done_timeout=DONE)
    return env, agents


def _sep(state) -> float:
    return pairwise_min_sep(state.states)


def _enu_pair(state):
    return (enu(state.states[0].lat, state.states[0].lon),
            enu(state.states[1].lat, state.states[1].lon))


def run_traced(a: argparse.Namespace) -> tuple[list, list, set, int | None]:
    """Run the instrumented IPS (mirrors ips_once, seed a.seed) and return
    ``(segments, survival, lineage, chosen)`` — segments carry paths + parentage; lineage is the
    set of segment indices on one traced-back LoS path (empty if none reached rpz)."""
    env, agents = build_pair(a)
    init_seq, evolve_seq = root_seed_sequence(a.seed).spawn(2)
    particles = []
    for _ in init_seq.spawn(a.n):  # geometry is fixed; the seed spawn matches ips_once's tree
        st = env.initial_state(agents)
        o, it = _enu_pair(st)
        particles.append({"state": st, "s_ts": (0.0, _sep(st)), "s_o": o, "s_i": it,
                          "parent": None})
    level_seqs = evolve_seq.spawn(len(a.levels))

    segments: list[dict] = []
    survival: list[float] = []
    for k, target in enumerate(a.levels):
        sub = level_seqs[k].spawn(a.n + 1)
        evolved = []
        for i, p in enumerate(particles):
            st = p["state"]
            ts, own, itr = [p["s_ts"]], [p["s_o"]], [p["s_i"]]
            streams = _streams(sub[i])
            while st.min_sep > target and not env.is_terminal(st):
                st = env.advance(st, streams)
                o, it = _enu_pair(st)
                ts.append((st.t, _sep(st)))
                own.append(o)
                itr.append(it)
            crossed = st.min_sep <= target
            segments.append({"ts": ts, "own": own, "int": itr, "level": k,
                             "crossed": crossed, "parent": p["parent"]})
            o, it = _enu_pair(st)
            evolved.append({"state": st, "crossed": crossed, "seg": len(segments) - 1,
                            "c_ts": (st.t, _sep(st)), "c_o": o, "c_i": it})
        survivors = [e for e in evolved if e["crossed"]]
        survival.append(len(survivors) / a.n)
        if not survivors:
            print(f"collapsed at level {k} (d={target}) — add particles or ease the shells")
            break
        idx = generator(sub[a.n]).integers(0, len(survivors), size=a.n)
        particles = [{"state": survivors[j]["state"], "s_ts": survivors[j]["c_ts"],
                      "s_o": survivors[j]["c_o"], "s_i": survivors[j]["c_i"],
                      "parent": survivors[j]["seg"]} for j in idx]

    lineage: set[int] = set()
    chosen: int | None = None
    if not a.no_highlight and len(survival) == len(a.levels):
        final = len(a.levels) - 1
        losers = [i for i, s in enumerate(segments) if s["level"] == final and s["crossed"]]
        if losers:
            chosen = min(losers, key=lambda i: segments[i]["ts"][-1][1])  # deepest LoS
            cur: int | None = chosen
            while cur is not None:
                lineage.add(cur)
                cur = segments[cur]["parent"]
    return segments, survival, lineage, chosen


def _descent(seg: dict) -> dict:
    """Killed segment truncated at its closest approach (drop the recede-to-timeout tail)."""
    if seg["crossed"]:
        return seg
    j = int(np.argmin([p[1] for p in seg["ts"]])) + 1
    return {**seg, "ts": seg["ts"][:j], "own": seg["own"][:j], "int": seg["int"][:j]}


def plot(a: argparse.Namespace, segments: list, survival: list, lineage: set,
         chosen: int | None, out: Path) -> None:
    fig, (axt, axg) = plt.subplots(1, 2, figsize=(15, 6.6))
    cmap = plt.get_cmap("viridis")
    nlev = len(a.levels)
    hl_c = "crimson"

    for idx, raw in enumerate(segments):
        seg = _descent(raw)
        hl = idx in lineage
        t = [p[0] for p in seg["ts"]]
        y = [p[1] for p in seg["ts"]]
        col = cmap(seg["level"] / max(nlev - 1, 1))
        if hl:
            axt.plot(t, y, color=hl_c, lw=2.6, zorder=6)
        else:
            axt.plot(t, y, color=col, lw=1.0, alpha=0.28, zorder=2)
        e = seg["ts"][-1]
        if seg["crossed"]:
            axt.plot(e[0], e[1], "o", color=hl_c if hl else col,
                     ms=6 if hl else 3.5, zorder=7 if hl else 3)
        elif not hl:
            axt.plot(e[0], e[1], "x", color="0.6", ms=4, mew=1.0, alpha=0.5, zorder=3)
    for d in a.levels:
        axt.axhline(d, color="0.8", ls=":", lw=0.8, zorder=1)
    axt.axhline(a.rpz, color="tab:red", ls="--", lw=1.6, zorder=4, label=f"rpz {a.rpz:.0f}m (LoS)")
    if lineage:
        axt.plot([], [], color=hl_c, lw=2.6, label="one LoS lineage")
    axt.set_xlabel("time [s]")
    axt.set_ylabel("pairwise separation [m]")
    axt.set_ylim(a.rpz - 2, max(a.levels) + 25)
    axt.set_title("splitting tree: separation vs time\n"
                  "dot = survivor reaches a shell and clones;  × = killed at its closest approach")
    axt.legend(loc="upper right", fontsize=9)

    for idx, raw in enumerate(segments):
        seg = _descent(raw)
        hl = idx in lineage
        oe, on = [p[0] for p in seg["own"]], [p[1] for p in seg["own"]]
        ie, ino = [p[0] for p in seg["int"]], [p[1] for p in seg["int"]]
        if hl:
            axg.plot(oe, on, color="navy", lw=2.6, zorder=6)
            axg.plot(ie, ino, color="darkred", lw=2.6, zorder=6)
        else:
            axg.plot(oe, on, color="tab:blue", lw=0.7, alpha=0.22, zorder=2)
            axg.plot(ie, ino, color="tab:orange", lw=0.7, alpha=0.22, zorder=2)
    env, agents = build_pair(a)
    o0, i0 = _enu_pair(env.initial_state(agents))
    axg.plot(*o0, "o", color="tab:blue", ms=9, zorder=5, label="OWN start (north)")
    axg.plot(*i0, "s", color="tab:orange", ms=9, zorder=5, label=f"INT start ({a.dpsi:.0f}°)")
    if chosen is not None:
        end = _descent(segments[chosen])["own"][-1]
        axg.plot(end[0], end[1], "*", color="k", ms=15, zorder=8, label="LoS (closest approach)")
        axg.plot([], [], color="navy", lw=2.6, label="LoS lineage: OWN")
        axg.plot([], [], color="darkred", lw=2.6, label="LoS lineage: INT")
    axg.set_xlabel("east [m]")
    axg.set_ylabel("north [m]")
    axg.set_aspect("equal")
    axg.set_title("ground tracks of the two aircraft (every particle)")
    axg.legend(loc="upper left", fontsize=8)

    p_est = float(np.prod(survival)) if len(survival) == len(a.levels) else 0.0
    fig.suptitle(f"IPS splitting tree — {a.dpsi:.0f}° crossing, pos_ci95={a.pos:g}m, "
                 f"margin={a.margin:.2f}, N={a.n}, P(LoS)~{p_est:.1e}", fontsize=11)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dpsi", type=float, default=90.0, help="crossing angle [deg]")
    p.add_argument("--pos", type=float, default=10.0, help="pos_ci95 [m] (vel_ci95 = pos/10)")
    p.add_argument("--margin", type=float, default=1.0, help="MVP resolution margin (>= 1)")
    p.add_argument("-N", "--particles", dest="n", type=int, default=200, help="particles/level")
    p.add_argument("--levels", type=float, nargs="+", default=DEFAULT_LEVELS,
                   help="shell distances [m], decreasing, ending at rpz")
    p.add_argument("--lookahead", type=float, default=60.0, help="detection lookahead [s]")
    p.add_argument("--tlos", type=float, default=90.0, help="time to loss of separation [s]")
    p.add_argument("--rpz", type=float, default=50.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-highlight", dest="no_highlight", action="store_true",
                   help="do not trace/highlight a LoS lineage")
    p.add_argument("--out", type=Path,
                   default=Path("vault/observations/img/ips-splitting-tree.png"))
    a = p.parse_args()

    segments, survival, lineage, chosen = run_traced(a)
    print(f"survival/shell: {[f'{s:.2f}' for s in survival]}")
    if chosen is not None:
        print(f"LoS lineage: {len(lineage)} segments, min sep "
              f"{segments[chosen]['ts'][-1][1]:.1f} m")
    plot(a, segments, survival, lineage, chosen, a.out)


if __name__ == "__main__":
    main()
