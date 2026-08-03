"""Handbook figure: asymmetric situational awareness — the whole CNS chain, in a 2x2.

Two aircraft, each measuring itself with isotropic Gaussian GNSS noise (pos_ci95 = 10 m, **N**),
broadcasting on a jittered 1 Hz schedule, and hearing the other over a directed lossy, latent link
(lognormal latency, **C**), holding the last fix (**S**):

- aircraft **i** — 10 m/s north-east, receives at probability 1.0;
- aircraft **j** — 5 m/s west, receives at probability 0.7 (exaggerated).

Each panel samples a perceived position against that aircraft's *current* ground truth, recentred
on the truth. On the diagonal an aircraft looks at **itself**: a fresh self-fix, so just the
Gaussian navigation blur, no lag. Off the diagonal it looks at the **other**: the same blur, now
biased opposite the source's motion because the held fix is stale — with a heavier tail on j's
lossy 0.7 link than on i's perfect 1.0 link. The four pictures disagree: awareness is asymmetric.

    PYTHONPATH=. python scripts/handbook/perceived_position.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402

from opencdarr import geo, rng  # noqa: E402
from opencdarr.cns.base import CommState  # noqa: E402
from opencdarr.cns.broadcast import BroadcastSchedule  # noqa: E402
from opencdarr.cns.communication import Comm, lognormal_latency  # noqa: E402
from opencdarr.cns.navigation import GnssNavigation  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0 = 52.0, 4.0
# the roster `Comm.step` is handed: aircraft, not ids (nothing here gates on geometry)
_ROSTER = (AircraftState(id="i", lat=LAT0, lon=LON0, trk=0.0, gs=0.0),
           AircraftState(id="j", lat=LAT0, lon=LON0, trk=0.0, gs=0.0))
POS_CI95 = 10.0  # 95% radial GNSS position accuracy [m], isotropic Gaussian
SEED = 20260725
# (track [deg], speed [m/s], reception-as-receiver)
I_TRACK, I_SPEED, I_RECEPTION = 45.0, 10.0, 1.0  # north-east, perfect receiver
J_TRACK, J_SPEED, J_RECEPTION = 270.0, 5.0, 0.7  # west, lossy receiver (exaggerated)
BLUE, RED = "#1f77b4", "#d62728"
IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"


def _true(track: float, speed: float, t: float) -> tuple[float, float]:
    """Ground-truth (lat, lon) of a constant-velocity aircraft at time ``t``."""
    return geo.forward(LAT0, LON0, track, speed * t)


def _offset(true_ll: tuple[float, float], lat: float, lon: float) -> tuple[float, float]:
    """(east, north) metres of a (lat, lon) relative to a ground-truth point."""
    qdr, dist = geo.qdrdist(true_ll[0], true_ll[1], lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def simulate(t_end: float = 1000.0, sample_dt: float = 0.5, poll: float = 0.02
             ) -> dict[str, np.ndarray]:
    """Fly both aircraft, broadcast noisy fixes over the two directed links, and sample every
    perceived position against the source's current truth. Returns the (east, north) offsets [m]
    for the four views 'ii', 'ij', 'ji', 'jj' (perceiver-target); diagonals are self-fixes."""
    nav = GnssNavigation()
    comm = Comm(reception_prob={("j", "i"): I_RECEPTION, ("i", "j"): J_RECEPTION},
                latency=lognormal_latency(0.1, 0.25))
    sched = BroadcastSchedule(interval=1.0, jitter=0.1)
    nav_i, nav_j, self_i, self_j, comm_rng, tx_i, tx_j = (
        rng.generator(s) for s in rng.spawn(rng.root_seed_sequence(SEED), 7)
    )

    def tx_times(gen: np.random.Generator) -> list[float]:
        clock, out = sched.initial(1)[0], []
        while clock < t_end:
            out.append(clock)
            clock = sched.advance(clock, gen)
        return out

    def measured(track: float, speed: float, ac: str, t: float, gen: np.random.Generator):
        lat, lon = _true(track, speed, t)
        src = AircraftState(id=ac, lat=lat, lon=lon, trk=track, gs=speed, pos_ci95=POS_CI95)
        return nav.measure(nav.initial_state(), src, t, gen)

    i_tx, j_tx = tx_times(tx_i), tx_times(tx_j)
    state, ni, nj, t, next_sample = CommState(), 0, 0, 0.0, 0.0
    keys = ("ii", "ij", "rel_i", "jj", "ji", "rel_j")
    out: dict[str, list[tuple[float, float]]] = {k: [] for k in keys}
    while t < t_end:
        bcast = []
        while ni < len(i_tx) and i_tx[ni] <= t + 1e-9:
            bcast.append(measured(I_TRACK, I_SPEED, "i", i_tx[ni], nav_i))
            ni += 1
        while nj < len(j_tx) and j_tx[nj] <= t + 1e-9:
            bcast.append(measured(J_TRACK, J_SPEED, "j", j_tx[nj], nav_j))
            nj += 1
        state = comm.step(state, bcast, _ROSTER, t, comm_rng)
        if t + 1e-9 >= next_sample:
            # diagonals: a fresh self-measurement (no communication) -> pure navigation blur
            fi = measured(I_TRACK, I_SPEED, "i", t, self_i)
            ii = _offset(_true(I_TRACK, I_SPEED, t), fi.state.lat, fi.state.lon)
            fj = measured(J_TRACK, J_SPEED, "j", t, self_j)
            jj = _offset(_true(J_TRACK, J_SPEED, t), fj.state.lat, fj.state.lon)
            out["ii"].append(ii)
            out["jj"].append(jj)
            # off-diagonal: the held fix of the other; relative = held-other minus own self-fix
            # (the position error conflict detection actually acts on)
            if (h := state.held.get(("i", "j"))) is not None:  # i's view of j
                ij = _offset(_true(J_TRACK, J_SPEED, t), h.state.lat, h.state.lon)
                out["ij"].append(ij)
                out["rel_i"].append((ij[0] - ii[0], ij[1] - ii[1]))
            if (h := state.held.get(("j", "i"))) is not None:  # j's view of i
                ji = _offset(_true(I_TRACK, I_SPEED, t), h.state.lat, h.state.lon)
                out["ji"].append(ji)
                out["rel_j"].append((ji[0] - jj[0], ji[1] - jj[1]))
            next_sample += sample_dt
        t += poll
    return {k: np.array(v) for k, v in out.items()}


def _panel(ax: Axes, off: np.ndarray, title: str, lim: float,
           motion: tuple[float, float] | None) -> None:
    e, n = off[:, 0], off[:, 1]
    me, mn = float(e.mean()), float(n.mean())
    ax.scatter(e, n, s=6, alpha=0.13, color=BLUE, edgecolors="none")
    ax.plot(0, 0, "*", color=RED, ms=14, zorder=5, label="truth (now)")
    ax.plot(me, mn, "X", color="k", ms=9, zorder=5, label=f"mean ({me:.0f}, {mn:.0f}) m")
    if motion is not None:  # arrow along the source's motion; the held fix lags opposite it
        track, speed = motion
        a = speed * 2.2
        ax.annotate("", xy=(a * math.sin(math.radians(track)), a * math.cos(math.radians(track))),
                    xytext=(0, 0), arrowprops={"arrowstyle": "-|>", "color": "0.4", "lw": 1.8})
    ax.axhline(0, color="0.9", lw=0.6, zorder=0)
    ax.axvline(0, color="0.9", lw=0.6, zorder=0)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=7, loc="upper left")


def plot(out: Path) -> None:
    v = simulate()
    lim = max(np.abs(a).max() for a in v.values()) * 1.05
    for key, name in (("ii", "i sees itself"), ("ij", "i sees j"), ("rel_i", "i: j relative"),
                      ("jj", "j sees itself"), ("ji", "j sees i"), ("rel_j", "j: i relative")):
        e, n = v[key][:, 0], v[key][:, 1]
        print(f"     {name:>14}: lag {math.hypot(e.mean(), n.mean()):.1f} m, "
              f"spread e±{e.std():.1f} n±{n.std():.1f} m")

    fig, ax = plt.subplots(2, 3, figsize=(15.5, 10.5), sharex=True, sharey=True)
    _panel(ax[0, 0], v["ii"], "i's view of itself — self-fix", lim, None)
    _panel(ax[0, 1], v["ij"], "i's view of j  (i receives 1.0)", lim, (J_TRACK, J_SPEED))
    _panel(ax[0, 2], v["rel_i"], "i's relative position of j", lim, (J_TRACK, J_SPEED))
    _panel(ax[1, 0], v["jj"], "j's view of itself — self-fix", lim, None)
    _panel(ax[1, 1], v["ji"], "j's view of i  (j receives 0.7)", lim, (I_TRACK, I_SPEED))
    _panel(ax[1, 2], v["rel_j"], "j's relative position of i", lim, (I_TRACK, I_SPEED))
    for a in ax[1, :]:
        a.set_xlabel("east offset from truth [m]")
    for a in ax[:, 0]:
        a.set_ylabel("north offset from truth [m]")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    plot(IMG / "perceived-position.png")


if __name__ == "__main__":
    main()
