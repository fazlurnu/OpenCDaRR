"""Illustrate multi-aircraft surveillance uncertainty (motivates Phase 6f).

One source (AC1) on a curving track is observed by two receivers (AC2, AC3) through the *real*
:class:`~opencdarr.cns.communication.Comm` + :class:`~opencdarr.cns.surveillance.LastKnown`
models, over **independent directed links** (1->2, 1->3). Two stacked uncertainty sources:

* **Communication / surveillance** — reception drops + latency mean each observer holds a *stale*
  last-known picture (hold-as-is), and because each link draws independently the two observers
  disagree with each other even at equal reception probability.
* **Navigation** — with GNSS self-noise (``pos_ci95`` / ``vel_ci95``) every broadcast is itself a
  jittered self-measurement, so even a *just-delivered* message carries error: an irreducible
  noise floor on top of the staleness sawtooth.

The figure is a 2x2: top row **without** navigation noise (staleness only), bottom row **with**
it. Reproduce with::

    PYTHONPATH=. python scripts/surveillance_asymmetric_demo.py

Writes ``vault/observations/img/surveillance-asymmetric-perception.png``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from opencdarr import geo
from opencdarr.cns.base import CommState, Message
from opencdarr.cns.communication import Comm, constant_latency
from opencdarr.cns.navigation import GnssNavigation
from opencdarr.cns.surveillance import LastKnown
from opencdarr.state import AircraftState

# --- scenario constants -------------------------------------------------------------------
LAT0, LON0 = 52.0, 4.0
GS = 20.0            # source ground speed [m/s]
DT = 0.5             # sampling / integration step [s]
T_MAX = 60.0         # run length [s]
BROADCAST = 1.0      # broadcast cadence [s]
LATENCY = 0.8        # constant link delay [s]
RECEPT = {("AC1", "AC2"): 0.85, ("AC1", "AC3"): 0.45}  # asymmetric links: AC3 hears less
POS_CI95 = 10.0      # GNSS 95% radial position accuracy [m] (matches the 6e sweep)
VEL_CI95 = 1.0       # GNSS 95% velocity accuracy [m/s]
SEED = 7

GREEN, PINK = "#1b7837", "#c51b7d"


@dataclass
class Trace:
    """Per-tick record of the truth and each observer's belief of AC1."""

    ts: list[float]
    true: list[tuple[float, float]]
    b2: list[tuple[float, float] | None]
    b3: list[tuple[float, float] | None]
    e2: list[float]
    e3: list[float]


def source_states() -> list[tuple[float, AircraftState]]:
    """AC1's true state every DT along a curving track (heading sweeps 090 -> 000)."""
    out: list[tuple[float, AircraftState]] = []
    lat, lon, t = LAT0, LON0, 0.0
    while t <= T_MAX + 1e-9:
        trk = 90.0 * (1.0 - min(t / T_MAX, 1.0))
        out.append((t, AircraftState(
            id="AC1", lat=lat, lon=lon, trk=trk, gs=GS,
            pos_ci95=POS_CI95, vel_ci95=VEL_CI95,
        )))
        lat, lon = geo.forward(lat, lon, trk, GS * DT)
        t += DT
    return out


def enu(state: AircraftState) -> tuple[float, float]:
    """East, north offset of a state from the source start [m]."""
    qdr, dist = geo.qdrdist(LAT0, LON0, state.lat, state.lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def run(nav_noise: bool) -> Trace:
    """Thread the real Comm/LastKnown (+ optional GnssNavigation) and record beliefs of AC1."""
    comm = Comm(reception_prob=RECEPT, latency=constant_latency(LATENCY))
    surveil = LastKnown()
    nav = GnssNavigation() if nav_noise else None
    nav_rng = np.random.default_rng(SEED)          # navigation substream (self-fix)
    comm_rng = np.random.default_rng(SEED + 1000)  # communication substream (reception/latency)
    cstate = CommState()
    truth = source_states()
    # the roster as true states: `Comm.step` takes aircraft, not ids
    receivers = [AircraftState(aid, LAT0, LON0, 0.0, GS) for aid in ("AC1", "AC2", "AC3")]

    tr = Trace([], [], [], [], [], [])
    next_bc = 0.0
    for t, ac1 in truth:
        if t + 1e-9 >= next_bc:
            # AC1 broadcasts its self-fix (noisy iff nav_noise); the source is what matters here
            if nav is not None:
                msg1 = nav.measure(nav.initial_state(), ac1, t, nav_rng)
            else:
                msg1 = Message(source="AC1", state=ac1, t_meas=t)
            bc = [msg1,
                  Message("AC2", AircraftState("AC2", LAT0, LON0, 0.0, GS), t),
                  Message("AC3", AircraftState("AC3", LAT0, LON0, 0.0, GS), t)]
            cstate = comm.step(cstate, bc, receivers, t, comm_rng)
            next_bc += BROADCAST

        tx, ty = enu(ac1)
        tr.ts.append(t)
        tr.true.append((tx, ty))
        for rcv, bel, err in (("AC2", tr.b2, tr.e2), ("AC3", tr.b3, tr.e3)):
            p = surveil.perceived(cstate, rcv, "AC1", t)
            if p is None:
                bel.append(None)
                err.append(np.nan)
            else:
                bx, by = enu(p)
                bel.append((bx, by))
                err.append(math.hypot(tx - bx, ty - by))
    return tr


def plot_track(ax: plt.Axes, tr: Trace, title: str) -> None:
    tx = [p[0] for p in tr.true]
    ty = [p[1] for p in tr.true]
    ax.plot(tx, ty, "-", color="0.2", lw=2, label="AC1 true track", zorder=3)
    for bel, col, name in ((tr.b2, GREEN, "AC2 belief"), (tr.b3, PINK, "AC3 belief")):
        first = True
        for k in range(0, len(bel), 6):
            if bel[k] is None:
                continue
            bx, by = bel[k]
            ax.plot([tx[k], bx], [ty[k], by], "-", color=col, lw=0.7, alpha=0.5, zorder=2)
            ax.plot(bx, by, "o", color=col, ms=5, zorder=4, label=name if first else None)
            first = False
    ax.plot(tx[0], ty[0], "k^", ms=8, label="start", zorder=5)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="upper left")


def plot_err(ax: plt.Axes, tr: Trace, title: str) -> None:
    ax.plot(tr.ts, tr.e2, color=GREEN, lw=1.8, label="AC2 perceived error")
    ax.plot(tr.ts, tr.e3, color=PINK, lw=1.8, label="AC3 perceived error")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|true − believed| [m]")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="upper left")


def _summary(tag: str, tr: Trace) -> None:
    for name, e in (("AC2", tr.e2), ("AC3", tr.e3)):
        e_arr = np.asarray(e, dtype=float)
        print(f"{tag:8s} {name}: mean={np.nanmean(e_arr):5.1f} m  "
              f"min(fresh)={np.nanmin(e_arr):5.1f} m  max(stale)={np.nanmax(e_arr):6.1f} m")


def main() -> None:
    clean = run(nav_noise=False)
    noisy = run(nav_noise=True)

    fig, axs = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle(
        "Multi-aircraft surveillance uncertainty — staleness (comm) and self-noise (nav) stack\n"
        "one source (AC1), two independent observers; links P(1→2)=0.85, P(1→3)=0.45",
        fontsize=12, fontweight="bold",
    )
    plot_track(axs[0, 0], clean, "A. No nav noise — belief lags on the curve (staleness only)")
    plot_err(axs[0, 1], clean, "A. Clean sawtooth: grows stale, snaps down on delivery")
    plot_track(axs[1, 0], noisy, "B. GNSS noise (10 m / 1 m/s) — beliefs scatter off-track")
    plot_err(axs[1, 1], noisy, "B. Troughs never reach the floor — irreducible noise floor")
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    out = "vault/observations/img/surveillance-asymmetric-perception.png"
    fig.savefig(out, dpi=130)
    print("saved", out)
    _summary("no-nav", clean)
    _summary("with-nav", noisy)


if __name__ == "__main__":
    main()
