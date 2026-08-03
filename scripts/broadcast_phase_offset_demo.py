"""Broadcast-phase offset: aligned vs staggered transmit clocks (motivates Phase 6f).

Aircraft "spawned" at different times run the *same* broadcast interval at *different phase* — A
transmits at 0,1,2,... while B (spawned 0.3 s later) transmits at 0.3,1.3,2.3,.... The current
``run_fleet`` / ``run_encounter`` loop hardcodes a **single global** ``next_broadcast`` from 0, so
every aircraft is phase-aligned; this shows what the realistic offset changes.

Setup: one ego observer watches **four** intruders (straight flights, 20 m/s) through the real
:class:`~opencdarr.cns.communication.Comm` + :class:`~opencdarr.cns.surveillance.LastKnown` models,
reception 1.0 and latency 0.5 s (drops off, to isolate the *phase* effect). We plot each intruder's
perceived-position error and, in bold, the **max over intruders** — the ego's *worst* stale target
at each instant, which is what a multi-intruder resolver actually rides.

* **Aligned** (all phase 0, today's model): every intruder's staleness sawtooth peaks on the *same*
  tick, so the max swings between all-fresh troughs and all-stale peaks — correlated.
* **Staggered** (phases 0, ¼, ½, ¾ of the interval): the sawteeth interleave, so the max is steady
  near its ceiling — the ego is *never* fresh on everyone, but also never simultaneously stalest on
  everyone. A different, more realistic uncertainty structure.

That the demo can produce the offset *at all* using the stock models is the point: they key on
per-message ``t_meas`` and already support it — only the loop's single cadence does not.

Reproduce::

    PYTHONPATH=. python scripts/broadcast_phase_offset_demo.py

Writes ``vault/observations/img/broadcast-phase-offset.png``.
"""
from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np

from opencdarr import geo
from opencdarr.cns.base import CommState, Message
from opencdarr.cns.communication import Comm, constant_latency
from opencdarr.cns.surveillance import LastKnown
from opencdarr.state import AircraftState

LAT0, LON0 = 52.0, 4.0
GS = 20.0
DT = 0.1
T_MAX = 15.0
INTERVAL = 1.0
LATENCY = 0.5
N_INT = 4
COLORS = ["#4393c3", "#7fbf7b", "#d6604d", "#c2a5cf"]


# EGO is the receiver, parked at the ring centre the intruders converge on. `Comm.step` takes
# the roster as true states, so it needs one even though EGO never broadcasts here.
def ego_true(t: float) -> AircraftState:
    return AircraftState(id="EGO", lat=LAT0, lon=LON0, trk=0.0, gs=0.0)


def intruder_true(i: int, t: float) -> AircraftState:
    """Intruder i flying straight from a ring position on its own heading."""
    bearing = 360.0 * i / N_INT
    start_lat, start_lon = geo.forward(LAT0, LON0, bearing, 1200.0)
    hdg = (bearing + 180.0) % 360.0  # inbound
    lat, lon = geo.forward(start_lat, start_lon, hdg, GS * t)
    return AircraftState(id=f"INT{i}", lat=lat, lon=lon, trk=hdg, gs=GS)


def enu(state: AircraftState) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, state.lat, state.lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def run(phases: list[float]) -> tuple[list[float], list[list[float]], list[float]]:
    """Ego perceives N_INT sources, each transmitting at its own phase.

    Returns (times, per-source error, per-source first-contact time). A source's first belief
    lands at ``phase + latency`` — for a small phase that is < 1 s of global time, spread per
    aircraft rather than gated to a single tick.
    """
    comm = Comm(reception_prob=1.0, latency=constant_latency(LATENCY))
    surveil = LastKnown()
    rng = np.random.default_rng(0)  # unused at reception 1.0 / constant latency, but threaded
    cstate = CommState()
    next_bc = list(phases)
    ts: list[float] = []
    errs: list[list[float]] = [[] for _ in range(N_INT)]
    first_contact = [math.inf] * N_INT

    t = 0.0
    while t <= T_MAX + 1e-9:
        # collect this tick's broadcasts, then step comm ONCE per grid tick (even if empty) so
        # in-flight deliveries land at their true deliver_t, uniformly across every phase policy
        broadcasts = []
        for i in range(N_INT):
            if t + 1e-9 >= next_bc[i]:
                broadcasts.append(Message(f"INT{i}", intruder_true(i, t), t))
                next_bc[i] += INTERVAL
        cstate = comm.step(cstate, broadcasts, [ego_true(t)], t, rng)
        ts.append(t)
        for i in range(N_INT):
            p = surveil.perceived(cstate, "EGO", f"INT{i}", t)
            if p is None:
                errs[i].append(np.nan)
            else:
                if first_contact[i] == math.inf:
                    first_contact[i] = t
                bx, by = enu(p)
                tx, ty = enu(intruder_true(i, t))
                errs[i].append(math.hypot(tx - bx, ty - by))
        t += DT
    return ts, errs, first_contact


def _maxline(errs: list[list[float]]) -> np.ndarray:
    arr = np.asarray(errs, dtype=float)
    out = np.full(arr.shape[1], np.nan)
    live = ~np.all(np.isnan(arr), axis=0)  # columns with at least one delivered belief
    out[live] = np.nanmax(arr[:, live], axis=0)
    return out


def plot(ax: plt.Axes, ts: list[float], errs: list[list[float]], title: str) -> None:
    for i in range(N_INT):
        ax.plot(ts, errs[i], color=COLORS[i], lw=1.0, alpha=0.8, label=f"INT{i}")
    mx = _maxline(errs)
    ax.plot(ts, mx, color="k", lw=2.4, label="max over intruders", zorder=5)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("perceived error [m]")
    ax.set_ylim(0, 45)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2, loc="lower right")


def main() -> None:
    aligned_phases = [0.0] * N_INT                             # today's model: one global cadence
    even_phases = [i / N_INT * INTERVAL for i in range(N_INT)]  # evenly staggered
    # realistic: each aircraft draws a random initial firing time in [0, interval), then every
    # interval after (ADS-B randomises its slot; would come from a per-aircraft RNG substream)
    rng_phase = np.random.default_rng(0)
    rand_phases = list(rng_phase.uniform(0.0, INTERVAL, N_INT))

    ts, e_align, _ = run(aligned_phases)
    _, e_even, _ = run(even_phases)
    _, e_rand, fc_rand = run(rand_phases)

    fig, axs = plt.subplots(1, 3, figsize=(18, 5.2), sharey=True)
    fig.suptitle(
        "Broadcast-phase offset — aligned (today) vs staggered vs randomised transmit clocks\n"
        "one ego watching 4 intruders; bold = worst stale intruder the resolver rides",
        fontsize=12, fontweight="bold",
    )
    plot(axs[0], ts, e_align, "A. Aligned (all t=0) — staleness peaks together (correlated)")
    plot(axs[1], ts, e_even, "B. Evenly staggered (0, ¼, ½, ¾) — interleaved, steady max")
    ph = ", ".join(f"{p:.2f}" for p in rand_phases)
    plot(axs[2], ts, e_rand, f"C. Randomised phase ({ph}) — realistic, irregular")
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    out = "vault/observations/img/broadcast-phase-offset.png"
    fig.savefig(out, dpi=130)
    print("saved", out)

    # settle past first-contact transient (t >= 3 s) before summarising the max line
    def stats(ts_: list[float], errs: list[list[float]]) -> tuple[float, float, float]:
        mx = _maxline(errs)
        m = mx[np.asarray(ts_) >= 3.0]
        return float(np.nanmean(m)), float(np.nanstd(m)), float(np.nanmin(m))

    for tag, e in (("aligned  ", e_align), ("even     ", e_even), ("randomised", e_rand)):
        mean, std, lo = stats(ts, e)
        print(f"{tag} max-staleness: mean={mean:5.1f}m  std={std:4.1f}m  floor={lo:5.1f}m")
    print("randomised first-contact times [s]:",
          "  ".join(f"INT{i}={fc:.1f}" for i, fc in enumerate(fc_rand)),
          "  (phase+latency; some < 1 s)")


if __name__ == "__main__":
    main()
