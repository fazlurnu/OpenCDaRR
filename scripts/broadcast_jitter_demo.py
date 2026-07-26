"""Broadcast-interval jitter: fixed vs per-transmission-dithered transmit clock.

The companion to [[broadcast-phase-offset]]. A *phase* offset shifts an aircraft's broadcast comb
but keeps it regular; **jitter** dithers *every* gap (``interval + U(-j, +j)``), the slot
randomisation real ADS-B uses to avoid systematic co-channel collisions. This drives the real
:class:`~opencdarr.cns.Comm` over one lossy link (reception 0.8, lognormal latency) under two
transmit schedules and compares the **inter-arrival gap of received messages** — panel 5 of
[[communication-reception-latency]]:

* **Fixed** (old): gaps land at exactly ``k·interval`` — sharp humps, ``k−1`` = run of drops.
* **Jittered** (new): each hump smears (and widens with ``k``, ~√k·jitter), the regular comb gone.

Mirrors ``run_fleet(schedule=BroadcastSchedule(jitter=…))`` — the same ``U(-j, +j)`` gap draw.
Reproduce::

    PYTHONPATH=. python scripts/broadcast_jitter_demo.py

Writes ``vault/observations/img/broadcast-jitter-comparison.png``.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.cns import Comm, CommState, Message, lognormal_latency  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

SRC, EGO = "SRC", "EGO"
RECEPTION = 0.8
LAT_MEDIAN, LAT_SIGMA = 0.1, 0.25  # s
INTERVAL = 1.0
JITTER = 0.1          # half-width [s]: each gap is INTERVAL + U(-JITTER, +JITTER)
N_TICKS = 4000
BLUE, ORANGE = "#1f77b4", "#ff7f0e"


def _msg(t: float) -> Message:
    return Message(SRC, AircraftState(id=SRC, lat=52.0, lon=4.0, trk=0.0, gs=10.0), t_meas=t)


def transmit_times(jitter: float, rng: np.random.Generator) -> list[float]:
    """The SRC broadcast schedule: fixed INTERVAL, or dithered by U(-jitter, +jitter) per gap."""
    ts, t = [], 0.0
    for _ in range(N_TICKS):
        ts.append(t)
        step = INTERVAL + (rng.uniform(-jitter, jitter) if jitter > 0.0 else 0.0)
        t += step
    return ts


def run(jitter: float) -> tuple[list[float], np.ndarray]:
    """Return (transmit times, inter-arrival gaps of *received* messages) for a schedule."""
    sched_seq, comm_seq = spawn(root_seed_sequence(0), 2)  # separate substreams (ADR 0001/0006)
    tx = transmit_times(jitter, generator(sched_seq))
    comm = Comm(reception_prob=RECEPTION, latency=lognormal_latency(LAT_MEDIAN, LAT_SIGMA))
    rng = generator(comm_seq)

    state = CommState()
    arrivals: list[float] = []  # deliver_t of every message this link actually accepted
    for t in tx:
        state = comm.step(state, [_msg(t)], [EGO], t, rng)
        for pending in state.in_flight:
            if pending.message.source == SRC and pending.message.t_meas == t:
                arrivals.append(pending.deliver_t)
                break
    gaps = np.diff(np.sort(arrivals))
    return tx, gaps


def main() -> None:
    tx_fixed, gaps_fixed = run(jitter=0.0)
    tx_jit, gaps_jit = run(jitter=JITTER)

    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(
        "Broadcast-interval jitter — fixed (old) vs per-transmission dither (new)\n"
        f"one lossy link (reception {RECEPTION}, lognormal latency); jitter = ±{JITTER}s per gap",
        fontsize=12, fontweight="bold",
    )

    # row 1: the transmit comb over the first 12 s — regular vs irregular
    for ax, tx, col, name in ((axs[0, 0], tx_fixed, BLUE, "Fixed"),
                              (axs[0, 1], tx_jit, ORANGE, "Jittered")):
        early = [t for t in tx if t <= 12.0]
        ax.eventplot(early, colors=col, lineoffsets=0.5, linelengths=0.8)
        for k in range(13):
            ax.axvline(k, color="0.8", lw=0.8, zorder=0)
        ax.set_title(f"{name} transmit times (first 12 s) — grey lines = k·interval", fontsize=10)
        ax.set_xlabel("time [s]")
        ax.set_yticks([])
        ax.set_xlim(0, 12)

    # row 2: inter-arrival gap of received messages — humps at k·interval vs smeared
    bins = np.arange(0.0, 4.0, 0.03)
    for ax, gaps, col, name in ((axs[1, 0], gaps_fixed, BLUE, "Fixed"),
                                (axs[1, 1], gaps_jit, ORANGE, "Jittered")):
        ax.hist(gaps, bins=bins, density=True, color=col, alpha=0.85)
        for k in (1, 2, 3):
            ax.axvline(k, color="0.6", ls="--", lw=0.9)
        sub = "sharp humps at k·interval" if name == "Fixed" else "smeared, widening with k"
        ax.set_title(f"{name} inter-arrival gaps — {sub}", fontsize=10)
        ax.set_xlabel("gap between received messages [s]")
        ax.set_ylabel("density")
        ax.set_xlim(0, 4)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = (Path(__file__).resolve().parents[1]
           / "vault/observations/img/broadcast-jitter-comparison.png")
    fig.savefig(out, dpi=120)
    print("wrote", out)

    # sharpness metric: fraction of gaps within ±0.1 s of the nearest integer multiple of INTERVAL
    def near_grid(gaps: np.ndarray) -> float:
        return float(np.mean(np.abs(gaps - np.round(gaps / INTERVAL) * INTERVAL) < 0.1))

    for name, gaps in (("fixed   ", gaps_fixed), ("jittered", gaps_jit)):
        print(f"{name}: mean gap={gaps.mean():.3f}s  "
              f"within ±0.1s of k·interval={near_grid(gaps) * 100:4.1f}%")


if __name__ == "__main__":
    main()
