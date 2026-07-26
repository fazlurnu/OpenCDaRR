"""Handbook figure: the communication channel — reception probability and the update interval.

The "Communication" page's picture, drawn from the real
:class:`~opencdarr.cns.communication.Comm` model, in the terms of the ADS-B reception-probability
model (Rahman et al.): reception is a Bernoulli trial, so the number of transmissions until the
next success is geometric, and the **update interval** — the time between received position updates
— lands in *bumps* at multiples of the broadcast interval. Two directed links are compared, a lossy
one (reception 0.80) and a reliable one (0.99):

  - left: the **time since the last received update**, tick by tick. Each delivery resets it to one
    broadcast interval; every missed message lengthens it.
  - right: the **update interval** distribution — geometric bumps at multiples of the broadcast
    interval, with mean interval / reception.

Handbook plot style: no suptitle, concise titles. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/communication.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import rng  # noqa: E402
from opencdarr.cns.base import CommState, LatencyDistribution, Message  # noqa: E402
from opencdarr.cns.broadcast import BroadcastSchedule  # noqa: E402
from opencdarr.cns.communication import Comm, constant_latency, lognormal_latency  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
INTERVAL, N_TICKS, SEED = 1.0, 4000, 20260725
BLUE, RED, ORANGE, PURPLE = "#1f77b4", "#d62728", "#ff7f0e", "#9467bd"
SRC = AircraftState(id="SRC", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def update_intervals(
    reception: float, latency: LatencyDistribution, schedule: BroadcastSchedule, seed: int,
    n_tx: int = 2000, poll: float = 0.01,
) -> np.ndarray:
    """The update intervals of one directed link — the gaps between the times the receiver actually
    *gets* each fresh message — for a reception probability, ``latency``, and transmit ``schedule``
    (interval + jitter). Reception is timed at a fine ``poll`` so the sub-interval spread that
    latency adds is visible (a transmission-time gap would hide it). Transmit and channel draws
    take separate substreams (ADR 0006 §6)."""
    tx_seq, comm_seq = rng.spawn(rng.root_seed_sequence(seed), 2)
    tx_rng, comm_rng = rng.generator(tx_seq), rng.generator(comm_seq)
    comm = Comm(reception_prob={("SRC", "RCV"): reception}, latency=latency)
    clock = schedule.initial(1)[0]
    tx_times = []
    for _ in range(n_tx):
        tx_times.append(clock)
        clock = schedule.advance(clock, tx_rng)
    state, received_at, last, ti, t = CommState(), [], None, 0, 0.0
    while t <= tx_times[-1] + 1e-9:
        broadcasts = []
        while ti < len(tx_times) and tx_times[ti] <= t + 1e-9:
            broadcasts.append(Message("SRC", SRC, tx_times[ti]))
            ti += 1
        state = comm.step(state, broadcasts, ["SRC", "RCV"], t, comm_rng)
        held = state.held.get(("RCV", "SRC"))
        if held is not None and held.t_meas != last:  # a fresh message just landed at time t
            received_at.append(t)
            last = held.t_meas
        t += poll
    return np.diff(received_at)


def simulate(reception: float, seed: int) -> tuple[list[float], list[float], list[float]]:
    """Run one directed link at ``reception`` probability, broadcasting every ``INTERVAL`` with a
    lognormal delay: per-tick (time, time-since-last-update) and the update intervals between
    successive received messages."""
    comm = Comm(reception_prob={("SRC", "RCV"): reception}, latency=lognormal_latency(0.1, 0.25))
    generator = rng.generator(rng.root_seed_sequence(seed))
    state = CommState()
    times, since_update, received = [], [], []
    last_tmeas: float | None = None
    for k in range(N_TICKS):
        t = k * INTERVAL
        state = comm.step(state, [Message(source="SRC", state=SRC, t_meas=t)], ["SRC", "RCV"],
                          t, generator)
        held = state.held.get(("RCV", "SRC"))
        if held is None:
            continue
        times.append(t)
        since_update.append(t - held.t_meas)
        if held.t_meas != last_tmeas:  # a fresh message just arrived at time t
            received.append(t)  # the reception time -> gaps are times between received messages
            last_tmeas = held.t_meas
    update_intervals = list(np.diff(received))
    return times, since_update, update_intervals


def plot(out: Path) -> None:
    links = [("reliable link (reception 0.99)", 0.99, BLUE),
             ("lossy link (reception 0.80)", 0.80, RED)]
    runs = {p: simulate(p, SEED + i) for i, (_, p, _) in enumerate(links)}
    for label, p, _ in links:
        _, since, intervals = runs[p]
        print(f"     {label:>32}: mean time-since-update {np.mean(since):.2f} s "
              f"(max {max(since):.0f}), mean update interval {np.mean(intervals):.3f} s "
              f"(interval / reception = {INTERVAL / p:.3f})")

    fig, (a_since, a_int) = plt.subplots(1, 2, figsize=(12.0, 5.2))

    # --- time since the last received update: each missed message lengthens it ---
    for label, p, col in links:
        times, since, _ = runs[p]
        window = [(t, s) for t, s in zip(times, since, strict=True) if t <= 60.0]
        a_since.step([t for t, _ in window], [s for _, s in window], where="post", color=col,
                     lw=2.0, label=label)
    a_since.set_xlabel("time [s]")
    a_since.set_ylabel("time since last received update [s]")
    a_since.set_title("A missed message lengthens the interval since the last update", fontsize=10)
    a_since.set_ylim(bottom=0)
    a_since.legend(fontsize=8, loc="upper right")
    a_since.set_box_aspect(1)

    # --- update-interval distribution: geometric bumps at multiples of the broadcast interval ---
    ivals = [np.asarray(runs[p][2]) for _, p, _ in links]
    a_int.hist(ivals, bins=np.arange(0.5, 7.0, 0.25),
               weights=[np.full(v.size, 1.0 / v.size) for v in ivals],
               color=[c for _, _, c in links], label=[lbl for lbl, _, _ in links])
    a_int.set_xlabel("update interval [s]")
    a_int.set_ylabel("fraction")
    a_int.set_title("Update interval: geometric bumps, mean = interval / reception", fontsize=10)
    a_int.set_xticks(range(1, 7))
    a_int.legend(fontsize=8, loc="upper right")
    a_int.set_box_aspect(1)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def latency_figure(out: Path) -> None:
    """The time between messages received by the receiver, with a lognormal(median 0.1 s, σ 0.25)
    latency at reception 0.9. Left: the full distribution — geometric bumps at multiples of the
    interval, set by dropped messages. Right: zoomed to the one-interval bump, where the sub-second
    latency only smears it around one interval, never lengthening it."""
    ui = update_intervals(0.9, lognormal_latency(0.1, 0.25), BroadcastSchedule(interval=INTERVAL),
                          SEED)
    w = np.full(ui.size, 1.0 / ui.size)
    print(f"        latency ~ lognormal(0.1, 0.25): mean {ui.mean():.3f} s, std {ui.std():.3f} s")

    fig, (a_full, a_zoom) = plt.subplots(1, 2, figsize=(12.0, 5.0))
    a_full.hist(ui, bins=np.arange(0.5, 4.5, 0.05), weights=w, color=ORANGE)
    a_full.set_xlabel("time between received messages [s]")
    a_full.set_ylabel("fraction")
    a_full.set_xticks(range(1, 5))
    a_full.set_title("Time between received messages (reception 0.9)", fontsize=10)
    a_full.set_box_aspect(1)

    a_zoom.hist(ui, bins=np.arange(0.80, 1.42, 0.01), weights=w, color=ORANGE)
    a_zoom.axvline(INTERVAL, color="0.6", ls=":", lw=1.2, label="one interval")
    a_zoom.set_xlabel("time between received messages [s]")
    a_zoom.set_title("Zoom on the one-interval bump: latency smears it", fontsize=10)
    a_zoom.legend(fontsize=8)
    a_zoom.set_box_aspect(1)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def jitter_figure(out: Path) -> None:
    """Broadcast jitter's effect on the update interval, in two panels: jitter alone spreads each
    geometric bump around its multiple of the interval; adding the realistic latency on top changes
    it negligibly (the spread is jitter's, not the channel's)."""
    jit = BroadcastSchedule(interval=INTERVAL, jitter=0.1)
    panels = [("jitter ±0.1 s", constant_latency(0.0), PURPLE),
              ("jitter ±0.1 s + latency ~ lognormal(0.1, 0.25)", lognormal_latency(0.1, 0.25),
               ORANGE)]
    runs = [update_intervals(0.9, latency, jit, SEED) for _, latency, _ in panels]
    for (label, _, _), ui in zip(panels, runs, strict=True):
        print(f"        [{label}]: mean {ui.mean():.2f} s, std {ui.std():.2f} s")

    bins = np.arange(0.5, 4.5, 0.05)
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.0), sharex=True, sharey=True)
    for ax, (label, _, col), ui in zip(axes, panels, runs, strict=True):
        ax.hist(ui, bins=bins, weights=np.full(ui.size, 1.0 / ui.size), color=col, alpha=0.85,
                label=label)
        ax.set_xlabel("update interval [s]")
        ax.set_xticks(range(1, 5))
        ax.legend(fontsize=8)
        ax.set_box_aspect(1)
    axes[0].set_ylabel("fraction")
    axes[0].set_title("Jitter spreads each bump around its multiple", fontsize=10)
    axes[1].set_title("Latency on top barely changes it", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    plot(IMG / "comm-update-interval.png")
    latency_figure(IMG / "comm-latency.png")
    jitter_figure(IMG / "comm-jitter.png")


if __name__ == "__main__":
    main()
