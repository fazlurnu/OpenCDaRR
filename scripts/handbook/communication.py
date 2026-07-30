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

import math
from dataclasses import replace
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import rng  # noqa: E402
from opencdarr.cns.base import CommState, LatencyDistribution, Message  # noqa: E402
from opencdarr.cns.broadcast import BroadcastSchedule  # noqa: E402
from opencdarr.cns.communication import (  # noqa: E402
    Comm,
    TransceiverComm,
    constant_latency,
    lognormal_latency,
    radio_health,
)
from opencdarr.cns.surveillance import LastKnown  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
INTERVAL, N_TICKS, SEED = 1.0, 4000, 20260725
BLUE, RED, ORANGE, PURPLE, GREY = "#1f77b4", "#d62728", "#ff7f0e", "#9467bd", "0.55"
SRC = AircraftState(id="SRC", lat=52.0, lon=4.0, trk=0.0, gs=10.0)

# One latency shape across every figure, matching examples/handbook/communication.ipynb.
LAT_MEDIAN, LAT_SIGMA = 0.2, 0.3
# The two figures need *different* reception probabilities to say what they say. The latency figure
# wants the geometric structure visible, so it runs lossy; the jitter figure wants the one-interval
# bump to dominate, because jitter's effect is the *spread* of that bump — at 0.6 the geometric
# spread (std ~1.0 s) swamps a ±0.1 s dither entirely and the figure would show nothing.
RECEPTION_LOSSY, RECEPTION_TIGHT = 0.6, 0.9
JITTER = 0.1


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
    comm = Comm(reception_prob={("SRC", "RCV"): reception},
                latency=lognormal_latency(LAT_MEDIAN, LAT_SIGMA))
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
    """The delay distribution beside what it does downstream. Left: the lognormal itself. Right:
    the update interval with and without it — without latency every gap is exactly k x interval,
    so that case is drawn as reference lines rather than three spikes that would flatten the
    smeared distribution beside them."""
    schedule = BroadcastSchedule(interval=INTERVAL)
    sharp = update_intervals(RECEPTION_LOSSY, constant_latency(0.0), schedule, SEED)
    smeared = update_intervals(RECEPTION_LOSSY,
                               lognormal_latency(LAT_MEDIAN, LAT_SIGMA), schedule, SEED)
    generator = rng.generator(rng.root_seed_sequence(SEED))
    draws = np.array([float(generator.lognormal(np.log(LAT_MEDIAN), LAT_SIGMA))
                      for _ in range(20000)])
    print(f"        no latency:  mean update interval {sharp.mean():.3f} s, "
          f"std {sharp.std():.3f} s")
    print(f"        lognormal({LAT_MEDIAN}, {LAT_SIGMA}): mean {smeared.mean():.3f} s, "
          f"std {smeared.std():.3f} s "
          f"(mean moves {abs(smeared.mean() - sharp.mean()):.3f} s, "
          f"std +{100 * (smeared.std() / sharp.std() - 1):.0f}%)")

    fig, (a_dist, a_gap) = plt.subplots(1, 2, figsize=(12.0, 5.0))
    a_dist.hist(draws, bins=np.arange(0.0, 1.2, 0.015),
                weights=np.full(draws.size, 1.0 / draws.size), color=PURPLE)
    a_dist.axvline(LAT_MEDIAN, color="0.4", ls=":", lw=1.2, label=f"median {LAT_MEDIAN} s")
    a_dist.set_xlabel("link delay [s]")
    a_dist.set_ylabel("fraction")
    a_dist.set_title(f"The delay: lognormal(median {LAT_MEDIAN}, sigma {LAT_SIGMA})", fontsize=10)
    a_dist.legend(fontsize=8)
    a_dist.set_box_aspect(1)

    for k in range(1, 5):
        a_gap.axvline(k * INTERVAL, color=GREY, ls="--", lw=1.2,
                      label="no latency: exactly k x interval" if k == 1 else None)
    a_gap.hist(smeared, bins=np.arange(0.5, 4.5, 0.04),
               weights=np.full(smeared.size, 1.0 / smeared.size), color=PURPLE,
               label="with the delay above")
    a_gap.set_xlabel("update interval [s]")
    a_gap.set_ylabel("fraction")
    a_gap.set_xticks(range(1, 5))
    a_gap.set_title(f"Effect on the update interval (reception {RECEPTION_LOSSY})", fontsize=10)
    a_gap.legend(fontsize=8)
    a_gap.set_box_aspect(1)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def jitter_figure(out: Path) -> None:
    """Broadcast jitter's effect on the update interval, in two panels: jitter alone spreads each
    geometric bump around its multiple of the interval; adding the realistic latency on top changes
    it negligibly (the spread is jitter's, not the channel's)."""
    jit = BroadcastSchedule(interval=INTERVAL, jitter=JITTER)
    panels = [(f"jitter ±{JITTER} s", constant_latency(0.0), PURPLE),
              (f"jitter ±{JITTER} s + latency ~ lognormal({LAT_MEDIAN}, {LAT_SIGMA})",
               lognormal_latency(LAT_MEDIAN, LAT_SIGMA), ORANGE)]
    runs = [update_intervals(RECEPTION_TIGHT, latency, jit, SEED) for _, latency, _ in panels]
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


_FLEET = ("AC1", "AC2", "AC3")
_T_FAIL, _T_MAX = 15.0, 40.0
# "from -> to": AC1's broadcasts reach AC2 over a good link and AC3 over a poor one; the return
# links match, so AC1's own picture of AC2 is the good one and of AC3 the poor one.
_LINKS = {("AC1", "AC2"): 0.9, ("AC2", "AC1"): 0.9,
          ("AC1", "AC3"): 0.5, ("AC3", "AC1"): 0.5}


def _gs(aid: str, t: float) -> float:
    """Neither period divides the 40 s window, so a frozen belief cannot coincide with the truth
    at the readout time and hide the effect the figure is drawn to show."""
    if aid == "AC1":
        return 15.0 + 5.0 * math.sin(2.0 * math.pi * t / 18.0)
    return 11.0 + 4.0 * math.sin(2.0 * math.pi * t / 31.0 + 1.2)


def _outage(subsystem: str) -> np.ndarray:
    """AC1's transmitter or receiver fails at ``_T_FAIL``; every rate is 0, so the failure lands on
    the tick we name rather than on a draw. Columns: t, AC1 truth, AC2 truth, the four beliefs."""
    comm = TransceiverComm(reception_prob=_LINKS)
    generator = rng.generator(rng.root_seed_sequence(11))
    state, rows, t = comm.initial_state(), [], 0.0
    while t <= _T_MAX + 1e-9:
        if t >= _T_FAIL:  # impose the outage on the RadioHealth gate's own state
            down = {f"{subsystem}_down": frozenset({"AC1"})}
            state = replace(state, gates=(replace(radio_health(state), **down),))
        broadcasts = [
            Message(a, AircraftState(id=a, lat=52.0, lon=4.0, trk=0.0, gs=_gs(a, t)), t)
            for a in _FLEET
        ]
        state = comm.step(state, broadcasts, _FLEET, t, generator)

        def seen(receiver: str, source: str, state: CommState = state, t: float = t) -> float:
            held = LastKnown().perceived(state, receiver, source, t)
            return float("nan") if held is None else held.gs

        rows.append((t, _gs("AC1", t), _gs("AC2", t),
                     seen("AC2", "AC1"), seen("AC3", "AC1"),
                     seen("AC1", "AC2"), seen("AC1", "AC3")))
        t += INTERVAL
    return np.array(rows)


def radio_failure_figure(out: Path) -> None:
    """The two halves of a failed radio. A down transmitter silences AC1 for everyone; a down
    receiver blinds AC1 to everyone. Three aircraft, because at two the failures are identical."""
    panels = [
        ("tx", "AC1's transmitter fails: nobody hears AC1", 1, "AC1 true", "lower left",
         [(3, BLUE, "AC2's view of AC1 (link AC1->AC2, p 0.9)"),
          (4, RED, "AC3's view of AC1 (link AC1->AC3, p 0.5)")]),
        ("rx", "AC1's receiver fails: AC1 hears nobody", 2, "AC2 / AC3 true", "upper right",
         [(5, BLUE, "AC1's view of AC2 (link AC2->AC1, p 0.9)"),
          (6, RED, "AC1's view of AC3 (link AC3->AC1, p 0.5)")]),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.2), sharey=True)
    for ax, (subsystem, title, truth_col, truth_label, loc, series) in zip(axes, panels,
                                                                          strict=True):
        d = _outage(subsystem)
        ax.plot(d[:, 0], d[:, truth_col], color="0.55", lw=2.2, label=truth_label, zorder=2)
        for col, colour, label in series:
            ax.step(d[:, 0], d[:, col], where="post", color=colour, lw=1.7, label=label, zorder=3)
        ax.axvline(_T_FAIL, color="0.75", ls=":", lw=1.2)
        ax.set_xlabel("time [s]")
        ax.set_title(title, fontsize=10)
        ax.legend(fontsize=7.5, loc=loc)
        ax.set_box_aspect(1)
        err = abs(d[-1, series[0][0]] - d[-1, truth_col])
        print(f"        [{subsystem}] at {_T_MAX:.0f}s the good link is off by {err:.1f} m/s")
    axes[0].set_ylabel("ground speed [m/s]")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    plot(IMG / "comm-update-interval.png")
    latency_figure(IMG / "comm-latency.png")
    jitter_figure(IMG / "comm-jitter.png")
    radio_failure_figure(IMG / "comm-radio-failure.png")


if __name__ == "__main__":
    main()
