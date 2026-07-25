"""Handbook figure: surveillance — hold-as-is over a lossy link.

The "Surveillance" page's picture, drawn from the real
:class:`~opencdarr.cns.communication.Comm` and
:class:`~opencdarr.cns.surveillance.LastKnown` models. A source flies a
**noise-free** but time-varying ground-speed profile and broadcasts it every
``INTERVAL``; the observer only receives each broadcast with probability
``RECEPTION`` (0.88). Between deliveries :class:`LastKnown` holds the last message
**unchanged** — no dead-reckoning — so the observed ground speed is a staircase
that lags the truth whenever an update is dropped, and lags most where the true
speed is changing fastest.

Handbook plot style: no suptitle, concise title. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/surveillance.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import rng  # noqa: E402
from opencdarr.cns.base import CommState, Message  # noqa: E402
from opencdarr.cns.communication import Comm  # noqa: E402
from opencdarr.cns.surveillance import LastKnown  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
INTERVAL, DURATION = 1.0, 60.0  # s — a broadcast every second, one-minute window
RECEPTION = 0.88  # probability the observer receives any one broadcast
SEED = 20260725
TRUE_COL, OBS_COL, DROP_COL = "0.15", "#1f77b4", "#d62728"


def true_gs(t: float) -> float:
    """The source's true ground speed [m/s] — a smooth, noise-free speed change."""
    return 24.0 + 6.0 * math.sin(2.0 * math.pi * t / 40.0)


def simulate() -> dict[str, list[float]]:
    """Step the source through the lossy link, recording the true and observed ground speed at
    every tick and which ticks delivered a fresh update."""
    comm = Comm(reception_prob={("SRC", "OBS"): RECEPTION})  # latency 0 — this is about drops
    surveil = LastKnown()
    gen = rng.generator(rng.root_seed_sequence(SEED))
    state = CommState()

    times, true, observed, drops = [], [], [], []
    last_tmeas: float | None = None
    for k in range(int(DURATION / INTERVAL) + 1):
        t = k * INTERVAL
        source = AircraftState(id="SRC", lat=52.0, lon=4.0, trk=90.0, gs=true_gs(t), vel_ci95=0.0)
        state = comm.step(state, [Message("SRC", source, t)], ["SRC", "OBS"], t, gen)
        held = surveil.perceived(state, "OBS", "SRC", t)

        times.append(t)
        true.append(source.gs)
        observed.append(math.nan if held is None else held.gs)
        delivered = state.held.get(("OBS", "SRC"))
        if delivered is not None and delivered.t_meas != last_tmeas:
            last_tmeas = delivered.t_meas  # a fresh message landed this tick
        elif k > 0:
            drops.append(t)  # nothing new arrived — the observer holds the stale value
    return {"t": times, "true": true, "observed": observed, "drops": drops}


def plot(out: Path) -> None:
    r = simulate()
    n_drop = len(r["drops"])
    print(f"     reception {RECEPTION}: {n_drop}/{int(DURATION)} ticks dropped, "
          f"mean update interval ~ {INTERVAL / RECEPTION:.2f} s")

    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    ax.plot(r["t"], r["true"], color=TRUE_COL, lw=2.2, label="true ground speed (no noise)")
    ax.step(r["t"], r["observed"], where="post", color=OBS_COL, lw=1.6,
            label=f"observed (reception {RECEPTION}, hold-as-is)")
    received = [t for t in r["t"] if t not in set(r["drops"])]
    obs_at = dict(zip(r["t"], r["observed"], strict=True))
    ax.scatter(received, [obs_at[t] for t in received], s=16, color=OBS_COL, zorder=3)
    for i, t in enumerate(r["drops"]):
        ax.axvline(t, color=DROP_COL, lw=0.8, alpha=0.35, zorder=0,
                   label="dropped update (observer holds stale)" if i == 0 else None)

    ax.set_xlabel("time [s]")
    ax.set_ylabel("ground speed [m/s]")
    ax.set_title("Hold-as-is: the observed speed goes stale on every dropped update", fontsize=10)
    ax.set_xlim(0, DURATION)
    ax.legend(fontsize=9, loc="upper right", framealpha=0.9)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


# --- asymmetric perception: one encounter, two directed links of different quality -------------
REC_AB, REC_BA = 0.90, 0.50  # A->B is reliable; B->A is lossy (per-link reception, ADR 0004)
GREEN = "#2ca02c"


def true_gs_for(craft: str, t: float) -> float:
    """Each aircraft flies its own noise-free speed profile, so the two panels are distinguishable."""
    return (24.0 + 6.0 * math.sin(2.0 * math.pi * t / 40.0) if craft == "A"
            else 21.0 + 5.0 * math.sin(2.0 * math.pi * t / 40.0 + math.pi / 2.0))


def simulate_pair() -> dict[tuple[str, str], dict[str, list[float]]]:
    """A and B both broadcast their noise-free state every tick over one asymmetric channel; record,
    per directed link (source -> receiver), the source's true speed and the receiver's held view."""
    comm = Comm(reception_prob={("A", "B"): REC_AB, ("B", "A"): REC_BA})
    surveil = LastKnown()
    gen = rng.generator(rng.root_seed_sequence(SEED))
    state = CommState()

    links = {("A", "B"): _blank(), ("B", "A"): _blank()}
    last_tmeas: dict[tuple[str, str], float | None] = {("A", "B"): None, ("B", "A"): None}
    for k in range(int(DURATION / INTERVAL) + 1):
        t = k * INTERVAL
        states = {c: AircraftState(id=c, lat=52.0, lon=4.0, trk=90.0,
                                   gs=true_gs_for(c, t), vel_ci95=0.0) for c in ("A", "B")}
        state = comm.step(state, [Message(c, states[c], t) for c in ("A", "B")], ["A", "B"], t, gen)
        for src, rcv in links:
            held = surveil.perceived(state, rcv, src, t)
            rec = links[(src, rcv)]
            rec["t"].append(t)
            rec["true"].append(states[src].gs)
            rec["observed"].append(math.nan if held is None else held.gs)
            delivered = state.held.get((rcv, src))
            if delivered is not None and delivered.t_meas != last_tmeas[(src, rcv)]:
                last_tmeas[(src, rcv)] = delivered.t_meas
            elif k > 0:
                rec["drops"].append(t)
    return links


def _blank() -> dict[str, list[float]]:
    return {"t": [], "true": [], "observed": [], "drops": []}


def plot_asymmetric(out: Path) -> None:
    links = simulate_pair()
    panels = [(("A", "B"), REC_AB, "B's view of A", OBS_COL),
              (("B", "A"), REC_BA, "A's view of B", GREEN)]
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.0), sharey=True)
    for ax, ((src, rcv), rec_p, title, col) in zip(axes, panels, strict=True):
        r = links[(src, rcv)]
        n_drop = len(r["drops"])
        print(f"     {src}->{rcv} reception {rec_p}: {n_drop}/{int(DURATION)} ticks dropped")
        ax.plot(r["t"], r["true"], color=TRUE_COL, lw=2.2, label=f"{src} true ground speed")
        ax.step(r["t"], r["observed"], where="post", color=col, lw=1.6,
                label=f"{rcv} holds (reception {rec_p})")
        received = [t for t in r["t"] if t not in set(r["drops"])]
        obs_at = dict(zip(r["t"], r["observed"], strict=True))
        ax.scatter(received, [obs_at[t] for t in received], s=14, color=col, zorder=3)
        for i, t in enumerate(r["drops"]):
            ax.axvline(t, color=DROP_COL, lw=0.8, alpha=0.32, zorder=0,
                       label="dropped update" if i == 0 else None)
        ax.set_title(f"{title} — {n_drop} of {int(DURATION)} updates dropped", fontsize=10)
        ax.set_xlabel("time [s]")
        ax.set_xlim(0, DURATION)
        ax.legend(fontsize=9, loc="upper right", framealpha=0.9)
    axes[0].set_ylabel("ground speed [m/s]")

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    plot(IMG / "surveillance-hold-as-is.png")
    plot_asymmetric(IMG / "surveillance-asymmetric.png")


if __name__ == "__main__":
    main()
