"""Demonstrate hold-as-is surveillance: prove ``perceived`` never moves between deliveries.

Runs the **real** :class:`~opencdarr.cns.Comm` and :class:`~opencdarr.cns.LastKnown` against one
source aircraft whose *true* ground speed changes continuously (ramp / hold / ramp), broadcasting
once per second over a lossy link (``reception_prob = 0.6``, so drops are frequent enough to see).
Plots truth (continuous) against what the receiver actually perceives (a step function that only
moves on delivery) — the defining, testable property of hold-as-is.

Usage:  python scripts/surveillance_demo.py

Writes ``vault/observations/img/surveillance-hold-as-is.png``. Backs
``vault/observations/surveillance-hold-as-is.md``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.cns import Comm, CommState, LastKnown, Message, age  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

SOURCE, RECEIVER = "INT", "OWN"
RECEPTION_PROB = 0.6  # deliberately lossy so drops are frequent and visible
BROADCAST_INTERVAL = 1.0
DT = 0.05  # true-state sampling for the smooth ground-truth curve
T_MAX = 30.0


def true_gs(t: float) -> float:
    """A hand-shaped ground-speed profile: ramp up, hold, ramp down, hold — never flat for long,
    so any drop is visible as perceived failing to track it."""
    if t < 6.0:
        return 10.0 + 1.0 * t
    if t < 12.0:
        return 16.0
    if t < 20.0:
        return 16.0 - 1.5 * (t - 12.0)
    return 4.0 + 0.6 * (t - 20.0)


def run() -> dict:
    surveillance = LastKnown()
    comm = Comm(reception_prob=RECEPTION_PROB, latency=0.0)  # isolate the drop effect: no delay
    (comm_seq,) = spawn(root_seed_sequence(1), 1)
    rng = generator(comm_seq)

    state = CommState()
    rows_true: list[tuple[float, float]] = []
    rows_perceived: list[tuple[float, float, float]] = []  # t, gs, age
    delivered_at: list[tuple[float, float]] = []  # t, gs — markers for actual deliveries

    t = 0.0
    next_broadcast = 0.0
    last_seen_t_meas: float | None = None
    while t <= T_MAX + 1e-9:
        gs = true_gs(t)
        rows_true.append((t, gs))

        if t + 1e-9 >= next_broadcast:
            msg = Message(
                source=SOURCE,
                state=AircraftState(id=SOURCE, lat=52.0, lon=4.0, trk=90.0, gs=true_gs(t)),
                t_meas=t,
            )
            state = comm.step(state, [msg], (SOURCE, RECEIVER), t, rng)
            next_broadcast += BROADCAST_INTERVAL

        perceived = surveillance.perceived(state, RECEIVER, SOURCE, t)
        a = age(state, RECEIVER, SOURCE, t)
        if perceived is not None:
            rows_perceived.append((t, perceived.gs, a))
            held = state.held[(RECEIVER, SOURCE)]
            if held.t_meas != last_seen_t_meas:  # a fresh delivery just landed
                delivered_at.append((t, perceived.gs))
                last_seen_t_meas = held.t_meas

        t += DT

    return {
        "true": np.array(rows_true),
        "perceived": np.array(rows_perceived),
        "delivered": np.array(delivered_at),
    }


def plot(data: dict, out: Path) -> None:
    true_t, true_gs_ = data["true"][:, 0], data["true"][:, 1]
    p_t, p_gs, p_age = data["perceived"][:, 0], data["perceived"][:, 1], data["perceived"][:, 2]
    d_t, d_gs = data["delivered"][:, 0], data["delivered"][:, 1]

    fig, ax = plt.subplots(2, 1, figsize=(13, 8), sharex=True, height_ratios=[2, 1])

    a = ax[0]
    a.plot(true_t, true_gs_, color="tab:red", lw=1.6, label="true gs(t) — continuous")
    a.step(p_t, p_gs, where="post", color="tab:blue", lw=1.8,
           label="perceived gs(t) — holds flat between deliveries")
    a.scatter(d_t, d_gs, color="tab:blue", s=28, zorder=5, label="message delivered here")
    a.set_ylabel("ground speed [m/s]")
    a.set_title(f"Hold-as-is surveillance: {SOURCE}'s true state vs what {RECEIVER} perceives "
                f"(reception_prob={RECEPTION_PROB})")
    a.legend(loc="upper right", fontsize=9); a.grid(True, alpha=0.3)

    a = ax[1]
    a.plot(p_t, p_age, color="tab:blue", lw=1.4)
    for t in d_t:
        a.axvline(t, color="tab:blue", lw=0.5, alpha=0.25)
    a.set_xlabel("t [s]"); a.set_ylabel("age of\nperceived msg [s]")
    a.set_title("Staleness: resets to 0 exactly at each delivery, grows linearly between")
    a.grid(True, alpha=0.3)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=105)
    print(f"wrote {out}")


def main() -> None:
    data = run()
    p_gs, p_age = data["perceived"][:, 1], data["perceived"][:, 2]
    n_deliveries = len(data["delivered"])
    # correctness check: age must reset to (near) 0 at every delivery timestamp and only there
    at_delivery_ages = []
    for dt_, _ in data["delivered"]:
        idx = np.argmin(np.abs(data["perceived"][:, 0] - dt_))
        at_delivery_ages.append(data["perceived"][idx, 2])
    print(f"deliveries: {n_deliveries} (expected ~{RECEPTION_PROB * T_MAX:.0f} "
          f"of {int(T_MAX)} broadcasts)")
    print(f"age at delivery instants: max={max(at_delivery_ages):.4f}s (should be ~0)")
    print(f"max age reached between deliveries: {p_age.max():.2f}s")
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/surveillance-hold-as-is.png"
    plot(data, out)


if __name__ == "__main__":
    main()
