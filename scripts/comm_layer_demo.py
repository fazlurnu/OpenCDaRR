"""Demonstrate the communication layer: Bernoulli reception + lognormal latency.

Runs the **real** :class:`~opencdarr.cns.Comm` over many broadcast ticks for one asymmetric
pair — OWN→INT delivers 80% of the time, INT→OWN 99% — and measures what actually came out:
delivery rates, the latency distribution, and how *stale* the information each aircraft is
holding actually is.

Usage:  python scripts/comm_layer_demo.py

Writes ``vault/observations/img/comm-reception-latency.png``. Backs
``vault/observations/communication-reception-latency.md``.
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.cns import Comm, CommState, Message, lognormal_latency  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

RECEIVERS = ("OWN", "INT")
LINKS = (("OWN", "INT"), ("INT", "OWN"))  # (source, receiver)
RECEPTION = {("OWN", "INT"): 0.80, ("INT", "OWN"): 0.99}
LAT_MEDIAN, LAT_SIGMA = 0.1, 0.25  # s
BROADCAST_INTERVAL = 1.0
N_TICKS = 4000
TIMELINE_S = 60.0


def _broadcast(source: str, t: float) -> Message:
    return Message(
        source=source,
        state=AircraftState(id=source, lat=52.0, lon=4.0, trk=0.0, gs=10.0),
        t_meas=t,
    )


def run() -> dict:
    comm = Comm(reception_prob=RECEPTION, latency=lognormal_latency(LAT_MEDIAN, LAT_SIGMA))
    # own substream for the comm layer, per ADR 0001/0006
    (comm_seq,) = spawn(root_seed_sequence(0), 1)
    rng = generator(comm_seq)

    state = CommState()
    offered = {link: 0 for link in LINKS}
    received = {link: 0 for link in LINKS}
    latencies = {link: [] for link in LINKS}
    arrivals = {link: [] for link in LINKS}  # deliver_t of every message actually received
    ages = {link: [] for link in LINKS}
    timeline = {link: ([], []) for link in LINKS}  # (t, age)

    for i in range(N_TICKS):
        t = i * BROADCAST_INTERVAL
        state = comm.step(state, [_broadcast(s, t) for s in RECEIVERS], RECEIVERS, t, rng)

        for source, receiver in LINKS:
            offered[(source, receiver)] += 1
            # a message just accepted by this link is still in flight (latency > 0)
            for pending in state.in_flight:
                if pending.message.source == source and pending.receiver == receiver \
                        and pending.message.t_meas == t:
                    received[(source, receiver)] += 1
                    latencies[(source, receiver)].append(pending.deliver_t - t)
                    arrivals[(source, receiver)].append(pending.deliver_t)
                    break
            # how stale is the information this receiver is currently acting on?
            held = state.held.get((receiver, source))
            if held is not None:
                age = t - held.t_meas
                ages[(source, receiver)].append(age)
                if t <= TIMELINE_S:
                    timeline[(source, receiver)][0].append(t)
                    timeline[(source, receiver)][1].append(age)

    # inter-arrival gap: time between one received message and the next, on the same link
    gaps = {}
    for link, times in arrivals.items():
        gaps[link] = np.diff(np.sort(np.array(times)))

    return {
        "offered": offered, "received": received,
        "latencies": {k: np.array(v) for k, v in latencies.items()},
        "gaps": gaps,
        "ages": {k: np.array(v) for k, v in ages.items()},
        "timeline": timeline,
    }


def plot(data: dict, out: Path) -> None:
    colors = {LINKS[0]: "tab:blue", LINKS[1]: "tab:orange"}
    label = {link: f"{link[0]}→{link[1]}  (p={RECEPTION[link]:.2f})" for link in LINKS}
    fig = plt.figure(figsize=(14, 13))
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1])
    ax = np.array(
        [[fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])],
         [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]]
    )

    # 1. Bernoulli reception: measured vs configured
    a = ax[0, 0]
    xs = np.arange(len(LINKS))
    empirical = [data["received"][k] / data["offered"][k] for k in LINKS]
    nominal = [RECEPTION[k] for k in LINKS]
    a.bar(xs - 0.18, empirical, 0.36, label="measured", color=[colors[k] for k in LINKS])
    a.bar(xs + 0.18, nominal, 0.36, label="configured", color="lightgrey", edgecolor="k")
    for x, (e, n) in enumerate(zip(empirical, nominal)):
        a.text(x - 0.18, e + 0.012, f"{e:.3f}", ha="center", fontsize=9)
        a.text(x + 0.18, n + 0.012, f"{n:.2f}", ha="center", fontsize=9)
    a.set_xticks(xs, [f"{s}→{r}" for s, r in LINKS])
    a.set_ylim(0, 1.15); a.set_ylabel("delivery rate")
    a.set_title(f"1. Bernoulli reception per directed link  ({N_TICKS} broadcasts each)")
    a.legend(loc="lower right"); a.grid(True, axis="y", alpha=0.3)

    # 2. latency distribution vs the lognormal it was drawn from
    a = ax[0, 1]
    pooled = np.concatenate([data["latencies"][k] for k in LINKS])
    a.hist(pooled, bins=80, density=True, color="tab:green", alpha=0.45,
           label=f"measured (n={pooled.size})")
    grid = np.linspace(1e-3, float(np.quantile(pooled, 0.999)), 400)
    mu = math.log(LAT_MEDIAN)
    pdf = np.exp(-((np.log(grid) - mu) ** 2) / (2 * LAT_SIGMA**2)) / (
        grid * LAT_SIGMA * math.sqrt(2 * math.pi))
    a.plot(grid, pdf, "k-", lw=2, label=f"LogNormal(median={LAT_MEDIAN}, σ={LAT_SIGMA})")
    a.axvline(LAT_MEDIAN, color="k", ls="--", lw=1.2, label=f"median = {LAT_MEDIAN}s")
    a.axvline(float(pooled.mean()), color="tab:red", ls=":", lw=1.5,
              label=f"mean = {pooled.mean():.3f}s (skew)")
    a.set_xlabel("link latency [s]"); a.set_ylabel("density")
    a.set_title("2. Latency is lognormal — positive, right-skewed")
    a.legend(fontsize=8); a.grid(True, alpha=0.3)

    # 3. how stale is the held information?
    a = ax[1, 0]
    hi = max(float(data["ages"][k].max()) for k in LINKS) + 1.0
    bins = np.arange(0, hi, 0.25)
    for link in LINKS:
        a.hist(data["ages"][link], bins=bins, density=True, alpha=0.55,
               color=colors[link], label=f"{label[link]}  mean={data['ages'][link].mean():.2f}s")
    a.set_xlim(0, hi)
    a.set_xlabel("age of held message when used [s]"); a.set_ylabel("density")
    a.set_title("3. Staleness of the information each receiver acts on")
    a.legend(fontsize=8); a.grid(True, alpha=0.3)

    # 4. staleness over time — the sawtooth, and the taller spikes on the lossy link
    a = ax[1, 1]
    for link in LINKS:
        ts, age = data["timeline"][link]
        a.step(ts, age, where="post", color=colors[link], lw=1.4, label=label[link])
    a.set_xlabel("t [s]"); a.set_ylabel("age of held message [s]")
    a.set_title(f"4. Staleness over time (first {TIMELINE_S:.0f}s) — drops make the spikes")
    a.legend(fontsize=8); a.grid(True, alpha=0.3)

    # 5. inter-arrival gap: time between one *received* message and the next, per link.
    # Bin counts computed explicitly with np.histogram (not ax.hist) and drawn as bars.
    a = fig.add_subplot(gs[2, :])
    hi_gap = max(float(data["gaps"][k].max()) for k in LINKS) + 1.0
    bin_edges = np.arange(0.0, hi_gap, 0.02)
    for link in LINKS:
        counts, edges = np.histogram(data["gaps"][link], bins=bin_edges, density=True)
        width = edges[1] - edges[0]
        mean_gap = float(data["gaps"][link].mean())
        a.bar(edges[:-1], counts, width=width, align="edge", alpha=0.55, color=colors[link],
              label=f"{label[link]}  mean gap={mean_gap:.3f}s")
        expected = BROADCAST_INTERVAL / RECEPTION[link]  # E[gap] for a Bernoulli(p) process
        a.axvline(expected, color=colors[link], ls="--", lw=1.3,
                  label=f"  E[gap] = interval/p = {expected:.3f}s")
    a.set_xlim(0, hi_gap)
    a.set_xlabel("time between consecutive received messages [s]"); a.set_ylabel("density")
    a.set_title("5. Inter-arrival gap per link — humps at k·interval are runs of k−1 drops "
                "(np.histogram)")
    a.legend(fontsize=8, ncol=2); a.grid(True, alpha=0.3)

    fig.suptitle(
        "Communication layer — Bernoulli reception + lognormal latency, asymmetric links",
        fontsize=13)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=105)
    print(f"wrote {out}")


def main() -> None:
    data = run()
    print(f"reception (measured vs configured), {N_TICKS} broadcasts per link:")
    for link in LINKS:
        rate = data["received"][link] / data["offered"][link]
        print(f"  {link[0]}→{link[1]}: {rate:.4f}  (configured {RECEPTION[link]:.2f})")
    pooled = np.concatenate([data["latencies"][k] for k in LINKS])
    print(f"latency: median={np.median(pooled):.4f}s mean={pooled.mean():.4f}s "
          f"p99={np.quantile(pooled, 0.99):.4f}s max={pooled.max():.4f}s")
    for link in LINKS:
        ages = data["ages"][link]
        print(f"held age {link[0]}→{link[1]}: mean={ages.mean():.3f}s "
              f"p95={np.quantile(ages, 0.95):.3f}s max={ages.max():.3f}s")
    for link in LINKS:
        gaps = data["gaps"][link]
        expected = BROADCAST_INTERVAL / RECEPTION[link]
        print(f"inter-arrival gap {link[0]}→{link[1]}: mean={gaps.mean():.3f}s "
              f"(E[interval/p]={expected:.3f}s) max={gaps.max():.3f}s")
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/comm-reception-latency.png"
    plot(data, out)


if __name__ == "__main__":
    main()
