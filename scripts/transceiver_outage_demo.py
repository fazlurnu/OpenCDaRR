"""Radio outages: what a failed transmitter and a failed receiver each do to perception.

:class:`~opencdarr.cns.TransceiverComm` loses *radios* where :class:`~opencdarr.cns.Comm` loses
*messages*, and it splits the radio in two because a transmitter and a receiver fail independently.
This drives the real CNS stack (`GnssNavigation` -> `TransceiverComm` -> `LastKnown`) across a
failure at ``t = 10 s`` and reads the perceived ground speed back against the truth.

Two figures, answering two different questions:

* **perception** — OWN's own radio fails, transmitter in one case and receiver in the other. The
  two are mirror images: a silent aircraft keeps its own picture and corrupts everyone else's; a
  deaf one corrupts only its own. Its *self*-fix survives both, because an aircraft's own
  measurement never goes over the air ([[surveillance-hold-as-is]] holds the rest).
* **observability** — whether a study can tell the two failures apart at all. At ``n = 2`` "INT's
  transmitter died" and "OWN's receiver died" sever the *same single link* and are
  indistinguishable in the state; at ``n = 3`` they separate.

All four hazard rates are zero and the outage is imposed on the
:class:`~opencdarr.cns.RadioHealth` gate's state directly, so the failure lands on the tick we name
rather than on a lucky draw — the same trick the gate tests in
``tests/test_cns_transceiver.py`` use. Reproduce::

    PYTHONPATH=. python scripts/transceiver_outage_demo.py

Writes ``vault/observations/img/transceiver-outage-perception.png`` and
``vault/observations/img/transceiver-outage-observability.png``.
"""
from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.cns import (  # noqa: E402
    CNS,
    CnsState,
    CnsStreams,
    GnssNavigation,
    LastKnown,
    RadioHealthState,
    TransceiverComm,
    radio_health,
)
from opencdarr.cns.surveillance import age  # noqa: E402
from opencdarr.rng import children, generator, root_seed_sequence  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

T_FAIL = 10.0  # when the radio dies [s]
T_MAX = 30.0
INTERVAL = 1.0  # broadcast cadence [s]
POS_CI95, VEL_CI95 = 10.0, 0.8  # GNSS self-measurement accuracy
SEED = 11

TRUTH_KW = dict(color="#333333", lw=2.0, zorder=2)
BELIEF_KW = dict(color="#c1121f", lw=1.6, marker="o", ms=3.5, drawstyle="steps-post", zorder=3)
SELF_KW = dict(color="#0466c8", lw=1.4, ls="--", zorder=3)


def _stack(n: int, noisy: bool) -> tuple[CNS, CnsState, CnsStreams]:
    """The real stack with every hazard rate at zero — the outage is scripted, not drawn."""
    nav_seq, comm_seq = children(root_seed_sequence(SEED), 0, 2)
    cns = CNS(
        navigation=GnssNavigation() if noisy else None,
        communication=TransceiverComm(reception_prob=1.0),  # lossless: isolate the outage
        surveillance=LastKnown(),
    )
    return cns, cns.initial_state(n), CnsStreams(
        nav=generator(nav_seq), comm=generator(comm_seq)
    )


def _fail(state: CnsState, aid: str, subsystem: str) -> CnsState:
    """Put ``aid``'s transmitter or receiver down, leaving the rest of the comm state alone.

    The health lives in the `RadioHealth` gate's own state, which rides positionally in
    ``CommState.gates`` -- so this rebuilds that one gate state and threads the rest through.
    """
    down = {"tx_down": frozenset({aid})} if subsystem == "tx" else {"rx_down": frozenset({aid})}
    healthy = replace(radio_health(state.comm), **down)
    gates = tuple(
        healthy if isinstance(own, RadioHealthState) else own for own in state.comm.gates
    )
    return replace(state, comm=replace(state.comm, gates=gates))


def _truth(aid: str, t: float) -> AircraftState:
    """Ground truth. Both speeds vary, so a belief frozen by an outage diverges visibly."""
    gs = (10.0 + 3.0 * math.sin(2.0 * math.pi * t / 20.0) if aid == "OWN"
          else 14.0 + 4.0 * math.sin(2.0 * math.pi * t / 15.0 + 1.0))
    return AircraftState(
        id=aid, lat=52.0, lon=4.0 + (0.01 if aid == "INT" else 0.0), trk=0.0, gs=gs,
        pos_ci95=POS_CI95, vel_ci95=VEL_CI95,
    )


def perception_run(subsystem: str) -> dict[str, list[float]]:
    """OWN's own ``subsystem`` fails at T_FAIL; track both sides' beliefs against the truth."""
    cns, state, streams = _stack(2, noisy=True)
    keys = ("t", "own_true", "int_true", "own_sees_int", "int_sees_own", "own_sees_self")
    out: dict[str, list[float]] = {k: [] for k in keys}

    t = 0.0
    while t <= T_MAX + 1e-9:
        if t >= T_FAIL:
            state = _fail(state, "OWN", subsystem)
        states = [_truth("OWN", t), _truth("INT", t)]
        state, perception = cns.sense(states, (0, 1), t, state, streams)

        own_sees = {ac.id: ac.gs for ac in perception[0].traffic}
        int_sees = {ac.id: ac.gs for ac in perception[1].traffic}
        out["t"].append(t)
        out["own_true"].append(states[0].gs)
        out["int_true"].append(states[1].gs)
        out["own_sees_self"].append(perception[0].own.gs)
        out["own_sees_int"].append(own_sees.get("INT", float("nan")))
        out["int_sees_own"].append(int_sees.get("OWN", float("nan")))
        t += INTERVAL
    return out


def staleness(ids: list[str], aid: str, subsystem: str, t_end: float = 15.0) -> dict[str, float]:
    """Age of every directed link at ``t_end`` after ``aid``'s ``subsystem`` fails at T_FAIL."""
    cns, state, streams = _stack(len(ids), noisy=False)
    t = 0.0
    while t <= t_end + 1e-9:
        if t >= T_FAIL:
            state = _fail(state, aid, subsystem)
        truths = [_truth_n(ids, i, t) for i in ids]
        state, _ = cns.sense(truths, range(len(ids)), t, state, streams)
        t += INTERVAL
    return {f"{r}<-{s}": age(state.comm, r, s, t_end) or 0.0
            for r in ids for s in ids if r != s}


def _truth_n(ids: list[str], aid: str, t: float) -> AircraftState:
    """Truth for the n-aircraft observability run: distinct, constant speeds, no noise."""
    i = ids.index(aid)
    return AircraftState(id=aid, lat=52.0, lon=4.0 + 0.01 * i, trk=0.0, gs=10.0 + i,
                         pos_ci95=0.0, vel_ci95=0.0)


def figure_perception(out_dir: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.4), sharex=True, sharey=True)
    for col, (subsystem, label) in enumerate(
        (("tx", "OWN transmitter fails"), ("rx", "OWN receiver fails"))
    ):
        d = perception_run(subsystem)
        top, bot = axes[0][col], axes[1][col]

        top.plot(d["t"], d["int_true"], label="INT true gs", **TRUTH_KW)
        top.plot(d["t"], d["own_sees_int"], label="OWN's belief of INT", **BELIEF_KW)
        top.plot(d["t"], d["own_sees_self"], label="OWN's own self-fix", **SELF_KW)
        top.set_title(f"{label} — what OWN sees", fontsize=10)

        bot.plot(d["t"], d["own_true"], label="OWN true gs", **TRUTH_KW)
        bot.plot(d["t"], d["int_sees_own"], label="INT's belief of OWN", **BELIEF_KW)
        bot.set_title(f"{label} — what INT sees", fontsize=10)

        for ax in (top, bot):
            ax.axvline(T_FAIL, color="#888888", lw=1.0, ls=":")
            ax.legend(fontsize=8, loc="upper right", framealpha=0.9)
        bot.set_xlabel("time [s]")

    for row in (0, 1):
        axes[row][0].set_ylabel("ground speed [m/s]")
    fig.tight_layout()
    path = out_dir / "transceiver-outage-perception.png"
    fig.savefig(path, dpi=120)
    print("wrote", path)


def figure_observability(out_dir: Path) -> None:
    """Can a study tell the two failures apart? Compare INT-transmitter against OWN-receiver."""
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.8))
    for ax, ids in zip(axes, (["OWN", "INT"], ["OWN", "INT", "3RD"]), strict=True):
        a = staleness(ids, "INT", "tx")
        b = staleness(ids, "OWN", "rx")
        links = list(a)
        x = np.arange(len(links))
        ax.bar(x - 0.2, [a[k] for k in links], 0.4, label="INT transmitter down", color="#c1121f")
        ax.bar(x + 0.2, [b[k] for k in links], 0.4, label="OWN receiver down", color="#0466c8")
        ax.set_xticks(x)
        ax.set_xticklabels(links, fontsize=8, rotation=30, ha="right")
        ax.set_ylabel("staleness at t = 15 s [s]")
        verdict = "indistinguishable" if a == b else "distinguishable"
        ax.set_title(f"n = {len(ids)} — {verdict}", fontsize=10)
        ax.legend(fontsize=8, loc="upper left", framealpha=0.9)
    fig.tight_layout()
    path = out_dir / "transceiver-outage-observability.png"
    fig.savefig(path, dpi=120)
    print("wrote", path)


def main() -> None:
    out_dir = Path(__file__).resolve().parents[1] / "vault/observations/img"
    figure_perception(out_dir)
    figure_observability(out_dir)

    for subsystem in ("tx", "rx"):
        d = perception_run(subsystem)
        i = d["t"].index(T_MAX)
        print(f"\n[OWN {subsystem} down] at t = {T_MAX:.0f} s")
        print(f"  INT true gs {d['int_true'][i]:6.2f}   OWN believes {d['own_sees_int'][i]:6.2f}"
              f"   error {d['own_sees_int'][i] - d['int_true'][i]:+6.2f}")
        print(f"  OWN true gs {d['own_true'][i]:6.2f}   INT believes {d['int_sees_own'][i]:6.2f}"
              f"   error {d['int_sees_own'][i] - d['own_true'][i]:+6.2f}")
        print(f"  OWN self-fix {d['own_sees_self'][i]:5.2f}  (never sent over the air)")

    for ids in (["OWN", "INT"], ["OWN", "INT", "3RD"]):
        same = staleness(ids, "INT", "tx") == staleness(ids, "OWN", "rx")
        print(f"\nn = {len(ids)}: INT-transmitter-down and OWN-receiver-down give the same "
              f"comm state: {same}")


if __name__ == "__main__":
    main()
