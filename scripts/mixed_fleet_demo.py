"""Mixed-fleet encounter: OWN flies DubinsDynamics, INT flies HolonomicDynamics.

A genuine two-aircraft conflict (detect -> resolve -> recover: StateBased + MVP + PastCPA), where
each aircraft is advanced by its *own* Dynamics model. CD/CR/CRR never change: they only ever read
`trk`/`gs` off `AircraftState`, and neither model changes what those mean (ADR 0009). This is the
"mixed fleet" case both ADR 0009's Consequences and `controlling-dubins-vs-holonomic.md`'s "What
this doesn't cover yet" flagged as untried.

`run_encounter` doesn't take per-aircraft `dynamics=`/`perf=` yet (that's a bigger, separate
change), so this threads the CDR decide step (`loop._decide`) manually — same pattern as
`scripts/trajectory_comparison/run_ours.py` — but calls each aircraft's own `Dynamics.step` to
advance it, instead of one shared `step_dynamics` call for both.

Usage:  python scripts/mixed_fleet_demo.py

Writes ``vault/observations/img/mixed-fleet-dubins-holonomic.png``. Backs
``vault/observations/mixed-fleet-dubins-holonomic.md``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import Command, DubinsDynamics, HolonomicDynamics  # noqa: E402
from opencdarr.loop import _INACTIVE, _decide  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

SPEED, RPZ, MARGIN = 10.2889, 50.0, 1.05
LOOKAHEAD, TLOS, DPSI, DCPA = 40.0, 25.0, 90.0, 0.0
DT, BCAST, T_MAX = 0.2, 1.0, 80.0
M_PER_DEG_LAT = 111320.0


def _spans(t: np.ndarray, active: np.ndarray) -> list[tuple[float, float]]:
    """Contiguous [start, end) intervals where `active` is truthy — for shading."""
    out: list[tuple[float, float]] = []
    start = None
    for i, a in enumerate(active):
        if a > 0.5 and start is None:
            start = t[i]
        elif a <= 0.5 and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def run() -> dict[str, np.ndarray]:
    det, res, rec = StateBased(), MVP(MARGIN), PastCPA(bouncing_guard=True)
    own_dyn, intr_dyn = DubinsDynamics(), HolonomicDynamics()

    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED)
    intr = create_conflict(own, intr_id="INT", dpsi=DPSI, dcpa=DCPA, tlos=TLOS, rpz=RPZ, side=1)
    nom_own = Command.from_track_speed(own.trk, own.gs)
    nom_intr = Command.from_track_speed(intr.trk, intr.gs)
    cmd_own, cmd_intr = nom_own, nom_intr
    mem_own = mem_intr = _INACTIVE

    m_lon = M_PER_DEG_LAT * np.cos(np.radians(52.0))
    t, next_bcast = 0.0, 0.0
    rows = []
    min_sep = float("inf")
    while t < T_MAX + 1e-9:
        if t + 1e-9 >= next_bcast:  # no noise: decide on the true states, once per broadcast tick
            cmd_own, mem_own = _decide(
                own, intr, nom_own, mem_own, RPZ, LOOKAHEAD, det, res, rec)
            cmd_intr, mem_intr = _decide(
                intr, own, nom_intr, mem_intr, RPZ, LOOKAHEAD, det, res, rec)
            next_bcast += BCAST

        _, sep = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        min_sep = min(min_sep, sep)
        rows.append((
            t,
            (own.lat - 52.0) * M_PER_DEG_LAT, (own.lon - 4.0) * m_lon, own.gs, own.trk,
            (intr.lat - 52.0) * M_PER_DEG_LAT, (intr.lon - 4.0) * m_lon, intr.gs, intr.trk,
            sep, float(mem_own.resolving), float(mem_intr.resolving),
        ))

        own = own_dyn.step(own, cmd_own, M600, DT)     # OWN: turn-rate-limited, coupled heading
        intr = intr_dyn.step(intr, cmd_intr, M600, DT)  # INT: isotropic accel, no coupled heading
        t += DT

    a = np.array(rows)
    keys = ("t", "own_n", "own_e", "own_gs", "own_trk", "int_n", "int_e", "int_gs", "int_trk",
             "sep", "own_active", "int_active")
    data = {k: a[:, i] for i, k in enumerate(keys)}
    data["min_sep"] = np.array([min_sep])
    return data


def plot(d: dict[str, np.ndarray], out: Path) -> None:
    own_spans = _spans(d["t"], d["own_active"])
    int_spans = _spans(d["t"], d["int_active"])

    fig, ax = plt.subplots(2, 2, figsize=(14, 10))

    a = ax[0, 0]
    a.plot(d["own_e"], d["own_n"], color="tab:orange", lw=2.0, label="OWN (DubinsDynamics)")
    a.plot(d["int_e"], d["int_n"], color="tab:blue", lw=2.0, label="INT (HolonomicDynamics)")
    a.scatter([d["own_e"][0]], [d["own_n"][0]], color="tab:orange", marker="^", s=60, zorder=5)
    a.scatter([d["int_e"][0]], [d["int_n"][0]], color="tab:blue", marker="^", s=60, zorder=5)
    a.set_xlabel("East [m]")
    a.set_ylabel("North [m]")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=9, loc="best")

    # zoom to the manoeuvre: the long straight cruise legs before/after would otherwise dwarf the
    # (much smaller) avoidance deflection at this scale. adjustable="box" (not the "datalim"
    # default) keeps these exact limits and reshapes the subplot box instead, so equal-aspect
    # doesn't silently override them back out to the full data range.
    all_spans = own_spans + int_spans
    if all_spans:
        pad = 5.0
        t0, t1 = min(s for s, _ in all_spans) - pad, max(e for _, e in all_spans) + pad
        mask = (d["t"] >= t0) & (d["t"] <= t1)
        xs = np.concatenate([d["own_e"][mask], d["int_e"][mask]])
        ys = np.concatenate([d["own_n"][mask], d["int_n"][mask]])
        mx, my = 0.1 * (xs.max() - xs.min() + 1.0), 0.1 * (ys.max() - ys.min() + 1.0)
        a.set_xlim(xs.min() - mx, xs.max() + mx)
        a.set_ylim(ys.min() - my, ys.max() + my)
        a.set_aspect("equal", adjustable="box")
        a.set_title("Ground tracks, zoomed to the manoeuvre (triangle = start)")
    else:
        a.axis("equal")
        a.set_title("Ground tracks (triangle = start)")

    a = ax[0, 1]
    a.plot(d["t"], d["sep"], color="k", lw=1.8, label="separation")
    a.axhline(RPZ, color="tab:red", lw=1.2, ls="--", label=f"rpz = {RPZ:.0f} m")
    for s, e in own_spans:
        a.axvspan(s, e, color="tab:orange", alpha=0.10, lw=0)
    for s, e in int_spans:
        a.axvspan(s, e, color="tab:blue", alpha=0.10, lw=0)
    a.set_xlabel("t [s]")
    a.set_ylabel("separation [m]")
    a.set_title("Separation over time (shaded = resolving: orange OWN, blue INT)")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=9)

    for col, (prefix, name, color) in enumerate(
        [("own", "OWN (Dubins)", "tab:orange"), ("int", "INT (Holonomic)", "tab:blue")]
    ):
        a = ax[1, col]
        spans = own_spans if prefix == "own" else int_spans
        for s, e in spans:
            a.axvspan(s, e, color=color, alpha=0.10, lw=0)
        a.plot(d["t"], d[f"{prefix}_gs"], color=color, lw=1.8, label="gs")
        a.set_xlabel("t [s]")
        a.set_ylabel("ground speed [m/s]")
        a2 = a.twinx()
        a2.plot(d["t"], d[f"{prefix}_trk"], color=color, lw=1.0, ls="--", alpha=0.6, label="trk")
        a2.set_ylabel("track [deg]")
        a.set_title(f"{name}: speed / track (shaded = resolving)")
        lines1, labels1 = a.get_legend_handles_labels()
        lines2, labels2 = a2.get_legend_handles_labels()
        a.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="best")
        a.grid(True, alpha=0.3)

    fig.suptitle(
        f"Mixed-fleet encounter: OWN (DubinsDynamics) vs INT (HolonomicDynamics), "
        f"dpsi={DPSI:.0f} deg, dcpa={DCPA:.0f} m — same CD/CR/CRR (StateBased+MVP+PastCPA)",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=105)
    print(f"wrote {out}")


def main() -> None:
    d = run()
    min_sep = float(d["min_sep"][0])
    own_resolving_time = float(np.sum(d["own_active"]) * DT)
    int_resolving_time = float(np.sum(d["int_active"]) * DT)
    own_min_gs = float(d["own_gs"].min())
    int_min_gs = float(d["int_gs"].min())
    print(f"min separation: {min_sep:.2f} m (rpz={RPZ:.0f} m) -> "
          f"{'LOSS OF SEPARATION' if min_sep < RPZ else 'clear'}")
    print(f"OWN (Dubins)    resolving for {own_resolving_time:5.1f} s, "
          f"min gs={own_min_gs:.3f} m/s")
    print(f"INT (Holonomic) resolving for {int_resolving_time:5.1f} s, "
          f"min gs={int_min_gs:.3f} m/s")

    out = (
        Path(__file__).resolve().parents[1]
        / "vault/observations/img/mixed-fleet-dubins-holonomic.png"
    )
    plot(d, out)


if __name__ == "__main__":
    main()
