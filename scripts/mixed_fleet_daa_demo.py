"""Phase-4e demo: mixed-fleet DAA — how a fixed-wing and a multirotor fly the *same* avoidance.

MVP emits a vehicle-neutral avoidance **velocity**. A multirotor takes it as a native setpoint; a
fixed-wing cannot (it flies course + airspeed), so the loop projects the velocity onto its channels
(``project_to_fixedwing``) and the airframe **converges** to it under a bank-limited turn and a
stall-floored airspeed (ADR 0013 §4, Phase 4e).

To isolate the airframe response, one ownship of each type resolves the **same** conflict against
the **same** non-cooperative intruder (StateBased + MVP + PastCPA). The contrast:

- the **multirotor** changes its velocity almost freely — it can slow well down and side-step;
- the **fixed-wing** cannot slow below stall (``v_min``) and can only *turn* through a feasible,
  bank-limited arc, so it resolves the same geometry mostly by heading, and converges to the
  commanded course rather than snapping to it.

The headline ``min_sep`` from ``run_encounter`` with a real mixed pair (**both** running DAA, each
its own ``kinematics``/``perf`` — ADR 0011 §7) is printed too, tying the picture back to the gate.
In the ``mixed-fleet-dubins-holonomic`` lineage (that demo's Dubins side is now a real FixedWing).

Writes ``vault/observations/img/mixed-fleet-daa.png``.

    PYTHONPATH=. python scripts/mixed_fleet_daa_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.kinematics import FixedWing, Kinematics, MotionCommand, Multirotor  # noqa: E402
from opencdarr.loop import run_encounter  # noqa: E402
from opencdarr.performance import M600, SMALL_FIXEDWING, Performance  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager, project_to_fixedwing  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0 = 52.0, 4.0
RPZ, LOOKAHEAD = 50.0, 120.0
DT, BCAST, T_MAX = 0.2, 1.0, 130.0
GS, DPSI, TLOS, MARGIN = 15.0, 90.0, 50.0, 1.6
_FW, _MR = FixedWing(), Multirotor()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _pair(own_gs: float) -> tuple[AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=own_gs, yaw=0.0, bank=0.0)
    intr = create_conflict(own, intr_id="INT", dpsi=DPSI, dcpa=0.0, tlos=TLOS, rpz=RPZ, side=1)
    return own, intr


def avoid(kinematics: Kinematics, perf: Performance, *, project: bool) -> list[tuple[float, ...]]:
    """One ownship (``kinematics``/``perf``) resolves against a straight, non-cooperative intruder.

    ``project=True`` wires the fixed-wing velocity→course/airspeed adapter (Phase 4e); a multirotor
    passes ``project=False`` and flies the resolver velocity directly.
    """
    own, intr = _pair(GS)
    sep = SeparationManager()
    det, res, rec = StateBased(), MVP(margin=MARGIN), PastCPA(bouncing_guard=True)
    nom = MotionCommand.from_track_speed(own.trk, own.gs)  # frozen cruise nominal
    intr_cmd = MotionCommand.from_track_speed(intr.trk, intr.gs)  # INT holds its track (no DAA)
    adapter = (lambda c: project_to_fixedwing(c, perf)) if project else None
    mem = INACTIVE
    cmd = adapter(nom) if adapter is not None else nom

    rows: list[tuple[float, ...]] = []
    t, nb = 0.0, 0.0
    while t < T_MAX:
        if t + 1e-9 >= nb:
            cmd, mem = sep.step(own, [intr], nom, mem, RPZ, LOOKAHEAD, det, res, rec, adapter)
            nb += BCAST
        oe, on = _enu(own.lat, own.lon)
        ie, in_ = _enu(intr.lat, intr.lon)
        _, sepd = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        heading = own.yaw if own.yaw is not None else own.trk
        # the track the command asks the airframe to fly (fixed-wing: course channel; multirotor:
        # direction of its velocity channel) vs the track it is currently making good -> the gap is
        # the convergence lag (a multirotor closes it near-instantly; a fixed-wing bank-limited)
        cmd_track = cmd.target_course if cmd.target_course is not None else cmd.trk
        gap = abs(((own.trk - cmd_track + 180.0) % 360.0) - 180.0)
        rows.append((t, oe, on, ie, in_, sepd, heading, own.bank, own.gs,
                     float(mem.resolving), gap))
        own = kinematics.step(own, cmd, perf, DT)
        intr = _MR.step(intr, intr_cmd, M600, DT)
        if on > 1500.0:
            break
        t += DT
    return rows


def mixed_min_sep() -> float:
    """``run_encounter`` with a real mixed pair (fixed-wing OWN + multirotor INT, both DAA)."""
    own, intr = _pair(GS)
    return run_encounter(
        own, intr, perf=M600, rpz=RPZ, t_lookahead=LOOKAHEAD, dt=DT,
        detector=StateBased(), resolver=MVP(margin=1.1), recovery=PastCPA(bouncing_guard=True),
        own_kinematics=_FW, own_perf=SMALL_FIXEDWING, intr_kinematics=_MR, intr_perf=M600,
    ).min_sep


def _col(rows: list[tuple[float, ...]], k: int) -> list[float]:
    return [r[k] for r in rows]


def _spans(t: list[float], active: list[float]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    start: float | None = None
    for i, a in enumerate(active):
        if a > 0.5 and start is None:
            start = t[i]
        elif a <= 0.5 and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def plot(
    fw: list[tuple[float, ...]],
    mr: list[tuple[float, ...]],
    mixed: float,
    out: Path,
) -> None:
    fig, ax = plt.subplots(2, 2, figsize=(13.5, 9.5))

    # --- ground tracks (east axis exaggerated so the gentle fixed-wing arc is legible) ---
    a = ax[0, 0]
    a.plot(_col(fw, 3), _col(fw, 4), color="0.6", lw=1.6, label="INT (straight, non-cooperative)")
    a.plot(_col(fw, 1), _col(fw, 2), color="tab:orange", lw=2.4, label="OWN FixedWing (arc)")
    a.plot(_col(mr, 1), _col(mr, 2), color="tab:blue", lw=2.4, label="OWN Multirotor (side-step)")
    a.scatter([0], [0], color="k", marker="^", s=60, zorder=5)
    for rows, color in ((fw, "tab:orange"), (mr, "tab:blue")):
        sep = _col(rows, 5)
        i = min(range(len(sep)), key=lambda j: sep[j])
        a.annotate(f"{sep[i]:.0f} m", (_col(rows, 1)[i], _col(rows, 2)[i]),
                   textcoords="offset points", xytext=(6, 0), fontsize=8, color=color)
    a.axvline(0.0, color="0.85", lw=0.8, zorder=0)
    a.set_xlim(-140, 90)
    a.set_ylim(-40, 1200)
    a.set_xlabel("East [m]  (axis exaggerated)")
    a.set_ylabel("North [m]")
    a.set_title("Same geometry, two ownships vs one intruder (triangle = OWN start)")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8, loc="upper left")

    # --- separation over time ---
    a = ax[0, 1]
    a.plot(_col(fw, 0), _col(fw, 5), color="tab:orange", lw=2.0, label="FixedWing ownship")
    a.plot(_col(mr, 0), _col(mr, 5), color="tab:blue", lw=2.0, label="Multirotor ownship")
    a.axhline(RPZ, color="tab:red", lw=1.2, ls="--", label=f"rpz = {RPZ:.0f} m")
    a.set_xlabel("t [s]")
    a.set_ylabel("separation [m]")
    a.set_title("Separation — both clear the same conflict")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8)

    # --- fixed-wing course + bank: the feasible, bank-limited arc it converges through ---
    a = ax[1, 0]
    for s, e in _spans(_col(fw, 0), _col(fw, 9)):
        a.axvspan(s, e, color="tab:orange", alpha=0.10, lw=0)
    unwrapped = [((h + 180.0) % 360.0) - 180.0 for h in _col(fw, 6)]  # signed course about north
    a.plot(_col(fw, 0), unwrapped, color="tab:orange", lw=2.0, label="course psi (deg from north)")
    a.set_xlabel("t [s]")
    a.set_ylabel("course [deg]")
    a2 = a.twinx()
    a2.plot(_col(fw, 0), _col(fw, 7), color="tab:red", lw=1.2, ls="--", alpha=0.7,
            label="bank phi")
    a2.set_ylabel("bank [deg]")
    a.set_title("FixedWing: converges to the avoidance course via a bank-limited turn")
    lines1, labels1 = a.get_legend_handles_labels()
    lines2, labels2 = a2.get_legend_handles_labels()
    a.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="best")
    a.grid(True, alpha=0.3)

    # --- convergence: the gap between the commanded track and the achieved track. This is the
    #     Phase-4e point — the multirotor reaches the MVP velocity near-instantly, the fixed-wing
    #     converges to it under its bank/roll limit (the projection's documented approximation).
    a = ax[1, 1]
    a.plot(_col(fw, 0), _col(fw, 10), color="tab:orange", lw=2.0,
           label="FixedWing (converges under bank limit)")
    a.plot(_col(mr, 0), _col(mr, 10), color="tab:blue", lw=2.0,
           label="Multirotor (velocity reached ~instantly)")
    a.set_xlabel("t [s]")
    a.set_ylabel("|commanded track - achieved track| [deg]")
    a.set_title("Convergence to the MVP velocity — the airframe difference the projection encodes")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8)

    fig.suptitle(
        f"Phase 4e mixed-fleet DAA: one MVP velocity, two airframe responses "
        f"(dpsi={DPSI:.0f} deg); real mixed pair clears at {mixed:.0f} m (rpz {RPZ:.0f} m)",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    fw = avoid(_FW, SMALL_FIXEDWING, project=True)
    mr = avoid(_MR, M600, project=False)
    mixed = mixed_min_sep()
    fw_min = min(_col(fw, 5))
    mr_min = min(_col(mr, 5))
    print(f"single-ownship vs straight INT:  FixedWing min sep {fw_min:.1f} m, "
          f"Multirotor min sep {mr_min:.1f} m (rpz {RPZ:.0f})")
    print(f"real mixed pair (both DAA) via run_encounter: min sep {mixed:.1f} m -> "
          f"{'clear' if mixed >= RPZ else 'LOSS'}")
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/mixed-fleet-daa.png"
    plot(fw, mr, mixed, out)


if __name__ == "__main__":
    main()
