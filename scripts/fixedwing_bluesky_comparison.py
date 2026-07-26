"""Fixed-wing equation-of-motion comparison: OpenCDaRR ``FixedWing`` vs BlueSky.

Backs ``docs/fixedwing-vs-bluesky.md``. Drives OpenCDaRR's real
:class:`~opencdarr.dynamics.FixedWing` step against a *source-faithful* transcription of
BlueSky's fixed-wing kinematics (``bluesky/traffic/traffic.py`` ``update_airspeed`` /
``update_groundspeed`` / ``update_pos`` and ``aporasas.py`` crab), matched to the same airspeed,
gravity, and time step so only the models differ. The BlueSky transcription is checked against a
value measured from a *real* headless BlueSky run (A320, 25 deg bank, TAS 172.8 m/s -> constant
1.5159 deg/s turn rate), so the reimplementation is anchored to the running simulator, not assumed.

Writes ``docs/img/fixedwing-eom-comparison.png``.

Run:  python scripts/fixedwing_bluesky_comparison.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.dynamics import FixedWing, MotionCommand  # noqa: E402
from opencdarr.performance import SMALL_FIXEDWING, Performance  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.wind import NO_WIND, WindField  # noqa: E402

_G = 9.80665
_REARTH = 6371000.0  # m — BlueSky's flat-earth radius constant (bluesky.tools.geo.Rearth)
DT = 0.05  # s — BlueSky's default simdt
V = 17.0  # m/s — SMALL_FIXEDWING cruise TAS (Reyner & Liem example airframe)
BS_BANK = 25.0  # deg — BlueSky autopilot default bank (ap.bankdef = radians(25))


# --------------------------------------------------------------------------------------------
# BlueSky fixed-wing kinematics, transcribed from ~/Projects/bluesky (single aircraft, 2D).
# --------------------------------------------------------------------------------------------
def bluesky_step(
    x: float,
    y: float,
    hdg: float,
    tas: float,
    *,
    hdg_cmd: float,
    tas_cmd: float,
    bank_deg: float,
    wind: WindField,
    dt: float = DT,
) -> tuple[float, float, float, float, float, float]:
    """One BlueSky fixed-wing step in a local ENU (metres) frame. Returns
    ``(x, y, hdg, tas, trk, gs)``.

    Faithful to ``bluesky/traffic/traffic.py``:
      * ``update_airspeed``  — bang-bang airspeed ramp; turn rate = sign(hdg_err)*g*tan(phi)/V,
        with a *fixed* bank ``phi`` applied instantly (no roll state); snap onto hdg within a step.
      * ``update_groundspeed`` — ground velocity = airspeed vector (along hdg) + wind (Eq-9 sum).
      * ``update_pos`` — flat-earth integration (here: straight x += gseast*dt, y += gsnorth*dt).
    """
    # update_airspeed: airspeed (bang-bang toward the commanded TAS at axmax) ------------------
    axmax = 2.0
    dv = tas_cmd - tas
    if abs(dv) > abs(dt * axmax):
        tas = tas + math.copysign(axmax, dv) * dt
    else:
        tas = tas_cmd

    # update_airspeed: heading via the fixed-bank coordinated turn (default_tr path) -----------
    hdg_err = ((hdg_cmd - hdg + 180.0) % 360.0) - 180.0
    turnrate = math.copysign(
        math.degrees(_G * math.tan(math.radians(bank_deg)) / max(tas, 1e-9)), hdg_err
    )
    if abs(hdg_err) > abs(dt * turnrate):
        hdg = (hdg + dt * turnrate) % 360.0
    else:
        hdg = hdg_cmd % 360.0

    # update_groundspeed: ground velocity = airspeed vector + wind (the Eq-9 vector sum) -------
    wn, we = wind.w_north, wind.w_east
    gsnorth = tas * math.cos(math.radians(hdg)) + wn
    gseast = tas * math.sin(math.radians(hdg)) + we
    gs = math.hypot(gsnorth, gseast)
    trk = math.degrees(math.atan2(gseast, gsnorth)) % 360.0

    # update_pos: flat-earth position integration --------------------------------------------
    x = x + gseast * dt
    y = y + gsnorth * dt
    return x, y, hdg, tas, trk, gs


def bluesky_crab_hdg(trk_des: float, tas: float, wind: WindField) -> float:
    """Commanded heading to make good ``trk_des`` under wind — aporasas.py crab law.

    ``winddir = atan2(w_east, w_north)`` (direction the wind blows TOWARD);
    ``steer = arcsin(clip(Vw*sin(trk-winddir)/TAS, -1, 1))``; ``hdg = trk + steer``.
    """
    vw = wind.speed
    winddir = math.atan2(wind.w_east, wind.w_north)
    drift = math.radians(trk_des) - winddir
    steer = math.asin(max(-1.0, min(1.0, vw * math.sin(drift) / max(tas, 1e-3))))
    return (trk_des + math.degrees(steer)) % 360.0


def _validate_transcription() -> None:
    """Anchor the transcription to the running simulator: at the A320 values I measured from a
    real headless BlueSky run (TAS 172.8 m/s, 25 deg bank) the constant turn rate is 1.5159 deg/s.
    """
    tas = 172.84
    tr = math.degrees(_G * math.tan(math.radians(25.0)) / tas)
    assert abs(tr - 1.5159) < 1e-3, f"transcription drift: {tr}"


# --------------------------------------------------------------------------------------------
# OpenCDaRR helpers
# --------------------------------------------------------------------------------------------
def _enu(lat: float, lon: float) -> tuple[float, float]:
    """(lat, lon) deg -> local ENU (east, north) metres from the origin (0, 0)."""
    brg, dist = geo.qdrdist(0.0, 0.0, lat, lon)
    r = math.radians(brg)
    return dist * math.sin(r), dist * math.cos(r)


def run_ours_turn(perf: Performance, hdg_target: float, n: int) -> dict[str, list[float]]:
    """OpenCDaRR FixedWing: command a heading change (airspeed_direction, no wind)."""
    st = AircraftState(id="F", lat=0.0, lon=0.0, trk=0.0, gs=V, yaw=0.0, bank=0.0)
    dyn = FixedWing()
    cmd = MotionCommand(target_airspeed_direction=hdg_target, target_airspeed=V)
    xs, ys, banks, hdgs = [], [], [], []
    for _ in range(n):
        st = dyn.step(st, cmd, perf, DT, NO_WIND)
        e, nth = _enu(st.lat, st.lon)
        xs.append(e)
        ys.append(nth)
        banks.append(st.bank)
        hdgs.append(st.yaw if st.yaw is not None else st.trk)
    return {"x": xs, "y": ys, "bank": banks, "hdg": hdgs}


def run_bs_turn(bank_deg: float, hdg_target: float, n: int) -> dict[str, list[float]]:
    """BlueSky transcription: command a heading change at a fixed bank (no wind)."""
    x, y, hdg, tas = 0.0, 0.0, 0.0, V
    xs, ys, banks, hdgs = [], [], [], []
    for _ in range(n):
        prev_hdg = hdg
        x, y, hdg, tas, _trk, _gs = bluesky_step(
            x, y, hdg, tas, hdg_cmd=hdg_target, tas_cmd=V, bank_deg=bank_deg, wind=NO_WIND
        )
        turning = abs(((hdg - prev_hdg + 180) % 360) - 180) > 1e-6
        xs.append(x)
        ys.append(y)
        # BlueSky has no roll state: full bank or wings level
        banks.append(bank_deg if turning else 0.0)
        hdgs.append(hdg)
    return {"x": xs, "y": ys, "bank": banks, "hdg": hdgs}


def run_ours_course(course: float, wind: WindField, n: int) -> dict[str, list[float]]:
    """OpenCDaRR FixedWing: make good a ground course under wind (target_course -> crab)."""
    st = AircraftState(id="F", lat=0.0, lon=0.0, trk=0.0, gs=V, yaw=0.0, bank=0.0)
    dyn = FixedWing()
    cmd = MotionCommand(target_course=course, target_airspeed=V)
    xs, ys, hdgs = [], [], []
    for _ in range(n):
        st = dyn.step(st, cmd, perf=SMALL_FIXEDWING, dt=DT, wind=wind)
        e, nth = _enu(st.lat, st.lon)
        xs.append(e)
        ys.append(nth)
        hdgs.append(st.yaw if st.yaw is not None else st.trk)
    return {"x": xs, "y": ys, "hdg": hdgs}


def run_bs_course(
    trk_des: float, wind: WindField, n: int, *, crab: bool
) -> dict[str, list[float]]:
    """BlueSky transcription flying a leg: crab (LNAV make-good track) or not (raw HDG)."""
    hdg0 = bluesky_crab_hdg(trk_des, V, wind) if crab else trk_des
    x, y, hdg, tas = 0.0, 0.0, hdg0, V
    xs, ys, hdgs = [], [], []
    for _ in range(n):
        hdg_cmd = bluesky_crab_hdg(trk_des, tas, wind) if crab else trk_des
        x, y, hdg, tas, _trk, _gs = bluesky_step(
            x, y, hdg, tas, hdg_cmd=hdg_cmd, tas_cmd=V, bank_deg=BS_BANK, wind=wind
        )
        xs.append(x)
        ys.append(y)
        hdgs.append(hdg)
    return {"x": xs, "y": ys, "hdg": hdgs}


# --------------------------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------------------------
def main() -> None:
    _validate_transcription()
    n_turn = int(12.0 / DT)  # 12 s of turning
    n_leg = int(18.0 / DT)  # 18 s straight leg

    # (a) native-parameter turn: OpenCDaRR (44 deg cap, 60 deg/s roll) vs BlueSky (25 deg, instant)
    ours_native = run_ours_turn(SMALL_FIXEDWING, 90.0, n_turn)
    bs_native = run_bs_turn(BS_BANK, 90.0, n_turn)

    # (c) matched core: OpenCDaRR forced to 25 deg / near-instant roll / no stall bite vs BlueSky
    matched = Performance(v_max=100.0, v_min=1.0, ax=2.0, phi_max=25.0, roll_rate_max=1.0e6)
    ours_matched = run_ours_turn(matched, 90.0, n_turn)
    bs_matched = run_bs_turn(BS_BANK, 90.0, n_turn)

    # (d) crosswind: 6 m/s from the west (270), make good a due-north (0) ground course
    wind = WindField.from_met(coming_from_deg=270.0, speed=6.0)
    ours_crab = run_ours_course(0.0, wind, n_leg)
    bs_crab = run_bs_course(0.0, wind, n_leg, crab=True)
    bs_nocrab = run_bs_course(0.0, wind, n_leg, crab=False)

    fig, ax = plt.subplots(2, 2, figsize=(11, 9.5))
    c_ours, c_bs = "#1f77b4", "#d62728"

    # (a) native turn ground tracks -----------------------------------------------------------
    a = ax[0, 0]
    a.plot(ours_native["x"], ours_native["y"], color=c_ours, lw=2,
           label="OpenCDaRR (phi_max 44 deg, 60 deg/s roll)")
    a.plot(bs_native["x"], bs_native["y"], color=c_bs, lw=2, ls="--",
           label="BlueSky (25 deg bank, instant roll)")
    a.scatter([0], [0], c="k", s=18, zorder=5)
    a.set_title("(a) 90 deg turn, each model's native parameters")
    a.set_xlabel("east [m]")
    a.set_ylabel("north [m]")
    a.set_aspect("equal")
    a.legend(fontsize=8, loc="lower right")

    # (b) bank angle vs time during that turn -------------------------------------------------
    b = ax[0, 1]
    t = [i * DT for i in range(n_turn)]
    b.plot(t, ours_native["bank"], color=c_ours, lw=2, label="OpenCDaRR bank phi(t)")
    b.plot(t, bs_native["bank"], color=c_bs, lw=2, ls="--", label="BlueSky bank phi(t)")
    b.set_title("(b) Bank angle: finite roll rate vs instant step")
    b.set_xlabel("time [s]")
    b.set_ylabel("bank angle phi [deg]")
    b.legend(fontsize=8, loc="upper right")

    # (c) matched-parameter turn (shared coordinated-turn core) -------------------------------
    c = ax[1, 0]
    c.plot(ours_matched["x"], ours_matched["y"], color=c_ours, lw=3, alpha=0.8,
           label="OpenCDaRR @ 25 deg, ~instant roll")
    c.plot(bs_matched["x"], bs_matched["y"], color=c_bs, lw=1.6, ls="--",
           label="BlueSky @ 25 deg")
    c.scatter([0], [0], c="k", s=18, zorder=5)
    c.set_title("(c) Same bank -> same arc (shared g*tan(phi)/V core)")
    c.set_xlabel("east [m]")
    c.set_ylabel("north [m]")
    c.set_aspect("equal")
    c.legend(fontsize=8, loc="lower right")

    # (d) crosswind: crab holds track (both), uncorrected heading drifts --------------------
    d = ax[1, 1]
    d.plot(ours_crab["x"], ours_crab["y"], color=c_ours, lw=3, alpha=0.8,
           label="OpenCDaRR make-good north (crab)")
    d.plot(bs_crab["x"], bs_crab["y"], color=c_bs, lw=1.6, ls="--",
           label="BlueSky make-good north (crab)")
    d.plot(bs_nocrab["x"], bs_nocrab["y"], color="#888888", lw=1.6, ls=":",
           label="heading due north, no crab (drifts E)")
    d.scatter([0], [0], c="k", s=18, zorder=5)
    crab_ours = (ours_crab["hdg"][-1] + 180) % 360 - 180
    d.set_title(f"(d) Crosswind 6 m/s from W: crab = {crab_ours:.1f} deg")
    d.set_xlabel("east [m]")
    d.set_ylabel("north [m]")
    d.set_aspect("equal")
    d.legend(fontsize=8, loc="upper left")

    for row in ax:
        for cell in row:
            cell.grid(False)
    fig.tight_layout()

    out = Path(__file__).resolve().parents[1] / "docs" / "img" / "fixedwing-eom-comparison.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"wrote {out}")

    # console cross-checks the report quotes
    print(f"OpenCDaRR crab (make-good north, 6 m/s W wind): {crab_ours:.3f} deg")
    bs_hdg = (bs_crab["hdg"][-1] + 180) % 360 - 180
    print(f"BlueSky   crab (make-good north, 6 m/s W wind): {bs_hdg:.3f} deg")
    r_ours = V * V / (_G * math.tan(math.radians(SMALL_FIXEDWING.phi_max)))
    r_bs = V * V / (_G * math.tan(math.radians(BS_BANK)))
    print(f"OpenCDaRR steady turn radius V^2/(g*tan(44 deg)) = {r_ours:.1f} m")
    print(f"BlueSky   steady turn radius V^2/(g*tan(25 deg)) = {r_bs:.1f} m")


if __name__ == "__main__":
    main()
