"""IPR-under-wind sweep (Phase 5d, the research payoff).

Sweeps steady-wind magnitude and bearing across airframe mixes and records how the IPR (and
the median safety margin) move. The physical question the whole wind build exists to answer: does a
steady wind degrade detect-and-avoid, and for which airframe?

The headline finding (see ``vault/observations/ipr-under-wind.md``): a *uniform* wind translates
both aircraft together, and the CD/CR/CRR stack reads the wind-blown **ground** frame, so a
slack-envelope **multirotor pair stays robust (IPR ~ 1)**. A **fixed-wing pair**, whose turn rate
and stall limit its wind-relative maneuvering, **loses a couple of IPR points in strong wind**,
bearing-dependent — turning into a crosswind costs margin. A demanding conflict (short warning,
heavy GNSS noise, near-head-on) is used so the baseline has headroom to degrade.

    PYTHONPATH=. python scripts/ipr_wind_sweep.py            # default sweep + plot
    PYTHONPATH=. python scripts/ipr_wind_sweep.py --n 400 --jobs 8

Writes ``vault/observations/img/ipr-under-wind.png``.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from joblib import Parallel, delayed  # noqa: E402

from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import GnssNavigation  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.fleet import Agent, run_fleet  # noqa: E402
from opencdarr.kinematics import FixedWing, Multirotor  # noqa: E402
from opencdarr.performance import M600, SMALL_FIXEDWING, Performance  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.wind import WindField  # noqa: E402

_FW, _MR = FixedWing(), Multirotor()
# (own, intr) airframe pairs; each entry is (label, own_kinematics, own_perf, intr_kinematics,
# intr_perf).
_MIXES = {
    "both fixed-wing": (_FW, SMALL_FIXEDWING, _FW, SMALL_FIXEDWING),
    "fixed-wing vs multirotor": (_FW, SMALL_FIXEDWING, _MR, M600),
    "both multirotor": (_MR, M600, _MR, M600),
}
_BEARINGS = (0.0, 90.0, 180.0, 270.0)  # meteorological "coming from"


def _one(
    seq: np.random.SeedSequence, wind: WindField, mix: tuple, cfg: argparse.Namespace
) -> tuple:
    own_kinematics, own_perf, intr_kinematics, intr_perf = mix
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=cfg.gs, yaw=0.0, bank=0.0,
        pos_ci95=cfg.pos_ci95, vel_ci95=cfg.vel_ci95,
    )
    intr = create_conflict(
        own, intr_id="INT", dpsi=cfg.dpsi, dcpa=0.0, tlos=cfg.tlos, rpz=cfg.rpz, side=1)
    out = run_fleet(
        [Agent(own, own_perf, kinematics=own_kinematics),
         Agent(intr, intr_perf, kinematics=intr_kinematics)],
        rpz=cfg.rpz, t_lookahead=cfg.lookahead, dt=cfg.dt,
        detector=StateBased(), resolver=MVP(margin=cfg.margin),
        recovery=PastCPA(bouncing_guard=True),
        wind=wind, navigation=GnssNavigation(), rng=generator(seq),
    )
    return out.los, out.min_sep


def _ipr_med(
    seqs: list, wind: WindField, mix: tuple, cfg: argparse.Namespace
) -> tuple[float, float]:
    rows = Parallel(n_jobs=cfg.jobs)(delayed(_one)(s, wind, mix, cfg) for s in seqs)
    los = sum(r[0] for r in rows)
    return 1.0 - los / len(seqs), float(np.median([r[1] for r in rows]))


def _perf_label(p: Performance) -> str:
    return "M600" if p is M600 else "small fixed-wing"


def plot(
    speeds: np.ndarray,
    ipr_by_mix: dict[str, list[float]],
    med_fw_by_bearing: dict[float, list[float]],
    cfg: argparse.Namespace,
    out: Path,
) -> None:
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 5.4))

    for label, iprs in ipr_by_mix.items():
        a1.plot(speeds, iprs, marker="o", lw=2.0, label=label)
    a1.set_xlabel("wind speed [m/s]")
    a1.set_ylabel("IPR  (1 − LoS / n)")
    a1.set_title(f"IPR vs wind (crosswind from 90°) — {cfg.n} seeds/point")
    a1.grid(True, alpha=0.3)
    a1.legend(fontsize=8)

    for bearing, meds in med_fw_by_bearing.items():
        a2.plot(speeds, meds, marker="o", lw=2.0, label=f"wind from {bearing:.0f}°")
    a2.axhline(cfg.rpz, color="tab:red", ls="--", lw=1.0, label=f"rpz = {cfg.rpz:.0f} m")
    a2.set_xlabel("wind speed [m/s]")
    a2.set_ylabel("median min-sep [m]")
    a2.set_title("Safety margin vs wind (both fixed-wing) — bearing-dependent")
    a2.grid(True, alpha=0.3)
    a2.legend(fontsize=8)

    fig.suptitle(
        f"IPR under steady wind: dpsi={cfg.dpsi:.0f}°, tlos={cfg.tlos:.0f}s, lookahead="
        f"{cfg.lookahead:.0f}s, margin={cfg.margin}, GNSS noise {cfg.pos_ci95:.0f}m/"
        f"{cfg.vel_ci95:.0f}(m/s)",
        fontsize=11,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--n", type=int, default=200, help="noise realisations per point")
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--speeds", type=float, nargs="+",
                   default=[0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0])
    p.add_argument("--dpsi", type=float, default=130.0, help="crossing angle (near head-on)")
    p.add_argument("--tlos", type=float, default=20.0, help="spawn time-to-LoS [s], short warning")
    p.add_argument("--lookahead", type=float, default=60.0)
    p.add_argument("--margin", type=float, default=1.0)
    p.add_argument("--gs", type=float, default=17.0)
    p.add_argument("--rpz", type=float, default=50.0)
    p.add_argument("--dt", type=float, default=0.2)
    p.add_argument("--pos-ci95", dest="pos_ci95", type=float, default=25.0)
    p.add_argument("--vel-ci95", dest="vel_ci95", type=float, default=4.0)
    cfg = p.parse_args()

    seqs = list(spawn(root_seed_sequence(cfg.seed), cfg.n))
    speeds = np.array(cfg.speeds)
    t0 = time.time()
    print(f"IPR-under-wind sweep — {cfg.n} seeds/point, dpsi={cfg.dpsi}, tlos={cfg.tlos}, "
          f"lookahead={cfg.lookahead}, margin={cfg.margin}")

    # Panel A: IPR vs wind speed for each airframe mix, at a fixed crosswind bearing (90°).
    ipr_by_mix: dict[str, list[float]] = {}
    print(f"\n{'mix':>26} {'wind[m/s]':>10} {'IPR':>7} {'median':>8}")
    for label, mix in _MIXES.items():
        iprs = []
        for sp in speeds:
            ipr, med = _ipr_med(seqs, WindField.from_met(90.0, float(sp)), mix, cfg)
            iprs.append(ipr)
            print(f"{label:>26} {sp:>10.0f} {ipr:>7.3f} {med:>8.1f}")
        ipr_by_mix[label] = iprs

    # Panel B: median min-sep vs wind speed for the fixed-wing pair, across bearings.
    fw_mix = _MIXES["both fixed-wing"]
    med_fw_by_bearing: dict[float, list[float]] = {}
    for bearing in _BEARINGS:
        med_fw_by_bearing[bearing] = [
            _ipr_med(seqs, WindField.from_met(bearing, float(sp)), fw_mix, cfg)[1] for sp in speeds
        ]

    print(f"\n(elapsed {time.time() - t0:.1f} s)")
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/ipr-under-wind.png"
    plot(speeds, ipr_by_mix, med_fw_by_bearing, cfg, out)


if __name__ == "__main__":
    main()
