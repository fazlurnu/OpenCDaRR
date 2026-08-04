"""What :class:`~rmvp.RMVP` claims, checked one claim at a time.

Five sections, in the order the README makes the claims:

1. ``confidence = 0.5`` reproduces :class:`~opencdarr.cr.MVP` — the controlled-comparison claim.
2. ``sigma -> 0`` reproduces MVP at any confidence — the deterministic-limit claim.
3. The root solver converges, and the commanded step stays **bounded** as the relative speed
   vanishes — the claim that separates this rule from a naive angular one.
4. The rule delivers its stated confidence, measured end-to-end through the simulator's own
   :class:`~opencdarr.cns.GnssNavigation` perturbation rather than through the design model.
5. The same measurement for MVP and for ``uncertainty-aware-mvp``'s UAMVP, so the three sit in one
   table.

    python robust-mvp/verify_rmvp.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "uncertainty-aware-mvp"))

from rmvp import RMVP, rotation  # noqa: E402
from uamvp import UAMVP  # noqa: E402

from opencdarr.cns import GnssNavigation  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.relative import relative_enu, velocity_enu  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

SPEED = 10.2889  # m/s = 20 kt, both aircraft
RPZ = 50.0  # m
MARGIN = 1.05
TLOS = 180.0  # s
DCPA = 0.0  # m
ANGLES = (2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 90.0)  # deg
BLUE, RED, ORANGE, PURPLE = "#1f77b4", "#d62728", "#ff7f0e", "#9467bd"


def pair(dpsi: float, pos_ci95: float, vel_ci95: float) -> tuple[AircraftState, AircraftState]:
    """The campaign's encounter at one crossing angle — both aircraft at 20 kt, ``dcpa`` 0."""
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED, pos_ci95=pos_ci95, vel_ci95=vel_ci95
    )
    intr = create_conflict(
        own, intr_id="INT", dpsi=dpsi, dcpa=DCPA, tlos=TLOS, rpz=RPZ, side=1
    )
    return own, intr


def step_magnitude(resolver: ConflictResolver, own: AircraftState, intr: AircraftState) -> float:
    """``||dv||`` [m/s] the resolver commands for this pair, from noise-free perception."""
    ve, vn = velocity_enu(own)
    ne, nn = resolver.resolve(own, [intr], RPZ).target_velocity
    return math.hypot(ne - ve, nn - vn)


# --- 1 & 2: the two limits in which this must *be* MVP ---------------------------------------


def check_reduces_to_mvp() -> None:
    """``confidence = 0.5`` and ``sigma -> 0`` both have to give MVP's own step."""
    mvp = MVP(margin=MARGIN)
    print("1. confidence = 0.5 reproduces MVP (declared uncertainty non-zero)")
    worst = 0.0
    for dpsi in ANGLES:
        for pos_ci95, vel_ci95 in ((10.0, 1.0), (10.0, 3.0), (25.0, 5.0)):
            own, intr = pair(dpsi, pos_ci95, vel_ci95)
            a = step_magnitude(mvp, own, intr)
            b = step_magnitude(RMVP(margin=MARGIN, confidence=0.5), own, intr)
            worst = max(worst, abs(a - b) / max(a, 1e-12))
    print(f"   worst relative difference in ||dv|| over {len(ANGLES) * 3} geometries: {worst:.2e}")

    print("2. sigma -> 0 reproduces MVP at every confidence")
    worst = 0.0
    for confidence in (0.5, 0.9, 0.95, 0.99, 0.999):
        for dpsi in ANGLES:
            own, intr = pair(dpsi, 0.0, 0.0)
            a = step_magnitude(mvp, own, intr)
            b = step_magnitude(RMVP(margin=MARGIN, confidence=confidence), own, intr)
            worst = max(worst, abs(a - b) / max(a, 1e-12))
    print(f"   worst relative difference in ||dv|| over 5 confidences: {worst:.2e}")


# --- 3: the solver, and the limit that makes the rule usable ---------------------------------


def check_solver_and_bound() -> None:
    """The root is solved to tolerance, and ``||dv||`` stays finite as ``|v_rel| -> 0``."""
    print("3. the angular root, and the step as the relative speed vanishes")
    k = RMVP(confidence=0.95).k
    worst = 0.0
    for alpha in (1e-4, 0.01, 0.1, 0.5, 1.0):
        for gamma in (0.02, 0.1, 0.5, 1.0):
            for sigma_phi in (0.001, 0.01, 0.1, 1.0, 5.0, 50.0):
                for sigma_los in (0.0, 0.001, 0.05, 0.2):
                    theta = rotation(alpha, gamma, k, sigma_phi, sigma_los)
                    if theta in (0.0, 0.5 * math.pi - alpha):
                        continue  # a branch, not a root
                    sigma_m = math.hypot(sigma_phi * math.cos(theta), sigma_los)
                    worst = max(worst, abs(alpha + theta - gamma - k * sigma_m))
    print(f"   worst residual |f(theta*)| over 480 cases: {worst:.2e} rad")

    # the degenerate limit: fixed geometry and declared accuracy, relative speed driven to zero
    dist, sigma_v, sigma_los = 114.6, 1.733, 0.057
    alpha, gamma = math.asin(0.1 / dist), math.asin(52.5 / dist)
    limit = k * sigma_v / (0.5 * math.pi + alpha - gamma)
    perpendicular = 0.5 * math.pi - alpha
    print(f"   |v_rel| [m/s]   ||dv|| [m/s]   (analytic limit {limit:.3f})")
    for v_mag in (5.0, 2.0, 1.0, 0.359, 0.1, 0.01, 1e-3, 1e-4):
        theta = rotation(alpha, gamma, k, sigma_v / v_mag, sigma_los)
        flag = "  (perpendicular)" if theta >= perpendicular - 1e-15 else ""
        print(f"   {v_mag:>11.4g}   {v_mag * math.tan(theta):>11.3f}{flag}")


# --- 4 & 5: what the manoeuvre actually achieves ----------------------------------------------


@dataclass(frozen=True)
class Achieved:
    """One cell of the end-to-end measurement."""

    p_clear: float  # P(true closest-approach offset > rpz) after the manoeuvre
    dv_mean: float  # mean commanded ||dv|| [m/s]


def measure_achieved(
    resolver: ConflictResolver,
    dpsi: float,
    pos_ci95: float,
    vel_ci95: float,
    n: int = 20_000,
    seed: int = 7,
) -> Achieved:
    """Perceive, resolve, then score the manoeuvre on the **true** states.

    Each draw takes two fixes through :class:`~opencdarr.cns.GnssNavigation` — the ownship's fix of
    itself and its fix of the intruder — hands the pair to the resolver, and evaluates the
    closest-approach offset the commanded velocity actually produces against the true geometry. The
    ownship's own fix error stays in: it commands ``perceived own velocity - dv``, so it flies a
    velocity offset from the one it meant to. The intruder holds, which is the conservative
    single-sided reading of a cooperative encounter.
    """
    own, intr = pair(dpsi, pos_ci95, vel_ci95)
    nav = GnssNavigation()
    nav_state = nav.initial_state()
    rng = np.random.default_rng(seed)
    ve_true, vn_true = velocity_enu(own)

    cleared = 0
    dv_total = 0.0
    for _ in range(n):
        own_fix = nav.measure(nav_state, own, 0.0, rng).state
        intr_fix = nav.measure(nav_state, intr, 0.0, rng).state
        cmd_e, cmd_n = resolver.resolve(own_fix, [intr_fix], RPZ).target_velocity
        dv_total += math.hypot(cmd_e - ve_true, cmd_n - vn_true)
        # the ownship now truly flies (cmd_e, cmd_n) from its true position
        flown = AircraftState(
            id=own.id, lat=own.lat, lon=own.lon,
            trk=math.degrees(math.atan2(cmd_e, cmd_n)) % 360.0, gs=math.hypot(cmd_e, cmd_n),
        )
        rel = relative_enu(flown, intr)
        v_mag = math.hypot(rel.vx, rel.vy)
        offset = abs(rel.rx * rel.vy - rel.ry * rel.vx) / v_mag if v_mag > 1e-12 else rel.dist
        cleared += offset > RPZ
    return Achieved(p_clear=cleared / n, dv_mean=dv_total / n)


def check_achieved() -> dict[tuple[str, float, float], Achieved]:
    """Design confidence against delivered probability, for the three resolvers."""
    resolvers: dict[str, ConflictResolver] = {
        "MVP": MVP(margin=MARGIN),
        "UAMVP0.95": UAMVP(margin=MARGIN, confidence=0.95),
        "RMVP0.95": RMVP(margin=MARGIN, confidence=0.95),
    }
    print("4/5. delivered P(offset > rpz) after one manoeuvre, 20 000 perception draws per cell")
    out: dict[tuple[str, float, float], Achieved] = {}
    for vel_ci95 in (1.0, 3.0):
        print(f"   pos_ci95 = 10 m, vel_ci95 = {vel_ci95:g} m/s      "
              + "  ".join(f"{d:g}deg" for d in ANGLES))
        for name, resolver in resolvers.items():
            row = []
            for dpsi in ANGLES:
                got = measure_achieved(resolver, dpsi, 10.0, vel_ci95)
                out[(name, dpsi, vel_ci95)] = got
                row.append(got.p_clear)
            print(f"   {name:>12}  P(clear)   " + "  ".join(f"{p:5.3f}" for p in row))
            print(f"   {'':>12}  ||dv||     "
                  + "  ".join(f"{out[(name, d, vel_ci95)].dv_mean:5.2f}" for d in ANGLES))
    return out


# --- the figure --------------------------------------------------------------------------------


def figure(achieved: dict[tuple[str, float, float], Achieved]) -> None:
    """Left: the design intent. Right: what the noise leaves of it."""
    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(10.0, 4.6))
    colours = {"MVP": "0.45", "UAMVP0.95": ORANGE, "RMVP0.95": RED}
    resolvers: dict[str, ConflictResolver] = {
        "MVP": MVP(margin=MARGIN),
        "UAMVP0.95": UAMVP(margin=MARGIN, confidence=0.95),
        "RMVP0.95": RMVP(margin=MARGIN, confidence=0.95),
    }

    fine = np.linspace(1.0, 90.0, 60)
    for name, resolver in resolvers.items():
        steps = [step_magnitude(resolver, *pair(float(d), 10.0, 3.0)) for d in fine]
        ax_l.plot(fine, steps, color=colours[name], label=name, lw=1.6)
    ax_l.set_xlabel("crossing angle [deg]")
    ax_l.set_ylabel(r"commanded $\|\Delta v\|$ [m/s]")
    ax_l.set_ylim(0.0, 8.0)
    ax_l.legend(frameon=False, fontsize=9)
    ax_l.set_box_aspect(1)

    for name in resolvers:
        for vel_ci95, style in ((1.0, "-"), (3.0, "--")):
            ax_r.plot(
                ANGLES, [achieved[(name, d, vel_ci95)].p_clear for d in ANGLES],
                style, color=colours[name], marker="o", ms=3.5, lw=1.4,
                label=f"{name}, {vel_ci95:g} m/s",
            )
    ax_r.axhline(0.95, color="0.25", ls=":", lw=1.0)
    ax_r.set_xlabel("crossing angle [deg]")
    ax_r.set_ylabel(r"delivered $P(d > r_{PZ})$")
    ax_r.set_ylim(0.4, 1.02)
    ax_r.legend(frameon=False, fontsize=7.5, ncol=2)
    ax_r.set_box_aspect(1)

    fig.tight_layout()
    fig.savefig(HERE / "fig_validation.png", dpi=130)
    print(f"\nwrote {HERE / 'fig_validation.png'}")


def main() -> None:
    check_reduces_to_mvp()
    print()
    check_solver_and_bound()
    print()
    achieved = check_achieved()
    figure(achieved)


if __name__ == "__main__":
    main()
