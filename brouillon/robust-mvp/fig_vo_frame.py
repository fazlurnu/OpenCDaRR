"""MVP against RMVP in the velocity-obstacle frame, under each source of navigation uncertainty.

The construction is the paper's (``Journal___RESS___UQ_CD_R___v2``, §4.2): draw Monte Carlo samples
of the perturbed input, push each one through the resolver, and plot the resulting resolution
velocity in the ownship's velocity space against the velocity obstacle. A sample outside the
obstacle is a resolution that would actually have worked; a sample inside is one the noise turned
into a manoeuvre that still leaves the pair in conflict. The percentage in each panel is that
count, evaluated against the **true** geometry rather than the perceived one.

Three columns, one per uncertainty source, holding everything else fixed:

- **position** — positions perturbed, velocities exact;
- **velocity** — velocities perturbed, positions exact;
- **both** — the campaign's actual condition.

The *declared* accuracy is (10 m, 3 m/s) in all three, so RMVP sizes the same margin in every panel
and the only thing changing is which input is noisy. Without that, the "position" column would also
be measuring a resolver that had been told the velocity was perfect.

    python robust-mvp/fig_vo_frame.py
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from conditions_rm import CONFIDENCE, DCPA, MARGIN, RPZ, SPEED, TLOS  # noqa: E402
from rmvp import RMVP  # noqa: E402

from opencdarr.cns import GnssNavigation  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.relative import relative_enu, velocity_enu  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

MS_TO_KT = 1.9438445
V_MAX_KT = M600.v_max * MS_TO_KT  # the M600's flight envelope, 18 m/s = 35 kt
DECLARED = (10.0, 3.0)  # (pos_ci95 [m], vel_ci95 [m/s]) every panel's resolver is told
# (column label, true pos_ci95 [m], true vel_ci95 [m/s]) — which input actually carries noise
SOURCES = (("position only", DECLARED[0], 0.0),
           ("velocity only", 0.0, DECLARED[1]),
           ("position and velocity", DECLARED[0], DECLARED[1]))
ANGLES = (2.0, 30.0)  # deg, the campaign's two geometries
GREY, RED = "0.45", "#d62728"


def spawn_tlos(dpsi: float, gamma_deg: float | None) -> float:
    """The ``tlos`` that puts the pair at a range whose velocity-obstacle half-angle is
    ``gamma_deg``.

    The obstacle's half-angle is ``gamma = asin(rpz / |r|)``, so asking for a wider cone is asking
    for a shorter range, and range is what ``tlos`` sets. At ``dcpa = 0`` the pair closes straight
    down the line of sight, so ``|r| = rpz + |v_rel| tlos`` exactly and the inverse is one line.
    Pinning ``gamma`` rather than ``tlos`` is what lets two crossing angles share a cone: the
    default ``tlos`` puts Δψ = 2° at 114.6 m and Δψ = 30° at 1008.7 m, which are different pictures
    for two reasons at once.

    ``None`` keeps the campaign's own ``TLOS``.
    """
    if gamma_deg is None:
        return TLOS
    v_rel = 2.0 * SPEED * math.sin(math.radians(dpsi) / 2.0)
    return (RPZ / math.sin(math.radians(gamma_deg)) - RPZ) / v_rel


def states(
    dpsi: float, pos_ci95: float, vel_ci95: float, tlos: float = TLOS
) -> tuple[AircraftState, AircraftState]:
    """The encounter, with the error drawn from ``pos_ci95``/``vel_ci95`` and (10 m, 3 m/s)
    declared.

    ``pos_ci95_declared`` / ``vel_ci95_declared`` are what makes the three columns a controlled
    comparison: the broadcast claim stays at ``DECLARED`` while the error the aircraft actually
    suffers is switched off one channel at a time.
    """
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED,
        pos_ci95=pos_ci95, vel_ci95=vel_ci95,
        pos_ci95_declared=DECLARED[0], vel_ci95_declared=DECLARED[1],
    )
    intr = create_conflict(
        own, intr_id="INT", dpsi=dpsi, dcpa=DCPA, tlos=tlos, rpz=RPZ, side=1
    )
    return own, replace(
        intr, pos_ci95_declared=DECLARED[0], vel_ci95_declared=DECLARED[1]
    )


def truth(dpsi: float, tlos: float = TLOS) -> tuple[AircraftState, AircraftState]:
    """The same encounter with no noise at all, whose ``pos_ci95``/``vel_ci95`` *are* the declared
    pair — the states a resolver reads when nothing has gone wrong."""
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED,
        pos_ci95=DECLARED[0], vel_ci95=DECLARED[1],
    )
    return own, create_conflict(
        own, intr_id="INT", dpsi=dpsi, dcpa=DCPA, tlos=tlos, rpz=RPZ, side=1
    )


def in_velocity_obstacle(
    own: AircraftState, intr: AircraftState, v_own: tuple[float, float]
) -> bool:
    """Would the ownship flying ``v_own`` still be in conflict, on the **true** geometry?

    The velocity-obstacle membership test written without constructing the obstacle: the pair is in
    conflict when the relative velocity is closing and the predicted closest-approach offset is
    inside ``rpz``. Same condition, one line, no cone.
    """
    rel = relative_enu(own, intr)
    vie, vin = velocity_enu(intr)
    wx, wy = vie - v_own[0], vin - v_own[1]
    w2 = wx * wx + wy * wy
    if w2 < 1e-12:
        return rel.dist < RPZ
    t_cpa = -(rel.rx * wx + rel.ry * wy) / w2
    if t_cpa <= 0.0:
        return False  # diverging: no future approach to be in conflict about
    return math.hypot(rel.rx + wx * t_cpa, rel.ry + wy * t_cpa) < RPZ


def samples(
    resolver: ConflictResolver,
    own: AircraftState,
    intr: AircraftState,
    n: int,
    seed: int,
) -> tuple[np.ndarray, float]:
    """``n`` resolution velocities [kt] from perturbed perception, and the fraction still in
    conflict."""
    nav = GnssNavigation()
    nav_state = nav.initial_state()
    rng = np.random.default_rng(seed)
    out = np.empty((n, 2))
    inside = 0
    for i in range(n):
        own_fix = nav.measure(nav_state, own, 0.0, rng).state
        intr_fix = nav.measure(nav_state, intr, 0.0, rng).state
        v_own = resolver.resolve(own_fix, [intr_fix], RPZ).target_velocity
        out[i] = v_own
        inside += in_velocity_obstacle(own, intr, v_own)
    return out * MS_TO_KT, inside / n


def draw_obstacle(ax: plt.Axes, own: AircraftState, intr: AircraftState, reach: float) -> None:
    """The velocity obstacle as the ownship sees it: a cone apexed at the intruder's velocity.

    In ownship-velocity space the forbidden set is ``{v_i - w}`` over every closing relative
    velocity ``w``, which is the cone with apex ``v_i``, axis along the line of sight ``r_hat`` and
    half-angle ``asin(rpz / |r|)``. Drawn at ``rpz``, not at ``margin * rpz``: the obstacle is the
    protected zone, and the margin is the resolvers' own buffer on top of it.
    """
    rel = relative_enu(own, intr)
    vie, vin = velocity_enu(intr)
    axis = math.atan2(rel.rx, rel.ry)  # bearing of the line of sight, aviation convention
    gamma = math.asin(min(RPZ / rel.dist, 1.0))
    apex = np.array([vie, vin]) * MS_TO_KT
    legs = [apex + reach * np.array([math.sin(axis + s * gamma), math.cos(axis + s * gamma)])
            for s in (-1.0, 1.0)]
    ax.fill(*zip(apex, legs[0], legs[1], strict=True), color=RED, alpha=0.10, lw=0, zorder=0)
    for leg in legs:
        ax.plot([apex[0], leg[0]], [apex[1], leg[1]], color="0.1", lw=1.0, zorder=1)

    # the d_CPA direction: every one of these resolvers steps along +-c_hat, the unit vector to the
    # predicted closest-approach point, so both clouds have to lie on this one line through the
    # ownship's own velocity. At dcpa = 0 that direction is perpendicular to the line of sight.
    voe, von = velocity_enu(own)
    origin = np.array([voe, von]) * MS_TO_KT
    c_hat = np.array([math.cos(axis), -math.sin(axis)])  # perpendicular to r_hat
    ax.plot(*zip(origin - reach * c_hat, origin + reach * c_hat, strict=True),
            color=RED, ls="--", lw=1.0, alpha=0.7, zorder=1)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--n", type=int, default=10_000, help="Monte Carlo samples per resolver")
    p.add_argument("--plot", type=int, default=1500, help="samples actually drawn")
    p.add_argument("--seed", type=int, default=11)
    p.add_argument("--gamma", type=float, default=None,
                   help="pin the velocity-obstacle half-angle [deg] instead of using the "
                        "campaign's tlos; larger gamma = shorter range")
    p.add_argument("--out", type=Path, default=HERE / "fig_vo_frame.png")
    cfg = p.parse_args()

    resolvers: dict[str, ConflictResolver] = {
        "MVP": MVP(margin=MARGIN),
        f"RMVP{CONFIDENCE:.2f}": RMVP(margin=MARGIN, confidence=CONFIDENCE),
    }
    colours = {"MVP": GREY, f"RMVP{CONFIDENCE:.2f}": RED}

    fig, axes = plt.subplots(len(ANGLES), len(SOURCES), figsize=(11.0, 7.6))
    for row, dpsi in enumerate(ANGLES):
        tlos = spawn_tlos(dpsi, cfg.gamma)
        own_t, intr_t = truth(dpsi, tlos)
        dist = relative_enu(own_t, intr_t).dist
        gamma = math.degrees(math.asin(min(RPZ / dist, 1.0)))
        print(f"{dpsi:g}deg: range {dist:.1f} m, VO half-angle {gamma:.1f} deg, "
              f"tlos {tlos:.1f} s", flush=True)
        for col, (source, pos_ci95, vel_ci95) in enumerate(SOURCES):
            ax = axes[row][col]
            own, intr = states(dpsi, pos_ci95, vel_ci95, tlos)
            cloud, share, star = {}, {}, {}
            for name, resolver in resolvers.items():
                cloud[name], share[name] = samples(resolver, own, intr, cfg.n, cfg.seed)
                v = resolver.resolve(own_t, [intr_t], RPZ).target_velocity
                star[name] = np.array(v) * MS_TO_KT

            # Framed on the 1st-99th percentile of both clouds *and* both true solutions, then
            # squared. Percentiles alone would crop the second velocity-obstacle leg: at dcpa = 0
            # the perceived passing side is a coin flip, so each cloud is genuinely bimodal and the
            # two modes are the two legs. That split is the paper's own result, not an artefact, so
            # the frame has to hold it.
            anchors = np.vstack([*(np.percentile(c, [2, 98], axis=0) for c in cloud.values()),
                                 *star.values()])
            lo, hi = anchors.min(axis=0), anchors.max(axis=0)
            centre, half = (lo + hi) / 2.0, float(np.max(hi - lo)) / 2.0 * 1.30
            draw_obstacle(ax, own_t, intr_t, reach=8.0 * half)
            # the airframe's own limit. A resolution velocity outside it is not a manoeuvre, it is
            # a request: the kinematics layer clamps the command into [v_min, v_max], so the
            # aircraft flies the nearest speed it has and the intended geometry is not achieved.
            ax.add_patch(plt.Circle((0.0, 0.0), V_MAX_KT, fill=False, ls=":", lw=1.0,
                                    color="0.25", zorder=2))

            for name in resolvers:
                unflyable = float(np.mean(np.hypot(*cloud[name].T) > V_MAX_KT))
                ax.scatter(*cloud[name][:cfg.plot].T, s=5, alpha=0.30, color=colours[name], lw=0,
                           label=f"{name}  {share[name]:.1%} in VO, {unflyable:.1%} > envelope")
                # open, not filled: under position-only noise a mode is ~0.1 kt across and a solid
                # marker hides it completely — and that mode is exactly where the true solution
                # sits, so the panel would read as "no samples here"
                ax.scatter(*star[name], marker="*", s=260, facecolor="none",
                           edgecolor=colours[name], lw=1.5, zorder=5)
            ax.set_xlim(centre[0] - half, centre[0] + half)
            ax.set_ylim(centre[1] - half, centre[1] + half)
            ax.set_aspect("equal")
            ax.set_box_aspect(1)
            ax.legend(frameon=False, fontsize=7.5, loc="upper left", markerscale=2.2,
                      handletextpad=0.2)
            if row == 0:
                ax.set_title(f"{source} uncertainty", fontsize=10)
            if row == len(ANGLES) - 1:
                ax.set_xlabel("cross-track resolution speed [kt]")
            if col == 0:
                ax.set_ylabel(f"$\\Delta\\psi$ = {dpsi:g}°, range {dist:.0f} m\n"
                              "along-track speed [kt]")
            print(f"  {source:>22}: " + "   ".join(
                f"{n} {share[n]:5.1%} in VO / "
                f"{float(np.mean(np.hypot(*cloud[n].T) > V_MAX_KT)):5.1%} > envelope"
                for n in resolvers), flush=True)

    fig.tight_layout()
    fig.savefig(cfg.out, dpi=130)
    print(f"\nwrote {cfg.out}")


if __name__ == "__main__":
    main()
