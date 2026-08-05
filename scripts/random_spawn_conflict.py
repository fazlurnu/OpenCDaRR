"""Conflict probability for the simultaneous random-spawn disc — analytically, then measured.

The traffic model of ``examples/handbook/ring_mc_vs_ips.ipynb`` part 2: ``N`` aircraft drawn by
the entry rule of Groot, Ellerbroek & Hoekstra (2024), released together on a spawn circle of
radius ``R_OUTER`` at a common speed, with separation measured only inside the disc of radius
``R_INNER``. Every aircraft then flies a straight line from a common start time, so once the rule
has been drawn the encounter is fully determined and its closest approach is a closed form — no
integration of a trajectory is needed to know whether a pair conflicts.

That makes the pair conflict probability an integral of an indicator over three dimensions (the
absolute heading drops out by rotational symmetry), evaluated here by Sobol quasi-Monte-Carlo::

    p = (2*pi * (2 R_INNER)^2)^-1 * int int int 1[ min_t |r0 + v t| < D ] d(dpsi) dx1 dx2

``--sim`` re-flies the same draws through :func:`~opencdarr.fleet.build_env` with the resolver
removed, which is a genuinely independent route to the same number: closed-form CPA algebra on one
side, the full environment (kinematics, waypoint guidance, the measurement gate) on the other.

Written up in ``vault/derivations/random-spawn-conflict-probability.md``.

    PYTHONPATH=. python scripts/random_spawn_conflict.py
    PYTHONPATH=. python scripts/random_spawn_conflict.py --sim --draws 20000
"""

from __future__ import annotations

import argparse
import dataclasses
import math
from itertools import combinations

import numpy as np
from scipy.stats import qmc

from opencdarr import (
    M600,
    Agent,
    MeasurementArea,
    StateBased,
    random_traffic,
    run_fleet,
)
from opencdarr.autopilot import WaypointAutopilot
from opencdarr.mission import Mission
from opencdarr.rng import children, generator, root_seed_sequence

CENTRE = (52.0, 4.0)
R_INNER = 1000.0     # the measured disc [m]
R_OUTER = 1200.0     # the spawn circle [m]
D = 50.0             # protected zone [m]
V = 10.0             # common ground speed [m/s]


# --- the closed form ---------------------------------------------------------------------------
def closest_approach(psi1, x1, psi2, x2):
    """Closest approach [m] of a pair, over the window in which both are inside the disc.

    ``inf`` when the two are never inside together — a pair that shares no measured time cannot
    produce a measured conflict, however close their tracks pass.
    """
    d1 = np.stack([np.sin(psi1), np.cos(psi1)], axis=-1)
    d2 = np.stack([np.sin(psi2), np.cos(psi2)], axis=-1)
    n1 = np.stack([np.cos(psi1), -np.sin(psi1)], axis=-1)
    n2 = np.stack([np.cos(psi2), -np.sin(psi2)], axis=-1)
    half1, half2 = np.sqrt(R_OUTER**2 - x1**2), np.sqrt(R_OUTER**2 - x2**2)

    r0 = ((x1[..., None] * n1 - half1[..., None] * d1)      # relative position at t = 0
          - (x2[..., None] * n2 - half2[..., None] * d2))
    vrel = V * (d1 - d2)                                     # relative velocity (constant)

    c1 = np.sqrt(np.maximum(R_INNER**2 - x1**2, 0.0))
    c2 = np.sqrt(np.maximum(R_INNER**2 - x2**2, 0.0))
    t_lo = np.maximum((half1 - c1) / V, (half2 - c2) / V)
    t_hi = np.minimum((half1 + c1) / V, (half2 + c2) / V)

    bb = np.einsum("...i,...i->...", vrel, vrel)
    ab = np.einsum("...i,...i->...", r0, vrel)
    with np.errstate(divide="ignore", invalid="ignore"):
        t_star = np.where(bb > 1e-12, -ab / np.maximum(bb, 1e-12), t_lo)
    t = np.clip(t_star, t_lo, t_hi)
    return np.where(t_hi > t_lo, np.linalg.norm(r0 + t[..., None] * vrel, axis=-1), np.inf)


def tracks_cross_inside(psi1, x1, psi2, x2):
    """Do the two straight *tracks* intersect inside the disc — the time-blind question."""
    d1 = np.stack([np.sin(psi1), np.cos(psi1)], axis=-1)
    d2 = np.stack([np.sin(psi2), np.cos(psi2)], axis=-1)
    n1 = np.stack([np.cos(psi1), -np.sin(psi1)], axis=-1)
    n2 = np.stack([np.cos(psi2), -np.sin(psi2)], axis=-1)
    p1 = x1[..., None] * n1 - np.sqrt(R_OUTER**2 - x1**2)[..., None] * d1
    p2 = x2[..., None] * n2 - np.sqrt(R_OUTER**2 - x2**2)[..., None] * d2
    denom = d1[..., 0] * d2[..., 1] - d1[..., 1] * d2[..., 0]
    dp = p2 - p1
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (dp[..., 0] * d2[..., 1] - dp[..., 1] * d2[..., 0]) / denom
    inside = np.linalg.norm(p1 + s[..., None] * d1, axis=-1) <= R_INNER
    return np.where(np.abs(denom) > 1e-12, inside, False)


def shifted_closest(psi1, x1, psi2, x2, dt):
    """As :func:`closest_approach`, but aircraft 2's clock is offset by ``dt`` seconds."""
    d1 = np.stack([np.sin(psi1), np.cos(psi1)], axis=-1)
    d2 = np.stack([np.sin(psi2), np.cos(psi2)], axis=-1)
    n1 = np.stack([np.cos(psi1), -np.sin(psi1)], axis=-1)
    n2 = np.stack([np.cos(psi2), -np.sin(psi2)], axis=-1)
    half1, half2 = np.sqrt(R_OUTER**2 - x1**2), np.sqrt(R_OUTER**2 - x2**2)
    p1 = x1[..., None] * n1 - half1[..., None] * d1
    p2 = x2[..., None] * n2 - half2[..., None] * d2
    r0 = p1 - (p2 - V * dt[..., None] * d2)
    vrel = V * (d1 - d2)
    c1 = np.sqrt(np.maximum(R_INNER**2 - x1**2, 0.0))
    c2 = np.sqrt(np.maximum(R_INNER**2 - x2**2, 0.0))
    t_lo = np.maximum((half1 - c1) / V, (half2 - c2) / V + dt)
    t_hi = np.minimum((half1 + c1) / V, (half2 + c2) / V + dt)
    bb = np.einsum("...i,...i->...", vrel, vrel)
    ab = np.einsum("...i,...i->...", r0, vrel)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.clip(np.where(bb > 1e-12, -ab / np.maximum(bb, 1e-12), t_lo), t_lo, t_hi)
    return np.where(t_hi > t_lo, np.linalg.norm(r0 + t[..., None] * vrel, axis=-1), np.inf)


def pair_probability(log2_points: int, seed: int = 0) -> tuple[float, float]:
    """Sobol QMC estimate of the pair conflict probability, with a half-sample spread."""
    m = 1 << log2_points
    u = qmc.Sobol(d=3, scramble=True, seed=seed).random(m)
    dpsi = 2 * math.pi * u[:, 0]
    x1, x2 = R_INNER * (2 * u[:, 1] - 1), R_INNER * (2 * u[:, 2] - 1)
    hit = closest_approach(np.zeros_like(dpsi), x1, dpsi, x2) < D
    return float(hit.mean()), abs(hit[: m // 2].mean() - hit[m // 2:].mean()) / 2


def fleet_counts(n: int, draws: int, seed: int = 7) -> np.ndarray:
    """Number of conflicting pairs in each of ``draws`` fleets of ``n`` — exact sampling."""
    rng = np.random.default_rng(seed)
    psi = rng.uniform(0, 2 * math.pi, size=(draws, n))
    x = rng.uniform(-R_INNER, R_INNER, size=(draws, n))
    count = np.zeros(draws, dtype=np.int16)
    for i, j in combinations(range(n), 2):
        count += closest_approach(psi[:, i], x[:, i], psi[:, j], x[:, j]) < D
    return count


def gas_model(n: int) -> float:
    """Swept-area estimate of the expected conflict count, as an independent order check."""
    e_vrel = 4 * V / math.pi                       # E|v_rel| for uniform relative heading
    t_mean = (math.pi * R_INNER / 2) / V           # mean chord / speed
    return math.comb(n, 2) * 2 * D * e_vrel * t_mean / (math.pi * R_INNER**2)


# --- the same thing, flown ---------------------------------------------------------------------
PERF = dataclasses.replace(M600, v_max=V, v_min=-V)
STRAIGHT = dict(rpz=D, t_lookahead=30.0, dt=0.5, detector=StateBased(), resolver=None,
                recovery=None, t_max=400.0, done_timeout=10.0, stop_within=50.0)


def flown_min_sep(n: int, seq) -> float:
    """One draw flown straight with no resolver and no noise: the geometry, integrated.

    The scenario and the measurement gate are the package's
    (:func:`~opencdarr.scenario.random_traffic`, :class:`~opencdarr.fleet.MeasurementArea`), so
    this is the same environment the handbook notebook and the campaign runner fly — which is what
    makes the agreement in §7 of the write-up a check on the analysis rather than on a re-typing.
    """
    fleet = random_traffic(n, generator(seq), speed=V, r_inner=R_INNER, r_outer=R_OUTER,
                           lat0=CENTRE[0], lon0=CENTRE[1])
    agents = [Agent(state, PERF, autopilot=WaypointAutopilot(Mission(goto=goal),
                                                             capture_radius=30.0))
              for state, goal in fleet]
    return run_fleet(agents, measure_within=MeasurementArea(CENTRE, R_INNER), **STRAIGHT).min_sep


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sizes", type=int, nargs="+", default=[2, 4, 6])
    p.add_argument("--log2-points", dest="log2", type=int, default=24)
    p.add_argument("--fleet-draws", dest="fleet_draws", type=int, default=1_000_000)
    p.add_argument("--sim", action="store_true", help="also fly the draws (slow)")
    p.add_argument("--draws", type=int, default=20_000, help="encounters per size for --sim")
    p.add_argument("--jobs", type=int, default=-1)
    cfg = p.parse_args()

    print("pair conflict probability — Sobol QMC convergence")
    for k in range(16, cfg.log2 + 1, 2):
        prob, spread = pair_probability(k)
        print(f"  2^{k:<2} = {1 << k:>10,} pts   p = {prob:.5f}   half-sample spread {spread:.5f}")
    p_pair, _ = pair_probability(cfg.log2)

    print(f"\nfrom pairs to fleets  (p = {p_pair:.5f}, {cfg.fleet_draws:,} sampled fleets)")
    print(f"{'N':>3}  {'pairs':>6}  {'E[count]':>9}  {'sampled':>8}  {'P(>=1)':>7}  "
          f"{'indep.':>7}  {'Var':>6}  {'binom Var':>9}  {'gas model':>9}")
    for n in cfg.sizes:
        m = math.comb(n, 2)
        count = fleet_counts(n, cfg.fleet_draws)
        print(f"{n:>3}  {m:>6}  {m * p_pair:>9.4f}  {count.mean():>8.4f}  "
              f"{(count > 0).mean():>7.4f}  {1 - (1 - p_pair) ** m:>7.4f}  {count.var():>6.4f}  "
              f"{m * p_pair * (1 - p_pair):>9.4f}  {gas_model(n):>9.4f}")

    n_show = max(cfg.sizes)
    count = fleet_counts(n_show, cfg.fleet_draws)
    print(f"\nconflicting pairs per fleet, N = {n_show} (per 1000 fleets)")
    for k, share in enumerate(np.bincount(count) / count.size):
        if share * 1000 < 0.05:
            break
        print(f"  {share * 1000:7.1f} fleets with {k}  ->  {share * 1000 * k:7.1f} pairs")

    print("\nwhat the timing contributes")
    m = 1 << min(cfg.log2, 24)
    u = qmc.Sobol(d=4, scramble=True, seed=5).random(m)
    dpsi = 2 * math.pi * u[:, 0]
    x1, x2 = R_INNER * (2 * u[:, 1] - 1), R_INNER * (2 * u[:, 2] - 1)
    zeros = np.zeros_like(dpsi)
    cross = tracks_cross_inside(zeros, x1, dpsi, x2)
    conflict = closest_approach(zeros, x1, dpsi, x2) < D
    transit = (math.pi * R_INNER / 2) / V
    stagger = shifted_closest(zeros, x1, dpsi, x2, 2 * transit * (2 * u[:, 3] - 1)) < D
    print(f"  tracks cross inside the disc            {cross.mean():.4f}")
    print(f"  aircraft pass within {D:.0f} m               {conflict.mean():.4f}")
    print(f"  of the crossing pairs, those in conflict "
          f"{(conflict & cross).sum() / cross.sum():.4f}")
    print(f"  conflicts whose tracks never cross       "
          f"{(conflict & ~cross).sum() / conflict.sum():.4f}")
    print(f"  start times decorrelated                {stagger.mean():.4f}")

    if not cfg.sim:
        print("\n(--sim re-flies the draws through the environment; slow)")
        return

    from joblib import Parallel, delayed

    from opencdarr.estimator import wilson_interval
    print(f"\nflown: no resolver, no noise, {cfg.draws:,} draws per size")
    for n in cfg.sizes:
        seqs = children(root_seed_sequence(3), 0, cfg.draws)
        ms = np.array(Parallel(n_jobs=cfg.jobs, batch_size=64)(
            delayed(flown_min_sep)(n, s) for s in seqs), dtype=float)
        hits = int(np.sum(ms < D))
        lo, hi = wilson_interval(hits, cfg.draws)
        print(f"  N = {n}:  P(conflict) = {hits / cfg.draws:.4f}   "
              f"95% Wilson [{lo:.4f}, {hi:.4f}]", flush=True)


if __name__ == "__main__":
    main()
