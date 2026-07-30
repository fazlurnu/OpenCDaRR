"""Multi-aircraft IPR-vs-density sweep (Phase 6e — the quantitative fleet payoff).

The N-aircraft analogue of :mod:`ipr_angle_sweep` / :mod:`ipr_wind_sweep`: instead of one conflict
pair over noise realisations, a whole **ring** of ``N`` aircraft, each crossing to the
diametrically-opposite start (the swap-ring superconflict of [[fleet-cooperative-ring]]), run
through :func:`~opencdarr.fleet.run_fleet` over ``--n`` seeded GNSS-noise realisations. The IPR is

    IPR = 1 − (realisations with **any-pair** LoS) / n

— LoS anywhere in the fleet fails the realisation, measured on the true separations. Sweeping the
fleet size answers "does detect-and-avoid hold as traffic thickens?": more aircraft on the same
ring means a denser centre crossing and less room to manoeuvre, so the IPR degrades with density.

**Reduces to the pairwise IPR at N = 2** (the plan's check): the two-aircraft ring is a head-on
pair, and ``run_fleet`` on it is bit-for-bit ``run_encounter`` (``test_fleet.py``). ``--verify-n2``
re-runs that pair through *both* runners on the same substreams and asserts LoS / min-sep match.

    PYTHONPATH=. python scripts/ipr_fleet_sweep.py                 # MVP + VO, default sweep + plot
    PYTHONPATH=. python scripts/ipr_fleet_sweep.py --n 500 --jobs 8
    PYTHONPATH=. python scripts/ipr_fleet_sweep.py --sizes 2 4 6 8 --resolvers mvp

Writes ``vault/observations/img/ipr-fleet-sweep.png``.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from joblib import Parallel, delayed  # noqa: E402

from opencdarr import scenario as sc  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import (
    BroadcastSchedule,  # noqa: E402
    GnssNavigation,  # noqa: E402
)
from opencdarr.cr import MVP, VO  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.fleet import Agent, FleetOutcome, run_fleet  # noqa: E402
from opencdarr.loop import run_encounter  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402

_COLORS = {"mvp": "tab:blue", "vo": "tab:orange"}


def _resolver(name: str, margin: float) -> ConflictResolver:
    return {"mvp": MVP, "vo": VO}[name](margin=margin)


def _ring_agents(n: int, cfg: argparse.Namespace) -> list[Agent]:
    """``n`` cruise aircraft on the swap-ring, each carrying the declared GNSS accuracy."""
    fleet = sc.swap_ring(n, speed=cfg.speed, radius=cfg.radius)
    return [
        Agent(replace(s, pos_ci95=cfg.pos_ci95, vel_ci95=cfg.vel_ci95), M600)
        for s, _ in fleet
    ]


def _kwargs(cfg: argparse.Namespace, resolver_name: str) -> dict[str, object]:
    return dict(
        rpz=cfg.rpz, t_lookahead=cfg.lookahead, dt=cfg.dt, detector=StateBased(),
        resolver=_resolver(resolver_name, cfg.margin), recovery=PastCPA(bouncing_guard=True),
        t_max=cfg.t_max, done_timeout=cfg.done_timeout,
        schedule=BroadcastSchedule(interval=cfg.broadcast_interval),
    )


def _one(
    n: int, resolver_name: str, seq: np.random.SeedSequence, cfg: argparse.Namespace
) -> tuple[bool, float]:
    out: FleetOutcome = run_fleet(
        _ring_agents(n, cfg), navigation=GnssNavigation(), rng=generator(seq),
        **_kwargs(cfg, resolver_name),
    )
    return out.los, out.min_sep


def _baseline_min_sep(n: int, resolver_name: str, cfg: argparse.Namespace) -> float:
    """The deterministic (noiseless) fleet min-sep — the margin noise then eats into."""
    return run_fleet(_ring_agents(n, cfg), **_kwargs(cfg, resolver_name)).min_sep


def _verify_n2(seqs: list[np.random.SeedSequence], cfg: argparse.Namespace) -> None:
    """At N = 2 the ring is a head-on pair; run_fleet must match run_encounter on each seed."""
    fleet = sc.swap_ring(2, speed=cfg.speed, radius=cfg.radius)
    own = replace(fleet[0][0], pos_ci95=cfg.pos_ci95, vel_ci95=cfg.vel_ci95)
    intr = replace(fleet[1][0], pos_ci95=cfg.pos_ci95, vel_ci95=cfg.vel_ci95)
    # both runners take the same BroadcastSchedule now, so the comparison uses one kwargs bundle.
    # It used to strip the schedule and substitute its interval, which was equivalent only while
    # the schedule stayed aligned and jitter-free — adding either would have quietly compared a
    # dithered fleet run against an undithered pairwise one.
    kw = _kwargs(cfg, "mvp")
    mismatches = 0
    for seq in seqs:
        enc = run_encounter(own, intr, perf=M600, navigation=GnssNavigation(),
                            rng=generator(seq), **kw)
        flt = run_fleet([Agent(own, M600), Agent(intr, M600)], navigation=GnssNavigation(),
                        rng=generator(seq), **kw)
        if (enc.los, enc.min_sep) != (flt.los, flt.min_sep):
            mismatches += 1
    verdict = "IDENTICAL" if mismatches == 0 else f"{mismatches} MISMATCH(es)"
    print(f"verify N=2: run_fleet == run_encounter over {len(seqs)} substreams -> {verdict}")


def plot(
    sizes: list[int],
    ipr: dict[str, list[float]],
    med: dict[str, list[float]],
    base: dict[str, list[float]],
    cfg: argparse.Namespace,
    out: Path,
) -> None:
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 5.4))

    for name in ipr:
        a1.plot(sizes, ipr[name], marker="o", lw=2.0, color=_COLORS[name], label=name.upper())
    a1.set_xlabel("fleet size N (aircraft on the ring)")
    a1.set_ylabel("IPR  (1 − any-pair LoS / n)")
    a1.set_ylim(-0.02, 1.02)
    a1.set_xticks(sizes)
    a1.set_title(f"Fleet IPR degrades with density — {cfg.n} seeds/point")
    a1.grid(True, alpha=0.3)
    a1.legend(fontsize=9)

    for name in med:
        a2.plot(sizes, med[name], marker="o", lw=2.0, color=_COLORS[name],
                label=f"{name.upper()} median (noisy)")
        a2.plot(sizes, base[name], marker="^", ls=":", lw=1.6, color=_COLORS[name],
                label=f"{name.upper()} noiseless")
    a2.axhline(cfg.rpz, color="tab:red", ls="--", lw=1.0, label=f"rpz = {cfg.rpz:.0f} m")
    a2.set_xlabel("fleet size N (aircraft on the ring)")
    a2.set_ylabel("fleet min pairwise sep [m]")
    a2.set_xticks(sizes)
    a2.set_title("Safety margin vs density — noisy median vs noiseless")
    a2.grid(True, alpha=0.3)
    a2.legend(fontsize=8)

    fig.suptitle(
        f"Phase 6e: multi-aircraft IPR vs fleet density (swap-ring, radius {cfg.radius:.0f} m, "
        f"{cfg.speed:.0f} m/s) — GNSS noise {cfg.pos_ci95:.0f} m / {cfg.vel_ci95:.0f} (m/s), "
        f"lookahead {cfg.lookahead:.0f} s, margin {cfg.margin}",
        fontsize=11,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--resolvers", nargs="+", choices=("mvp", "vo"), default=["mvp", "vo"])
    p.add_argument("--sizes", type=int, nargs="+", default=[2, 4, 6, 8, 10, 12, 14, 16],
                   help="fleet sizes to sweep (use even N for exact diametric opposition)")
    p.add_argument("--radius", type=float, default=1500.0, help="ring radius [m]")
    p.add_argument("--speed", type=float, default=10.0, help="cruise ground speed [m/s]")
    p.add_argument("--pos-ci95", dest="pos_ci95", type=float, default=10.0)
    p.add_argument("--vel-ci95", dest="vel_ci95", type=float, default=1.0)
    p.add_argument("--rpz", type=float, default=50.0)
    p.add_argument("--lookahead", type=float, default=30.0,
                   help="shorter than the pairwise 120 s (a long horizon livelocks the ring)")
    p.add_argument("--margin", type=float, default=1.05)
    p.add_argument("--dt", type=float, default=0.5)
    p.add_argument("--broadcast-interval", dest="broadcast_interval", type=float, default=1.0)
    p.add_argument("--t-max", dest="t_max", type=float, default=600.0)
    p.add_argument("--done-timeout", dest="done_timeout", type=float, default=10.0)
    p.add_argument("--n", type=int, default=200, help="noise realisations per point")
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-verify-n2", dest="verify_n2", action="store_false", default=True)
    cfg = p.parse_args()

    # same substreams per point, reused across sizes/resolvers -> a controlled comparison
    seqs = list(spawn(root_seed_sequence(cfg.seed), cfg.n))
    print(f"Fleet IPR sweep — resolvers={[r.upper() for r in cfg.resolvers]}, sizes={cfg.sizes}, "
          f"radius={cfg.radius:.0f} m, speed={cfg.speed:.0f} m/s, GNSS {cfg.pos_ci95:.0f} m/"
          f"{cfg.vel_ci95:.0f}(m/s), lookahead={cfg.lookahead:.0f}, margin={cfg.margin}, "
          f"{cfg.n} seeds, joblib {cfg.jobs} cores")
    if cfg.verify_n2:
        _verify_n2(seqs, cfg)

    print(f"{'resolver':>9} {'N':>4} {'IPR':>8} {'any-LoS':>10} "
          f"{'median min-sep':>16} {'noiseless':>12}")
    t0 = time.time()
    ipr: dict[str, list[float]] = {r: [] for r in cfg.resolvers}
    med: dict[str, list[float]] = {r: [] for r in cfg.resolvers}
    base: dict[str, list[float]] = {r: [] for r in cfg.resolvers}
    for name in cfg.resolvers:
        for n in cfg.sizes:
            rows = Parallel(n_jobs=cfg.jobs)(delayed(_one)(n, name, s, cfg) for s in seqs)
            n_los = sum(r[0] for r in rows)
            median = float(np.median([r[1] for r in rows]))
            baseline = _baseline_min_sep(n, name, cfg)
            ipr[name].append(1.0 - n_los / cfg.n)
            med[name].append(median)
            base[name].append(baseline)
            print(f"{name.upper():>9} {n:4d} {1 - n_los / cfg.n:8.4f} {f'{n_los}/{cfg.n}':>10} "
                  f"{median:>14.1f} m {baseline:>10.1f} m")
    print(f"(elapsed {time.time() - t0:.1f} s)")

    out = Path(__file__).resolve().parents[1] / "vault/observations/img/ipr-fleet-sweep.png"
    plot(cfg.sizes, ipr, med, base, cfg, out)


if __name__ == "__main__":
    main()
