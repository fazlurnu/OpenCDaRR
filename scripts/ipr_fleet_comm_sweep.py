"""Fleet IPR under lossy perception — the 6f payoff (perfect vs lossy comm, vs density).

The lossy counterpart of :mod:`ipr_fleet_sweep` (6e). Same swap-ring superconflict and seeded GNSS
noise, but each realisation is run **twice on the same noise**: once with **perfect** perception
(6e's baseline — every aircraft sees every other's true broadcast instantly) and once with a
**lossy** link (per-link Bernoulli reception + lognormal latency, ADR 0006), so aircraft act on
dropped, stale, *asymmetric* pictures. The gap between the two IPR curves is what asymmetric
perception costs — the erosion [[fleet-ipr-sweep]] predicted the perfect baseline was hiding.

    IPR = 1 − (realisations with any-pair LoS) / n     (measured on true separations)

Running both arms on the *same* nav substream isolates perception as the difference. Reproduce::

    PYTHONPATH=. python scripts/ipr_fleet_comm_sweep.py --n 200 --jobs 8

The base link's asymmetry is only *statistical* (i.i.d. Bernoulli). Three optional knobs — all
default **off** — add the *structural* / correlated effects that bite harder (each writes its own
figure so the baseline is never clobbered):

* ``--blind-pairs K`` — K aircraft pairs persistently mutually blind (both directed links at
  reception 0): a structural down link, not a lucky run of drops.
* ``--random-phase`` — the lossy fleet gets random per-aircraft phase ([[broadcast-phase-offset]]).
* ``--broadcast-jitter S`` — per-transmission slot dither ([[broadcast-jitter]]).

Writes ``…/ipr-fleet-comm-sweep.png`` (base) or a ``-blind…/-phase/-jit…`` variant.
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
from opencdarr.cns import Comm, GnssNavigation, lognormal_latency  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.fleet import Agent, random_broadcast_phase, run_fleet  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402


def _ring_agents(n: int, cfg: argparse.Namespace) -> list[Agent]:
    fleet = sc.swap_ring(n, speed=cfg.speed, radius=cfg.radius)
    return [Agent(replace(s, pos_ci95=cfg.pos_ci95, vel_ci95=cfg.vel_ci95), M600)
            for s, _ in fleet]


def _kwargs(cfg: argparse.Namespace) -> dict[str, object]:
    return dict(
        rpz=cfg.rpz, t_lookahead=cfg.lookahead, dt=cfg.dt, detector=StateBased(),
        resolver=MVP(margin=cfg.margin), recovery=PastCPA(bouncing_guard=True),
        t_max=cfg.t_max, done_timeout=cfg.done_timeout, broadcast_interval=cfg.broadcast_interval,
    )


def _blind_pairs(ids: list[str], k: int) -> list[tuple[str, str]]:
    """K non-overlapping aircraft pairs to render mutually blind (structural down links)."""
    return [(ids[2 * i], ids[2 * i + 1]) for i in range(min(k, len(ids) // 2))]


def _lossy_comm(cfg: argparse.Namespace, ids: list[str]) -> Comm:
    """The lossy link: base i.i.d. reception, optionally with structurally-down *blind pairs*.

    ``--blind-pairs K`` sets both directions of K aircraft pairs to reception 0 (persistently
    down), a *structural* asymmetry rather than the i.i.d. Bernoulli of the base link.
    """
    latency = lognormal_latency(cfg.lat_median, cfg.lat_sigma)
    if cfg.blind_pairs <= 0:
        return Comm(reception_prob=cfg.reception, latency=latency)
    recep = {(a, b): cfg.reception for a in ids for b in ids if a != b}
    for a, b in _blind_pairs(ids, cfg.blind_pairs):
        recep[(a, b)] = recep[(b, a)] = 0.0  # both directions persistently down
    return Comm(reception_prob=recep, latency=latency)


def _one(
    n: int, nav_seq: np.random.SeedSequence, comm_seq: np.random.SeedSequence,
    phase_seq: np.random.SeedSequence, jit_seq: np.random.SeedSequence, cfg: argparse.Namespace,
) -> tuple[bool, bool, float, float]:
    """One realisation on the *same* nav noise: (perfect, lossy) LoS + min-sep.

    The perfect arm is the aligned 6e baseline. Structural asymmetry (``--blind-pairs``) and the
    transmit-timing effects (``--random-phase``, ``--broadcast-jitter``) apply to the *lossy* arm
    only; each defaults off, so the plain sweep is unchanged.
    """
    kw = _kwargs(cfg)
    agents = _ring_agents(n, cfg)
    ids = [a.state.id for a in agents]
    perfect = run_fleet(agents, navigation=GnssNavigation(), rng=generator(nav_seq), **kw)
    phase = (random_broadcast_phase(n, cfg.broadcast_interval, generator(phase_seq))
             if cfg.random_phase else None)
    lossy_kw = dict(kw)
    if cfg.broadcast_jitter > 0.0:
        lossy_kw["broadcast_jitter"] = cfg.broadcast_jitter
        lossy_kw["broadcast_rng"] = generator(jit_seq)
    lossy = run_fleet(agents, navigation=GnssNavigation(), rng=generator(nav_seq),
                      communication=_lossy_comm(cfg, ids), comm_rng=generator(comm_seq),
                      broadcast_phase=phase, **lossy_kw)
    return perfect.los, lossy.los, perfect.min_sep, lossy.min_sep


def _lossy_label(cfg: argparse.Namespace) -> str:
    """A one-line description of the lossy arm's configuration for the legend / logs."""
    bits = [f"reception {cfg.reception}", f"latency ~{cfg.lat_median}s"]
    if cfg.blind_pairs:
        bits.append(f"{cfg.blind_pairs} blind pair(s)")
    if cfg.random_phase:
        bits.append("random phase")
    if cfg.broadcast_jitter:
        bits.append(f"jitter ±{cfg.broadcast_jitter}s")
    return "lossy (" + ", ".join(bits) + ")"


def plot(sizes: list[int], res: dict[str, dict[str, list[float]]], cfg: argparse.Namespace,
         out: Path) -> None:
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 5.4))
    a1.plot(sizes, res["perfect"]["ipr"], marker="o", lw=2.0, color="tab:blue",
            label="perfect perception (6e)")
    a1.plot(sizes, res["lossy"]["ipr"], marker="s", lw=2.0, color="tab:red",
            label=_lossy_label(cfg))
    a1.set_xlabel("fleet size N (aircraft on the ring)")
    a1.set_ylabel("IPR  (1 − any-pair LoS / n)")
    a1.set_ylim(-0.02, 1.02)
    a1.set_xticks(sizes)
    a1.set_title(f"Lossy perception erodes fleet IPR — {cfg.n} seeds/point")
    a1.grid(True, alpha=0.3)
    a1.legend(fontsize=9)

    a2.plot(sizes, res["perfect"]["med"], marker="o", lw=2.0, color="tab:blue", label="perfect")
    a2.plot(sizes, res["lossy"]["med"], marker="s", lw=2.0, color="tab:red", label="lossy")
    a2.axhline(cfg.rpz, color="0.4", ls="--", lw=1.0, label=f"rpz = {cfg.rpz:.0f} m")
    a2.set_xlabel("fleet size N (aircraft on the ring)")
    a2.set_ylabel("fleet min pairwise sep [m], median")
    a2.set_xticks(sizes)
    a2.set_title("Safety margin vs density — perfect vs lossy")
    a2.grid(True, alpha=0.3)
    a2.legend(fontsize=9)

    fig.suptitle(
        f"Phase 6f: fleet IPR under lossy perception (swap-ring MVP, radius {cfg.radius:.0f} m, "
        f"{cfg.speed:.0f} m/s) — GNSS {cfg.pos_ci95:.0f} m/{cfg.vel_ci95:.0f}(m/s), "
        f"lookahead {cfg.lookahead:.0f} s, margin {cfg.margin}",
        fontsize=11,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sizes", type=int, nargs="+", default=[2, 4, 6, 8, 10, 12])
    p.add_argument("--radius", type=float, default=1500.0)
    p.add_argument("--speed", type=float, default=10.0)
    p.add_argument("--pos-ci95", dest="pos_ci95", type=float, default=10.0)
    p.add_argument("--vel-ci95", dest="vel_ci95", type=float, default=1.0)
    p.add_argument("--rpz", type=float, default=50.0)
    p.add_argument("--lookahead", type=float, default=30.0)
    p.add_argument("--margin", type=float, default=1.05)
    p.add_argument("--dt", type=float, default=0.5)
    p.add_argument("--broadcast-interval", dest="broadcast_interval", type=float, default=1.0)
    p.add_argument("--reception", type=float, default=0.8, help="per-link Bernoulli delivery prob")
    p.add_argument("--lat-median", dest="lat_median", type=float, default=0.1)
    p.add_argument("--lat-sigma", dest="lat_sigma", type=float, default=0.25)
    # structural asymmetry / transmit-timing knobs — all default OFF, so the plain sweep is intact
    p.add_argument("--blind-pairs", dest="blind_pairs", type=int, default=0,
                   help="structurally-down (mutually blind) aircraft pairs, lossy arm (0=off)")
    p.add_argument("--random-phase", dest="random_phase", action="store_true",
                   help="give the lossy fleet random per-aircraft broadcast phase (else aligned)")
    p.add_argument("--broadcast-jitter", dest="broadcast_jitter", type=float, default=0.0,
                   help="per-transmission jitter half-width [s] in the lossy arm (0=off)")
    p.add_argument("--t-max", dest="t_max", type=float, default=600.0)
    p.add_argument("--done-timeout", dest="done_timeout", type=float, default=10.0)
    p.add_argument("--n", type=int, default=200, help="noise realisations per point")
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    cfg = p.parse_args()

    # one (nav, comm, phase, jitter) substream quad per realisation, reused across sizes so the
    # perfect/lossy comparison is controlled and reproducible from --seed
    quads = [spawn(s, 4) for s in spawn(root_seed_sequence(cfg.seed), cfg.n)]
    print(f"Fleet lossy-IPR sweep — sizes={cfg.sizes}, {_lossy_label(cfg)}, "
          f"{cfg.n} seeds, joblib {cfg.jobs} cores")
    print(f"{'N':>4} {'IPR perfect':>12} {'IPR lossy':>10} {'Δ':>7} "
          f"{'med perfect':>12} {'med lossy':>10}")

    t0 = time.time()
    res = {arm: {"ipr": [], "med": []} for arm in ("perfect", "lossy")}
    for n in cfg.sizes:
        rows = Parallel(n_jobs=cfg.jobs)(
            delayed(_one)(n, nav, comm, phase, jit, cfg) for nav, comm, phase, jit in quads
        )
        los_p = sum(r[0] for r in rows)
        los_l = sum(r[1] for r in rows)
        ipr_p = 1.0 - los_p / cfg.n
        ipr_l = 1.0 - los_l / cfg.n
        res["perfect"]["ipr"].append(ipr_p)
        res["lossy"]["ipr"].append(ipr_l)
        res["perfect"]["med"].append(float(np.median([r[2] for r in rows])))
        res["lossy"]["med"].append(float(np.median([r[3] for r in rows])))
        print(f"{n:4d} {ipr_p:12.4f} {ipr_l:10.4f} {ipr_p - ipr_l:7.4f} "
              f"{res['perfect']['med'][-1]:10.1f} m {res['lossy']['med'][-1]:8.1f} m")
    print(f"(elapsed {time.time() - t0:.1f} s)")

    # base run writes the 6f observation figure; structural/timing variants get their own name so a
    # --blind-pairs / --random-phase / --broadcast-jitter run never clobbers the committed baseline
    tag = ""
    if cfg.blind_pairs:
        tag += f"-blind{cfg.blind_pairs}"
    if cfg.random_phase:
        tag += "-phase"
    if cfg.broadcast_jitter:
        tag += f"-jit{cfg.broadcast_jitter}"
    out = (Path(__file__).resolve().parents[1]
           / f"vault/observations/img/ipr-fleet-comm-sweep{tag}.png")
    plot(cfg.sizes, res, cfg, out)


if __name__ == "__main__":
    main()
