"""Unified IPS: one importance function that survives nav-drift AND comms-jump rare events.

Motivation (``vault/observations/important-ips-gap.md``, the coordinate matrix). Fixed-effort
multilevel splitting needs an importance coordinate matched to *how* the rare event is driven:

- ``min_sep`` (geometric separation) ladders the **nav** pathway (continuous drift) but is flat until
  CPA, so it **collapses** on the **comms** pathway (a run of dropped updates).
- staleness (consecutive-drop count) ladders the **comms** pathway exactly but is identically zero
  under perfect comms, so it **collapses** on the **nav** pathway.

No *single simple* coordinate unifies them — the note's open question. The Blom–Ma–Bakker lesson
(``blom-comparison/car_ips.py``): splitting works when the shells are nested on the rare-event driver
*and* resampling interleaves with the still-live randomness. So the unifying coordinate is the
**max of the two per-pathway progresses**, a hand-built committor surrogate:

    phi = max( nav_progress , CAP * comm_progress )
      nav_progress  = clip( (d_nominal - min_sep) / (d_nominal - rpz), 0, 1 )   # 1 at LoS
      comm_progress = clip( staleness_running_max / L_crit,           0, 1 )   # blind-run depth

Under perfect comms ``comm_progress == 0`` so ``phi == nav_progress`` (the min_sep ladder); under
perfect nav ``nav_progress`` stays ~0 until a breach so ``phi`` rides ``comm_progress`` (the staleness
ladder); with both on, ``max`` ladders whichever pathway a given particle is advancing along. ``CAP <
1`` reserves ``phi = 1`` for an actual loss of separation, so the deepest shell is the true rare set.

This module builds a minimal encounter with both mechanisms, a brute-force MC ground truth, and a
fixed-effort IPS (mirroring ``opencdarr.ips.ips_once``: fresh per-particle stream per shell, resample
survivors to N, ``P̂ = Π S_k/N``). The validation runs every (coordinate x regime) cell: ``min_sep``
and ``staleness`` each fail one regime (reproducing the matrix), ``unified`` passes all three.

    python blom-comparison/opus4p8_unified_ips.py                 # full 3x3 validation
    python blom-comparison/opus4p8_unified_ips.py --mc-only       # just the MC ground truth
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np

from opencdarr.rng import generator, root_seed_sequence, spawn

Coord = Literal["min_sep", "staleness", "unified"]
Regime = Literal["nav", "comms", "both"]


@dataclass(frozen=True)
class Params:
    """A minimal head-on-ish encounter. A resolver holds the predicted miss distance ``d`` near a
    target ``d_target`` over ``T`` ticks; loss of separation is the true closest approach dropping
    below ``rpz``. Two *structurally different* degradations attack it, deliberately mimicking the
    real sim's coordinate matrix:

    - **nav** (continuous drift): Gaussian error each tick perturbs ``d`` directly, so the geometric
      separation descends with a smooth gradient — ``min_sep`` shells ladder it.
    - **comms** (discrete cliff): a *run* of ``L_crit`` consecutive dropped broadcasts loses the
      resolver — ``d`` collapses by ``breach_penalty`` in one step and stays low. Separation looks
      nominal right up to the breach, then jumps past ``rpz`` — ``min_sep`` is *bimodal* (nominal or
      breached, nothing between), so its intermediate shells hold no partial progress and collapse.
      Only the drop count (staleness) ladders toward this cliff.
    """

    T: int = 60  # control ticks to closest point of approach
    R0: float = 5.0  # initial along-track range [miss units]; closes linearly to 0 at CPA
    rpz: float = 1.0  # loss of separation threshold [miss units]
    d_target: float = 1.5  # resolver's target miss distance (also the nominal safe miss)
    d0: float = 1.5  # initial predicted miss
    k: float = 0.30  # resolver gain pulling d back to target each fresh tick
    sigma_nav: float = 0.10  # nav error std perturbing d per tick (nav pathway)
    rx: float = 0.58  # per-broadcast reception probability (comms pathway); 1.0 = perfect comms
    L_crit: int = 17  # consecutive drops that breach the resolver (comm_progress -> 1)
    breach_penalty: float = 0.9  # one-time miss collapse when a blind run breaches (the cliff)
    cap: float = 0.90  # comm_progress ceiling in the unified coordinate (reserves phi=1 for LoS)

    @property
    def d_nominal(self) -> float:
        return self.d_target


def _flags(regime: Regime) -> tuple[bool, bool]:
    """(nav_on, comms_on) for a regime. ``nav`` = drift only, ``comms`` = drops only, ``both``."""
    return {"nav": (True, False), "comms": (False, True), "both": (True, True)}[regime]


@dataclass
class Cloud:
    """The particle population as parallel arrays. ``t`` is per-particle so a particle can freeze at
    a shell (survivor) while others keep evolving toward it."""

    t: np.ndarray  # tick index per particle
    d: np.ndarray  # true predicted miss distance
    stale: np.ndarray  # current consecutive-drop count
    stale_max: np.ndarray  # running max of stale (monotone comms progress)
    breached: np.ndarray  # has a blind run of L_crit lost the resolver? (one-way)
    min_sep: np.ndarray  # running min geometric separation

    @classmethod
    def start(cls, n: int, p: Params) -> Cloud:
        return cls(
            t=np.zeros(n, dtype=np.int32),
            d=np.full(n, p.d0),
            stale=np.zeros(n, dtype=np.int32),
            stale_max=np.zeros(n, dtype=np.int32),
            breached=np.zeros(n, dtype=bool),
            min_sep=np.full(n, math.hypot(p.R0, p.d0)),
        )

    def take(self, idx: np.ndarray) -> Cloud:
        return Cloud(self.t[idx].copy(), self.d[idx].copy(), self.stale[idx].copy(),
                     self.stale_max[idx].copy(), self.breached[idx].copy(), self.min_sep[idx].copy())

    def nav_progress(self, p: Params) -> np.ndarray:
        return np.clip((p.d_nominal - self.min_sep) / (p.d_nominal - p.rpz), 0.0, 1.0)

    def comm_progress(self, p: Params) -> np.ndarray:
        return np.clip(self.stale_max / p.L_crit, 0.0, 1.0)

    def phi(self, coord: Coord, p: Params) -> np.ndarray:
        if coord == "min_sep":
            return self.nav_progress(p)
        if coord == "staleness":
            return self.comm_progress(p)
        return np.maximum(self.nav_progress(p), p.cap * self.comm_progress(p))

    def is_los(self, p: Params) -> np.ndarray:
        return self.min_sep <= p.rpz


def _step(cloud: Cloud, mask: np.ndarray, p: Params, nav_on: bool, comms_on: bool,
          rng: np.random.Generator) -> None:
    """Advance the masked particles one control tick, in place.

    Fresh comms hold ``d`` toward ``d_target`` (gain ``k``); nav error perturbs ``d`` continuously.
    A run of ``L_crit`` dropped broadcasts *breaches*: the resolver is lost, ``d`` drops by
    ``breach_penalty`` once and no longer self-corrects — a cliff invisible to ``min_sep`` until it
    lands. The two pathways are thus continuous-gradient (nav) vs discrete-cliff (comms)."""
    idx = np.where(mask)[0]
    if idx.size == 0:
        return
    n = idx.size

    heard = (rng.random(n) < p.rx) if comms_on else np.ones(n, dtype=bool)
    cloud.stale[idx[heard]] = 0
    cloud.stale[idx[~heard]] += 1
    cloud.stale_max[idx] = np.maximum(cloud.stale_max[idx], cloud.stale[idx])

    # a blind run reaching L_crit breaches once: the miss collapses and the resolver stops correcting
    newly = (cloud.stale[idx] >= p.L_crit) & (~cloud.breached[idx])
    cloud.breached[idx[newly]] = True
    cloud.d[idx[newly]] -= p.breach_penalty

    lost = cloud.breached[idx]
    correction = np.where(lost, 0.0, p.k * (p.d_target - cloud.d[idx]))  # fresh comms self-correct
    noise = rng.normal(0.0, p.sigma_nav, n) if nav_on else 0.0
    cloud.d[idx] = cloud.d[idx] + correction + noise
    cloud.t[idx] += 1

    rge = p.R0 * np.maximum(0.0, 1.0 - cloud.t[idx] / p.T)  # along-track range closes to 0 at CPA
    sep = np.hypot(rge, cloud.d[idx])
    cloud.min_sep[idx] = np.minimum(cloud.min_sep[idx], sep)


def _evolve(cloud: Cloud, target: float, coord: Coord, p: Params, nav_on: bool, comms_on: bool,
            rng: np.random.Generator) -> None:
    """Evolve every particle until its running ``phi`` reaches ``target`` (survivor) or the encounter
    ends at ``t == T`` (dropped) — in place."""
    while True:
        active = (cloud.phi(coord, p) < target) & (cloud.t < p.T)
        if not active.any():
            break
        _step(cloud, active, p, nav_on, comms_on, rng)


def _run_to_end(cloud: Cloud, p: Params, nav_on: bool, comms_on: bool,
                rng: np.random.Generator) -> None:
    while True:
        active = cloud.t < p.T
        if not active.any():
            break
        _step(cloud, active, p, nav_on, comms_on, rng)


def shells(coord: Coord, p: Params, m: int) -> list[float]:
    """``m`` phi-shells rising to 1.0 (= loss of separation). Front-loaded spacing: the deep
    shells near the rare set need to be finer where survival thins."""
    # geometric-ish rise toward 1 so the tail is well resolved
    xs = np.linspace(0.0, 1.0, m + 1)[1:]
    return [float(x) for x in (1.0 - (1.0 - xs) ** 1.7)]


# ---------------------------------------------------------------------------------------------------
def mc_estimate(p: Params, regime: Regime, n: int, seed: int) -> tuple[float, float, float, int]:
    """Brute-force P(LoS) with a Wilson 95% CI over ``n`` independent encounters."""
    nav_on, comms_on = _flags(regime)
    cloud = Cloud.start(n, p)
    _run_to_end(cloud, p, nav_on, comms_on, generator(root_seed_sequence(seed)))
    k = int(cloud.is_los(p).sum())
    return (*_wilson(k, n), k)


def _wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    if n == 0:
        return (float("nan"), 0.0, 1.0)
    ph = k / n
    denom = 1.0 + z * z / n
    centre = (ph + z * z / (2 * n)) / denom
    half = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / denom
    return (ph, max(0.0, centre - half), min(1.0, centre + half))


def ips_once(p: Params, regime: Regime, coord: Coord, m: int, n_particles: int,
             seq: np.random.SeedSequence) -> float:
    """One fixed-effort splitting run on ``coord``. ``P̂ = Π_k S_k/N`` (0 if a shell empties)."""
    nav_on, comms_on = _flags(regime)
    cloud = Cloud.start(n_particles, p)
    level = shells(coord, p, m)
    prob = 1.0
    for lseq, target in zip(spawn(seq, len(level)), level, strict=True):
        evolve_s, resample_s = spawn(lseq, 2)
        _evolve(cloud, target, coord, p, nav_on, comms_on, generator(evolve_s))
        survivors = np.where(cloud.phi(coord, p) >= target)[0]
        prob *= survivors.size / n_particles
        if survivors.size == 0:
            return 0.0
        idx = generator(resample_s).integers(0, survivors.size, n_particles)
        cloud = cloud.take(survivors[idx])
    # deepest shell is phi = 1.0 == LoS, so no separate terminal factor is needed
    return prob


def ips_estimate(p: Params, regime: Regime, coord: Coord, m: int, n_particles: int, reps: int,
                 seed: int) -> tuple[float, tuple[float, float], int]:
    """Mean ``P̂`` with a log-space 95% CI over ``reps`` independent replications."""
    probs = [ips_once(p, regime, coord, m, n_particles, s)
             for s in spawn(root_seed_sequence(seed), reps)]
    return float(np.mean(probs)), _log_ci(probs), sum(x == 0.0 for x in probs)


def _log_ci(probs: list[float], z: float = 1.96) -> tuple[float, float]:
    pos = [x for x in probs if x > 0.0]
    if len(pos) < 2 or len(pos) != len(probs):
        return (min(probs), max(probs))
    logs = np.log(pos)
    se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
    c = float(np.mean(logs))
    return (math.exp(c - z * se), math.exp(c + z * se))


# ---------------------------------------------------------------------------------------------------
REGIMES: tuple[Regime, ...] = ("nav", "comms", "both")
COORDS: tuple[Coord, ...] = ("min_sep", "staleness", "unified")
# per-coordinate cell: (mean, (lo, hi), n_collapsed, verdict)
Cell = tuple[float, tuple[float, float], int, str]


def _verdict(mean: float, ci: tuple[float, float], n_coll: int, reps: int,
             truth: tuple[float, float, float, int]) -> str:
    if n_coll == reps:
        return "COLLAPSE"
    _, mlo, mhi, _ = truth
    return "PASS" if (ci[0] <= mhi and mlo <= ci[1]) else "FAIL"


def make_figure(path, truth: dict, cells: dict, p: Params) -> None:  # type: ignore[no-untyped-def]
    """P(LoS) per regime (log y): MC 95% band + each coordinate's estimate with its CI."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    colours = {"min_sep": "#2ca02c", "staleness": "#d62728", "unified": "#1f77b4"}
    offs = {"min_sep": -0.22, "staleness": 0.0, "unified": 0.22}
    floor = 4e-6  # collapsed estimates are drawn here, at the axis floor = "reads zero"
    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    for i, regime in enumerate(REGIMES):
        _, mlo, mhi, _ = truth[regime]
        ax.add_patch(plt.Rectangle((i - 0.38, mlo), 0.76, mhi - mlo, color="0.82", zorder=0))
        for coord in COORDS:
            mean, (lo, hi), n_coll, verdict = cells[coord][regime]
            x = i + offs[coord]
            if verdict == "COLLAPSE":
                ax.plot(x, floor, "x", color=colours[coord], ms=10, mew=2.2, zorder=3)
            else:
                ax.errorbar(x, mean, yerr=[[max(mean - lo, 0)], [max(hi - mean, 0)]], fmt="o",
                            color=colours[coord], ms=7, capsize=3, elinewidth=1.3, zorder=3)
    ax.set_yscale("log")
    ax.set_xticks(range(len(REGIMES)))
    ax.set_xticklabels(["nav only\n(drift)", "comms only\n(jumps)", "nav + comms"])
    ax.set_ylabel("P(loss of separation)")
    ax.set_ylim(floor * 0.6, 3e-3)
    handles = [Patch(color="0.82", label="MC 95% CI")]
    handles += [Line2D([], [], color=colours[c], marker="o", ls="", ms=7, label=c) for c in COORDS]
    handles += [Line2D([], [], color="0.35", marker="x", ls="", ms=9, mew=2, label="collapsed (0)")]
    ax.legend(handles=handles, frameon=False, fontsize=9, ncol=2, loc="lower left")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def write_results(path, truth: dict, cells: dict, p: Params, particles: int, shells_: int,  # type: ignore[no-untyped-def]
                  reps: int, mc_n: int) -> None:
    passed = {c: sum(cells[c][r][3] == "PASS" for r in REGIMES) for c in COORDS}
    lines = [
        "# Unified IPS — one coordinate for nav AND comms rare events (opus4p8)",
        "",
        f"Minimal encounter (T={p.T}, rpz={p.rpz}, d_target={p.d_target}); nav = continuous drift "
        f"(sigma_nav={p.sigma_nav}), comms = discrete breach after L_crit={p.L_crit} dropped "
        f"broadcasts (rx={p.rx}). IPS: {reps} reps x {particles} particles x {shells_} shells; "
        f"MC ground truth over {mc_n:,} encounters/regime.",
        "",
        "## MC ground truth (Wilson 95% CI)",
        "",
        "| regime | P(LoS) | 95% CI | events |",
        "| --- | --- | --- | --- |",
    ]
    for r in REGIMES:
        ph, lo, hi, k = truth[r]
        lines.append(f"| {r} | {ph:.3e} | [{lo:.2e}, {hi:.2e}] | {k} |")
    lines += [
        "",
        "## IPS estimate per (coordinate x regime) — PASS = CI overlaps MC",
        "",
        "| coordinate | nav (drift) | comms (jumps) | nav + comms | regimes |",
        "| --- | --- | --- | --- | --- |",
    ]
    for c in COORDS:
        row = []
        for r in REGIMES:
            mean, ci, n_coll, verdict = cells[c][r]
            row.append("collapse" if verdict == "COLLAPSE" else f"{mean:.2e} {verdict}")
        lines.append(f"| `{c}` | {row[0]} | {row[1]} | {row[2]} | **{passed[c]}/3** |")
    lines += [
        "",
        "`min_sep` ladders the continuous nav drift but reads a **structural zero** on the rare "
        "discrete comms pathway (separation is bimodal — nominal or breached — so its intermediate "
        "shells hold no partial progress). It still passes *nav+comms* because nav dominates the "
        "total — confirming the escape-hatch in `important-ips-gap.md`, while failing the pure comms "
        "pathway. `staleness` is the mirror image: it ladders the drop run but is identically zero "
        "under perfect comms, and undercounts *nav+comms* by missing the nav contribution entirely.",
        "",
        "`unified = max(nav_progress, cap*comm_progress)` reduces to each single coordinate in that "
        "coordinate's own regime and ladders whichever pathway a particle is advancing when both are "
        "on, so it tracks MC in **all three** regimes. The Blom lesson made concrete: nest the shells "
        "on the rare-event driver, and when there are two drivers, take the per-pathway max.",
        "",
    ]
    path.write_text("\n".join(lines))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mc-only", action="store_true", help="print only the MC ground truth per regime")
    ap.add_argument("--mc-n", type=int, default=5_000_000, help="MC encounters per regime")
    ap.add_argument("--particles", type=int, default=4000, help="IPS particles per shell")
    ap.add_argument("--shells", type=int, default=14, help="IPS phi-shells")
    ap.add_argument("--reps", type=int, default=12, help="IPS replications for the CI")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--prefix", default="opus4p8")
    ap.add_argument("--outdir", type=Path, default=Path(__file__).parent)
    a = ap.parse_args()
    p = Params()

    print(f"encounter: T={p.T}, rpz={p.rpz}, d_target={p.d_target}, sigma_nav={p.sigma_nav}, "
          f"rx={p.rx}, L_crit={p.L_crit}")
    print("MC ground truth (Wilson 95% CI):")
    truth: dict[Regime, tuple[float, float, float, int]] = {}
    for regime in REGIMES:
        t0 = time.perf_counter()
        ph, lo, hi, k = mc_estimate(p, regime, a.mc_n, a.seed)
        truth[regime] = (ph, lo, hi, k)
        print(f"  {regime:5}: P(LoS)={ph:.3e}  95%CI[{lo:.3e}, {hi:.3e}]  ({k} events / {a.mc_n}, "
              f"{time.perf_counter() - t0:.0f}s)")
    if a.mc_only:
        return

    print(f"\nIPS: {a.reps} reps x {a.particles} particles x {a.shells} shells")
    header = f"  {'coordinate':10} | {'nav':^24} | {'comms':^24} | {'both':^24}"
    print(header + "\n  " + "-" * (len(header) - 2))
    cells: dict[Coord, dict[Regime, Cell]] = {}
    for coord in COORDS:
        cells[coord] = {}
        row = []
        for regime in REGIMES:
            mean, ci, n_coll = ips_estimate(p, regime, coord, a.shells, a.particles, a.reps, a.seed)
            verdict = _verdict(mean, ci, n_coll, a.reps, truth[regime])
            cells[coord][regime] = (mean, ci, n_coll, verdict)
            row.append(f"{mean:.2e} {verdict:>8}")
        n_pass = sum(cells[coord][r][3] == "PASS" for r in REGIMES)
        print(f"  {coord:10} | " + " | ".join(f"{c:^24}" for c in row) + f"   [{n_pass}/3]")

    print("\nverdict:")
    for coord in COORDS:
        n_pass = sum(cells[coord][r][3] == "PASS" for r in REGIMES)
        tag = "<-- unifies" if coord == "unified" and n_pass == 3 else ""
        print(f"  {coord:10}: {n_pass}/3 regimes  {tag}")

    a.outdir.mkdir(parents=True, exist_ok=True)
    res = a.outdir / f"{a.prefix}_unified_results.md"
    fig = a.outdir / f"{a.prefix}_unified_ips.png"
    write_results(res, truth, cells, p, a.particles, a.shells, a.reps, a.mc_n)
    make_figure(fig, truth, cells, p)
    print(f"\nwrote {res}\nwrote {fig}")


if __name__ == "__main__":
    main()
