"""Blom–Ma–Bakker (2018) hypothetical-car GSHS example, reproduced with a fixed-effort IPS.

Reference: H.A.P. Blom, H. Ma, G.J. Bakker, *Interacting Particle System-based Estimation of Reach
Probability for a Generalized Stochastic Hybrid System*, IFAC PapersOnLine 51-16 (2018) 79–84
(``vault/papers/rare-event-sim/interacting-particle-system.pdf``), Section 5.

**The scenario (Fig. 1 / Section 5.1).** A car drives at constant ``v0`` through dense fog toward a
wall. At distance ``d_fog`` from the wall the driver sees it for the first time and enters a reaction
*delay* mode; the delay is exponential with mean ``mu``. During the delay the car keeps its speed;
when the delay ends it decelerates at constant ``a_min`` (mode ``-1``). If it reaches the wall — while
still reacting, or before braking has stopped it — that is a *hit*. There is no Brownian motion, so
the whole trajectory is deterministic **once the reaction time is fixed**: the car hits the wall iff
the reaction delay exceeds ``s* = (d_fog - v0^2/2|a_min|) / v0``. Hence the exact answer is

    p_hit(mu) = P(delay > s*) = exp(-s* / mu),   s* = (5400 - 1800)/60 = 60 s

matching the paper's Table 4 (2.48e-3 at mu=10 down to 9.36e-14 at mu=2).

**What the paper compares — and this module reproduces.** The reach set is nested in ``m`` shells and
IPS resamples survivors at each shell (fixed effort ``P̂ = Π S_k/N``). Two independent choices decide
whether it works:

1. *How the Poisson reaction is sampled* (the paper's headline, Tables 5 vs 6).
   - **Exponential sampling** draws the whole reaction time up front. With no Brownian motion the
     future is then deterministic, so a cloned survivor and all its copies share one already-fixed
     reaction time and never diverge — IPS cannot manufacture a delay longer than the largest one
     drawn at initialisation, and for ``mu <= 5`` it simply *collapses*.
   - **Bernoulli sampling** tosses a per-step coin (fire with prob ``1 - e^{-dt/mu}``), so a cloned
     survivor re-rolls its future step by step. Clones diverge, splitting bites.

2. *What the shells are nested on* (the importance function).
   - **Position shells** ``d_k = (k/m) d_fog`` — the paper's ``D_k`` (Section 5.3). Equidistant in
     car position. The catch: a braked car still coasts ~1800 m, so the first few shells select
     nobody, and the "still-reacting" sub-population — the sole carrier of the rare event — is spent
     before selection begins. Bernoulli reaches ``mu=5`` this way but starves in the deep tail
     (the paper's own Table 6 reports a *larger-than-the-estimate* error at ``mu=2``).
   - **Delay-progress shells** ``ell_k = (k/m)(d_fog - brake)`` — nested on how far the car coasts
     while still reacting (the rare-event driver itself). Uniform per-shell survival ``e^{-s*/(m*mu)}``,
     so Bernoulli reaches the full ``mu=2`` tail (~1e-13) accurately. Exponential still collapses —
     which cleanly isolates the *sampling* scheme, holding the importance function fixed.

    python blom-comparison/car_ips.py                          # full sweep, prefix opus4p8
    python blom-comparison/car_ips.py --particles 2000 --reps 8   # quicker
"""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402

# --- Physical constants of the Blom car example (paper Section 5.1) ---------------------------------
D_FOG = 5400.0  # [m] distance wall - fog boundary: the driver sees the wall from here
V0 = 60.0  # [m/s] cruise speed (216 km/h)
A_MIN = 1.0  # [m/s^2] deceleration magnitude once braking starts
BRAKE_DIST = V0 * V0 / (2.0 * A_MIN)  # 1800 m: distance to stop from V0
S_STAR = (D_FOG - BRAKE_DIST) / V0  # 60 s: reaction delay above which the car hits the wall
POINT_OF_NO_RETURN = D_FOG - BRAKE_DIST  # 3600 m: coast this far still reacting and the wall is hit
MU_TABLE = (10.0, 5.0, 10.0 / 3.0, 2.5, 2.0)  # paper Table 4 (its "3.33" row is exactly 10/3)

# discrete GSHS modes, collapsed to what matters for the reach event
MODE_DELAY, MODE_BRAKE, MODE_STOP = np.int8(0), np.int8(1), np.int8(2)

LevelVar = Literal["position", "delay"]


def p_hit_exact(mu: float) -> float:
    """The analytic hit probability ``exp(-s*/mu)`` (paper Table 4)."""
    return math.exp(-S_STAR / mu)


def levels_for(kind: LevelVar, m: int) -> list[float]:
    """The ``m`` shells for one importance function.

    ``position``: equidistant car-position shells to the wall (the paper's ``D_k``, Section 5.3).
    ``delay``: equidistant coast-while-reacting shells to the point of no return — crossing the last
    one *is* a hit, so both variants nest the identical rare event, only measured differently.
    """
    top = D_FOG if kind == "position" else POINT_OF_NO_RETURN
    return [(k / m) * top for k in range(1, m + 1)]


@dataclass
class Population:
    """A cloud of ``N`` cars as parallel arrays — one fixed-effort IPS shell operates on all at once.

    ``y`` is position past the fog boundary, ``v`` the speed, ``mode`` the discrete GSHS mode.
    ``fire_pos`` is the position at which the reaction fires: for exponential sampling it is *pre-drawn*
    at initialisation (``v0 * delay``); for Bernoulli sampling it starts at ``+inf`` and is filled in
    with the realised position the step the coin comes up. Cloning a survivor copies ``fire_pos`` too
    — which is exactly why an exponential clone (whose value is already set) cannot diverge from its
    parent, while a Bernoulli clone (still ``+inf``) re-rolls its own.
    """

    y: np.ndarray
    v: np.ndarray
    mode: np.ndarray
    fire_pos: np.ndarray

    def take(self, idx: np.ndarray) -> Population:
        """Resample-with-replacement: the new cloud carries copies of the selected particles."""
        return Population(self.y[idx].copy(), self.v[idx].copy(),
                          self.mode[idx].copy(), self.fire_pos[idx].copy())

    def progress(self, kind: LevelVar) -> np.ndarray:
        """The monotone importance coordinate the shells are crossed on. ``position`` is just ``y``;
        ``delay`` is how far the car coasted while still reacting — its live position until it fires,
        frozen at ``fire_pos`` afterward (``min(y, fire_pos)`` does both)."""
        return self.y if kind == "position" else np.minimum(self.y, self.fire_pos)


def _evolve_to(pop: Population, target: float, mu: float, dt: float, bernoulli: bool,
               kind: LevelVar, rng: np.random.Generator) -> Population:
    """Advance every particle until its importance coordinate reaches ``target`` (a survivor) or it
    can no longer get there (dropped). Constant-speed motion while reacting; constant deceleration once
    braking. Bernoulli firing draws a fresh per-step coin (so clones diverge); exponential firing
    triggers at the pre-drawn ``fire_pos`` (so clones stay identical). For ``delay`` shells a car that
    fires below ``target`` is finished — its coast distance is frozen short — so braking need not be
    simulated; for ``position`` shells the braking roll-out decides whether it reaches the shell."""
    y, v, mode, fire = pop.y.copy(), pop.v.copy(), pop.mode.copy(), pop.fire_pos.copy()
    p_fire = -math.expm1(-dt / mu)  # 1 - exp(-dt/mu): P(reaction fires this step) for Bernoulli

    def _active() -> np.ndarray:
        if kind == "position":
            return (y < target) & (mode != MODE_STOP)
        return (mode == MODE_DELAY) & (y < target)  # only a still-reacting car can still advance phi

    while True:
        active = _active()
        if not active.any():
            break

        # reacting cars: coast forward, then possibly start braking (recording where they fired)
        react = np.where(active & (mode == MODE_DELAY))[0]
        if react.size:
            y[react] += V0 * dt
            fires = (rng.random(react.size) < p_fire) if bernoulli else (y[react] >= fire[react])
            hit = react[fires]
            fire[hit] = y[hit]  # realised coast-while-reacting distance
            mode[hit] = MODE_BRAKE

        if kind == "position":
            # braking cars: decelerate; if the step would cross v=0, place the exact stop point
            brake = np.where(active & (mode == MODE_BRAKE))[0]
            if brake.size:
                vb = v[brake]
                v_next = vb - A_MIN * dt
                stops = v_next <= 0.0
                si, ci = brake[stops], brake[~stops]
                y[si] += vb[stops] ** 2 / (2.0 * A_MIN)  # remaining distance to a full stop
                v[si] = 0.0
                mode[si] = MODE_STOP
                y[ci] += vb[~stops] * dt - 0.5 * A_MIN * dt * dt
                v[ci] = v_next[~stops]

    return Population(y, v, mode, fire)


@dataclass(frozen=True)
class IPSResult:
    """One replication: ``P̂ = Π_k S_k/N`` (0 if some shell emptied) and its per-shell survival."""

    prob: float
    survival: tuple[float, ...]
    collapsed_at: int | None  # index of the first empty shell, or None


def ips_once(kind: LevelVar, levels: Sequence[float], n_particles: int, mu: float, dt: float,
             bernoulli: bool, seq: np.random.SeedSequence) -> IPSResult:
    """One fixed-effort multilevel-splitting run for the car example.

    Mirrors :func:`opencdarr.ips.ips_once`: an initial cloud at the fog boundary, then per shell
    evolve → keep survivors → resample with replacement back to ``N``. Everything before the fog
    boundary is deterministic, so all particles start identically at ``(delay, y=0, v=v0)``; the
    reaction delay is either pre-drawn (exponential) or rolled per step (Bernoulli).
    """
    init_seq, evolve_seq = seq.spawn(2)
    y = np.zeros(n_particles)
    v = np.full(n_particles, V0)
    mode = np.full(n_particles, MODE_DELAY)
    if bernoulli:
        fire = np.full(n_particles, np.inf)  # firing decided per step, never pre-committed
    else:
        fire = V0 * generator(init_seq).exponential(mu, n_particles)  # fixed reaction position
    pop = Population(y, v, mode, fire)

    survival: list[float] = []
    for k, (lseq, target) in enumerate(zip(spawn(evolve_seq, len(levels)), levels, strict=True)):
        evolve_s, resample_s = spawn(lseq, 2)
        pop = _evolve_to(pop, target, mu, dt, bernoulli, kind, generator(evolve_s))
        survivors = np.where(pop.progress(kind) >= target)[0]
        survival.append(survivors.size / n_particles)
        if survivors.size == 0:
            return IPSResult(0.0, tuple(survival), collapsed_at=k)
        idx = generator(resample_s).integers(0, survivors.size, n_particles)
        pop = pop.take(survivors[idx])

    return IPSResult(float(np.prod(survival)), tuple(survival), collapsed_at=None)


@dataclass(frozen=True)
class Estimate:
    """A replicated estimate: mean ``P̂`` with a log-space 95% CI, plus how many reps collapsed."""

    prob: float
    ci: tuple[float, float]
    n_collapsed: int
    n_reps: int


def _log_ci(probs: list[float], z: float = 1.96) -> tuple[float, float]:
    """95% CI for the mean rare probability: log-space when every replication is positive (the
    product estimator is right-skewed), otherwise the raw min/max span (a collapsed rep has no log)."""
    positive = [p for p in probs if p > 0.0]
    if len(positive) < 2 or len(positive) != len(probs):
        return (min(probs), max(probs))
    logs = np.log(positive)
    se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
    centre = float(np.mean(logs))
    return (math.exp(centre - z * se), math.exp(centre + z * se))


def estimate(kind: LevelVar, levels: Sequence[float], n_particles: int, mu: float, dt: float,
             bernoulli: bool, reps: int, seed: int) -> Estimate:
    """Run ``reps`` independent IPS replications and combine (mean ``P̂`` + log-space CI)."""
    probs = [ips_once(kind, levels, n_particles, mu, dt, bernoulli, s).prob
             for s in spawn(root_seed_sequence(seed), reps)]
    return Estimate(prob=float(np.mean(probs)), ci=_log_ci(probs),
                    n_collapsed=sum(p == 0.0 for p in probs), n_reps=reps)


def crude_mc(mu: float, n: int, seed: int) -> tuple[float, int]:
    """A brute-force cross-check for the tractable regime: sample delays, count wall hits.
    Returns ``(p_hat, n_hits)``; useless once ``p_hit`` drops below ~``1/n`` (reads 0), which is the
    whole reason IPS is needed."""
    delays = generator(root_seed_sequence(seed)).exponential(mu, n)
    hits = int(np.sum(delays > S_STAR))
    return hits / n, hits


# --- reporting -------------------------------------------------------------------------------------
def _fmt(est: Estimate) -> str:
    if est.n_collapsed == est.n_reps:
        return "collapsed"
    tag = f"  [{est.n_collapsed}/{est.n_reps} reps collapsed]" if est.n_collapsed else ""
    return f"{est.prob:.3e}  95%CI[{est.ci[0]:.2e}, {est.ci[1]:.2e}]{tag}"


# one estimator variant = (importance function, sampling scheme)
VARIANTS: tuple[tuple[str, LevelVar, bool], ...] = (
    ("pos_exp", "position", False),
    ("pos_ber", "position", True),
    ("delay_exp", "delay", False),
    ("delay_ber", "delay", True),
)


def run_sweep(mus: Sequence[float], n_particles: int, m: int, dt: float, reps: int,
              seed: int) -> dict[float, dict[str, object]]:
    """For every ``mu``: analytic ``p_hit`` plus all four (importance function x sampling) variants."""
    out: dict[float, dict[str, object]] = {}
    for mu in mus:
        row: dict[str, object] = {"exact": p_hit_exact(mu)}
        for name, kind, bernoulli in VARIANTS:
            row[name] = estimate(kind, levels_for(kind, m), n_particles, mu, dt, bernoulli, reps, seed)
        out[mu] = row
        print(f"  mu={mu:<6.3g} exact={row['exact']:.2e}  "  # type: ignore[str-format]
              f"pos:  exp={_fmt(row['pos_exp'])!s:<34} ber={_fmt(row['pos_ber'])}")  # type: ignore[arg-type]
        print(f"  {'':13} delay:exp={_fmt(row['delay_exp'])!s:<34} "  # type: ignore[arg-type]
              f"ber={_fmt(row['delay_ber'])}")
    return out


def _table(sweep: dict[float, dict[str, object]], kind_keys: tuple[str, str], title: str) -> list[str]:
    exp_key, ber_key = kind_keys
    lines = [f"### {title}", "",
             "| mu (s) | exact p_hit | exponential sampling | Bernoulli sampling |",
             "| --- | --- | --- | --- |"]
    for mu, r in sweep.items():
        lines.append(f"| {mu:.3g} | {r['exact']:.3e} | {_fmt(r[exp_key])} | {_fmt(r[ber_key])} |")  # type: ignore[arg-type]
    return lines + [""]


def write_table(path: Path, sweep: dict[float, dict[str, object]], n_particles: int, m: int,
                dt: float, reps: int) -> None:
    """Markdown reproducing paper Tables 4/5/6, for both importance functions."""
    lines = [
        "# Blom car example — IPS reproduction (opus4p8)",
        "",
        f"Fixed-effort multilevel splitting, N_p={n_particles}, m={m} shells, dt={dt} s, "
        f"{reps} replications. Exact p_hit = exp(-60/mu) (paper Table 4).",
        "",
        "*Exponential sampling* draws the reaction time in one shot; with no Brownian motion clones "
        "then never diverge, so it collapses for mu <= 5 (paper Table 5). *Bernoulli sampling* rolls "
        "the reaction per step so clones diverge (paper Table 6).",
        "",
    ]
    lines += _table(sweep, ("pos_exp", "pos_ber"),
                    "Paper's equidistant position shells (D_k, Section 5.3)")
    lines += [
        "Position shells reproduce Tables 5/6: exponential collapses for mu <= 5; Bernoulli reaches "
        "mu = 5 but starves in the deep tail, because a braked car coasts ~1800 m so the first shells "
        "select nobody and the still-reacting sub-population is spent before selection begins. (The "
        "paper's own Table 6 shows a larger-than-the-estimate error at mu = 2, the same edge.)",
        "",
    ]
    lines += _table(sweep, ("delay_exp", "delay_ber"),
                    "Delay-progress shells (nested on the rare-event driver)")
    lines += [
        "Nesting the shells on how far the car coasts while still reacting gives uniform per-shell "
        "survival, so Bernoulli reaches the full mu = 2 tail (~1e-13) accurately. Exponential still "
        "collapses under the identical shells — isolating the *sampling* scheme as the paper's point, "
        "while the importance function decides how deep Bernoulli can go.",
        "",
    ]
    path.write_text("\n".join(lines))


def make_figure(path: Path, sweep: dict[float, dict[str, object]], reps: int) -> None:
    """p_hit vs mu (log y): exact curve; Bernoulli (both shell choices) and exponential estimates."""
    mus = sorted(sweep)
    grid = np.linspace(min(mus) * 0.95, max(mus) * 1.05, 200)

    fig, ax = plt.subplots(figsize=(7.4, 5.2))
    ax.plot(grid, [p_hit_exact(m) for m in grid], color="0.35", lw=1.6, label="exact  exp(-60/μ)")

    def _scatter(key: str, color: str, marker: str, label: str, dx: float) -> None:
        xs, ys, lo, hi = [], [], [], []
        for mu in mus:
            est: Estimate = sweep[mu][key]  # type: ignore[assignment]
            if est.n_collapsed == est.n_reps:
                continue  # fully collapsed: nothing to plot
            xs.append(mu + dx)
            ys.append(est.prob)
            lo.append(max(est.prob - est.ci[0], 0.0))
            hi.append(max(est.ci[1] - est.prob, 0.0))
        ax.errorbar(xs, ys, yerr=[lo, hi], fmt=marker, color=color, ms=7, capsize=3,
                    lw=0, elinewidth=1.2, label=label)

    _scatter("delay_ber", "#1f77b4", "o", "Bernoulli, delay-progress shells", dx=0.0)
    _scatter("pos_ber", "#2ca02c", "^", "Bernoulli, position shells (paper)", dx=0.05)
    _scatter("delay_exp", "#d62728", "s", "exponential (either shell)", dx=-0.05)

    ax.set_yscale("log")
    ax.set_xlabel("mean reaction delay  μ  [s]")
    ax.set_ylabel("hit probability  $p_{hit}$")
    ax.set_ylim(1e-14, 1e-2)
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--prefix", default="opus4p8", help="output filename prefix")
    p.add_argument("--outdir", type=Path, default=Path(__file__).parent, help="output directory")
    p.add_argument("--particles", type=int, default=10000, help="N_p particles per shell (paper: 1e4)")
    p.add_argument("--levels", type=int, default=10, help="m equidistant shells (paper: 10)")
    p.add_argument("--dt", type=float, default=0.01, help="Bernoulli / integration step [s]")
    p.add_argument("--reps", type=int, default=20, help="independent IPS replications for the CI")
    p.add_argument("--mu", type=float, nargs="+", default=list(MU_TABLE),
                   help="mean reaction delays [s] (default: paper Table 4)")
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()

    print(f"Blom car GSHS: d_fog={D_FOG:g} m, v0={V0:g} m/s, a_min=-{A_MIN:g} m/s^2, "
          f"hit iff reaction delay > s*={S_STAR:g} s (coast past {POINT_OF_NO_RETURN:g} m)")
    print(f"IPS: N_p={a.particles}, m={a.levels} shells, dt={a.dt}, reps={a.reps}\n")

    mc_p, mc_hits = crude_mc(MU_TABLE[0], a.particles, a.seed)
    print(f"crude MC cross-check (mu=10, n={a.particles}): p_hit={mc_p:.3e} "
          f"({mc_hits} hits; exact {p_hit_exact(10.0):.3e})\n")

    t0 = time.perf_counter()
    sweep = run_sweep(a.mu, a.particles, a.levels, a.dt, a.reps, a.seed)
    print(f"\nswept {len(a.mu)} values of mu in {time.perf_counter() - t0:.0f}s")

    a.outdir.mkdir(parents=True, exist_ok=True)
    table_path = a.outdir / f"{a.prefix}_results.md"
    fig_path = a.outdir / f"{a.prefix}_car_ips.png"
    write_table(table_path, sweep, a.particles, a.levels, a.dt, a.reps)
    make_figure(fig_path, sweep, a.reps)
    print(f"\nwrote {table_path}\nwrote {fig_path}")


if __name__ == "__main__":
    main()
