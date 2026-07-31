"""What does the IPS estimator's *sampling distribution* look like at P(LoS) ~ 1e-4?

Every other IPS script reports one number with a confidence interval. This one asks the question
behind the interval: run the whole estimator ``--reps`` times over independent seed subtrees
(:func:`opencdarr.ips.replication_seeds`, ADR 0001) and look at the spread of the 100 estimates
themselves. The scenario is the [[ips-gate2-efficiency]] Case-2 rung — a fixed 90 deg crossing at
``pos_ci95 = 10`` m, lookahead 60 — where the truth is around 1e-4 and plain Monte Carlo at 30 000
encounters reads exactly zero.

**What theory says.** The fixed-effort estimator is a product of per-shell survival fractions,
``P_hat = prod_k S_k/N`` (``opencdarr/ips.py``). Two classical results bear on its distribution:

1. *Asymptotic normality in N.* The splitting/Feynman-Kac central limit theorem (Del Moral 2004;
   Cerou-Del Moral-Furon-Guyader 2012; Garvels 2000) gives ``sqrt(N) (P_hat/P - 1) -> Normal(0,
   sigma^2)`` with the idealised relative variance ``sigma^2 = sum_k (1 - p_k)/p_k``, so the
   relative standard deviation is ``sqrt(sigma^2 / N)`` — it shrinks as ``1/sqrt(N)``.
2. *Log-normality at finite N.* ``log P_hat = sum_k log(S_k/N)`` is a sum of m terms, so the CLT
   applies *in log space* over the shells, not just in N. That makes ``P_hat`` right-skewed and
   approximately log-normal — the reason ``opencdarr.ips._log_ci`` builds its interval on logs.

The two agree when ``N p_k`` is large (a tight log-normal is indistinguishable from a normal) and
diverge as N shrinks. So the script sweeps ``--particles``: the headline arm is the validated
N = 2000, and the smaller arms are where the skew, the ``1/sqrt(N)`` scaling and — at the bottom —
level collapse become visible. A collapsed replication returns ``P_hat = 0``, which has no log at
all; the distribution is then a zero-inflated mixture and *neither* limit law applies.

Two modes, so the analysis can be re-cut without paying for the sampling again:

    python scripts/ips_distribution.py run --reps 100 --particles 2000 500 200 --jobs 8
    python scripts/ips_distribution.py analyse --out scripts/ips_dist_<stamp>
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from ips_validate import Scenario  # the one scenario builder both IPS gates use
from scipy import stats

from opencdarr.ips import IPSResult, combine_replications, ips_once, replication_seeds

# The [[ips-gate2-efficiency]] Case-2 rung: fixed 90 deg crossing, GNSS pos_ci95 = 10 m, lookahead
# 60 s. Shell distances come from that note's measured min-sep percentiles (survival ~0.4/shell).
LEVELS: tuple[float, ...] = (150, 100, 75, 66, 61, 58, 56, 54, 52, 51, 50)
SCENARIO = Scenario(pos_ci95=10.0, vel_ci95=1.0, dpsi=90.0, lookahead=60.0)

# Mean survival per shell measured on this rung (8 reps x 2000 particles). The ``theory`` mode
# feeds these to the idealised model the splitting CLT is stated for, giving the reference shape
# the real replications are then judged against. Product = 6.8e-5, the rung's own P(LoS).
SURVIVAL_HINT: tuple[float, ...] = (
    0.476, 0.485, 0.404, 0.428, 0.386, 0.371, 0.420, 0.340, 0.299, 0.547, 0.509,
)


@dataclass(frozen=True)
class Arm:
    """One particle count's worth of replications: the raw estimates and their survival vectors."""

    n_particles: int
    probs: tuple[float, ...]  # one P_hat per replication (0.0 where a level collapsed)
    survival: tuple[tuple[float, ...], ...]  # per-replication S_k/N, truncated if it collapsed
    n_collapsed: int
    seconds: float

    def to_json(self) -> dict[str, Any]:
        return {
            "n_particles": self.n_particles,
            "probs": list(self.probs),
            "survival": [list(s) for s in self.survival],
            "n_collapsed": self.n_collapsed,
            "seconds": self.seconds,
        }

    @staticmethod
    def from_json(d: dict[str, Any]) -> Arm:
        return Arm(
            n_particles=int(d["n_particles"]),
            probs=tuple(float(p) for p in d["probs"]),
            survival=tuple(tuple(float(x) for x in s) for s in d["survival"]),
            n_collapsed=int(d["n_collapsed"]),
            seconds=float(d["seconds"]),
        )


# ----------------------------------------------------------------------------- sampling


def run_arm(n_particles: int, reps: int, seed: int, jobs: int) -> Arm:
    """``reps`` independent whole-estimator runs at one particle count.

    Parallelised over *replications* rather than through :mod:`opencdarr.parallel`'s lockstep
    scheduler: with ``reps >> jobs`` the replications already pack the machine evenly, and each one
    stays entirely inside its worker, so no particle cloud ever crosses a process boundary. The
    results are the same either way (ADR 0018) — this is only which schedule is cheaper here.
    """
    from joblib import Parallel, delayed

    seqs = replication_seeds(seed, reps)
    t0 = time.perf_counter()
    results: list[IPSResult] = Parallel(n_jobs=jobs, batch_size=1)(
        delayed(ips_once)(SCENARIO.build_initial, LEVELS, n_particles, s) for s in seqs
    )
    seconds = time.perf_counter() - t0
    return Arm(
        n_particles=n_particles,
        probs=tuple(r.prob for r in results),
        survival=tuple(r.survival for r in results),
        n_collapsed=sum(1 for r in results if r.collapsed_at is not None),
        seconds=seconds,
    )


# ----------------------------------------------------------------------------- statistics


@dataclass(frozen=True)
class ArmStats:
    """The empirical distribution of one arm's estimates, beside what the CLT predicts for it."""

    n_particles: int
    n_reps: int
    n_collapsed: int
    mean: float  # the point estimate a single 100-rep run would report
    median: float
    gmean: float  # exp(mean log P_hat) — the *typical* run, below the mean under log-normality
    cv: float  # observed relative standard deviation of P_hat
    cv_pred: float  # sqrt(sum_k (1-p_k)/p_k / N) — the idealised splitting CLT
    sigma_log: float  # sd of log P_hat
    skew: float  # sample skewness of P_hat
    skew_lognormal: float  # 3s + s^3 for s = sigma_log — what a log-normal would show
    shapiro_p: float  # normality of P_hat
    shapiro_log_p: float  # normality of log P_hat  (log-normality of P_hat)
    ad_log: float  # Anderson-Darling A^2 on log P_hat
    ad_crit5: float  # its 5% critical value
    seconds: float


def effective_particles(arm: Arm) -> list[float]:
    """Per shell, the number of *independent* particles the observed survival spread implies.

    Each ``S_k/N`` would have variance ``p_k(1-p_k)/N`` if the N particles were independent, which
    is the assumption behind ``sigma^2 = sum_k (1-p_k)/p_k``. Inverting the measured across-
    replication variance instead gives ``N_eff = p_k(1-p_k) / Var(S_k/N)`` — the count that would
    explain what actually happened. Shell 1 must come back at ``N_eff = N`` (a pinned geometry
    starts every particle identical and independent, so the first crossing really is binomial);
    anything below that at deeper shells is resampling correlation, i.e. the particle depletion
    Blom et al. name. Only replications that reached every shell are used, since a collapsed one
    has no survival fraction to contribute past the level it died on.
    """
    complete = np.array([s for s in arm.survival if len(s) == len(LEVELS)])
    if len(complete) < 2:
        return [float("nan")] * len(LEVELS)
    out: list[float] = []
    for k in range(len(LEVELS)):
        p = float(complete[:, k].mean())
        var = float(complete[:, k].var(ddof=1))
        out.append(p * (1.0 - p) / var if var > 0.0 else float("inf"))
    return out


def _relative_variance(survival: tuple[tuple[float, ...], ...], n_particles: int) -> float:
    """The idealised splitting relative variance ``sum_k (1-p_k)/p_k / N``, at the measured ``p_k``.

    ``p_k`` is the mean survival at shell k over the replications that reached it. This is the
    textbook variance for *independent* particles; real IPS particles share ancestors after
    resampling, so it is a lower bound on what the sampling actually shows.
    """
    depth = max(len(s) for s in survival)
    total = 0.0
    for k in range(depth):
        vals = [s[k] for s in survival if len(s) > k and s[k] > 0.0]
        if not vals:
            continue
        p = float(np.mean(vals))
        total += (1.0 - p) / p
    return total / n_particles


def analyse_arm(arm: Arm) -> ArmStats:
    """Empirical moments and normality tests for one arm, with the CLT prediction alongside.

    Collapsed replications (``P_hat = 0``) are kept in the mean — that is the estimate a user would
    actually report — but dropped from every log-space quantity, where they have no value at all.
    An arm so degenerate that fewer than three replications survive reports ``nan`` for the
    distributional tests rather than a number the sample cannot support.
    """
    probs = np.asarray(arm.probs, dtype=float)
    positive = probs[probs > 0.0]
    logs = np.log(positive) if positive.size else np.array([])
    enough = positive.size >= 3  # scipy's minimum for shapiro / anderson
    sigma_log = float(np.std(logs, ddof=1)) if positive.size >= 2 else float("nan")
    ad = stats.anderson(logs, dist="norm") if enough else None
    return ArmStats(
        n_particles=arm.n_particles,
        n_reps=len(probs),
        n_collapsed=arm.n_collapsed,
        mean=float(np.mean(probs)),
        median=float(np.median(probs)),
        gmean=float(np.exp(np.mean(logs))) if positive.size else float("nan"),
        cv=float(np.std(probs, ddof=1) / np.mean(probs)),
        cv_pred=math.sqrt(_relative_variance(arm.survival, arm.n_particles)),
        sigma_log=sigma_log,
        skew=float(stats.skew(probs, bias=False)),
        skew_lognormal=3.0 * sigma_log + sigma_log**3,
        shapiro_p=float(stats.shapiro(probs).pvalue) if len(probs) >= 3 else float("nan"),
        shapiro_log_p=float(stats.shapiro(logs).pvalue) if enough else float("nan"),
        ad_log=float(ad.statistic) if ad else float("nan"),
        ad_crit5=float(ad.critical_values[2]) if ad else float("nan"),  # 5% level
        seconds=arm.seconds,
    )


def report(arms: list[Arm]) -> list[ArmStats]:
    """Print the comparison table: what the 100 estimates did, and what the CLT said they would."""
    stats_list = [analyse_arm(a) for a in arms]

    print(f"\nscenario: fixed 90deg crossing, pos_ci95={SCENARIO.pos_ci95} "
          f"vel_ci95={SCENARIO.vel_ci95} rpz={SCENARIO.rpz} lookahead={SCENARIO.lookahead}, "
          f"{len(LEVELS)} shells")
    print(f"levels: {' '.join(f'{d:.0f}' for d in LEVELS)}")

    print("\n--- the estimates themselves ---")
    head = f"{'N':>6} {'reps':>5} {'coll':>5} {'mean':>10} {'median':>10} {'geo-mean':>10} " \
           f"{'min':>10} {'max':>10} {'max/min':>8} {'wall':>7}"
    print(head)
    for a, s in zip(arms, stats_list, strict=True):
        pos = [p for p in a.probs if p > 0.0]
        print(f"{s.n_particles:>6} {s.n_reps:>5} {s.n_collapsed:>5} {s.mean:>10.3e} "
              f"{s.median:>10.3e} {s.gmean:>10.3e} {min(pos):>10.3e} {max(pos):>10.3e} "
              f"{max(pos) / min(pos):>8.1f} {s.seconds:>6.0f}s")

    print("\n--- spread: observed vs the splitting CLT (relative sd) ---")
    print(f"{'N':>6} {'CV obs':>8} {'CV pred':>8} {'ratio':>7} {'sigma_log':>10} "
          f"{'skew obs':>9} {'skew LN':>8} {'med/mean':>9} {'exp(-s2/2)':>11}")
    for s in stats_list:
        print(f"{s.n_particles:>6} {s.cv:>8.3f} {s.cv_pred:>8.3f} {s.cv / s.cv_pred:>7.2f} "
              f"{s.sigma_log:>10.3f} {s.skew:>9.2f} {s.skew_lognormal:>8.2f} "
              f"{s.median / s.mean:>9.3f} {math.exp(-s.sigma_log**2 / 2):>11.3f}")

    print("\n--- distributional fit (p > 0.05 = the model is not rejected) ---")
    print(f"{'N':>6} {'Shapiro P':>11} {'Shapiro logP':>13} {'AD logP':>9} {'AD 5% crit':>11} "
          f"{'verdict':>28}")
    for s in stats_list:
        if s.n_collapsed:
            verdict = f"zero-inflated ({s.n_collapsed} collapsed)"
        elif s.shapiro_log_p > 0.05 and s.shapiro_p <= 0.05:
            verdict = "log-normal, normal rejected"
        elif s.shapiro_log_p > 0.05:
            verdict = "log-normal (normal also ok)"
        else:
            verdict = "log-normal rejected"
        print(f"{s.n_particles:>6} {s.shapiro_p:>11.4f} {s.shapiro_log_p:>13.4f} "
              f"{s.ad_log:>9.3f} {s.ad_crit5:>11.3f} {verdict:>28}")

    print("\n--- effective independent particles per shell (N_eff; = N would mean binomial) ---")
    print(f"{'N':>6} " + " ".join(f"{d:>6.0f}" for d in LEVELS))
    for a in arms:
        cells = " ".join(f"{v:>6.0f}" for v in effective_particles(a))
        print(f"{a.n_particles:>6} {cells}")
    return stats_list


# ----------------------------------------------------------------------------- figure


def plot(arms: list[Arm], stats_list: list[ArmStats], path: Path) -> None:
    """Histogram + normal QQ of ``log P_hat`` per arm, and the ``1/sqrt(N)`` scaling of the spread.

    Log-normality shows up twice: as a straight line in the QQ panel, and as the fitted curve
    tracking the histogram. The right-hand panel is the other prediction — that the spread falls as
    ``1/sqrt(N)`` — with the idealised CLT value beside the measured one.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n_arms = len(arms)
    fig, axes = plt.subplots(2, n_arms + 1, figsize=(4.0 * (n_arms + 1), 7.0))
    axes = np.atleast_2d(axes)

    for j, (arm, st) in enumerate(zip(arms, stats_list, strict=True)):
        positive = np.array([p for p in arm.probs if p > 0.0])
        if positive.size < 3:  # nothing to draw a distribution from
            for ax in (axes[0, j], axes[1, j]):
                ax.set_axis_off()
                ax.set_title(f"N = {st.n_particles}: {st.n_collapsed} collapsed")
            continue
        logs = np.log(positive)

        ax = axes[0, j]
        ax.hist(logs, bins=14, density=True, color="0.75", edgecolor="0.35", linewidth=0.6)
        grid = np.linspace(logs.min(), logs.max(), 200)
        ax.plot(grid, stats.norm.pdf(grid, logs.mean(), logs.std(ddof=1)), color="C3", lw=1.6,
                label="fitted normal")
        ax.axvline(math.log(st.mean), color="C0", lw=1.2, ls="--", label="mean $\\hat{P}$")
        ax.set_xlabel(r"$\log \hat{P}$")
        ax.set_ylabel("density" if j == 0 else "")
        ax.set_title(f"N = {st.n_particles}"
                     + (f", {st.n_collapsed} collapsed" if st.n_collapsed else ""))
        ax.legend(fontsize=7, frameon=False)

        ax = axes[1, j]
        stats.probplot(logs, dist="norm", plot=ax)
        ax.get_lines()[0].set(marker="o", ms=3.0, mfc="none", mec="C0", ls="none")
        ax.get_lines()[1].set(color="C3", lw=1.2)
        ax.set_title(f"Shapiro p = {st.shapiro_log_p:.3f}")
        ax.set_xlabel("normal quantile")
        ax.set_ylabel(r"$\log \hat{P}$" if j == 0 else "")

    ns = np.array([s.n_particles for s in stats_list], dtype=float)
    ax = axes[0, n_arms]
    ax.loglog(ns, [s.cv for s in stats_list], "o-", color="C0", ms=5, label="observed")
    ax.loglog(ns, [s.cv_pred for s in stats_list], "s--", color="C3", ms=5,
              label=r"CLT $\sqrt{\sigma^2/N}$")
    ax.set_xlabel("particles per shell $N$")
    ax.set_ylabel(r"relative sd of $\hat{P}$")
    ax.set_title("observed spread is ~10x the CLT\nand nearly flat in $N$", fontsize=10)
    ax.legend(fontsize=7, frameon=False)

    # why: the cloud loses independence as it descends, so N is not what the deep shells have
    ax = axes[1, n_arms]
    for arm, st in zip(arms, stats_list, strict=True):
        ax.semilogy(LEVELS, effective_particles(arm), "o-", ms=3.5, lw=1.3,
                    label=f"N = {st.n_particles}")
    ax.invert_xaxis()
    ax.set_xlabel("shell distance [m]")
    ax.set_ylabel(r"$N_{\mathrm{eff}}$ per shell")
    ax.set_title("particle depletion down the ladder", fontsize=10)
    ax.legend(fontsize=7, frameon=False)

    for ax in axes.ravel():
        ax.grid(False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"\nfigure -> {path}")


# ----------------------------------------------------------------------------- theory reference


def idealised_draws(p_k: Sequence[float], n_particles: int, draws: int, seed: int) -> np.ndarray:
    """Sample ``P_hat = prod_k Binomial(N, p_k)/N`` — the model the splitting CLT is stated for.

    Every shell's survivor count is an independent binomial, which is exactly the assumption behind
    ``sigma^2 = sum_k (1-p_k)/p_k``: N *independent* particles per level. Real IPS particles are not
    independent — resampling makes them share ancestors — so this is the reference shape, not a
    prediction of what the simulator does. Cheap enough (pure numpy) to draw 10^5 replicates, which
    is what makes the tail behaviour readable where 100 real replications only hint at it.
    """
    rng = np.random.default_rng(seed)
    counts = np.stack([rng.binomial(n_particles, p, size=draws) for p in p_k])
    return np.prod(counts / n_particles, axis=0)


def theory_figure(p_k: Sequence[float], counts: Sequence[int], draws: int, seed: int,
                  path: Path) -> None:
    """Mean, variance and shape of the estimator against the Gaussian with the same two moments.

    One column per particle count. The top row overlays a Gaussian and a log-normal on the sampled
    density; the middle row is the Gaussian QQ of ``P_hat``, the bottom row the Gaussian QQ of
    ``log P_hat``. The Gaussian is not wrong so much as *symmetric*: it is fitted to the right mean
    and variance and still misses, because the product estimator is skewed — and the miss is
    entirely on the low side, the side that matters for a safety claim.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    truth = float(np.prod(p_k))
    fig, axes = plt.subplots(3, len(counts), figsize=(4.6 * len(counts), 10.5), squeeze=False)

    for j, n_particles in enumerate(counts):
        p_hat = idealised_draws(p_k, n_particles, draws, seed)
        mean, sd = float(np.mean(p_hat)), float(np.std(p_hat, ddof=1))
        logs = np.log(p_hat[p_hat > 0.0])
        cv_pred = math.sqrt(sum((1.0 - p) / p for p in p_k) / n_particles)

        # everything on this rung sits in units of 1e-5; scaling the axis kills five leading
        # zeros per tick label and leaves the numbers readable
        unit = 1e-5
        ax = axes[0][j]
        lo, hi = mean - 4.0 * sd, mean + 4.5 * sd
        ax.hist(p_hat / unit, bins=160, density=True, color="0.8", edgecolor="none",
                label=f"{draws} draws")
        grid = np.linspace(min(lo, 0.0), hi, 500)
        ax.plot(grid / unit, stats.norm.pdf(grid, mean, sd) * unit, color="C3", lw=1.7,
                label="Gaussian (same mean, sd)")
        ax.plot(grid / unit,
                stats.lognorm.pdf(grid, s=logs.std(ddof=1), scale=math.exp(logs.mean())) * unit,
                color="C0", lw=1.7, label="log-normal")
        ax.axvline(mean / unit, color="C3", lw=1.0, ls="--", label="mean")
        ax.axvline(float(np.median(p_hat)) / unit, color="C0", lw=1.0, ls=":", label="median")
        ax.axvline(truth / unit, color="0.2", lw=1.0, label="true $P$")
        # only reach below zero when the Gaussian actually puts mass there — that overhang is the
        # point at small N, and dead space at large N
        ax.set_xlim((lo if lo > 0.0 else -0.2 * sd) / unit, hi / unit)
        ax.set_xlabel(r"$\hat{P}\ [\times 10^{-5}]$")
        ax.set_ylabel("density" if j == 0 else "")
        ax.set_title(f"N = {n_particles}\nmean {mean / unit:.2f}, sd {sd / unit:.2f} "
                     f"($\\times 10^{{-5}}$),  CV {sd / mean:.3f} (CLT {cv_pred:.3f})",
                     fontsize=10)
        ax.legend(fontsize=7, frameon=False, loc="upper right")

        # a thinned sample keeps the QQ panels legible and the file small; the shape is the point
        sub = p_hat[:: max(1, draws // 4000)]
        for row, (data, label) in enumerate(
            ((sub, r"$\hat{P}$"), (np.log(sub[sub > 0.0]), r"$\log \hat{P}$")), start=1
        ):
            ax = axes[row][j]
            stats.probplot(data, dist="norm", plot=ax)
            ax.get_lines()[0].set(marker="o", ms=2.2, mfc="none", mec="C0", ls="none")
            ax.get_lines()[1].set(color="C3", lw=1.3)
            r2 = stats.pearsonr(*stats.probplot(data, dist="norm", fit=False)).statistic ** 2
            ax.set_title(f"{label} vs Gaussian   $R^2$ = {r2:.4f}", fontsize=10)
            ax.set_xlabel("Gaussian quantile")
            ax.set_ylabel(label if j == 0 else "")

    for ax in np.ravel(axes):
        ax.grid(False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"figure -> {path}")


def shells_for(target: float, survival: float) -> tuple[float, ...]:
    """Per-shell survivals whose product is exactly ``target``, as close to ``survival`` as an
    integer shell count allows.

    Rarity does not enter the estimator's shape directly — the shell *count* does. Holding the
    per-shell survival near a working value and letting the ladder lengthen is how a splitting
    design actually reaches a smaller probability, so it is also how the shape should be swept.
    """
    m = max(1, round(math.log(target) / math.log(survival)))
    return (target ** (1.0 / m),) * m


def rarity_figure(targets: Sequence[float], counts: Sequence[int], survival: float,
                  draws: int, seed: int, path: Path) -> None:
    """How the estimator's spread and shape move as the target probability shrinks.

    The headline is that ``sigma^2 = sum_k (1-p_k)/p_k`` has one term per shell and the ladder
    grows as ``log P / log p``, so the relative spread of splitting rises only as
    ``sqrt(|log P| / N)`` — while plain Monte Carlo's rises as ``1/sqrt(n P)``. Logarithmic against
    exponential: that gap *is* the method. The cost is that the same lengthening ladder makes the
    estimator steadily more skewed, so the log-normal reading matters more the rarer the event.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.6))
    show = [t for t in (1e-3, 1e-5, 1e-7) if t in set(targets)] or list(targets[:3])

    # (a) spread against rarity, splitting vs plain MC at a matched segment budget
    ax = axes[0][0]
    for n_particles in counts:
        cv, mc = [], []
        for t in targets:
            p_k = shells_for(t, survival)
            cv.append(math.sqrt(sum((1.0 - p) / p for p in p_k) / n_particles))
            budget = n_particles * len(p_k)  # what the ladder costs, spent on MC encounters
            mc.append(math.sqrt((1.0 - t) / (budget * t)))
        ax.loglog(targets, cv, "o-", ms=4, label=f"IPS, N = {n_particles}")
        ax.loglog(targets, mc, ":", lw=1.2, color="0.5",
                  label="plain MC, matched budget" if n_particles == counts[0] else None)
    ax.axvspan(1e-5, 1e-4, color="C1", alpha=0.12, lw=0)
    ax.invert_xaxis()
    ax.set_xlabel("true $P$")
    ax.set_ylabel(r"relative sd of $\hat{P}$")
    ax.set_title(r"spread grows as $\sqrt{|\log P|}$, not $1/\sqrt{P}$")
    ax.legend(fontsize=7, frameon=False)

    # (b) the shape itself, rescaled by the truth so the three rarities are comparable
    ax = axes[0][1]
    n_ref = counts[len(counts) // 2]
    ratios: dict[float, np.ndarray] = {}
    for t in show:
        p_k = shells_for(t, survival)
        ratios[t] = idealised_draws(p_k, n_ref, draws, seed) / t
        ax.hist(ratios[t], bins=200, density=True, histtype="step", lw=1.4,
                label=f"$P$ = {t:.0e} ({len(p_k)} shells)")
    ax.axvline(1.0, color="0.2", lw=1.0)
    ax.set_xlim(0.0, 3.0)
    ax.set_xlabel(r"$\hat{P} / P$")
    ax.set_ylabel("density")
    ax.set_title(f"rarer = longer ladder = more skew (N = {n_ref})")
    ax.legend(fontsize=7, frameon=False)

    # (c) log-normality holds throughout — the QQ stays straight as the ladder lengthens
    ax = axes[1][0]
    for t in show:
        sub = ratios[t][:: max(1, draws // 3000)]
        q, v = stats.probplot(np.log(sub[sub > 0.0]), dist="norm", fit=False)
        ax.plot(q, v, ".", ms=2.6, label=f"$P$ = {t:.0e}")
    ax.set_xlabel("Gaussian quantile")
    ax.set_ylabel(r"$\log(\hat{P}/P)$")
    ax.set_title("log-normal at every rarity")
    ax.legend(fontsize=7, frameon=False)

    # (d) what a single run actually pins down: the central 90% of P_hat/P
    ax = axes[1][1]
    for i, n_particles in enumerate(counts):
        lo, hi = [], []
        for t in targets:
            d = idealised_draws(shells_for(t, survival), n_particles, draws // 4, seed) / t
            a, b = np.percentile(d, [5.0, 95.0])
            lo.append(float(a))
            hi.append(float(b))
        ax.fill_between(targets, lo, hi, color=f"C{i}", alpha=0.18, lw=0)
        ax.plot(targets, hi, "-", color=f"C{i}", lw=1.4, label=f"N = {n_particles}")
        ax.plot(targets, lo, "-", color=f"C{i}", lw=1.4)
    ax.axhline(1.0, color="0.2", lw=1.0)
    ax.set_xscale("log")
    ax.invert_xaxis()
    ax.set_xlabel("true $P$")
    ax.set_ylabel(r"$\hat{P} / P$")
    ax.set_title("central 90% of a single run's answer")
    ax.legend(fontsize=7, frameon=False)

    for ax in np.ravel(axes):
        ax.grid(False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    print(f"figure -> {path}")


# ----------------------------------------------------------------------------- cli


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    r = sub.add_parser("run", help="sample the estimator and write the raw estimates")
    r.add_argument("--reps", type=int, default=100, help="independent whole-estimator runs per arm")
    r.add_argument("--particles", type=int, nargs="+", default=[2000, 500, 200],
                   help="particle counts to sample; the first is the headline arm")
    r.add_argument("--seed", type=int, default=20260730)
    r.add_argument("--jobs", type=int, default=8)
    r.add_argument("--out", type=Path, default=None, help="output directory (default: timestamped)")

    a = sub.add_parser("analyse", help="re-cut an existing run's estimates")
    a.add_argument("--out", type=Path, required=True, help="a directory written by `run`")

    t = sub.add_parser("theory", help="the idealised reference shape, no simulation")
    t.add_argument("--particles", type=int, nargs="+", default=[2000, 200])
    t.add_argument("--draws", type=int, default=200_000)
    t.add_argument("--seed", type=int, default=20260730)
    t.add_argument("--out", type=Path, required=True, help="output directory")

    y = sub.add_parser("rarity", help="how the shape moves as the target probability shrinks")
    y.add_argument("--targets", type=float, nargs="+",
                   default=[1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8])
    y.add_argument("--particles", type=int, nargs="+", default=[500, 2000, 10000])
    y.add_argument("--survival", type=float, default=0.4, help="per-shell survival to hold")
    y.add_argument("--draws", type=int, default=200_000)
    y.add_argument("--seed", type=int, default=20260730)
    y.add_argument("--out", type=Path, required=True, help="output directory")

    args = p.parse_args()

    if args.mode == "rarity":
        args.out.mkdir(parents=True, exist_ok=True)
        rarity_figure(args.targets, args.particles, args.survival, args.draws, args.seed,
                      args.out / "ips_rarity_shape.png")
        return

    if args.mode == "theory":
        args.out.mkdir(parents=True, exist_ok=True)
        theory_figure(SURVIVAL_HINT, args.particles, args.draws, args.seed,
                      args.out / "ips_theory_shape.png")
        return

    if args.mode == "analyse":
        out = args.out
        payload = json.loads((out / "estimates.json").read_text())
        arms = [Arm.from_json(d) for d in payload["arms"]]
    else:
        out = args.out or Path("scripts") / f"ips_dist_{time.strftime('%Y%m%d_%H%M%S')}"
        out.mkdir(parents=True, exist_ok=True)
        arms = []
        for n_particles in args.particles:
            budget = args.reps * n_particles * len(LEVELS)
            print(f"[{time.strftime('%H:%M:%S')}] N={n_particles}: {args.reps} reps x "
                  f"{n_particles}p x {len(LEVELS)} shells (~{budget} seg) on {args.jobs} jobs")
            arm = run_arm(n_particles, args.reps, args.seed, args.jobs)
            est = combine_replications(
                [IPSResult(prob=p_, levels=LEVELS, survival=s, n_particles=n_particles,
                           collapsed_at=None if p_ > 0 else 0)
                 for p_, s in zip(arm.probs, arm.survival, strict=True)]
            )
            print(f"           P={est.prob:.3e}  95%CI[{est.ci[0]:.3e}, {est.ci[1]:.3e}]  "
                  f"collapsed={arm.n_collapsed}/{args.reps}  ({arm.seconds:.0f}s)")
            arms.append(arm)
            # written after every arm, so an interrupted sweep still leaves usable data
            (out / "estimates.json").write_text(json.dumps(
                {"reps": args.reps, "seed": args.seed, "levels": list(LEVELS),
                 "scenario": {"pos_ci95": SCENARIO.pos_ci95, "vel_ci95": SCENARIO.vel_ci95,
                              "dpsi": SCENARIO.dpsi, "lookahead": SCENARIO.lookahead,
                              "tlos": SCENARIO.tlos, "rpz": SCENARIO.rpz, "dt": SCENARIO.dt},
                 "arms": [x.to_json() for x in arms]}, indent=2))

    stats_list = report(arms)
    plot(arms, stats_list, out / "ips_distribution.png")


if __name__ == "__main__":
    main()
