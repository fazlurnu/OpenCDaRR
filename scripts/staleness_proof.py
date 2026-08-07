"""PROOF (important-ips-gap): fixed-effort splitting on staleness recovers (1-rx)^W exactly.

Validates the rare-event *estimator machinery* against a closed form, isolated from the resolver
physics. The rare event is a **communication blackout**: a single directed link drops every one of
``W`` broadcasts in a row. Its probability is exactly ``P = (1 - rx)^W`` — no simulation needed to
know the answer — so it is the perfect check that our fixed-effort multilevel splitting (the logic
of ``opencdarr.estimate.ips.ips_once``) returns the right number with real variance reduction.

Why this and not P(LoS): at perfect nav a one-link blackout does *not* reliably cause loss of
separation (a blind aircraft extrapolating a constant-velocity intruder is accurate), so P(LoS) has
no clean closed form. The blackout event does, and it exercises the same machinery.

The level variable is **staleness** — the count of consecutive drops — the accumulating discrete
cause, not a downstream geometric readout. Each shell's conditional survival is exactly P(drop) =
(1 - rx), so the product over ``W`` shells is ``(1 - rx)^W``. Splitting keeps ``N`` particles alive
at every depth, where plain Monte Carlo would need ~``1/(1-rx)^W`` runs to see one blackout.

Drop semantics match ``opencdarr.cns.communication.Comm``: a broadcast is dropped iff
``rng.random() >= reception_prob``.

    python scripts/staleness_proof.py --rx 0.15 --window 60 --particles 200 --reps 16
"""
from __future__ import annotations

import argparse
import math

import numpy as np

from opencdarr.rng import generator, root_seed_sequence, spawn


def _dropped(draw: float, rx: float) -> bool:
    """One broadcast on one link is dropped iff ``rng.random() >= reception_prob`` (matches Comm)."""
    return draw >= rx


def staleness_split_once(rx: float, window: int, n_particles: int,
                         seq: np.random.SeedSequence) -> tuple[float, list[float]]:
    """One fixed-effort run: split on consecutive-drop count over ``window`` shells.

    Mirrors ``ips_once``: fresh per-particle stream each shell, survivors resampled with replacement
    back to ``N``. A particle survives shell k iff its k-th broadcast is dropped (it stays blind);
    a received broadcast kills it (the blackout can no longer complete). Returns ``(P_hat, survival
    fractions)`` with ``P_hat = prod_k S_k / N``.
    """
    level_seqs = seq.spawn(window)
    # particle state is just "still blind so far" — identical across survivors, but we resample by
    # index exactly as ips_once does, so the estimator code path is the real one.
    n_alive = n_particles
    survival: list[float] = []
    for k in range(window):
        sub = level_seqs[k].spawn(n_particles + 1)
        draws = np.array([generator(sub[i]).random() for i in range(n_particles)])
        survivors = int(np.sum([_dropped(d, rx) for d in draws]))
        survival.append(survivors / n_particles)
        if survivors == 0:
            return 0.0, survival
        # resample survivors to N (trivial here since blind states are identical, but done for real)
        _ = generator(sub[n_particles]).integers(0, survivors, size=n_particles)
        n_alive = n_particles
    return float(np.prod(survival)), survival


def log_ci(probs: list[float], z: float = 1.96) -> tuple[float, float]:
    pos = [p for p in probs if p > 0.0]
    if len(pos) < 2 or len(pos) != len(probs):
        return (min(probs), max(probs))
    logs = np.log(pos)
    se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
    c = float(np.mean(logs))
    return (math.exp(c - z * se), math.exp(c + z * se))


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    if n == 0:
        return (float("nan"), 0.0, 1.0)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (p, max(0.0, centre - half), min(1.0, centre + half))


def mc_blackout(rx: float, window: int, n_runs: int, seq: np.random.SeedSequence) -> tuple[int, int]:
    """Plain MC: simulate ``n_runs`` full windows, count complete blackouts (all ``window`` dropped)."""
    blackouts = 0
    for s in spawn(seq, n_runs):
        rng = generator(s)
        if all(_dropped(float(rng.random()), rx) for _ in range(window)):
            blackouts += 1
    return blackouts, n_runs


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--rx", type=float, default=0.15, help="reception probability per broadcast")
    p.add_argument("--window", type=int, default=60, help="broadcasts in the blackout window")
    p.add_argument("--particles", type=int, default=200, help="IPS particles per shell")
    p.add_argument("--reps", type=int, default=16, help="independent replications")
    p.add_argument("--mc-n", dest="mc_n", type=int, default=20000, help="plain-MC windows")
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()

    theo = (1.0 - a.rx) ** a.window
    print(f"blackout event: rx={a.rx}, window={a.window} broadcasts  "
          f"->  theoretical P = (1-{a.rx})^{a.window} = {theo:.3e}")

    root = root_seed_sequence(a.seed)
    mc_seq, ips_seq = spawn(root, 2)

    k, n = mc_blackout(a.rx, a.window, a.mc_n, mc_seq)
    mc_p, mc_lo, mc_hi = wilson(k, n)
    print(f"\nMC        n={n:>7}  blackouts={k}  P={mc_p:.3e}  95%CI[{mc_lo:.3e}, {mc_hi:.3e}]")

    seeds = spawn(ips_seq, a.reps)
    results = [staleness_split_once(a.rx, a.window, a.particles, s) for s in seeds]
    probs = [r[0] for r in results]
    mean = float(np.mean(probs))
    lo, hi = log_ci(probs)
    n_coll = sum(1 for x in probs if x == 0.0)
    print(f"staleness {a.reps}x{a.particles}p x{a.window} shells  P={mean:.3e}  "
          f"95%CI[{lo:.3e}, {hi:.3e}]  collapsed={n_coll}/{a.reps}")

    # mean survival per shell — should hover at (1-rx)
    good = [r[1] for r in results if r[0] > 0.0]
    if good:
        per = np.mean([g for g in good if len(g) == a.window], axis=0)
        print(f"  mean survival/shell = {per.mean():.4f}  (expected 1-rx = {1 - a.rx:.4f})")

    err = abs(mean - theo) / theo
    within = lo <= theo <= hi
    print(f"\nverdict: staleness estimate is {err * 100:.1f}% from theory; "
          f"theory {'INSIDE' if within else 'OUTSIDE'} its 95%CI  "
          f"-> {'PASS' if within else 'CHECK'}")
    print(f"         (MC saw {k} event(s) in {n} runs; splitting resolves it from "
          f"{a.reps * a.particles * a.window} cheap Bernoulli draws)")


if __name__ == "__main__":
    main()
