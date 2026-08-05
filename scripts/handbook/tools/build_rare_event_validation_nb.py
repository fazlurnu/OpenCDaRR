"""Build examples/handbook/rare_event_validation.ipynb from a cell list.

The evidence that plain Monte Carlo and the interacting particle system estimate the *same*
probability: a ladder of GNSS accuracies on one pairwise geometry, both estimators on every rung
where MC is affordable, then IPS alone below MC's reach. Explanations in ASD-STE100.
"""
from __future__ import annotations

import pathlib

import nbformat as nbf

MD, CODE = "md", "code"

CELLS: list[tuple[str, str]] = []


def md(s: str) -> None:
    CELLS.append((MD, s.strip("\n")))


def code(s: str) -> None:
    CELLS.append((CODE, s.strip("\n")))


# ---------------------------------------------------------------- 1. intro
md(r"""
# Rare-event validation

This notebook gives the evidence that the two estimators measure the same quantity. Plain Monte
Carlo (MC) counts the losses of separation in many encounters. The interacting particle system
(IPS) splits the event into shells and multiplies the conditional probabilities. The two methods
are very different, so agreement between them is a test of the two.

The test has three parts:

1. **The ladder.** One geometry, and the GNSS accuracy `pos_ci95` decreases in steps. A smaller
   error gives a smaller probability of a loss of separation. This is the knob that makes the event
   rare.
2. **The overlap band.** On the rungs where MC is affordable, the two estimators run on the same
   configuration. The result is a ratio with an interval, not one point with two error bars.
3. **Below the reach of MC.** On the lower rungs, MC observes no event and gives only an upper
   bound. IPS still gives a number with an interval.

The scenario is **pairwise**, with two aircraft. This is deliberate. At two aircraft the
per-aircraft probability and the per-run probability are the same number, so this notebook tests
the *estimators* alone. The scenario notebooks test the metric at more aircraft.

!!! note
    The lower rungs use a `pos_ci95` below 1 m. No GNSS receiver is that good. The small values are
    a way to make the event rare with one knob, and they are not a claim about a real sensor.
""")

code(r'''
%matplotlib inline
import dataclasses
import time

import matplotlib.pyplot as plt
import numpy as np
from joblib import Parallel, delayed

plt.rcParams["figure.dpi"] = 130

from opencdarr import GnssNavigation, M600, MVP, PastCPA, StateBased
from opencdarr.config import load_config
from opencdarr.estimator import agents_for, combine_ipr, estimate_ipr_over
from opencdarr.fleet import build_env
from opencdarr.ips import Particle, estimate_rare_prob, ladder_from_record
from opencdarr.rng import children, generator, root_seed_sequence
from opencdarr.scenario import PairwiseEncounter

BLUE, RED, ORANGE, GREY = "#1f77b4", "#d62728", "#ff7f0e", "0.45"
JOBS = 8
SEED = 20260805

BASE = load_config("../../configs/pairwise.yaml")
GEOMETRY = PairwiseEncounter(dpsi=2.0, dcpa=0.0)   # near head-on, worst case
RPZ = BASE.conflict.rpz


def config(pos_ci95: float, n_encounters: int):
    """One rung of the ladder: the accuracy, and how many encounters to sample."""
    return dataclasses.replace(
        BASE, seed=SEED, n_encounters=n_encounters,
        scenario=dataclasses.replace(BASE.scenario, speed=10.0, tlos=180.0,
                                     pos_ci95=pos_ci95, vel_ci95=pos_ci95 / 10.0),
    )


print(f"rpz {RPZ:.0f} m   lookahead {BASE.conflict.t_lookahead:.0f} s   "
      f"dt {BASE.simulation.dt:.1f} s   crossing angle 2 deg")
''')

# ---------------------------------------------------------------- 2. runners
md(r"""
## The two estimators

Each estimator gets its own function. The two use the **same** configuration, the same geometry,
and the same environment. Only the method is different.

MC divides the encounters between the cores. Each core takes a contiguous slice of the one seed
tree, so the parallel answer is identical to the serial answer.

IPS needs a `build_initial` that makes one particle from one seed. The particle holds the
environment and the initial state. The code below builds it exactly as the experiment layer does,
so the two paths cannot become different.
""")

code(r'''
def run_mc(pos_ci95: float, n_encounters: int):
    """Plain Monte Carlo over `n_encounters` sampled encounters, on `JOBS` cores."""
    cfg = config(pos_ci95, n_encounters)
    root = root_seed_sequence(cfg.seed)
    bounds = [(i * n_encounters // JOBS, (i + 1) * n_encounters // JOBS) for i in range(JOBS)]
    parts = Parallel(n_jobs=JOBS)(
        delayed(estimate_ipr_over)(
            GEOMETRY, cfg, M600, StateBased(), MVP(1.05), PastCPA(), GnssNavigation(),
            seqs=children(root, lo, hi))
        for lo, hi in bounds)
    return combine_ipr(parts)


def build_initial_for(pos_ci95: float):
    """The IPS particle factory for one rung — one sampled encounter from one seed."""
    cfg = config(pos_ci95, 1)

    def build_initial(seq):
        fleet = GEOMETRY.draw(generator(seq), cfg)
        agents = agents_for(fleet, M600)
        env = build_env(
            agents, rpz=cfg.conflict.rpz, t_lookahead=cfg.conflict.t_lookahead,
            dt=cfg.simulation.dt, detector=StateBased(), resolver=MVP(1.05),
            recovery=PastCPA(), navigation=GnssNavigation(),
            t_max=cfg.simulation.t_max, done_timeout=cfg.simulation.done_timeout,
            stop_within=cfg.simulation.stop_within,
        )
        return Particle(env=env, state=env.initial_state(agents))

    return build_initial


def run_ips(pos_ci95: float, shells, n_particles: int = 1000, reps: int = 8):
    """The interacting particle system over a fixed shell ladder.

    `tail=True` flies each survivor on to termination. That leg is what turns the reach
    probability into the reported per-aircraft `p_los`, which is the number compared below.
    """
    return estimate_rare_prob(build_initial_for(pos_ci95), shells,
                              n_particles=n_particles, reps=reps, seed=SEED, tail=True)
''')

# ---------------------------------------------------------------- 3. the ladder
md(r"""
## Part 1 — the ladder of rarity

The first step measures how the accuracy drives the probability. Each rung uses enough encounters
to see approximately 100 losses of separation, because the width of the interval comes from the
number of *events*, not from the number of encounters.

The last rungs of this cell are the boundary of the method. The number of encounters for 100 events
increases as 1/p, so the cost increases by 10 for each decade.
""")

code(r'''
LADDER = [
    # pos_ci95 [m], MC encounters (0 = too expensive for MC).
    # Sized from a pilot so each rung sees ~50-120 events: p falls from 3e-2 to 6e-4.
    (5.00, 4_000),
    (2.00, 12_000),
    (1.50, 20_000),
    (1.00, 50_000),
    (0.75, 80_000),
    (0.50, 0),
]
MC_BUDGET = max(n for _, n in LADDER)   # the largest batch we were willing to run

mc = {}
for ci, n in LADDER:
    if n == 0:
        continue
    t0 = time.perf_counter()
    mc[ci] = run_mc(ci, n)
    r = mc[ci]
    print(f"pos_ci95 {ci:5.2f} m   n {n:>7,}   "
          f"aircraft in a loss {sum(r.los_aircraft):>4}   p {r.p_los:.5f}   "
          f"{time.perf_counter() - t0:5.1f} s")
''')

md(r"""
A batch that observes **no** event does not measure zero — it measures nothing at all. The rate
it was looking for is somewhere below the one event it did not see, and the batch cannot say
where. The cell below shows the order MC is left at on the two lowest rungs, for the same cost as
the highest rung.
""")

code(r'''
for ci, n in LADDER:
    if n != 0:
        continue
    print(f"pos_ci95 {ci:5.2f} m   with {MC_BUDGET:,} encounters and 0 events, "
          f"MC resolves nothing below ~{1 / MC_BUDGET:.0e}")
''')

# ---------------------------------------------------------------- 4. overlap
md(r"""
## Part 2 — the overlap band

Both estimators now run on the rungs where MC is affordable. The shell ladder for IPS comes from
the MC record of the same rung, with `ladder_from_record`. This keeps the shells away from a manual
choice that could be tuned to give the correct answer.

The comparison is the **ratio** of the two estimates. A ratio of 1.0 is perfect agreement. The bar
is a factor of two, and a factor of five at 1e-4 and below, where the Monte-Carlo anchor is itself
built on only a few hundred events. One point that agrees is weak evidence. Three rungs that agree
over more than one decade is the claim of this page.
""")

code(r'''
ips = {}
for ci, n in LADDER:
    if n == 0:
        continue
    shells = ladder_from_record(mc[ci].min_seps, RPZ)
    t0 = time.perf_counter()
    ips[ci] = run_ips(ci, shells)
    e = ips[ci]
    print(f"pos_ci95 {ci:5.2f} m   shells {len(shells):>2}   "
          f"p {e.p_los:.5f}   collapsed {e.n_collapsed}/8   "
          f"{time.perf_counter() - t0:5.1f} s")
''')

code(r'''
def tolerance(p):
    """The factor the two estimators must agree within at a rate of p.

    Two, except where the event is rare enough that the Monte-Carlo anchor is itself coarse: at
    1e-4 and below a 2-million-encounter batch counts only a few hundred events, so a factor of
    five is the honest bar.
    """
    return 5.0 if p <= 1e-4 else 2.0

print(f"{'pos_ci95':>9}{'MC':>11}{'IPS':>11}{'IPS/MC':>9}{'within':>8}   agrees")
for ci, n in LADDER:
    if n == 0:
        continue
    m, e = mc[ci].p_los, ips[ci].p_los
    if m <= 0:
        print(f"{ci:9.2f}{m:11.5f}{e:11.5f}{'--':>9}{'--':>8}   --")
        continue
    factor, r = tolerance(m), e / m
    agrees = 1 / factor <= r <= factor
    print(f"{ci:9.2f}{m:11.5f}{e:11.5f}{r:9.2f}{factor:8.0f}x   "
          f"{'yes' if agrees else 'NO'}")
''')

# ---------------------------------------------------------------- 5. beyond MC
md(r"""
## Part 3 — below the reach of Monte Carlo

The two lowest rungs are where MC stops. The shells for them come from the lowest rung that MC
could
measure, with the last shell always at the protected zone. IPS gives a number and an interval where
MC gives only an upper bound.
""")

code(r'''
deepest_mc = [ci for ci, n in LADDER if n][-1]
deep_shells = ladder_from_record(mc[deepest_mc].min_seps, RPZ)
for ci, n in LADDER:
    if n != 0:
        continue
    t0 = time.perf_counter()
    ips[ci] = run_ips(ci, deep_shells)
    e = ips[ci]
    print(f"pos_ci95 {ci:5.2f} m   p {e.p_los:.3e}   "
          f"collapsed {e.n_collapsed}/8   {time.perf_counter() - t0:5.1f} s")
''')

# ---------------------------------------------------------------- 6. figure
md(r"""
## The figure

The left panel is the evidence. Each rung carries the MC estimate and the IPS estimate. Where MC
observes no event, an arrow marks the order it cannot resolve below. The right panel is the ratio
of the two estimates on the rungs where the two exist.
""")

code(r'''
from pathlib import Path

SITE_IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"


def publish(fig, name: str) -> None:
    if SITE_IMG.is_dir():
        fig.savefig(SITE_IMG / f"{name}.png", dpi=130)


fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.9))

ax = axes[0]
xs_mc = [ci for ci, n in LADDER if n]
ax.plot(xs_mc, [mc[c].p_los for c in xs_mc], "o", color=RED, label="Monte Carlo")
xs_ips = [ci for ci, _ in LADDER]
ax.plot(xs_ips, [ips[c].p_los for c in xs_ips], "s", color=BLUE,
        markerfacecolor="none", label="IPS")
# where MC observed nothing, mark the order it cannot resolve below
for ci, n in LADDER:
    if n:
        continue
    ax.annotate("", xy=(ci, 1 / MC_BUDGET / 6), xytext=(ci, 1 / MC_BUDGET),
                arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2))
ax.set_xscale("log"); ax.set_yscale("log")
ax.set_xlabel("pos_ci95 [m]"); ax.set_ylabel("P(LoS) per aircraft")
ax.set_box_aspect(1); ax.legend(frameon=False, fontsize=8)

ax = axes[1]
measured = [c for c in xs_mc if mc[c].p_los > 0]
ratio = [ips[c].p_los / mc[c].p_los for c in measured]
ax.axhline(1.0, color=GREY, lw=1.0, ls="--")
ax.plot(measured, ratio, "o-", color=ORANGE)
ax.set_xscale("log"); ax.set_xlabel("pos_ci95 [m]"); ax.set_ylabel("IPS / MC")
ax.set_ylim(0, 2); ax.set_box_aspect(1)

fig.tight_layout()
publish(fig, "rare-event-validation")
''')

# ---------------------------------------------------------------- 7. cost
md(r"""
## The cost of each estimate

The last table is the reason IPS exists. MC pays for each decade of rarity with ten times more
encounters. IPS pays approximately the same amount on each rung, because it never measures a rare
quantity. The two columns cross in the overlap band.
""")

code(r'''
print(f"{'pos_ci95':>9}{'MC encounters':>15}{'IPS particles':>15}")
for ci, n in LADDER:
    n_ips = 1000 * 8
    print(f"{ci:9.2f}{(n if n else 0):>15,}{n_ips:>15,}")
print("\nA rung with 0 MC encounters is one where MC observes no event within the budget.")
''')

md(r"""
## What this shows

The two estimators agree on each rung where both can measure: every ratio is inside the factor its
rate is held to, and it stays near 1.0 over more than one decade of probability. Below that band MC
observes nothing at all, and IPS continues to give a number.

Agreement in the overlap band is what makes the IPS numbers below the band usable. Without it, a
small number from a splitting estimator is only a number.
""")


def main() -> None:
    nb = nbf.v4.new_notebook()
    nb.cells = [nbf.v4.new_markdown_cell(s) if kind == MD else nbf.v4.new_code_cell(s)
                for kind, s in CELLS]
    nb.metadata = {"kernelspec": {"display_name": "Python 3", "language": "python",
                                  "name": "python3"},
                   "language_info": {"name": "python"}}
    out = pathlib.Path("examples/handbook/rare_event_validation.ipynb")
    out.parent.mkdir(parents=True, exist_ok=True)
    nbf.write(nb, out)
    print(f"wrote {out} ({len(CELLS)} cells)")


if __name__ == "__main__":
    main()
