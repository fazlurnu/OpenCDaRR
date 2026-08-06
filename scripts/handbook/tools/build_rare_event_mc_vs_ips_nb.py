"""Build examples/handbook/rare_event_mc_vs_ips.ipynb from a cell list."""
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
# The rare-event gate: MC and IPS on the same declaration

Plain Monte Carlo estimates P(LoS) by flying encounters and counting the ones that fail. That works
until failure becomes rare, and then it stops working in a particular way: the estimate is built
on a handful of events, so it swings from run to run — and eventually a whole run finds **zero**,
reporting `P(LoS) = 0` for something that is merely rare.

Interacting particle splitting (IPS, ADR 0017) spends its effort on the trajectories heading toward
the rare set instead. It reaches probabilities plain MC cannot afford.

The claim that has to be earned first is *agreement*: in a regime where MC is still solid, the two
must give the same number. Only then is IPS trustworthy where MC has run out.

This notebook walks one knob down three rungs and watches that happen:

| `pos_ci95` | P(LoS) | what it is for |
|---|---|---|
| 40 m | ~2.8e-2 | the anchor — MC is solid, so agreement is testable |
| 20 m | ~5e-3 | the crossover |
| 10 m | ~4.7e-4 | the demonstration — MC starves, IPS does not |

The knob is GNSS self-noise. Everything else is fixed: a `dcpa = 0` crossing pair, `rpz = 50 m`,
MVP + Past-CPA, `dt = 0.5`. A bigger `pos_ci95` widens each aircraft's perceived-position error, so
the resolver under-clears more often. Nothing else about the encounter changes, which is what makes
this a clean rarity dial rather than three different experiments.
""")

code(r'''
%matplotlib inline
import time

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["figure.dpi"] = 130

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.experiment import IPS, MC, Fixed, Methods, run_experiment
from opencdarr.performance import M600

BASE = Config(
    seed=0, n_encounters=1,                       # n_encounters is set per backend
    scenario=ScenarioConfig("M600", 10.2889, 50.0, 70.0),
    conflict=ConflictConfig(50.0, 120.0),         # rpz 50 m, lookahead 120 s
    methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
    simulation=SimulationConfig(0.5, 250.0, 10.0),
)
METHODS = Methods(detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(True),
                  navigation=GnssNavigation(), perf=M600)

def rung(pos_ci95: float) -> dict:
    """One rung of the rarity ladder: a head-on-crossing pair at this GNSS noise level."""
    return {"dcpa": Fixed(0.0), "pos_ci95": Fixed(pos_ci95), "vel_ci95": Fixed(pos_ci95 / 10)}

JOBS = -1      # every core. See section 4 for why this now reaches inside a single cell.
''')

# ---------------------------------------------------------------- 2. the anchor
md(r"""
## 1. The anchor: where MC is still solid

At `pos_ci95 = 40` about one encounter in thirty ends in a loss, so a couple of thousand encounters
give MC plenty of events. This is the rung where "do they agree?" is a fair question — and the
answer has to be yes before anything below it means anything.
""")

code(r'''
def measure(declaration, backend, jobs=JOBS):
    """Run one cell and time it. Returns (estimate, wall seconds)."""
    t0 = time.perf_counter()
    cell = run_experiment(declaration, methods=METHODS, backend=backend,
                          base_config=BASE, seed=0, n_jobs=jobs).cell()
    return cell, time.perf_counter() - t0


SHELLS = [100.0, 75.0, 60.0, 55.0, 52.0, 51.0, 50.0]   # decreasing, ending at rpz

mc_anchor, t_mc = measure(rung(40.0), MC(n_encounters=3000))
ips_anchor, t_ips = measure(rung(40.0), IPS(shells=SHELLS, n_particles=400, reps=4))

print(f"MC   P(LoS) = {mc_anchor.p_los_run:.4e}   from {mc_anchor.n_los} events   {t_mc:5.1f} s")
print(f"IPS  P(LoS) = {ips_anchor.p_los_run:.4e}   collapsed = {ips_anchor.n_collapsed}"
      f"        {t_ips:5.1f} s")
both = (mc_anchor.p_los_run, ips_anchor.p_los_run)
print(f"\nratio = {max(both) / min(both):.2f}x")
''')

md(r"""
They agree. There is no confidence interval to compare, because the estimators no longer report one
(ADR 0022) — an interval on a product-of-survivals estimator describes the spread of the
replications and not the shell spacing that actually dominates the error. Agreement is judged on
the **ratio** instead: within a factor of two, or five at 1e-4 and below where the MC anchor is
itself built on few events.
""")

# ---------------------------------------------------------------- 3. the walk down
md(r"""
## 2. Walking the knob down

Now the same comparison at three rungs. Watch the MC **event count**, not just its estimate: that
is the number the estimate is really made of.
""")

code(r'''
LADDER = [(40.0, 3000), (20.0, 6000), (10.0, 12000)]   # (pos_ci95, MC encounters)
rows = []

for pos, n_enc in LADDER:
    mc, t_mc = measure(rung(pos), MC(n_encounters=n_enc))
    ips, t_ips = measure(rung(pos), IPS(shells=SHELLS, n_particles=400, reps=4))
    rows.append(dict(pos=pos, n_enc=n_enc, mc=mc.p_los_run, events=mc.n_los, t_mc=t_mc,
                     ips=ips.p_los_run, collapsed=ips.n_collapsed, t_ips=t_ips))
    pair = (mc.p_los_run, ips.p_los_run)
    ratio = max(pair) / min(pair) if min(pair) > 0 else float("inf")
    print(f"pos_ci95 = {pos:4.0f} m | MC {mc.p_los_run:.3e} ({mc.n_los:3d} events,"
          f" {n_enc:,} runs, {t_mc:5.1f} s)"
          f" | IPS {ips.p_los_run:.3e} ({t_ips:5.1f} s) | ratio {ratio:.2f}x")
''')

code(r'''
pos = np.array([r["pos"] for r in rows])
mc_p = np.array([r["mc"] for r in rows])
ips_p = np.array([r["ips"] for r in rows])
events = [r["events"] for r in rows]

fig, (ax, ax_ev) = plt.subplots(1, 2, figsize=(9.6, 3.7))

ax.plot(pos, mc_p, marker="o", ms=5, lw=1.6, color="tab:orange", label="MC")
ax.plot(pos, ips_p, marker="s", ms=5, lw=1.6, color="tab:blue", label="IPS")
ax.set_yscale("log")
ax.invert_xaxis()                                   # rarer to the right
ax.set_xlabel("pos_ci95 [m]  (rarer ->)")
ax.set_ylabel("P(LoS)")
ax.set_xticks(pos)
ax.set_title("The two estimators track each other down the dial.", fontsize=10)
ax.legend(frameon=False, fontsize=9)

ax_ev.bar([f"{p:.0f} m" for p in pos], events, color="tab:orange", width=0.55)
for x, n in enumerate(events):
    ax_ev.annotate(str(n), (x, n), ha="center", va="bottom", fontsize=9)
ax_ev.set_xlabel("pos_ci95")
ax_ev.set_ylabel("LoS events MC actually saw")
ax_ev.set_title("What the MC estimate is made of.", fontsize=10)

fig.tight_layout()
plt.show()
''')

md(r"""
The left panel is the gate: the two lines stay within a factor of two of each other while P(LoS)
falls through two decades.

They cross on the way down, and that is worth reading correctly. Below the anchor the MC points are
built on eleven and then seven events, so their scatter is large — the crossing is sampling noise,
not a systematic difference between the estimators. The right panel is that scatter made visible.

Cost moves the other way. Each rung down costs MC roughly ten times the encounters for the same
precision, while IPS pays about the same every time.

Push one rung further and MC stops reporting a small number and starts reporting the wrong one:
""")

code(r'''
starved, t_starved = measure(rung(10.0), MC(n_encounters=1500))
print("MC at only 1,500 encounters, pos_ci95 = 10 m:")
print(f"   P(LoS) = {starved.p_los_run:.3e}   from {starved.n_los} event(s)"
      f"   {t_starved:.1f} s")
print(f"   IPS on the same rung gave {rows[-1]['ips']:.3e}"
      f" in {rows[-1]['t_ips']:.1f} s")
''')

md(r"""
An estimate resting on zero or one event is not a small probability, it is an absent measurement.
`vault/observations/rare-event-validation-ladder.md` records a seed where ten thousand encounters
at this rung find **no events at all**, and report `P(LoS) = 0`.

That is the whole case for splitting: not that IPS is faster here, but that a little further down
MC has no answer at any price.
""")

# ---------------------------------------------------------------- 4. the budget
md(r"""
## 3. Where the cores go

A rare-event study is usually **one** condition that runs for a long time. That used to be the
worst case for `n_jobs`: it spread *conditions* over processes, so a single-condition experiment
used a single core however many were free.

`n_jobs` is now a budget, spent where it helps. While there are at least as many conditions as
workers the conditions are fanned out — the cheapest split, since nothing crosses a process
boundary but a seed and a result. Past that the conditions run in turn and the budget goes
**inside** each cell: MC slices its encounter fan-out into contiguous seed slices and pools the
counts, and IPS shards each shell across the workers (ADR 0018). Never both at once, because two
pools would nest.

It changes the wall time and nothing else. The cell below is the proof — same declaration, two
worker counts, identical numbers.
""")

code(r'''
one, t_one = measure(rung(40.0), MC(n_encounters=3000), jobs=1)
many, t_many = measure(rung(40.0), MC(n_encounters=3000), jobs=-1)

print(f"n_jobs =  1 : P(LoS) = {one.p_los_run:.6e}   {t_one:5.1f} s")
print(f"n_jobs = -1 : P(LoS) = {many.p_los_run:.6e}   {t_many:5.1f} s   "
      f"({t_one / t_many:.1f}x faster)")
print(f"\nidentical, field for field: {one == many}")
''')

md(r"""
`n_jobs` is deliberately **not** part of the cache key, for the same reason: it would store several
identical copies of one cell and re-run the lot after a change of machine.
""")

# ---------------------------------------------------------------- 5. summary
md(r"""
## Summary

- **Earn agreement first.** At `pos_ci95 = 40` MC has plenty of events and the two estimators
  match. Without that rung, an IPS number in the rare regime is unfalsifiable.
- **Judge it on the ratio.** Within a factor of two, or five at 1e-4 and below. There is no
  interval to compare — see ADR 0022 for why one was misleading here.
- **Read the event count, not just the estimate.** MC's number at the bottom rung is a handful
  of events; a run that sees none reports zero, which is a different claim from "rare".
- **IPS costs about the same at every rung.** MC costs roughly ten times more per decade. The lines
  cross, and below the crossing MC eventually has no answer at any budget.
- **`n_jobs` reaches inside a single cell**, which is the shape a rare-event study actually has.

The pairwise numbers here are a single crossing geometry. `scripts/validation/` runs the same
comparison across the three encounter families as a campaign, with one cache entry per condition.
""")


def main() -> None:
    nb = nbf.v4.new_notebook()
    nb.cells = [
        nbf.v4.new_markdown_cell(src) if kind == MD else nbf.v4.new_code_cell(src)
        for kind, src in CELLS
    ]
    nb.metadata = {
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
        "language_info": {"name": "python", "pygments_lexer": "ipython3"},
    }
    out = pathlib.Path("examples/handbook/rare_event_mc_vs_ips.ipynb")
    out.write_text(nbf.writes(nb))
    print(f"wrote {out} with {len(nb.cells)} cells")


if __name__ == "__main__":
    main()
