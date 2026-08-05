"""Build examples/handbook/scenario_random_traffic.ipynb from a cell list.

Random traffic: N aircraft crossing a measured disc on random headings. The scenario where the
per-aircraft rate per flight hour is the comparable number. Explanations in ASD-STE100.
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
# The random-traffic scenario

`N` aircraft cross a disc on random headings. Each aircraft draws a heading and a perpendicular
offset, then flies the chord that the offset cuts. The aircraft start on a larger circle and enter
the measured disc already in flight, so a pair that starts close together has had time to separate
before the measurement begins.

Two things make this scenario different from the ring:

- **The geometry is random.** The ring is the same in each encounter, so only the noise changes.
  Here the geometry changes as well, and a conflict is itself a random event.
- **There is a measurement area.** A pair contributes to the result only while the two aircraft are
  inside the disc. Thus the run-in outside the disc is flown but not measured.

The measurement area also gives the scenario a measured **time**. That is what makes a rate for
each aircraft for each flight hour available here, and it is the form that a traffic-density study
reports.
""")

code(r'''
%matplotlib inline
import dataclasses
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from joblib import Parallel, delayed

plt.rcParams["figure.dpi"] = 130

from opencdarr import GnssNavigation, M600, MVP, PastCPA, StateBased, geo
from opencdarr.config import load_config
from opencdarr.estimator import agents_for, combine_ipr, estimate_ipr_over
from opencdarr.fleet import build_env, run_fleet
from opencdarr.ips import Particle, estimate_rare_prob, ladder_from_record
from opencdarr.rng import children, generator, root_seed_sequence, spawn
from opencdarr.scenario import RandomTraffic

BLUE, RED, ORANGE, GREY = "#1f77b4", "#d62728", "#ff7f0e", "0.45"
JOBS = 8
SEED = 20260805
R_INNER, R_OUTER = 1000.0, 1200.0
STOP_WITHIN = 50.0
POS_CI95 = 15.0

BASE = load_config("../../configs/pairwise.yaml")
RPZ = BASE.conflict.rpz

SITE_IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"


def publish(fig, name: str) -> None:
    if SITE_IMG.is_dir():
        fig.savefig(SITE_IMG / f"{name}.png", dpi=130)


def config(pos_ci95: float, n_encounters: int):
    """`done_timeout` is disabled, so only the waypoints or the departure end a run."""
    return dataclasses.replace(
        BASE, seed=SEED, n_encounters=n_encounters,
        scenario=dataclasses.replace(BASE.scenario, speed=10.0,
                                     pos_ci95=pos_ci95, vel_ci95=pos_ci95 / 10.0),
        simulation=dataclasses.replace(BASE.simulation, t_max=1800.0,
                                       done_timeout=1e9, stop_within=STOP_WITHIN),
    )


print(f"measured disc {R_INNER:.0f} m   spawn circle {R_OUTER:.0f} m   "
      f"rpz {RPZ:.0f} m   pos_ci95 {POS_CI95:.0f} m")
''')

# ---------------------------------------------------------------- 2. the picture
md(r"""
## Part 1 — what an encounter looks like

The figure shows the ground tracks of a few encounters, with one column for each fleet size. Each
aircraft has its own colour, the circle is its start and the star is its end. The dashed circle is
the measured disc. Each chord crosses that disc, because the offset is drawn across the inner
diameter.

The bottom row is the centre of the disc, at the scale of the protected zone.
""")

code(r'''
SIZES = [4, 6, 8]
N_SHOW = 30         # enough for the mean time inside the disc
N_PLOT = 3          # a few samples: the figure is an illustration, not a measurement


def fly_one(seq, n_ac: int, pos_ci95: float):
    cfg = config(pos_ci95, 1)
    sc = RandomTraffic(n=n_ac, r_inner=R_INNER, r_outer=R_OUTER)
    fleet = sc.draw(generator(seq), cfg)
    nav, _, _ = spawn(seq, 3)
    return run_fleet(
        agents_for(fleet, M600), rpz=cfg.conflict.rpz,
        t_lookahead=cfg.conflict.t_lookahead, dt=cfg.simulation.dt,
        detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(),
        navigation=GnssNavigation(), rng=generator(nav), t_max=cfg.simulation.t_max,
        done_timeout=cfg.simulation.done_timeout, stop_within=STOP_WITHIN,
        measure_within=sc.measurement_area(), record=True)


shown = {}
for n_ac in SIZES:
    t0 = time.perf_counter()
    shown[n_ac] = [fly_one(s, n_ac, POS_CI95)
                   for s in spawn(root_seed_sequence(SEED), N_SHOW)]
    outs = shown[n_ac]
    print(f"N {n_ac}   {N_SHOW} encounters   losses {sum(o.los for o in outs):>3}   "
          f"closest approach overall {min(o.min_sep for o in outs):6.1f} m   "
          f"mean duration {np.mean([o.frames[-1].t for o in outs]):6.1f} s   "
          f"{time.perf_counter() - t0:5.1f} s")
''')

code(r'''
def tracks_enu(outcome):
    frames = outcome.frames
    lat0, lon0 = 52.0, 4.0
    out = []
    for i in range(len(frames[0].states)):
        xy = []
        for f in frames:
            qdr, dist = geo.qdrdist(lat0, lon0, f.states[i].lat, f.states[i].lon)
            a = np.radians(qdr)
            xy.append((dist * np.sin(a), dist * np.cos(a)))
        out.append(np.array(xy))
    return out


fig, axes = plt.subplots(2, len(SIZES), figsize=(9.6, 6.6))

for col, n_ac in enumerate(SIZES):
    colours = plt.cm.tab10(np.linspace(0, 1, 10))[:n_ac]
    for row, (scale, half, unit) in enumerate([(1000.0, 1.4, "km"), (1.0, 400.0, "m")]):
        ax = axes[row, col]
        for outcome in shown[n_ac][:N_PLOT]:
            for k, track in enumerate(tracks_enu(outcome)):
                ax.plot(track[:, 0] / scale, track[:, 1] / scale,
                        color=colours[k], lw=0.9, alpha=0.85)
                if row == 0:
                    ax.plot(track[0, 0] / scale, track[0, 1] / scale, "o",
                            color=colours[k], ms=4)
                    ax.plot(track[-1, 0] / scale, track[-1, 1] / scale, "*",
                            color=colours[k], ms=9)
        if row == 0:
            ax.add_patch(plt.Circle((0, 0), R_INNER / scale, color="0.25",
                                    fill=False, lw=1.0, ls="--"))
            ax.set_title(f"N = {n_ac}", fontsize=9)
        else:
            ax.add_patch(plt.Circle((0, 0), RPZ / scale, color="0.25", fill=False, lw=1.0))
        ax.set_xlim(-half, half); ax.set_ylim(-half, half)
        ax.set_xlabel(f"east [{unit}]")
        if col == 0:
            ax.set_ylabel(f"north [{unit}]")
        ax.set_box_aspect(1)

fig.tight_layout()
publish(fig, "scenario-random-traffic-tracks")
''')

# ---------------------------------------------------------------- 3. metrics
md(r"""
## Part 2 — the three metrics, and the rate for each flight hour

The sweep runs `N = 4, 6, 8`. The three probabilities come from the estimator. The rate for
each flight hour needs the measured time as well:

$$\lambda = \frac{P_{ac}}{T}$$

$T$ is the mean time that one aircraft spends inside the measured disc. The cell below takes it
from
the recorded encounters of Part 1.
""")

code(r'''
N_ENC = 10_000


def run_mc(n_ac: int, pos_ci95: float, n_encounters: int):
    cfg = config(pos_ci95, n_encounters)
    sc = RandomTraffic(n=n_ac, r_inner=R_INNER, r_outer=R_OUTER)
    root = root_seed_sequence(cfg.seed)
    bounds = [(i * n_encounters // JOBS, (i + 1) * n_encounters // JOBS) for i in range(JOBS)]
    parts = Parallel(n_jobs=JOBS)(
        delayed(estimate_ipr_over)(sc, cfg, M600, StateBased(), MVP(1.05), PastCPA(),
                                   GnssNavigation(), seqs=children(root, lo, hi))
        for lo, hi in bounds)
    return combine_ipr(parts)


def time_inside(outcome) -> float:
    """Mean time [s] that one aircraft of this encounter spent inside the measured disc."""
    lat0, lon0 = 52.0, 4.0
    dt = outcome.frames[1].t - outcome.frames[0].t
    n = len(outcome.frames[0].states)
    steps = 0
    for f in outcome.frames:
        for ac in f.states:
            if geo.qdrdist(lat0, lon0, ac.lat, ac.lon)[1] <= R_INNER:
                steps += 1
    return steps * dt / n


sweep, dwell = {}, {}
print(f"{'N':>3}{'P_ac':>11}{'P_run':>10}{'E[K]':>10}{'T inside [s]':>15}"
      f"{'lambda [1/h]':>15}")
for n_ac in SIZES:
    sweep[n_ac] = run_mc(n_ac, POS_CI95, N_ENC)
    dwell[n_ac] = float(np.mean([time_inside(o) for o in shown[n_ac]]))
    r = sweep[n_ac]
    lam = r.p_los / (dwell[n_ac] / 3600.0)
    print(f"{n_ac:>3}{r.p_los:11.5f}{r.p_los:10.5f}{r.mean_los_pairs:10.5f}"
          f"{dwell[n_ac]:15.1f}{lam:15.4f}")
''')

code(r'''
fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.9))

ax = axes[0]
ax.plot(SIZES, [sweep[n].p_los for n in SIZES], "o-", color=BLUE, label="$P_{ac}$")
ax.plot(SIZES, [sweep[n].p_los for n in SIZES], "s-", color=RED, label="$P_{run}$")
ax.plot(SIZES, [sweep[n].mean_los_pairs for n in SIZES], "^-", color=ORANGE, label="E[K]")
ax.set_xticks(SIZES); ax.set_xlabel("aircraft in the disc")
ax.set_ylabel("value"); ax.set_yscale("log")
ax.set_box_aspect(1); ax.legend(frameon=False, fontsize=8)

ax = axes[1]
lam = [sweep[n].p_los / (dwell[n] / 3600.0) for n in SIZES]
density = [n / (np.pi * (R_INNER / 1000.0) ** 2) for n in SIZES]
ax.plot(density, lam, "o-", color=BLUE)
ax.set_xlabel("traffic density [aircraft / km$^2$]")
ax.set_ylabel("losses per aircraft per flight hour")
ax.set_box_aspect(1)

fig.tight_layout()
publish(fig, "scenario-random-traffic-metrics")
''')

# ---------------------------------------------------------------- 4. MC vs IPS
md(r"""
## Part 3 — Monte Carlo against the interacting particle system

The two estimators run on the same configuration at `N = 8`. IPS uses `tail=True`, so each survivor
flies on to the end of its encounter and the per-aircraft number is available.

The shells come from the Monte Carlo record of the same cell.
""")

code(r'''
def build_initial_for(n_ac: int, pos_ci95: float):
    cfg = config(pos_ci95, 1)
    sc = RandomTraffic(n=n_ac, r_inner=R_INNER, r_outer=R_OUTER)

    def build_initial(seq):
        fleet = sc.draw(generator(seq), cfg)
        agents = agents_for(fleet, M600)
        env = build_env(
            agents, rpz=cfg.conflict.rpz, t_lookahead=cfg.conflict.t_lookahead,
            dt=cfg.simulation.dt, detector=StateBased(), resolver=MVP(1.05),
            recovery=PastCPA(), navigation=GnssNavigation(),
            t_max=cfg.simulation.t_max, done_timeout=cfg.simulation.done_timeout,
            stop_within=cfg.simulation.stop_within,
            measure_within=sc.measurement_area(),
        )
        return Particle(env=env, state=env.initial_state(agents))

    return build_initial


shells = ladder_from_record(sweep[8].min_seps, RPZ)
t0 = time.perf_counter()
ips8 = estimate_rare_prob(build_initial_for(8, POS_CI95), shells,
                          n_particles=600, reps=8, seed=SEED, tail=True)
print(f"shells {len(shells)}   {shells}")
print(f"IPS  P(LoS) {ips8.p_los:.5f}   collapsed {ips8.n_collapsed}")
      f"collapsed {ips8.n_collapsed}/8   {time.perf_counter() - t0:6.1f} s")

p_los_reps = [r.p_los for r in ips8.reps if r.prob > 0]
print(f"IPS  P_ac  {np.mean(p_los_reps):.5f}   from {len(p_los_reps)} replications")
print(f"     distinct lineages per replication: {[r.n_lineages for r in ips8.reps]}")
print(f"MC   P(LoS) {sweep[8].p_los:.5f}")
if sweep[8].p_los > 0 and sweep[8].p_los > 0:
    print(f"\nratio IPS/MC on P_run {ips8.prob / sweep[8].p_los:.2f}   "
          f"on P_ac {np.mean(p_los_reps) / sweep[8].p_los:.2f}")
else:
    print("\nMonte Carlo saw no loss of separation in this cell, so there is no ratio. "
          "Increase N_ENC until it sees approximately 50 events.")
''')

md(r"""
## What this shows

Random traffic is the scenario closest to a density study. The geometry is drawn, not arranged, so
the result depends on how much traffic is in the area and not on one chosen conflict.

The rate for each aircraft for each flight hour is the form that removes both the fleet size
and the run length from the number. It is the quantity to compare against a traffic-density
result in the
literature.

The measurement area is what makes that possible, and it is also what keeps the two estimators
measuring one event. IPS splits on the same gated minimum separation the Monte Carlo counts.
""")


def main() -> None:
    nb = nbf.v4.new_notebook()
    nb.cells = [nbf.v4.new_markdown_cell(s) if kind == MD else nbf.v4.new_code_cell(s)
                for kind, s in CELLS]
    nb.metadata = {"kernelspec": {"display_name": "Python 3", "language": "python",
                                  "name": "python3"},
                   "language_info": {"name": "python"}}
    out = pathlib.Path("examples/handbook/scenario_random_traffic.ipynb")
    out.parent.mkdir(parents=True, exist_ok=True)
    nbf.write(nb, out)
    print(f"wrote {out} ({len(CELLS)} cells)")


if __name__ == "__main__":
    main()
