"""Production runner for the two campaigns in ``examples/handbook/ring_mc_vs_ips.ipynb``.

Part 1 is the ring: ``N`` aircraft on a circle, each flying to the point diametrically opposite --
one arranged worst case. Part 2 is random traffic drawn by the entry rule of Groot, Ellerbroek &
Hoekstra (2024): ``N`` drones crossing a 1 km disc on random headings, released on a 1.2 km spawn
circle so a pair that starts close has room to resolve before it is measured. Both are estimated
twice, by plain Monte Carlo and by the fixed-effort IPS of :mod:`opencdarr.ips`, through the same
environment.

**Monte Carlo runs to a number of events, not a number of encounters.** The precision of a
binomial estimate is set by how many losses were *seen*: the relative standard error is
``1/sqrt(k)`` for ``k`` events, so 25 events is +-20%, 100 is +-10% and 400 is +-5% — whatever the
underlying probability is. Sizing a campaign instead in encounters requires knowing the answer
first, which is the thing being measured. So ``--target-events`` runs in chunks and stops when the
target is reached or ``--max-encounters`` is exhausted; a run that exhausts the cap reports the
Wilson interval it earned and is flagged ``short``.

    PYTHONPATH=. python scripts/mc_vs_ips_campaign.py --calibrate
    PYTHONPATH=. python scripts/mc_vs_ips_campaign.py --target-events 100 --reps 20 --out run.json
    PYTHONPATH=. python scripts/mc_vs_ips_campaign.py --part 2 --dt 0.2 --jobs 100

``--calibrate`` times a few encounters per cell on the machine it is on and prints what the
campaign would cost there, which is the thing to run first on a new box.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
from joblib import Parallel, delayed

from opencdarr import (
    M600,
    MVP,
    Agent,
    GnssNavigation,
    MeasurementArea,
    StateBased,
    crossing_ring,
    random_traffic,
)
from opencdarr.autopilot import WaypointAutopilot
from opencdarr.crr import ProbabilisticFTR
from opencdarr.estimator import wilson_interval
from opencdarr.fleet import CnsStreams, FleetEnv, FleetStreams, build_env
from opencdarr.ips import Particle
from opencdarr.mission import Mission
from opencdarr.parallel import estimate_rare_prob, resolve_jobs
from opencdarr.rng import children, generator, root_seed_sequence

CENTRE = (52.0, 4.0)
SPEED = 10.0          # common ground speed [m/s]
RPZ = 50.0            # protected zone [m]
LOOKAHEAD = 30.0      # detection horizon [s]
MARGIN = 1.05         # MVP resolution margin
POS_CI95 = 10.0       # GNSS fix, declared and drawn [m]
VEL_CI95 = 1.0        # [m/s]

PERF = dataclasses.replace(M600, v_max=SPEED, v_min=-SPEED)


@dataclasses.dataclass(frozen=True)
class Arena:
    """The geometry every cell is run in — the part a user is expected to change.

    ``ring_radius`` is half the ring's diameter; ``r_inner`` is the measured disc and ``r_outer``
    the circle traffic is released on, far enough out that a pair starting close has room to
    resolve before it counts. Carried as a value rather than read from globals so it survives the
    trip into a worker process.
    """

    ring_radius: float = 500.0
    r_inner: float = 1000.0
    r_outer: float = 1200.0
    centre: tuple[float, float] = CENTRE


DEFAULT_ARENA = Arena()


def rules(dt: float, t_max: float) -> dict[str, Any]:
    """The separation stack, identical for both parts and both estimators."""
    return dict(
        rpz=RPZ, t_lookahead=LOOKAHEAD, dt=dt,
        detector=StateBased(), resolver=MVP(MARGIN), recovery=ProbabilisticFTR(),
        t_max=t_max, done_timeout=10.0, stop_within=50.0,
    )


# --- part 1: the ring ---------------------------------------------------------------------------
def _agents(fleet) -> list[Agent]:
    """Wrap a scenario in the airframe and guidance this study flies."""
    return [
        Agent(state, PERF, autopilot=WaypointAutopilot(Mission(goto=goal), capture_radius=30.0))
        for state, goal in fleet
    ]


def ring(n: int, arena: Arena) -> list[Agent]:
    """n multirotors on the ring, each aimed at the point diametrically opposite."""
    return _agents(crossing_ring(n, speed=SPEED, radius=arena.ring_radius,
                                 lat0=arena.centre[0], lon0=arena.centre[1],
                                 pos_ci95=POS_CI95, vel_ci95=VEL_CI95))


def ring_env(n: int, dt: float, arena: Arena) -> FleetEnv:
    return build_env(ring(n, arena), navigation=GnssNavigation(), **rules(dt, 600.0))


def ring_encounter(n: int, seq, dt: float, arena: Arena) -> tuple[bool, float]:
    """One ring encounter. The geometry is fixed, so the seed feeds the CNS noise alone."""
    agents = ring(n, arena)
    env = ring_env(n, dt, arena)
    state = env.initial_state(agents)
    streams = FleetStreams(cns=CnsStreams(nav=generator(seq)))
    while not env.is_terminal(state):
        state = env.advance(state, streams)
    return state.los, state.min_sep


def ring_cloud(n: int, dt: float, arena: Arena):
    """Every particle starts from the same world; the seed feeds the forward noise."""
    agents = ring(n, arena)
    env = ring_env(n, dt, arena)
    particle = Particle(env=env, state=env.initial_state(agents))

    def build_initial(seq):
        return particle
    return build_initial


# --- part 2: random traffic ---------------------------------------------------------------------
def sample_traffic(n: int, rng: np.random.Generator, arena: Arena) -> list[Agent]:
    """n drones crossing the measured disc, drawn by the entry rule (see opencdarr.scenario)."""
    return _agents(random_traffic(n, rng, speed=SPEED, r_inner=arena.r_inner,
                                  r_outer=arena.r_outer, lat0=arena.centre[0],
                                  lon0=arena.centre[1],
                                  pos_ci95=POS_CI95, vel_ci95=VEL_CI95))


def traffic_env(agents: list[Agent], dt: float, arena: Arena) -> FleetEnv:
    """The same rules as the ring, plus the disc that separation is measured inside."""
    return build_env(agents, navigation=GnssNavigation(),
                     measure_within=MeasurementArea(arena.centre, arena.r_inner),
                     **rules(dt, 400.0))


def traffic_encounter(n: int, seq, dt: float, arena: Arena) -> tuple[bool, float]:
    """One traffic encounter: the seed draws the geometry *and* the noise, on split streams."""
    geom, fwd = children(seq, 0, 2)
    agents = sample_traffic(n, generator(geom), arena)
    env = traffic_env(agents, dt, arena)
    state = env.initial_state(agents)
    streams = FleetStreams(cns=CnsStreams(nav=generator(fwd)))
    while not env.is_terminal(state):
        state = env.advance(state, streams)
    return state.los, state.min_sep


def traffic_cloud(n: int, dt: float, arena: Arena):
    """Each particle draws its own traffic — the distribution Monte Carlo integrates over."""
    def build_initial(seq):
        agents = sample_traffic(n, generator(seq), arena)
        env = traffic_env(agents, dt, arena)
        return Particle(env=env, state=env.initial_state(agents))
    return build_initial


# --- the campaign -------------------------------------------------------------------------------
CELLS = {
    1: dict(label="ring", encounter=ring_encounter, cloud=ring_cloud, step=0.5),
    2: dict(label="traffic", encounter=traffic_encounter, cloud=traffic_cloud, step=1.0),
}


def build_ladder(min_seps, rpz: float = RPZ, *, halving: float = 0.5,
                 min_count: int = 30, step: float = 0.5) -> list[float]:
    """Shells from the Monte Carlo record, then a fixed-step run-in below its resolvable floor."""
    ms = np.asarray([m for m in min_seps if np.isfinite(m)], dtype=float)
    shells: list[float] = []
    p, floor = halving, min_count / ms.size
    while p >= floor:
        d = float(np.percentile(ms, p * 100))
        if d > rpz and (not shells or d < shells[-1] - step):
            shells.append(d)
        p *= halving
    d = (shells[-1] if shells else float(ms.max())) - step
    while d > rpz:
        shells.append(d)
        d -= step
    shells.append(rpz)
    return [round(x, 2) for x in shells]


def run_mc(part: int, n: int, cfg: argparse.Namespace) -> dict[str, Any]:
    """Monte Carlo to ``--target-events`` events, in chunks, capped at ``--max-encounters``."""
    fn = CELLS[part]["encounter"]
    root = root_seed_sequence(cfg.seed + 1000 * part + n)
    los, min_seps, done = 0, [], 0
    t0 = time.perf_counter()
    while los < cfg.target_events and done < cfg.max_encounters:
        take = min(cfg.chunk, cfg.max_encounters - done)
        seqs = children(root, done, done + take)
        out = Parallel(n_jobs=cfg.jobs, batch_size=32)(
            delayed(fn)(n, s, cfg.dt, cfg.arena) for s in seqs)
        los += sum(1 for o in out if o[0])
        min_seps.extend(o[1] for o in out)
        done += take
        print(f"    [{CELLS[part]['label']} N={n}] {done:,} encounters, {los} events",
              flush=True)
    wall = time.perf_counter() - t0
    lo, hi = wilson_interval(los, done)
    return dict(encounters=done, events=los, p=los / done, ci=[lo, hi], wall_s=wall,
                short=los < cfg.target_events,
                rel_se=(1 / math.sqrt(los)) if los else None,
                min_sep=[float(m) for m in min_seps])


def run_ips(part: int, n: int, shells: list[float], particles: int,
            cfg: argparse.Namespace) -> dict[str, Any]:
    t0 = time.perf_counter()
    est = estimate_rare_prob(CELLS[part]["cloud"](n, cfg.dt, cfg.arena), shells,
                             n_particles=particles, reps=cfg.reps, seed=cfg.seed + 1,
                             n_jobs=cfg.jobs)
    good = [r for r in est.reps if r.collapsed_at is None]
    survival = ([sum(r.survival[k] for r in good) / len(good) for k in range(len(shells))]
                if good else [])
    return dict(p=est.prob, ci=list(est.ci), collapsed=est.n_collapsed, particles=particles,
                reps=cfg.reps, shells=shells, survival=survival,
                wall_s=time.perf_counter() - t0)


def calibrate(cfg: argparse.Namespace) -> None:
    """Time a few encounters per cell here, then price the campaign on this machine."""
    workers = resolve_jobs(cfg.jobs)
    print(f"calibrating on {workers} workers, {cfg.calibrate_n} encounters per cell, "
          f"dt = {cfg.dt}\n")
    print(f"{'cell':>14}  {'s/encounter':>11}  {'assumed p':>10}  {'encounters':>12}  "
          f"{'core-hours':>10}  {'wall':>9}")
    total = 0.0
    for part, sizes in ((1, cfg.ring_sizes), (2, cfg.traffic_sizes)):
        if part not in cfg.parts:
            continue
        fn = CELLS[part]["encounter"]
        for n in sizes:
            seqs = children(root_seed_sequence(99), 0, cfg.calibrate_n)
            t0 = time.perf_counter()
            for s in seqs:
                fn(n, s, cfg.dt, cfg.arena)
            per = (time.perf_counter() - t0) / cfg.calibrate_n
            p = cfg.assume_p.get((part, n))
            if p is None:
                print(f"{CELLS[part]['label'] + f' N={n}':>14}  {per:>11.4f}  "
                      f"{'unknown':>10}  {'-':>12}  {'-':>10}  {'-':>9}")
                continue
            need = math.ceil(cfg.target_events / p)
            core_h = need * per / 3600
            total += core_h
            wall = core_h / workers
            print(f"{CELLS[part]['label'] + f' N={n}':>14}  {per:>11.4f}  {p:>10.1e}  "
                  f"{need:>12,}  {core_h:>10.1f}  {wall * 60:>7.0f} m")
    print(f"\n  total {total:.0f} core-hours -> {total / workers:.1f} h wall on {workers} workers "
          f"({total / 100:.1f} h on 100)")
    print("  'assumed p' is the IPS estimate from the notebook; a lower true p costs "
          "proportionally more,\n  which is why the campaign runs to an event count rather "
          "than an encounter count.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", choices=("1", "2", "both"), default="both")
    ap.add_argument("--ring-sizes", type=int, nargs="+", default=[2, 3, 4])
    ap.add_argument("--traffic-sizes", type=int, nargs="+", default=[4, 6, 8])
    ap.add_argument("--target-events", type=int, default=100,
                    help="MC stops here; 25 = +-20%%, 100 = +-10%%, 400 = +-5%%")
    ap.add_argument("--max-encounters", type=int, default=5_000_000)
    ap.add_argument("--chunk", type=int, default=20_000)
    ap.add_argument("--particles", type=int, nargs="+", default=[2000],
                    help="per cell, or one value for all")
    ap.add_argument("--reps", type=int, default=20, help="IPS replications -> the interval")
    ap.add_argument("--ring-radius", type=float, default=500.0,
                    help="part 1: ring radius [m] -- half the ring diameter")
    ap.add_argument("--disc-radius", type=float, default=1000.0,
                    help="part 2: radius of the measured disc [m]")
    ap.add_argument("--release-radius", type=float, default=1200.0,
                    help="part 2: radius traffic is released on [m]; must exceed --disc-radius")
    ap.add_argument("--dt", type=float, default=0.5)
    ap.add_argument("--jobs", type=int, default=-1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--skip-mc", action="store_true")
    ap.add_argument("--skip-ips", action="store_true")
    ap.add_argument("--out", type=Path, default=None, help="write results as JSON")
    ap.add_argument("--calibrate", action="store_true", help="price the campaign and exit")
    ap.add_argument("--calibrate-n", type=int, default=20)
    cfg = ap.parse_args()
    cfg.parts = (1, 2) if cfg.part == "both" else (int(cfg.part),)
    cfg.arena = Arena(ring_radius=cfg.ring_radius, r_inner=cfg.disc_radius,
                      r_outer=cfg.release_radius)
    # the notebook's own IPS estimates, used only to price a calibration run
    cfg.assume_p = {(1, 2): 4.65e-5, (1, 3): 3.37e-4, (1, 4): 1.81e-3,
                    (2, 4): 3.90e-5, (2, 6): 1.17e-4, (2, 8): 3.59e-4}

    if cfg.calibrate:
        calibrate(cfg)
        return

    workers = resolve_jobs(cfg.jobs)
    print(f"{workers} workers, dt = {cfg.dt}, target {cfg.target_events} events, "
          f"{cfg.reps} replications, seed {cfg.seed}")
    print(f"arena: ring radius {cfg.arena.ring_radius:.0f} m, measured disc "
          f"{cfg.arena.r_inner:.0f} m, released at {cfg.arena.r_outer:.0f} m")
    settings = {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(cfg).items()
                if k not in ("assume_p", "arena")}
    results: dict[str, Any] = {"arena": dataclasses.asdict(cfg.arena), "settings": settings,
                               "workers": workers, "cells": []}
    t_start = time.perf_counter()

    for part, sizes in ((1, cfg.ring_sizes), (2, cfg.traffic_sizes)):
        if part not in cfg.parts:
            continue
        for k, n in enumerate(sizes):
            label = f"{CELLS[part]['label']} N={n}"
            print(f"\n=== {label} ===", flush=True)
            cell: dict[str, Any] = {"part": part, "n": n, "label": label}

            if not cfg.skip_mc:
                mc = run_mc(part, n, cfg)
                se = f"+-{mc['rel_se'] * 100:.0f}%" if mc["rel_se"] else "no events"
                print(f"  MC : {mc['events']}/{mc['encounters']:,}  p = {mc['p']:.3e}  "
                      f"[{mc['ci'][0]:.2e}, {mc['ci'][1]:.2e}]  {se}"
                      f"{'  SHORT of target' if mc['short'] else ''}  ({mc['wall_s']:.0f} s)",
                      flush=True)
                cell["mc"] = {kk: vv for kk, vv in mc.items() if kk != "min_sep"}
                record = mc["min_sep"]
            else:
                record = None

            if not cfg.skip_ips:
                if record is None:
                    raise SystemExit("--skip-mc leaves no record to build the ladder from")
                particles = (cfg.particles[k] if len(cfg.particles) > k else cfg.particles[-1])
                shells = build_ladder(record, step=CELLS[part]["step"])
                ips = run_ips(part, n, shells, particles, cfg)
                print(f"  IPS: p = {ips['p']:.3e}  [{ips['ci'][0]:.2e}, {ips['ci'][1]:.2e}]  "
                      f"collapsed {ips['collapsed']}/{cfg.reps}  {len(shells)} shells  "
                      f"({ips['wall_s']:.0f} s)", flush=True)
                cell["ips"] = ips
            results["cells"].append(cell)

    results["wall_s"] = time.perf_counter() - t_start
    print(f"\ntotal {results['wall_s'] / 60:.1f} min")
    print(f"\n{'cell':>14}  {'MC p':>11}  {'MC 95% CI':>24}  {'IPS p':>11}  {'IPS 95% CI':>24}")
    for cell in results["cells"]:
        mc, ips = cell.get("mc"), cell.get("ips")
        mc_s = f"{mc['p']:.3e}" if mc else "-"
        mc_ci = f"[{mc['ci'][0]:.2e}, {mc['ci'][1]:.2e}]" if mc else "-"
        ips_s = f"{ips['p']:.3e}" if ips else "-"
        ips_ci = f"[{ips['ci'][0]:.2e}, {ips['ci'][1]:.2e}]" if ips else "-"
        print(f"{cell['label']:>14}  {mc_s:>11}  {mc_ci:>24}  {ips_s:>11}  {ips_ci:>24}")

    if cfg.out:
        cfg.out.write_text(json.dumps(results, indent=1))
        print(f"\nwrote {cfg.out}")


if __name__ == "__main__":
    main()
