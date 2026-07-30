"""Pairwise Monte-Carlo environment: the engine behind the Pairwise-conflict handbook page.

Two waypoint scenarios (a 180 deg head-on; a 5 deg crossing), run through the full CNS stack
(position/velocity CI95 = 10 m / 1 m/s, reception 0.95, lognormal latency, broadcast jitter) for
the 2x2 of resolvers x recovery criteria ({MVP, VO} x {FTR, Past-CPA}), N noise realisations each.
Each run is a real n=2 fleet encounter (this file mirrors ``run_fleet``'s loop so jitter / comm /
nav / waypoints are the shipped models) and terminates when both aircraft reach their waypoints,
or at ``T_MAX``. It records the ground tracks + separation for the plots and the outcome
(conflict / LoS / min-sep) for P(LoS) and the CPA distribution.

``mc_plot.py`` imports :func:`run_all` from here and draws the figures. Run this file directly for
the timing + P(LoS)/CPA table only:

    PYTHONPATH=. python scripts/handbook/mc_pairwise.py [N]
"""

from __future__ import annotations

import hashlib
import math
import pickle
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from opencdarr import geo
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot, nominal_velocity
from opencdarr.cd import StateBased
from opencdarr.cns import CNS, CnsState, CnsStreams, Comm, GnssNavigation, lognormal_latency
from opencdarr.cns.broadcast import BroadcastSchedule
from opencdarr.cr import MVP, VO
from opencdarr.crr import FTR, PastCPA
from opencdarr.kinematics import FixedWing
from opencdarr.mission import Mission, Waypoint
from opencdarr.performance import SMALL_FIXEDWING
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.separation import INACTIVE, SeparationManager, project_to_fixedwing
from opencdarr.state import AircraftState, DesiredVelocity

LAT0, LON0 = 52.0, 4.0
SPEED, RPZ, LOOKAHEAD, DT = 17.0, 50.0, 120.0, 0.2  # fixed-wing cruise (stall 12, max 25)
POS_CI95, VEL_CI95 = 10.0, 1.0             # GNSS 95% radial accuracy, pos [m] / vel [m/s]
RECEPTION, LAT_MEDIAN, LAT_SIGMA = 0.95, 0.3, 0.4  # link: reception prob, lognormal latency [s]
JITTER = 0.2                                # broadcast slot dither U(-JITTER, +JITTER) [s]
CAPTURE = 30.0                             # m: fly-through tolerance = the pass-by radius
T_MAX = 600.0                              # s: hard cap so a run that never quite arrives ends
SEED = 20260726
CACHE_DIR = Path(__file__).resolve().parent / ".mc_cache"  # gitignore this; large pickled results
PERF = SMALL_FIXEDWING
_FW = FixedWing()
RESOLVERS = {"MVP": lambda: MVP(margin=1.05), "VO": lambda: VO(margin=1.05)}
RECOVERIES = {"FTR": lambda: FTR(), "Past-CPA": lambda: PastCPA(bouncing_guard=True)}


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _pass_ap(target: tuple[float, float], heading: float) -> WaypointAutopilot:
    """A pass-by autopilot: the waypoint (``target``) is a *fly-through* leg, with a dummy point
    2 km beyond along ``heading`` as the (never-reached) final. So the aircraft pursues the
    waypoint and passes it, rather than loitering it — a fixed-wing can't stop, and this keeps it
    flying straight through instead of orbiting. The run ends when it captures the waypoint (the
    leg advances), long before the dummy final."""
    beyond = geo.forward(target[0], target[1], heading, 2000.0)
    plan = Mission(flight_plan=(Waypoint(*target), Waypoint(*beyond)))
    return WaypointAutopilot(plan, cruise_airspeed=SPEED, capture_radius=CAPTURE,
                             loiter_radius=80.0)


def _fw(state: AircraftState) -> AircraftState:
    """Give a state the yaw/bank a fixed-wing needs (nose on track, wings level)."""
    return replace(state, yaw=state.trk, bank=0.0)


def scenario_headon():
    """Head-on: each aircraft's waypoint is the *other's* initial position, so they swap ends."""
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=SPEED,
                        pos_ci95=POS_CI95, vel_ci95=VEL_CI95)
    intr = create_conflict(own, intr_id="INT", dpsi=180.0, dcpa=0.0, tlos=60.0, rpz=RPZ, side=1)
    own_t, intr_t = (intr.lat, intr.lon), (own.lat, own.lon)
    return (_fw(own), _fw(intr), _pass_ap(own_t, own.trk), _pass_ap(intr_t, intr.trk),
            [_enu(*own_t), _enu(*intr_t)])


def scenario_crossing():
    """5 deg crossing: each waypoint is well past the crossing along its own heading (5 km), flown
    *through*. Placed far enough that the two destinations stay well apart, so we measure the
    crossing conflict, not a pile-up at the waypoints."""
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=SPEED,
                        pos_ci95=POS_CI95, vel_ci95=VEL_CI95)
    intr = create_conflict(own, intr_id="INT", dpsi=5.0, dcpa=0.0, tlos=180.0, rpz=RPZ, side=1)
    own_t = geo.forward(own.lat, own.lon, own.trk, 5000.0)
    intr_t = geo.forward(intr.lat, intr.lon, intr.trk, 5000.0)
    return (_fw(own), _fw(intr), _pass_ap(own_t, own.trk), _pass_ap(intr_t, intr.trk),
            [_enu(*own_t), _enu(*intr_t)])


def run_recorded(own0, intr0, own_ap, intr_ap, resolver, recovery,
                 nav_seq, comm_seq, bc_seq) -> dict:
    """One encounter with full CNS + jitter, recording tracks and separation (the n=2 fleet loop).
    Ends once both aircraft have flown *through* their waypoint, or at ``T_MAX`` (the cap)."""
    n = 2
    perfs = [PERF, PERF]
    aps = [own_ap, intr_ap]
    states = [replace(s, desired=DesiredVelocity.from_track_speed(s.trk, s.gs))
              for s in (own0, intr0)]
    gms = [GuidanceMemory(), GuidanceMemory()]
    mems = [INACTIVE, INACTIVE]
    sepmgr, det = SeparationManager(), StateBased()

    def adapt(cmd):  # a fixed-wing cannot fly a raw velocity: project it onto course/airspeed
        return project_to_fixedwing(cmd, PERF)

    # the whole datalink in one literal: noisy self-fixes, lossy delivery with a heavy-tailed
    # delay, intent kept private. The model config is immutable; each layer's generator rides in
    # the per-run CnsStreams, drawn from its own substream (ADR 0006 §6).
    cns = CNS(
        navigation=GnssNavigation(),
        communication=Comm(reception_prob=RECEPTION,
                           latency=lognormal_latency(LAT_MEDIAN, LAT_SIGMA)),
    )
    cns_streams = CnsStreams(nav=generator(nav_seq), comm=generator(comm_seq))
    schedule = BroadcastSchedule(interval=1.0, jitter=JITTER)
    bc_rng = generator(bc_seq)

    cmds = []
    for i in range(n):
        c, gms[i] = aps[i].step(states[i], gms[i], perfs[i])
        cmds.append(c)
    los = False
    min_sep = float("inf")
    t = 0.0
    next_bc = schedule.initial(n)
    cns_state = CnsState.initial(n, cns.communication)
    eps = 1e-9
    ts: list[float] = []
    tracks: list[list[tuple[float, float]]] = [[], []]
    seps: list[float] = []
    while t < T_MAX:
        for i in range(n):
            tracks[i].append(_enu(states[i].lat, states[i].lon))
        _, sep = geo.qdrdist(states[0].lat, states[0].lon, states[1].lat, states[1].lon)
        ts.append(t)
        seps.append(sep)
        min_sep = min(min_sep, sep)
        if sep < RPZ:
            los = True
        firing = schedule.due(next_bc, t, eps)
        if firing:
            cns_state, perception = cns.sense(states, firing, t, cns_state, cns_streams)
            for i in firing:
                see = perception[i]
                nom, gms[i] = aps[i].step(see.own, gms[i], perfs[i])
                # FTR's desired = the live nominal (bearing to the active waypoint), not a frozen
                # t=0 velocity; stamped for the decision and persisted on the true state (mirrors
                # run_fleet). share_intent is off here, so nothing of it goes on the air.
                self_i = replace(see.own, desired=nominal_velocity(nom, see.own))
                states[i] = replace(states[i], desired=self_i.desired)
                cmds[i], mems[i] = sepmgr.step(self_i, see.traffic, nom, mems[i], RPZ, LOOKAHEAD,
                                               det, resolver, recovery, adapt)
                next_bc[i] = schedule.advance(next_bc[i], bc_rng)
        states = [_FW.step(states[i], cmds[i], perfs[i], DT) for i in range(n)]
        t += DT
        if all(gm.leg_index >= 1 for gm in gms):  # both captured (flew through) their waypoint
            break
    return {"los": los, "min_sep": min_sep, "ts": ts, "tracks": tracks, "seps": seps,
            "t_end": t, "capped": t >= T_MAX}


def _mc(scenario_fn, resolver_name, recovery_name, n_runs, base_seed):
    own, intr, own_ap, intr_ap, wps = scenario_fn()
    runs = []
    for seq in spawn(root_seed_sequence(base_seed), n_runs):
        nav_seq, comm_seq, bc_seq = spawn(seq, 3)
        runs.append(run_recorded(own, intr, own_ap, intr_ap,
                                 RESOLVERS[resolver_name](), RECOVERIES[recovery_name](),
                                 nav_seq, comm_seq, bc_seq))
    return runs, wps


def summarize(runs) -> dict:
    los = np.array([r["los"] for r in runs])
    cpa = np.array([r["min_sep"] for r in runs])
    tend = np.array([r["t_end"] for r in runs])
    p_los = float(los.mean())
    return {"p_los": p_los, "ipr": 1.0 - p_los, "n_los": int(los.sum()),
            "cpa_med": float(np.median(cpa)), "cpa_p5": float(np.percentile(cpa, 5)),
            "cpa_min": float(cpa.min()), "tend_med": float(np.median(tend)),
            "n_capped": int(sum(r["capped"] for r in runs))}


def _config_sig() -> str:
    """A short fingerprint of the sim config, so changing a parameter invalidates the cache."""
    cfg = ("passby-fw-livenominal", SPEED, RPZ, LOOKAHEAD, DT, POS_CI95, VEL_CI95, RECEPTION,
           LAT_MEDIAN, LAT_SIGMA, JITTER, CAPTURE, T_MAX)
    return hashlib.md5(repr(cfg).encode()).hexdigest()[:8]


def _compute_all(n_runs: int, seed: int) -> dict:
    scenarios = {"headon": scenario_headon, "crossing": scenario_crossing}
    results = {}
    for sname, sfn in scenarios.items():
        for ri, rname in enumerate(RESOLVERS):
            for ci, cname in enumerate(RECOVERIES):
                t0 = time.perf_counter()
                runs, wps = _mc(sfn, rname, cname, n_runs, seed + ri * 10 + ci)
                results[(sname, rname, cname)] = (runs, wps, summarize(runs),
                                                  time.perf_counter() - t0)
    return results


def run_all(n_runs: int, seed: int = SEED, use_cache: bool = True) -> dict:
    """Run every (scenario, resolver, recovery) cell, ``n_runs`` each, and return
    ``{(scenario, resolver, recovery): (runs, waypoints, summary, seconds)}``.

    Results are cached to ``.mc_cache`` keyed by (n_runs, seed, config fingerprint), so a re-run
    with the same settings loads instantly and replotting never re-simulates. Pass
    ``use_cache=False`` to force a fresh run."""
    cache = CACHE_DIR / f"mc_n{n_runs}_s{seed}_{_config_sig()}.pkl"
    if use_cache and cache.exists():
        with cache.open("rb") as f:
            return pickle.load(f)
    results = _compute_all(n_runs, seed)
    if use_cache:
        CACHE_DIR.mkdir(exist_ok=True)
        with cache.open("wb") as f:
            pickle.dump(results, f)
    return results


def main() -> None:
    n_runs = int(sys.argv[1]) if len(sys.argv) > 1 else 250
    for (sname, rname, cname), (_, _, s, dt) in run_all(n_runs).items():
        print(f"{sname:8} {rname:3} x {cname:8}: {n_runs} runs in {dt:5.1f}s "
              f"({dt / n_runs * 1000:4.0f} ms/run)  P(LoS)={s['p_los']:.3f} IPR={s['ipr']:.3f}  "
              f"CPA med={s['cpa_med']:.0f} p5={s['cpa_p5']:.0f} min={s['cpa_min']:.0f}  "
              f"t_end~{s['tend_med']:.0f}s capped={s['n_capped']}")


if __name__ == "__main__":
    main()
