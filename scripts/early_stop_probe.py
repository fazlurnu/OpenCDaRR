"""Is the fleet's all-clear instant absorbing — do K, A or min_sep ever move after it?

The measurement behind TODO-ips-performance.md item 2b (2026-08-07). An early stop for the IPS
tail (and possibly the level legs) would end a flight once ``done_timer > 0`` — every pair
diverging and separated, nobody resolving — instead of waiting out ``done_timeout`` or ``t_max``.
Under noisy CDR a resolver can react to a phantom conflict and re-converge a diverging pair, so
the predicate is not absorbing by construction; this measures how often it fails in practice.

For each encounter: record the state at the first instant ``done_timer`` crosses each threshold,
fly on to the true terminal, and compare K, A and the running minimum separation. Run per case::

    PYTHONPATH=. python scripts/early_stop_probe.py --case ring8 --n 300 --out ring8.json

Findings (pos_ci95 = 30 m, seed 7; 300 encounters/case, 200 at n=15): K and A never moved after
a single clear step in any case — but not because dense traffic is benign. The predicate is
*global*, and the density sweep (random traffic, n = 5/8/10/15 on the 900 m disc) shows it
fires in 69 % / 14 % / 4 % / 0 % of encounters: while one aircraft is between its encounters,
some other pair is still converging, so the fleet-level clear never arrives — at n = 15 every
encounter dies at t_max without one. Where it can fire, firing means the wave has dispersed,
and in this encounter-based design (one wave through a disc, no arrivals) there is nobody left
to meet. min_sep is still not absorbing (n=5: a 24 m dip after five seconds of sustained
clear), and the expensive fleets present no stop to take — declined on those numbers. The
absorbing reading is a property of the encounter-based scenario design: in continuous traffic
(arrivals, stationary density) a global clear would be followed by fresh conflicts, and none of
this transfers.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts" / "validation"))

from campaign import base_config, methods_for  # noqa: E402

from opencdarr.cns.broadcast import schedule_for  # noqa: E402
from opencdarr.estimate.ips import Particle, _streams  # noqa: E402
from opencdarr.experiment.cell import _agents_from  # noqa: E402
from opencdarr.fleet import build_env  # noqa: E402
from opencdarr.rng import children, generator, root_seed_sequence, spawn  # noqa: E402
from opencdarr.scenario import CrossingRing, RandomTraffic  # noqa: E402

POS_CI95 = 30.0
THRESHOLDS = (0.5, 2.0, 5.0)  # seconds of sustained clear; 0.5 = one clear step at dt=0.5

CASES = {
    "ring6": lambda: CrossingRing(n=6, radius=1500.0),
    "ring8": lambda: CrossingRing(n=8, radius=1500.0),
    "random2": lambda: RandomTraffic(density=2.0, radius=900.0),
    # the density sweep: in denser traffic an aircraft can clear one encounter and then meet
    # another, so K/A growth after a clear instant should appear as n rises — the probe's
    # absorbing-in-practice reading at 5 aircraft is only as strong as the traffic is thin
    "random_n8": lambda: RandomTraffic(n=8, radius=900.0),
    "random_n10": lambda: RandomTraffic(n=10, radius=900.0),
    "random_n15": lambda: RandomTraffic(n=15, radius=900.0),
}


def _build_initial(scenario, m, cfg):
    """One particle from one seed — the campaign's own construction (experiment/cell.py)."""
    def build(seq):
        geom_rng = generator(seq)
        agents = _agents_from(scenario.draw(geom_rng, cfg), m)
        env = build_env(
            agents, rpz=cfg.conflict.rpz, t_lookahead=cfg.conflict.t_lookahead,
            dt=cfg.simulation.dt, detector=m.detector, resolver=m.resolver, recovery=m.recovery,
            navigation=m.navigation, communication=m.communication, surveillance=m.surveillance,
            t_max=cfg.simulation.t_max, done_timeout=cfg.simulation.done_timeout,
            wind=m.wind, area=scenario.measurement_area(),
            schedule=schedule_for(
                len(agents), cfg.simulation.broadcast_interval, geom_rng,
                jitter=cfg.simulation.broadcast_jitter,
                random_phase=cfg.simulation.broadcast_random_phase,
            ),
        )
        return Particle(env=env, state=env.initial_state(agents))
    return build


def run_case(name: str, n_encounters: int, seed: int) -> dict:
    scenario = CASES[name]()
    m = methods_for(scenario)
    cfg = base_config(n_encounters, POS_CI95)
    build = _build_initial(scenario, m, cfg)

    records = []
    t0 = time.perf_counter()
    for enc_seq in spawn(root_seed_sequence(seed), n_encounters):
        geom_seq, fwd_seq = children(enc_seq, 0, 2)
        particle = build(geom_seq)
        env, state = particle.env, particle.state
        streams = _streams(fwd_seq)

        first: dict[float, dict] = {}
        flickers = 0
        was_clear = False
        while not env.is_terminal(state):
            state = env.advance(state, streams)
            clear = state.done_timer > 0.0
            if was_clear and not clear:
                flickers += 1
            was_clear = clear
            for thr in THRESHOLDS:
                if thr not in first and state.done_timer >= thr:
                    first[thr] = {"t": state.t, "k": state.n_los_pairs,
                                  "a": state.n_los_aircraft, "min_sep": state.min_sep}

        records.append({
            "t_term": state.t, "k_term": state.n_los_pairs, "a_term": state.n_los_aircraft,
            "min_sep_term": state.min_sep, "los": state.los, "flickers": flickers,
            "first": {str(thr): first.get(thr) for thr in THRESHOLDS},
        })

    seconds = time.perf_counter() - t0
    out = {"case": name, "pos_ci95": POS_CI95, "n_encounters": n_encounters,
           "n_aircraft": scenario.size(), "probe_seconds": round(seconds, 1),
           "n_los": sum(1 for r in records if r["los"]),
           "mean_t_term": round(sum(r["t_term"] for r in records) / len(records), 1),
           "mean_flickers": round(sum(r["flickers"] for r in records) / len(records), 2),
           "thresholds": {}}

    for thr in THRESHOLDS:
        key = str(thr)
        reached = [r for r in records if r["first"][key] is not None]
        grew_k = [r for r in reached if r["k_term"] > r["first"][key]["k"]]
        grew_a = [r for r in reached if r["a_term"] > r["first"][key]["a"]]
        dipped = [r for r in reached
                  if r["min_sep_term"] < r["first"][key]["min_sep"] - 1e-9]
        dipped_1m = [r for r in reached
                     if r["min_sep_term"] < r["first"][key]["min_sep"] - 1.0]
        new_breach = [r for r in reached if r["los"] and r["first"][key]["k"] == 0]
        out["thresholds"][key] = {
            "n_reached": len(reached),
            "n_never": len(records) - len(reached),
            "grew_k": len(grew_k),
            "grew_a": len(grew_a),
            "k_growth_amounts": sorted(r["k_term"] - r["first"][key]["k"] for r in grew_k),
            "minsep_dipped": len(dipped),
            "minsep_dipped_1m": len(dipped_1m),
            "max_dip_m": round(max((r["first"][key]["min_sep"] - r["min_sep_term"]
                                    for r in dipped), default=0.0), 2),
            "breach_entirely_after_clear": len(new_breach),
            "mean_saved_s": round(sum(r["t_term"] - r["first"][key]["t"]
                                      for r in reached) / len(reached), 1) if reached else None,
        }
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--case", required=True, choices=list(CASES))
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    result = run_case(args.case, args.n, args.seed)
    pathlib.Path(args.out).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
