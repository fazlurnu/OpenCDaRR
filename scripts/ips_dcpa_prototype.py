"""PROTOTYPE (important-ips-gap): fixed-effort splitting on a dcpa-deficit importance function.

The production estimator (``opencdarr.ips``) splits on running-minimum separation ``min_sep``. That
is a *lagging* signal — it only moves near CPA — so it cannot stratify comms-driven (dropped-message)
rare events, where the failure builds long before separation reacts (see the trajectory probe in
``vault/observations/important-ips-gap.md``). This prototype splits instead on **how far behind the
nominal resolution a particle has fallen**, measured through the *predicted* miss distance (dcpa):

    imp(state) = dcpa_nominal(t) - dcpa_true(state)

``dcpa_nominal(t)`` is the perfect-comms, noise-free resolution trajectory (deterministic), which
climbs 0 -> ~rpz*margin as the resolver builds miss distance. Both nominal and a healthy particle
climb together, so ``imp ~ 0`` for the safe majority; a drop-starved particle stays on the collision
course (dcpa ~ 0) while nominal climbs, so its ``imp`` grows — monotone toward the rare event. Every
particle starts at ``imp = 0`` (nominal and actual both ~0 on the collision course), which fixes the
polarity problem raw dcpa has.

Fixed-effort ladder over increasing deficit shells, then a final conditional pass to termination
that counts actual loss of separation among the deep-deficit survivors:

    P(LoS) = [prod_k survival_k over deficit shells] * P(LoS | reached deepest shell)

Validated two ways:
  * nav-noise benchmark (``--pos 40``, perfect comms): must match plain MC, proving the dcpa
    importance also works where min_sep already did (unification).
  * comms benchmark (``--pos 0 --reception 0.03``): must match MC where min_sep-splitting collapsed.

    python scripts/ips_dcpa_prototype.py --pos 0 --reception 0.03 --mc-n 4000 --particles 400
    python scripts/ips_dcpa_prototype.py --pos 40 --mc-n 4000 --particles 400   # nav-noise gate
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
from joblib import Parallel, delayed

from opencdarr.fleet import FleetState
from opencdarr.ips import Particle, _streams
from opencdarr.relative import relative_enu
from opencdarr.rng import generator, root_seed_sequence, spawn
from scripts.ips_validate import Scenario, mc_estimate


def dcpa_true(state: FleetState) -> float:
    """Predicted closest-approach distance of the two aircraft from their true current states [m]."""
    own, intr = state.states[0], state.states[1]
    rel = relative_enu(own, intr)
    v2 = rel.vx * rel.vx + rel.vy * rel.vy
    if v2 <= 0.0:
        return rel.dist
    t_cpa = -(rel.rx * rel.vx + rel.ry * rel.vy) / v2
    if t_cpa < 0.0:
        return rel.dist  # past CPA: current range, not the (behind-us) linear prediction
    return math.hypot(rel.rx + rel.vx * t_cpa, rel.ry + rel.vy * t_cpa)


def _accumulate_until(particle: Particle, short0: float, target: float, dcpa_target: float,
                      dt: float, streams) -> tuple[FleetState, float]:
    """Advance one particle, accumulating dcpa *shortfall* ``max(0, dcpa_target - dcpa)*dt`` each
    step, until the cumulative shortfall reaches ``target`` [m·s] or the encounter terminates."""
    env, state = particle.env, particle.state
    short = short0
    while short < target and not env.is_terminal(state):
        state = env.advance(state, streams)
        short += max(0.0, dcpa_target - dcpa_true(state)) * dt  # shortfall floors at 0 above target
    return state, short


def dcpa_ips_once(scn: Scenario, dcpa_target: float, shells: list[float],
                  n_particles: int, seq: np.random.SeedSequence) -> dict:
    """One fixed-effort run over cumulative-shortfall ``shells`` (increasing [m·s]) + a final
    LoS-conditional pass. The importance ``∫ max(0, dcpa_target - dcpa) dt`` grows monotonically the
    whole time a particle sits below ``dcpa_target`` — so it ladders over the blind window instead of
    saturating, and it is a *leading* signal (dcpa) rather than a lagging one (min_sep)."""
    init_seq, evolve_seq = seq.spawn(2)
    entities = [((p := scn.build_initial(s)).env, p.state, 0.0) for s in init_seq.spawn(n_particles)]
    level_seqs = evolve_seq.spawn(len(shells) + 1)

    survival: list[float] = []
    for k, target in enumerate(shells):
        sub = level_seqs[k].spawn(n_particles + 1)
        evolved = []
        for (env, state, short), s in zip(entities, sub[:n_particles], strict=True):
            st, sh = _accumulate_until(Particle(env=env, state=state), short, target,
                                       dcpa_target, scn.dt, _streams(s))
            evolved.append((env, st, sh))
        survivors = [e for e in evolved if e[2] >= target]
        survival.append(len(survivors) / n_particles)
        if not survivors:
            return {"prob": 0.0, "survival": survival, "collapsed_at": k}
        idx = generator(sub[n_particles]).integers(0, len(survivors), size=n_particles)
        entities = [survivors[i] for i in idx]

    # final level: run deep-shortfall survivors to termination, count actual LoS
    sub = level_seqs[-1].spawn(n_particles)
    los = 0
    for (env, state, _), s in zip(entities, sub, strict=True):
        st, _ = _accumulate_until(Particle(env=env, state=state), 0.0, math.inf,
                                  dcpa_target, scn.dt, _streams(s))
        los += int(st.los)
    survival.append(los / n_particles)
    return {"prob": float(np.prod(survival)), "survival": survival, "collapsed_at": None}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pos", type=float, default=0.0)
    p.add_argument("--vel-ratio", dest="vel_ratio", type=float, default=0.1)
    p.add_argument("--reception", type=float, default=0.03)
    p.add_argument("--dpsi", type=float, default=90.0)
    p.add_argument("--tlos", type=float, default=65.0)
    p.add_argument("--lookahead", type=float, default=60.0)
    p.add_argument("--mc-n", dest="mc_n", type=int, default=4000)
    p.add_argument("--particles", type=int, default=400)
    p.add_argument("--reps", type=int, default=8)
    p.add_argument("--target-dcpa", dest="target_dcpa", type=float, default=50.0,
                   help="dcpa a resolved encounter should clear [m] (shortfall is measured below it)")
    p.add_argument("--shells", type=float, nargs="+",
                   default=[900, 1300, 1700, 2100, 2500],
                   help="increasing cumulative dcpa-shortfall thresholds [m·s]")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--jobs", type=int, default=8)
    a = p.parse_args()

    scn = Scenario(pos_ci95=a.pos, vel_ci95=a.pos * a.vel_ratio, dpsi=a.dpsi, tlos=a.tlos,
                   lookahead=a.lookahead, reception_prob=a.reception)
    cns = "perfect comms" if scn.comm() is None else f"rx={a.reception}"
    print(f"scenario: fixed {a.dpsi:.0f}deg, pos_ci95={a.pos} tlos={a.tlos} lookahead={a.lookahead} "
          f"rpz={scn.rpz}  ({cns})")

    t0 = time.perf_counter()
    mc_p, mc_lo, mc_hi, mc_n = mc_estimate(scn, a.mc_n, a.seed, a.jobs)
    print(f"\nMC        n={mc_n:>6}  P(LoS)={mc_p:.5f}  95%CI[{mc_lo:.5f}, {mc_hi:.5f}]  "
          f"({time.perf_counter()-t0:.0f}s)")

    t0 = time.perf_counter()
    seeds = spawn(root_seed_sequence(a.seed), a.reps)
    res = Parallel(n_jobs=a.jobs)(
        delayed(dcpa_ips_once)(scn, a.target_dcpa, a.shells, a.particles, s) for s in seeds)
    probs = [r["prob"] for r in res]
    n_coll = sum(1 for r in res if r["collapsed_at"] is not None)
    mean = float(np.mean(probs))
    pos = [x for x in probs if x > 0]
    if len(pos) >= 2 and len(pos) == len(probs):
        logs = np.log(pos)
        se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
        ci = (math.exp(logs.mean() - 1.96 * se), math.exp(logs.mean() + 1.96 * se))
    else:
        ci = (min(probs), max(probs))
    print(f"dcpa-IPS  {a.reps}x{a.particles}p x{len(a.shells)} shells  P={mean:.5f}  "
          f"95%CI[{ci[0]:.5f}, {ci[1]:.5f}]  collapsed={n_coll}/{a.reps}  "
          f"({time.perf_counter()-t0:.0f}s)")
    good = [r for r in res if r["collapsed_at"] is None]
    if good:
        per = [sum(r["survival"][k] for r in good) / len(good) for k in range(len(a.shells) + 1)]
        names = [f"{d:.0f}" for d in a.shells] + ["LoS"]
        print("  survival/shell: " + "  ".join(f"{n}:{s:.3f}" for n, s in zip(names, per)))

    agree = ci[0] <= mc_hi and mc_lo <= ci[1]
    print(f"\nverdict: CIs {'OVERLAP' if agree else 'DISJOINT'} -> {'PASS' if agree else 'FAIL'}")


if __name__ == "__main__":
    main()
