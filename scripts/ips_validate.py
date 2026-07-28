"""IPS validation gate: interacting-particle-system estimate vs brute-force MC (ADR 0017 §6).

Both estimators are derived from **one** :class:`Scenario`, so their parameters cannot drift: MC
via :func:`opencdarr.estimator.estimate_ipr`, IPS via :func:`opencdarr.ips.ips_once` over the same
``sample_pairwise`` geometry and :func:`opencdarr.fleet.build_env` rules. Reports both with 95% CI,
their wall-cost, and a verdict (do the intervals agree?).

Geometry is sampled crossing angles by default; ``--dpsi`` pins one fixed crossing (noise the only
variation), which both estimators then use identically. ``--reception``/``--latency`` add
communication uncertainty (dropped/late broadcasts) to *both* estimators through the one ``Comm``
model — so IPS's splitting acts on comms noise exactly as MC samples it.

- **Correctness** — ``--pos 40`` (P≈0.028): the IPS CI must overlap the MC CI. Proves unbiasedness.
- **Efficiency** — ``--pos 10`` (P≈5e-4): IPS returns a tight CI where MC's explodes / reads 0.
- **CNS-only** — ``--pos 0 --reception 0.7``: perfect nav, so message drops are the *sole* driver
  of loss of separation. This is the discrete-jump regime of ``important-ips-gap`` — the test of
  whether ``min_sep``-based splitting still tracks MC when the rare event is Bernoulli-caused.

    python scripts/ips_validate.py --pos 40 --mc-n 4000 --particles 300 --reps 8 --jobs 8
    python scripts/ips_validate.py --pos 10 --mc-n 30000 --particles 400 --reps 8 \
        --levels 90 75 65 58 54 52 51 50 --jobs 8
    # fixed 90° crossing, lookahead 60 (MC reads 0 at 30k; IPS estimates ~1e-4):
    python scripts/ips_validate.py --pos 10 --dpsi 90 --lookahead 60 --mc-n 30000 \
        --particles 2000 --reps 8 --levels 150 100 75 66 61 58 56 54 52 51 50 --jobs 8
    # CNS-only rung: perfect nav, drops drive LoS (reception/levels need tuning to the drop regime):
    python scripts/ips_validate.py --pos 0 --dpsi 90 --reception 0.7 --lookahead 60 --mc-n 30000 \
        --particles 2000 --reps 8 --levels 150 100 75 66 61 58 56 54 52 51 50 --jobs 8
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np
from joblib import Parallel, delayed

from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GnssNavigation
from opencdarr.cns.base import CommunicationModel
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.estimator import estimate_ipr
from opencdarr.fleet import Agent, build_env
from opencdarr.ips import Particle, combine_replications, ips_once, replication_seeds
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict, sample_pairwise
from opencdarr.state import AircraftState


@dataclass(frozen=True)
class Scenario:
    """One encounter distribution + CDR config, the single source both estimators derive from.

    ``dpsi=None`` samples crossing angles (``sample_pairwise``, the ladder distribution); a fixed
    ``dpsi`` pins one crossing geometry (``create_conflict``) whose only randomness is the GNSS
    noise — the noise-driven rare event on a single geometry (e.g. a 90° crossing).
    """

    pos_ci95: float
    vel_ci95: float
    dpsi: float | None = None  # None = sampled angles; a value = one fixed crossing geometry
    speed: float = 10.2889
    dcpa_max: float = 0.0
    tlos: float = 90.0
    rpz: float = 50.0
    lookahead: float = 120.0
    margin: float = 1.05
    dt: float = 0.5
    t_max: float = 250.0
    done_timeout: float = 10.0
    reception_prob: float = 1.0  # P(a broadcast reaches a receiver); < 1 => drops drive staleness
    latency: float = 0.0  # constant link delay [s]; a delivered message is this many seconds stale

    def comm(self) -> CommunicationModel | None:
        """The communication model shared by both estimators, or ``None`` when there is no CNS
        uncertainty (``reception_prob == 1`` and ``latency == 0``) — then the comm substream is
        reserved but never drawn, so the nav-noise gates behave exactly as before."""
        if self.reception_prob >= 1.0 and self.latency == 0.0:
            return None
        return Comm(reception_prob=self.reception_prob, latency=self.latency)

    def _env(self, agents: list[Agent]) -> object:
        return build_env(
            agents, rpz=self.rpz, t_lookahead=self.lookahead, dt=self.dt, detector=StateBased(),
            resolver=MVP(margin=self.margin), recovery=PastCPA(bouncing_guard=True),
            navigation=GnssNavigation(), communication=self.comm(),
            t_max=self.t_max, done_timeout=self.done_timeout,
        )

    def fixed_pair(self) -> tuple[AircraftState, AircraftState]:
        """The single fixed (own, intruder) pair for a pinned ``dpsi`` — noise the only spread."""
        own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=self.speed,
                            pos_ci95=self.pos_ci95, vel_ci95=self.vel_ci95)
        intr = create_conflict(own, intr_id="INT", dpsi=self.dpsi, dcpa=0.0, tlos=self.tlos,
                               rpz=self.rpz, side=1)
        return own, intr

    def mc_config(self, n: int, seed: int) -> Config:
        """The plain-MC config (``estimate_ipr``) for the sampled-geometry case (``dpsi=None``)."""
        return Config(
            seed=seed, n_encounters=n,
            scenario=ScenarioConfig(
                aircraft_type="M600", speed=self.speed, dcpa_max=self.dcpa_max,
                tlos=self.tlos, pos_ci95=self.pos_ci95, vel_ci95=self.vel_ci95),
            conflict=ConflictConfig(rpz=self.rpz, t_lookahead=self.lookahead),
            methods=MethodsConfig(detection="statebased", resolution="mvp", recovery="pastcpa",
                                  margin=self.margin, bouncing_guard=True),
            simulation=SimulationConfig(dt=self.dt, t_max=self.t_max,
                                        done_timeout=self.done_timeout),
        )

    def build_initial(self, seq: np.random.SeedSequence) -> Particle:
        """One IPS particle: the same geometry the MC path uses — sampled, or the fixed pair."""
        if self.dpsi is None:
            own, intr = sample_pairwise(
                generator(seq), speed=self.speed, dcpa_max=self.dcpa_max, tlos=self.tlos,
                rpz=self.rpz, pos_ci95=self.pos_ci95, vel_ci95=self.vel_ci95,
            )
        else:
            own, intr = self.fixed_pair()  # geometry fixed; the seed feeds forward noise, not it
        agents = [Agent(own, M600), Agent(intr, M600)]
        env = self._env(agents)
        return Particle(env=env, state=env.initial_state(agents))


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    if n == 0:
        return (float("nan"), 0.0, 1.0)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (p, max(0.0, centre - half), min(1.0, centre + half))


def _mc_chunk(scn: Scenario, n: int, seed: int) -> tuple[int, int]:
    """One MC chunk -> (n_conflict, n_los). Sampled geometry goes through ``estimate_ipr``; a fixed
    ``dpsi`` runs ``n`` encounters over the one pair, varying only the GNSS noise stream."""
    if scn.dpsi is None:
        # sampled geometry: estimate_ipr conditions on a detected conflict (its IPR denominator).
        # That coincides with the unconditional P(LoS) only when lookahead >= tlos (detection at
        # t=0), which the default satisfies; IPS's denominator is all N particles either way.
        r = estimate_ipr(scn.mc_config(n, seed), M600, StateBased(), MVP(margin=scn.margin),
                         PastCPA(bouncing_guard=True), navigation=GnssNavigation(),
                         communication=scn.comm())
        return r.n_conflict, r.n_los
    # fixed geometry: P(LoS) is unconditional over all n encounters (denominator n, not
    # n_conflict), to match IPS's "reachers / N". With lookahead < tlos, noise can let the resolver
    # deflect early so a true conflict never registers (n_conflict < n) — conditioning would drift.
    own, intr = scn.fixed_pair()
    comm = scn.comm()
    n_los = 0
    for s in spawn(root_seed_sequence(seed), n):
        # nav + comm substreams per encounter, mirroring estimate_ipr / the IPS particle layout, so
        # the stream tree stays config-invariant whether or not comms uncertainty is switched on.
        nav_seq, comm_seq = spawn(s, 2)
        out = run_encounter(
            own, intr, perf=M600, rpz=scn.rpz, t_lookahead=scn.lookahead, dt=scn.dt,
            detector=StateBased(), resolver=MVP(margin=scn.margin),
            recovery=PastCPA(bouncing_guard=True), navigation=GnssNavigation(),
            rng=generator(nav_seq), communication=comm, comm_rng=generator(comm_seq),
            t_max=scn.t_max, done_timeout=scn.done_timeout,
        )
        n_los += int(out.los)
    return n, n_los


def mc_estimate(scn: Scenario, n: int, seed: int, jobs: int) -> tuple[float, float, float, int]:
    """Pooled plain-MC P(LoS) over ``n`` encounters, split into ``jobs`` independent chunks."""
    chunks = [n // jobs + (1 if i < n % jobs else 0) for i in range(jobs)]
    out = Parallel(n_jobs=jobs)(
        delayed(_mc_chunk)(scn, c, seed + i) for i, c in enumerate(chunks) if c > 0
    )
    n_conf = sum(a for a, _ in out)
    n_los = sum(b for _, b in out)
    p, lo, hi = wilson(n_los, n_conf)
    return p, lo, hi, n_conf


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pos", type=float, default=40.0, help="pos_ci95 [m] (vel_ci95 = pos*ratio)")
    p.add_argument("--vel-ratio", dest="vel_ratio", type=float, default=0.1)
    p.add_argument("--vel", type=float, default=None, help="vel_ci95 [m/s]; overrides --vel-ratio "
                   "when set (velocity noise independent of --pos, incl. pos=0)")
    p.add_argument("--mc-n", dest="mc_n", type=int, default=4000, help="MC encounters")
    p.add_argument("--particles", type=int, default=300, help="IPS particles per level")
    p.add_argument("--reps", type=int, default=8, help="IPS independent replications")
    p.add_argument("--levels", type=float, nargs="+",
                   default=[70, 60, 55, 52, 51, 50], help="shell distances [m], ending at rpz")
    p.add_argument("--dt", type=float, default=0.5, help="integration step [s] (finer = less "
                   "shell overshoot; threaded into BOTH estimators)")
    p.add_argument("--dpsi", type=float, default=None, help="fix the crossing angle [deg] to one "
                   "geometry (default: sample angles); noise is then the only variation")
    p.add_argument("--lookahead", type=float, default=120.0, help="detection lookahead [s]")
    p.add_argument("--tlos", type=float, default=90.0, help="time to loss of separation [s]")
    p.add_argument("--reception", type=float, default=1.0, help="P(broadcast received) per link; "
                   "< 1 makes dropped/stale messages a driver of LoS (default: 1.0, perfect comms)")
    p.add_argument("--latency", type=float, default=0.0, help="constant link delay [s] (default: 0)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--jobs", type=int, default=1)
    a = p.parse_args()
    vel_ci95 = a.vel if a.vel is not None else a.pos * a.vel_ratio
    scn = Scenario(pos_ci95=a.pos, vel_ci95=vel_ci95, dpsi=a.dpsi, dt=a.dt,
                   lookahead=a.lookahead, tlos=a.tlos,
                   reception_prob=a.reception, latency=a.latency)

    geom = "sampled angles" if a.dpsi is None else f"fixed {a.dpsi:.0f}deg crossing"
    cns = "GNSS" if scn.comm() is None else f"GNSS+comms(rx={a.reception}, lat={a.latency}s)"
    print(f"scenario: {geom}, pos_ci95={a.pos} vel_ci95={vel_ci95} rpz={scn.rpz} "
          f"lookahead={scn.lookahead} margin={scn.margin} dt={scn.dt}  (MVP + Past-CPA, {cns})")
    if a.pos == 0.0 and vel_ci95 == 0.0 and a.dpsi is not None and scn.comm() is None:
        print("  WARNING: no nav noise (pos & vel = 0) on a fixed geometry with perfect comms -> "
              "degenerate estimate. Set --pos/--vel > 0, or --reception < 1.")
    if a.dpsi is None and scn.lookahead < scn.tlos:
        print("  WARNING: lookahead < tlos in sampled mode -> MC conditions on n_conflict, which "
              "drifts below IPS's all-N denominator. Prefer --dpsi, or set lookahead >= tlos.")

    t0 = time.perf_counter()
    mc_p, mc_lo, mc_hi, mc_n = mc_estimate(scn, a.mc_n, a.seed, a.jobs)
    t_mc = time.perf_counter() - t0
    print(f"\nMC   n={mc_n:>7}  P(LoS)={mc_p:.5f}  95%CI[{mc_lo:.5f}, {mc_hi:.5f}]  ({t_mc:.0f}s)")

    t0 = time.perf_counter()
    seeds = replication_seeds(a.seed, a.reps)
    results = Parallel(n_jobs=a.jobs)(
        delayed(ips_once)(scn.build_initial, a.levels, a.particles, s) for s in seeds
    )
    est = combine_replications(results)
    t_ips = time.perf_counter() - t0
    budget = a.particles * len(a.levels) * a.reps
    print(f"IPS  {a.reps}x{a.particles}p x{len(a.levels)} shells (~{budget} seg)  "
          f"P={est.prob:.6f}  95%CI[{est.ci[0]:.6f}, {est.ci[1]:.6f}]  "
          f"collapsed={est.n_collapsed}/{a.reps}  ({t_ips:.0f}s)")
    # mean survival per shell over non-collapsed reps — to tune shell spacing toward equal drops
    good = [r for r in est.reps if r.collapsed_at is None]
    if good:
        per = [sum(r.survival[k] for r in good) / len(good) for k in range(len(a.levels))]
        cells = "  ".join(f"{d:.0f}:{s:.3f}" for d, s in zip(a.levels, per, strict=True))
        print("  survival/shell: " + cells)

    # verdict: do the intervals overlap?
    agree = est.ci[0] <= mc_hi and mc_lo <= est.ci[1]
    within = mc_lo <= est.prob <= mc_hi
    print(f"\nverdict: CIs {'OVERLAP' if agree else 'DISJOINT'}; "
          f"IPS mean {'within' if within else 'OUTSIDE'} MC CI  "
          f"-> {'PASS' if agree else 'FAIL'}")


if __name__ == "__main__":
    main()
