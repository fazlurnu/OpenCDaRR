"""Rare-event IPS for the fleet sim, on the **look-ahead conflict coordinate** (Blom eq. 10.7).

This is the default rare-event simulator for our free-flight collision case. The importance function
is the *predicted-conflict distance* (dcpa) nested on a **shrinking look-ahead horizon** — Blom,
Krystul & Bakker, *Free Flight Collision Risk Estimation by Sequential Monte Carlo* (NLR-TP-2006-288),
§10.2.2, horizontal part of eq. 10.7:

    predicted separation at look-ahead tau:  s(tau) = |r + tau*v|   (r, v = relative pos/vel)
    smin_k = min over tau in [0, tau_k] of s(tau)          (closest predicted approach in the window)
    level D_k = { smin_k <= d_k },   k = 1..m
      d_1 >= ... >= d_m = rpz     (miss threshold shrinks to the protected zone)
      tau_1 >= ... >= tau_m = 0   (look-ahead shrinks to "now")
    phi = (deepest level reached) / m,   phi = 1  <=>  loss of separation

Because a smaller window *and* a tighter miss give a subset, D_1 ⊃ ... ⊃ D_m, so a particle crosses
the levels in order. Escaping a level means the resolver grew the predicted miss — which costs a
received message — so the per-level survival silently prices in the drop probability. That is why one
geometric ladder handles both nav drift and the comms blackout, where the earlier
``max(nav_progress, cap*comm_progress)`` collapsed 5/6 on comms. Validated 3/3 against MC on the real
``FleetEnv`` sim (nav 3.1e-4 vs 2.7e-4, comms 1.23e-3 vs 1.21e-3, both 2.64e-2 vs 2.68e-2). See
``vault/observations/lookahead-conflict-coordinate.md``.

Both MC and IPS run the same ``FleetEnv.advance`` for an apples-to-apples comparison; no core-sim
change (the coordinate is a read-only function of the true relative geometry).

    python scripts/ips_unified_validate.py --preset smoke --jobs -1   # quick machinery check
    python scripts/ips_unified_validate.py --preset rare  --jobs -1   # rare-event benchmark
    # crank on a many-core box (reps >= jobs saturates IPS); push toward 1e-6:
    python scripts/ips_unified_validate.py --preset rare --jobs -1 --reps 120 \
        --pos 14 --rx 0.10 --particles 4000 --mc-n 30000000
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, replace

import numpy as np
from joblib import Parallel, delayed

from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GnssNavigation
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, CnsStreams, FleetEnv, FleetState, FleetStreams, build_env
from opencdarr.performance import M600
from opencdarr.relative import relative_enu
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

Regime = str  # "nav" | "comms" | "both"
REGIMES: tuple[Regime, ...] = ("nav", "comms", "both")


def _streams(seq: np.random.SeedSequence) -> FleetStreams:
    """Mirror of :func:`opencdarr.ips._streams`: three substreams (nav, comm, broadcast)."""
    nav, comm, bc = seq.spawn(3)
    return FleetStreams(cns=CnsStreams(nav=generator(nav), comm=generator(comm)),
                        broadcast=generator(bc))


@dataclass(frozen=True)
class Scenario:
    """One fixed crossing geometry + CDR config; forward CNS noise is the only randomness.

    ``pos_ci95``/``reception_prob``/``latency`` are switched per regime by :meth:`for_regime`, so the
    geometry and CDR stack are identical across regimes — only the noise source changes.
    """

    dpsi: float = 90.0          # crossing angle [deg]
    tlos: float = 65.0          # time to loss of separation on the nominal collision course [s]
    rpz: float = 50.0           # protected-zone radius [m]
    lookahead: float = 60.0     # detection lookahead [s]
    margin: float = 1.05        # MVP resolution-zone margin
    speed: float = 10.2889      # 20 kts
    pos_ci95: float = 0.0       # GNSS 95% radial position accuracy [m] (nav pathway)
    reception_prob: float = 1.0 # P(a broadcast is received) per link (comms pathway)
    latency: float = 0.0        # constant link delay [s]
    dt: float = 0.5
    t_max: float = 200.0
    done_timeout: float = 10.0

    def for_regime(self, regime: Regime, pos_ci95: float, reception_prob: float) -> Scenario:
        if regime == "nav":
            return replace(self, pos_ci95=pos_ci95, reception_prob=1.0)
        if regime == "comms":
            return replace(self, pos_ci95=0.0, reception_prob=reception_prob)
        return replace(self, pos_ci95=pos_ci95, reception_prob=reception_prob)  # both

    def _comm(self) -> Comm | None:
        if self.reception_prob >= 1.0 and self.latency == 0.0:
            return None
        return Comm(reception_prob=self.reception_prob, latency=self.latency)

    def env(self) -> tuple[FleetEnv, list[Agent]]:
        own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=self.speed,
                            pos_ci95=self.pos_ci95, vel_ci95=self.pos_ci95 * 0.1)
        intr = create_conflict(own, intr_id="INT", dpsi=self.dpsi, dcpa=0.0, tlos=self.tlos,
                               rpz=self.rpz, side=1)
        agents = [Agent(own, M600), Agent(intr, M600)]
        env = build_env(agents, rpz=self.rpz, t_lookahead=self.lookahead, dt=self.dt,
                        detector=StateBased(), resolver=MVP(margin=self.margin),
                        recovery=PastCPA(bouncing_guard=True), navigation=GnssNavigation(),
                        communication=self._comm(), t_max=self.t_max,
                        done_timeout=self.done_timeout)
        return env, agents


@dataclass(frozen=True)
class Particle:
    """A fleet particle: its fixed rules (``env``) and current world (``state``). The look-ahead
    coordinate is a pure function of ``state``, so nothing else needs to ride along."""

    env: FleetEnv
    state: FleetState

    def advanced(self, streams: FleetStreams) -> Particle:
        return Particle(self.env, self.env.advance(self.state, streams))


def predicted_sep(state: FleetState, tau: float) -> float:
    """Closest predicted horizontal separation over the look-ahead window [0, tau] (Blom eq. 10.7):
    ``min_{0<=t<=tau} |r + t*v|`` for the pair's relative position/velocity. Straight-line
    extrapolation; the minimiser is clamped into the window."""
    own, intr = state.states[0], state.states[1]
    rel = relative_enu(own, intr)
    vsq = rel.vx * rel.vx + rel.vy * rel.vy
    if vsq < 1e-9:
        return math.hypot(rel.rx, rel.ry)  # not closing — window minimum is now
    t = min(max(-(rel.rx * rel.vx + rel.ry * rel.vy) / vsq, 0.0), tau)
    return math.hypot(rel.rx + t * rel.vx, rel.ry + t * rel.vy)


@dataclass(frozen=True)
class LookaheadCoord:
    """Nested conflict levels D_k, k=1..m: predicted sep within horizon ``tau_k`` <= ``d_k``, with
    both the horizon and the miss shrinking to ``(0, rpz)`` at the deepest level (= actual LoS).
    ``phi = (deepest level reached)/m``; the driver only ever needs the single ``in_level`` test."""

    rpz: float
    d_max: float
    tau_max: float
    m: int

    def d(self, k: int) -> float:
        return self.d_max - (self.d_max - self.rpz) * (k / self.m)

    def tau(self, k: int) -> float:
        return self.tau_max * (1.0 - k / self.m)

    def in_level(self, state: FleetState, k: int) -> bool:
        """Is the pair inside conflict level D_k (1..m)? D_m (tau=0, d=rpz) is an actual LoS."""
        return predicted_sep(state, self.tau(k)) <= self.d(k)


def run_to_terminal(p: Particle, streams: FleetStreams) -> Particle:
    while not p.env.is_terminal(p.state):
        p = p.advanced(streams)
    return p


# --- estimators ------------------------------------------------------------------------------------
def _mc_chunk(scn: Scenario, n: int, seed: int) -> tuple[int, int]:
    """Brute-force: run ``n`` encounters, return (n_los, n)."""
    env, agents = scn.env()
    n_los = 0
    for s in spawn(root_seed_sequence(seed), n):
        p = run_to_terminal(Particle(env, env.initial_state(agents)), _streams(s))
        n_los += int(p.state.los)
    return n_los, n


def mc_estimate(scn: Scenario, n: int, seed: int, jobs: int) -> tuple[float, float, float, int]:
    chunks = [n // jobs + (1 if i < n % jobs else 0) for i in range(jobs)]
    out = Parallel(n_jobs=jobs)(delayed(_mc_chunk)(scn, c, seed + i)
                                for i, c in enumerate(chunks) if c > 0)
    k = sum(a for a, _ in out)
    return (*_wilson(k, n), k)


def _wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    if n == 0:
        return (float("nan"), 0.0, 1.0)
    ph = k / n
    denom = 1.0 + z * z / n
    centre = (ph + z * z / (2 * n)) / denom
    half = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / denom
    return (ph, max(0.0, centre - half), min(1.0, centre + half))


def _evolve_to_level(p: Particle, coord: LookaheadCoord, k: int, streams: FleetStreams) -> Particle:
    """Advance until the pair enters conflict level D_k (survivor) or the encounter ends (dropped)."""
    while not coord.in_level(p.state, k) and not p.env.is_terminal(p.state):
        p = p.advanced(streams)
    return p


def ips_once(scn: Scenario, coord: LookaheadCoord, n_particles: int,
             seq: np.random.SeedSequence) -> float:
    """One fixed-effort splitting run on the look-ahead coordinate: ``P̂ = Π_k S_k/N`` (0 if a level
    empties). Fixed geometry initial cloud, fresh per-particle streams per level, resample survivors
    to N. The deepest level D_m *is* an actual LoS, so no separate terminal factor is needed."""
    env, agents = scn.env()
    init_seq, evolve_seq = seq.spawn(2)
    particles = [Particle(env, env.initial_state(agents))
                 for _ in range(n_particles)]  # fixed geometry: identical start, forward noise splits
    level_seqs = evolve_seq.spawn(coord.m)
    prob = 1.0
    for k in range(1, coord.m + 1):
        sub = level_seqs[k - 1].spawn(n_particles + 1)
        evolved = [_evolve_to_level(p, coord, k, _streams(s))
                   for p, s in zip(particles, sub[:n_particles], strict=True)]
        survivors = [p for p in evolved if coord.in_level(p.state, k)]
        prob *= len(survivors) / n_particles
        if not survivors:
            return 0.0
        idx = generator(sub[n_particles]).integers(0, len(survivors), n_particles)
        particles = [survivors[i] for i in idx]
    return prob


def ips_estimate(scn: Scenario, coord: LookaheadCoord, n_particles: int, reps: int, seed: int,
                 jobs: int) -> tuple[float, tuple[float, float], int]:
    """Mean ``P̂`` with a log-space 95% CI over ``reps`` independent replications."""
    seeds = spawn(root_seed_sequence(seed), reps)
    probs = Parallel(n_jobs=jobs)(
        delayed(ips_once)(scn, coord, n_particles, s) for s in seeds)
    return float(np.mean(probs)), _log_ci(probs), sum(x == 0.0 for x in probs)


def _log_ci(probs: list[float], z: float = 1.96) -> tuple[float, float]:
    pos = [x for x in probs if x > 0.0]
    if len(pos) < 2 or len(pos) != len(probs):
        return (min(probs), max(probs))
    logs = np.log(pos)
    se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
    c = float(np.mean(logs))
    return (math.exp(c - z * se), math.exp(c + z * se))


PRESETS = {
    # quick machinery check: rare-ish so all three regimes have signal, but small budget.
    "smoke": dict(pos_ci95=20.0, reception_prob=0.06, d_max=95.0, tau_max=55.0, levels=14,
                  mc_n=30000, particles=400, reps=6),
    # rare-event benchmark: the validated operating point (nav ~2.7e-4, comms ~1.2e-3, both ~2.7e-2).
    "rare": dict(pos_ci95=20.0, reception_prob=0.06, d_max=95.0, tau_max=55.0, levels=14,
                 mc_n=500_000, particles=1500, reps=12),
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", choices=list(PRESETS), default="smoke")
    ap.add_argument("--mc-only", action="store_true")
    ap.add_argument("--ips-only", action="store_true", help="skip MC; just run/ladder-check IPS")
    ap.add_argument("--jobs", type=int, default=8, help="parallel workers; -1 = all cores")
    ap.add_argument("--seed", type=int, default=0)
    # per-run overrides — crank these on a many-core box (reps >= jobs saturates IPS)
    ap.add_argument("--mc-n", type=int, help="override MC encounters/regime")
    ap.add_argument("--particles", type=int, help="override IPS particles/level")
    ap.add_argument("--reps", type=int, help="override IPS replications (raise to >= --jobs)")
    ap.add_argument("--levels", type=int, help="override m (number of nested conflict levels)")
    ap.add_argument("--pos", type=float, help="override GNSS pos_ci95 [m] (nav rarity)")
    ap.add_argument("--rx", type=float, help="override reception_prob (comms rarity)")
    ap.add_argument("--d-max", type=float, help="override d_1 miss threshold [m] (outer level)")
    ap.add_argument("--tau-max", type=float, help="override tau_1 look-ahead [s] (outer level)")
    a = ap.parse_args()
    cfg = dict(PRESETS[a.preset])  # copy so overrides don't mutate the shared preset
    for key, val in [("mc_n", a.mc_n), ("particles", a.particles), ("reps", a.reps),
                     ("levels", a.levels), ("pos_ci95", a.pos), ("reception_prob", a.rx),
                     ("d_max", a.d_max), ("tau_max", a.tau_max)]:
        if val is not None:
            cfg[key] = val

    scn0 = Scenario()
    coord = LookaheadCoord(rpz=scn0.rpz, d_max=cfg["d_max"], tau_max=cfg["tau_max"], m=cfg["levels"])
    print(f"scenario: {scn0.dpsi:.0f}deg crossing, rpz={scn0.rpz}, lookahead={scn0.lookahead}, "
          f"margin={scn0.margin}")
    print(f"preset={a.preset}: pos_ci95={cfg['pos_ci95']} reception={cfg['reception_prob']}; "
          f"look-ahead levels m={cfg['levels']} (d_max={cfg['d_max']}, tau_max={cfg['tau_max']}s); "
          f"MC n={cfg['mc_n']:,}, IPS {cfg['reps']}x{cfg['particles']}p\n")

    truth: dict[Regime, tuple[float, float, float, int]] = {}
    if not a.ips_only:
        for regime in REGIMES:
            scn = scn0.for_regime(regime, cfg["pos_ci95"], cfg["reception_prob"])
            t0 = time.perf_counter()
            ph, lo, hi, k = mc_estimate(scn, cfg["mc_n"], a.seed, a.jobs)
            truth[regime] = (ph, lo, hi, k)
            print(f"MC   {regime:5}: P(LoS)={ph:.3e}  95%CI[{lo:.3e}, {hi:.3e}]  "
                  f"({k} events / {cfg['mc_n']:,}, {time.perf_counter() - t0:.0f}s)", flush=True)
        if a.mc_only:
            return
        print()

    for regime in REGIMES:
        scn = scn0.for_regime(regime, cfg["pos_ci95"], cfg["reception_prob"])
        t0 = time.perf_counter()
        mean, (lo, hi), n_coll = ips_estimate(scn, coord, cfg["particles"], cfg["reps"], a.seed,
                                              a.jobs)
        if regime in truth:
            _, mlo, mhi, _ = truth[regime]
            verdict = "COLLAPSE" if n_coll == cfg["reps"] else (
                "PASS" if (lo <= mhi and mlo <= hi) else "FAIL")
            tail = f"{verdict}  (vs MC {truth[regime][0]:.3e}, {time.perf_counter() - t0:.0f}s)"
        else:
            tail = f"collapsed={n_coll}/{cfg['reps']}  ({time.perf_counter() - t0:.0f}s)"
        print(f"IPS  {regime:5}: P(LoS)={mean:.3e}  95%CI[{lo:.3e}, {hi:.3e}]  {tail}", flush=True)


if __name__ == "__main__":
    main()
