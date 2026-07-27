"""Unified-coordinate IPS on the real fleet sim: one importance function for nav AND comms rare events.

Implements the ``phi = max(nav_progress, cap * comm_progress)`` coordinate from
[[important-unified-coordinate]] against the *actual* ``opencdarr`` encounter (``FleetEnv.advance``,
GNSS nav noise, ``Comm`` drops), and validates it against brute-force MC in three regimes:

- **nav**   — nav noise only (``pos_ci95 > 0``, perfect comms): the continuous-drift pathway.
- **comms** — drops only (``pos_ci95 = 0``, ``reception_prob < 1``): the discrete-blackout pathway.
- **both**  — nav noise + drops.

`min_sep` (the current estimator's coordinate, ``opencdarr.ips.ips_once``) ladders nav but collapses
on the comms pathway ([[important-ips-gap]]); staleness is the mirror. The unified coordinate ladders
whichever pathway a particle is advancing along, so one estimator covers all three regimes.

**No core-sim change.** Staleness is read off ``state.cns_state.comm.held`` (via ``cns.surveillance.
age``) into a running-max the particle carries — parallel to ``FleetState.min_sep`` — so a clone keeps
its accumulated staleness. Both MC and IPS run the same ``FleetEnv.advance`` for an apples-to-apples
comparison (unlike ``ips_validate.py``, whose MC uses the pairwise ``run_encounter``).

    # smoke test at a non-rare setting (fast; IPS must match MC):
    python scripts/ips_unified_validate.py --preset smoke
    # the definitive rare-event benchmark (~1e-5; slow MC — run ONCE):
    python scripts/ips_unified_validate.py --preset rare --jobs 8
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

    @property
    def comms_on(self) -> bool:
        """Whether drops are possible (a real ``Comm`` model), so staleness is meaningful."""
        return self._comm() is not None

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


def _staleness(state: FleetState) -> float:
    """The **both-blind** duration [s]: time since *any* directed link last delivered a message.

    Comms-only LoS on a cooperative collision course needs *both* aircraft blind — one reception lets
    that aircraft resolve and the pair misses (probed: one-blind never breaches). So the quantity
    that ladders toward the rare event is the time since the most recent reception on *any* link
    (it resets whenever anyone hears), not the worst single-link age. No belief ever ⇒ blind since
    ``t = 0`` (returns ``t``)."""
    held = state.cns_state.comm.held
    latest = None
    for msg in held.values():
        latest = msg.t_meas if latest is None else max(latest, msg.t_meas)
    return state.t if latest is None else state.t - latest


@dataclass(frozen=True)
class Particle:
    """A fleet particle plus its running-max staleness (the parallel to ``state.min_sep``).

    ``comms_on`` gates staleness: under perfect comms ``build_env`` uses the perfect-delivery path,
    which never populates ``held`` (it threads ``last_tx``), so ``age`` is ``None`` everywhere and
    the belief is in fact always fresh — staleness must read 0, not ``t``. Only when drops are
    possible does the held-message age become a real blind-run measure."""

    env: FleetEnv
    state: FleetState
    max_stale: float
    comms_on: bool

    def advanced(self, streams: FleetStreams) -> Particle:
        s = self.env.advance(self.state, streams)
        stale = _staleness(s) if self.comms_on else 0.0
        return Particle(self.env, s, max(self.max_stale, stale), self.comms_on)


@dataclass(frozen=True)
class Coord:
    """The unified importance function ``phi = max(nav_progress, cap * comm_progress)`` in [0, 1].

    ``d_nominal`` is the miss distance the resolver achieves under perfect CNS (measured once), so
    ``nav_progress`` is the shortfall from that safe miss toward ``rpz`` (0 nominal, 1 at LoS).
    ``L_crit`` is the staleness at which a blackout is on the brink of breaching. ``cap < 1`` reserves
    ``phi = 1`` for a real geometric LoS, so the deepest shell is the true rare set (no terminal
    factor needed)."""

    rpz: float
    d_nominal: float
    l_crit: float
    cap: float = 0.9

    def nav_progress(self, min_sep: float) -> float:
        return min(1.0, max(0.0, (self.d_nominal - min_sep) / (self.d_nominal - self.rpz)))

    def comm_progress(self, max_stale: float) -> float:
        return min(1.0, max(0.0, max_stale / self.l_crit))

    def phi(self, p: Particle) -> float:
        return max(self.nav_progress(p.state.min_sep), self.cap * self.comm_progress(p.max_stale))


def run_to_terminal(p: Particle, streams: FleetStreams) -> Particle:
    while not p.env.is_terminal(p.state):
        p = p.advanced(streams)
    return p


def d_nominal(scn: Scenario) -> float:
    """The perfect-CNS closest approach [m]: run the deterministic (no noise, no drops) encounter."""
    env, agents = replace(scn, pos_ci95=0.0, reception_prob=1.0, latency=0.0).env()
    p = run_to_terminal(Particle(env, env.initial_state(agents), 0.0, comms_on=False),
                        _streams(root_seed_sequence(0)))
    return p.state.min_sep


# --- estimators ------------------------------------------------------------------------------------
def _mc_chunk(scn: Scenario, n: int, seed: int) -> tuple[int, int]:
    """Brute-force: run ``n`` encounters, return (n_los, n)."""
    env, agents = scn.env()
    comms_on = scn.comms_on
    n_los = 0
    for s in spawn(root_seed_sequence(seed), n):
        p = run_to_terminal(Particle(env, env.initial_state(agents), 0.0, comms_on), _streams(s))
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


def _evolve_to_shell(p: Particle, coord: Coord, target: float, streams: FleetStreams) -> Particle:
    while coord.phi(p) < target and not p.env.is_terminal(p.state):
        p = p.advanced(streams)
    return p


def ips_once(scn: Scenario, coord: Coord, shells: list[float], n_particles: int,
             seq: np.random.SeedSequence) -> float:
    """One fixed-effort splitting run on the unified coordinate. ``P̂ = Π S_k/N`` (0 if a shell empties).

    Mirrors :func:`opencdarr.ips.ips_once`: fixed geometry initial cloud, fresh per-particle streams
    per shell, resample survivors to N. Shells rise to 1.0 (= LoS, since ``phi = 1 ⇔ min_sep ≤ rpz``).
    """
    env, agents = scn.env()
    comms_on = scn.comms_on
    init_seq, evolve_seq = seq.spawn(2)
    particles = [Particle(env, env.initial_state(agents), 0.0, comms_on)
                 for _ in range(n_particles)]  # fixed geometry: identical start, forward noise splits
    level_seqs = evolve_seq.spawn(len(shells))
    prob = 1.0
    for k, target in enumerate(shells):
        sub = level_seqs[k].spawn(n_particles + 1)
        evolved = [_evolve_to_shell(p, coord, target, _streams(s))
                   for p, s in zip(particles, sub[:n_particles], strict=True)]
        survivors = [p for p in evolved if coord.phi(p) >= target]
        prob *= len(survivors) / n_particles
        if not survivors:
            return 0.0
        idx = generator(sub[n_particles]).integers(0, len(survivors), n_particles)
        particles = [survivors[i] for i in idx]
    return prob


def ips_estimate(scn: Scenario, coord: Coord, shells: list[float], n_particles: int, reps: int,
                 seed: int, jobs: int) -> tuple[float, tuple[float, float], int]:
    seeds = spawn(root_seed_sequence(seed), reps)
    probs = Parallel(n_jobs=jobs)(
        delayed(ips_once)(scn, coord, shells, n_particles, s) for s in seeds)
    return float(np.mean(probs)), _log_ci(probs), sum(x == 0.0 for x in probs)


def _log_ci(probs: list[float], z: float = 1.96) -> tuple[float, float]:
    pos = [x for x in probs if x > 0.0]
    if len(pos) < 2 or len(pos) != len(probs):
        return (min(probs), max(probs))
    logs = np.log(pos)
    se = float(np.std(logs, ddof=1)) / math.sqrt(len(logs))
    c = float(np.mean(logs))
    return (math.exp(c - z * se), math.exp(c + z * se))


def make_shells(m: int) -> list[float]:
    xs = np.linspace(0.0, 1.0, m + 1)[1:]
    return [float(x) for x in (1.0 - (1.0 - xs) ** 1.7)]


PRESETS = {
    # non-rare: fast, IPS must match MC — proves the machinery on the real sim
    "smoke": dict(pos_ci95=18.0, reception_prob=0.55, l_crit=25.0, mc_n=20000, particles=400,
                  reps=8, shells=10),
    # rare: the definitive benchmark — MC is expensive, run ONCE. Rarest point where all three
    # regimes stay robust (comms both-blind collapses if pushed rarer, since staleness ladders the
    # blind *duration* but not its *timing at CPA*). Probes: nav pos=20 -> ~2.5e-4, comms rx=0.06 ->
    # ~1.3e-3 (both-blind), staleness@LoS min ~24s -> L_crit=18. Deeper (1e-5/1e-6) is compute-bound
    # (IPS ~30min/regime, MC 1e-6 ~days) and needs a CPA-timed comm coordinate — see writeup.
    "rare": dict(pos_ci95=20.0, reception_prob=0.06, l_crit=18.0, mc_n=500_000, particles=1600,
                 reps=12, shells=14),
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--preset", choices=list(PRESETS), default="smoke")
    ap.add_argument("--mc-only", action="store_true")
    ap.add_argument("--ips-only", action="store_true", help="skip MC; just run/ladder-check IPS")
    ap.add_argument("--jobs", type=int, default=8, help="parallel workers; -1 = all cores")
    ap.add_argument("--seed", type=int, default=0)
    # per-run overrides of the preset — crank these on a many-core box (reps>=jobs saturates IPS)
    ap.add_argument("--mc-n", type=int, help="override MC encounters/regime")
    ap.add_argument("--particles", type=int, help="override IPS particles/shell")
    ap.add_argument("--reps", type=int, help="override IPS replications (raise to >= --jobs)")
    ap.add_argument("--shells", type=int, help="override IPS shell count")
    ap.add_argument("--pos", type=float, help="override GNSS pos_ci95 [m] (nav rarity)")
    ap.add_argument("--rx", type=float, help="override reception_prob (comms rarity)")
    ap.add_argument("--l-crit", type=float, help="override staleness scale L_crit [s]")
    a = ap.parse_args()
    cfg = dict(PRESETS[a.preset])  # copy so overrides don't mutate the shared preset
    for key, val in [("mc_n", a.mc_n), ("particles", a.particles), ("reps", a.reps),
                     ("shells", a.shells), ("pos_ci95", a.pos), ("reception_prob", a.rx),
                     ("l_crit", a.l_crit)]:
        if val is not None:
            cfg[key] = val

    scn0 = Scenario()
    dnom = d_nominal(scn0)
    print(f"scenario: {scn0.dpsi:.0f}deg crossing, rpz={scn0.rpz}, lookahead={scn0.lookahead}, "
          f"margin={scn0.margin}; perfect-CNS miss d_nominal={dnom:.1f} m")
    print(f"preset={a.preset}: pos_ci95={cfg['pos_ci95']} reception={cfg['reception_prob']} "
          f"L_crit={cfg['l_crit']}s; MC n={cfg['mc_n']:,}, IPS {cfg['reps']}x{cfg['particles']}p "
          f"x{cfg['shells']} shells\n")

    coord = Coord(rpz=scn0.rpz, d_nominal=dnom, l_crit=cfg["l_crit"])
    shells = make_shells(cfg["shells"])

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
        mean, (lo, hi), n_coll = ips_estimate(scn, coord, shells, cfg["particles"], cfg["reps"],
                                              a.seed, a.jobs)
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
