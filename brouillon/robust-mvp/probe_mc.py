"""Plain-MC probe over the campaign grid — sizes the IPS shell ladder before spending on it.

IPS needs a decreasing ladder of running-minimum separations, and a ladder spaced too aggressively
collapses and reports ``P = 0``, which is not a real zero (ADR 0017 §2). This measures where the
minimum separation actually lands per cell, so :func:`~conditions_rm.ladder` is placed on evidence.

Runs through the **same** ``env.advance`` path IPS uses (:class:`~opencdarr.ips.Particle`), not
through ``run_encounter``, so the distribution being laddered is the distribution being split. It
also records the manoeuvring cost, which is what the safety in the campaign has to be paid for
with.

    python robust-mvp/probe_mc.py --n 300 --jobs 6
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # runnable from any cwd

from conditions_rm import (  # noqa: E402
    DT,
    RESOLVERS,
    RPZ,
    SPEED,
    T_MAX,
    Cell,
    cells,
    ladder,
)
from joblib import Parallel, delayed  # noqa: E402

from opencdarr.fleet import CnsStreams, FleetState, FleetStreams  # noqa: E402
from opencdarr.relative import velocity_enu  # noqa: E402
from opencdarr.rng import children, generator, root_seed_sequence  # noqa: E402


def _streams(seq: np.random.SeedSequence) -> FleetStreams:
    """A run's forward RNG — nav + comm + broadcast, the same three substreams IPS spawns."""
    nav_seq, comm_seq, bc_seq = children(seq, 0, 3)
    return FleetStreams(
        cns=CnsStreams(nav=generator(nav_seq), comm=generator(comm_seq)),
        broadcast=generator(bc_seq),
    )


def _deviation(state: FleetState) -> float:
    """Total deviation from nominal across the fleet at this instant [m/s].

    Summed over aircraft rather than averaged, so a two-aircraft encounter where both are avoiding
    reads twice one where only one is. Aircraft with no declared intent contribute zero.
    """
    total = 0.0
    for ac in state.states:
        if ac.desired is None:
            continue
        ve, vn = velocity_enu(ac)
        total += float(np.hypot(ve - ac.desired.v_east, vn - ac.desired.v_north))
    return total


def one(cell: Cell, seq: np.random.SeedSequence) -> tuple[float, bool, float, float]:
    """One encounter to termination.

    Returns the minimum separation [m], whether separation was lost, the **time-averaged**
    deviation from nominal [m/s] and the encounter duration [s]. The deviation is time-averaged
    rather than integrated so it stays comparable between a cell that resolves in 90 s and one
    that runs to ``t_max``.
    """
    particle = cell.build_particle()
    env, state = particle.env, particle.state
    streams = _streams(seq)
    deviation = 0.0
    steps = 0
    while not env.is_terminal(state):
        state = env.advance(state, streams)
        deviation += _deviation(state)
        steps += 1
    duration = steps * DT
    return state.min_sep, state.los, deviation / max(steps, 1), duration


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--n", type=int, default=300, help="encounters per cell")
    p.add_argument("--jobs", type=int, default=6)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--tail-levels", type=int, default=12,
                   help="shells below the smallest observed run (the unobserved tail)")
    p.add_argument("--resolvers", nargs="+", default=None, choices=tuple(RESOLVERS))
    p.add_argument("--vel-ci95", nargs="+", type=float, default=None)
    p.add_argument("--dpsi", nargs="+", type=float, default=None)
    p.add_argument("--out", type=Path, default=HERE / "probe_results.json")
    cfg = p.parse_args()

    # the same noise substreams for every cell, so cells differ by their settings and nothing else
    seqs = list(children(root_seed_sequence(cfg.seed), 0, cfg.n))
    grid = cells(
        tuple(cfg.resolvers) if cfg.resolvers else None,
        tuple(cfg.vel_ci95) if cfg.vel_ci95 else None,
        tuple(cfg.dpsi) if cfg.dpsi else None,
    )
    print(f"MC probe — {len(grid)} cells x {cfg.n} encounters, rpz={RPZ:g} m, "
          f"speed={SPEED:g} m/s, t_max={T_MAX:g} s", flush=True)
    print(f"{'cell':>42} | {'P(LoS)':>7} {'min':>7} {'p1':>7} {'p5':>7} {'p25':>7} {'p40':>7} "
          f"{'p50':>7} | {'dev':>6} {'dur':>6} {'t/o':>5}", flush=True)
    t0 = time.time()

    # One Parallel over the *flattened* grid, not one per cell: a fresh Parallel object spawns a
    # fresh loky executor, and a loop of them leaves enough orphaned workers that the OS starts
    # killing them. ``batch_size`` also lets one unpickled Cell serve many encounters, so the env
    # is built once per batch rather than once per encounter.
    tasks = [(cell, s) for cell in grid for s in seqs]
    with Parallel(n_jobs=cfg.jobs, batch_size=25) as parallel:
        flat = parallel(delayed(one)(cell, s) for cell, s in tasks)

    out: dict[str, object] = {}
    for i, cell in enumerate(grid):
        res = flat[i * cfg.n:(i + 1) * cfg.n]
        sep = np.array([r[0] for r in res])
        p_los = float(np.mean([r[1] for r in res]))
        dev = float(np.mean([r[2] for r in res]))
        dur = float(np.mean([r[3] for r in res]))
        timeout = float(np.mean([r[3] >= T_MAX - DT for r in res]))
        q = np.percentile(sep, [1, 5, 25, 40, 50])
        out[cell.key] = {
            "label": cell.label, "dpsi": cell.dpsi, "pos_ci95": cell.pos_ci95,
            "vel_ci95": cell.vel_ci95, "resolver": cell.resolver, "n": cfg.n,
            "p_los": p_los, "min": float(sep.min()),
            "percentiles": {str(k): float(v) for k, v in zip((1, 5, 25, 40, 50), q, strict=True)},
            "dev_rate": dev, "duration": dur, "timeout_fraction": timeout,
            "ladder": list(ladder(sep, cfg.tail_levels)),
            "min_sep": sep.tolist(),
        }
        print(f"{cell.label:>42} | {p_los:7.3f} {sep.min():7.1f} {q[0]:7.1f} {q[1]:7.1f} "
              f"{q[2]:7.1f} {q[3]:7.1f} {q[4]:7.1f} | {dev:6.2f} {dur:6.0f} {timeout:5.2f}",
              flush=True)

    # Merge, never replace: the campaign is built up one arm at a time, and a run that probed a
    # subset would otherwise silently discard every cell it did not recompute.
    merged = json.loads(cfg.out.read_text()) if cfg.out.exists() else {}
    merged.update(out)
    cfg.out.write_text(json.dumps(merged, indent=1))
    print("\nladders", flush=True)
    for key, row in out.items():
        shells = row["ladder"]  # type: ignore[index]
        print(f"  {key:>28} ({len(shells):2d})  "  # type: ignore[arg-type]
              + " ".join(f"{d:6.1f}" for d in shells), flush=True)
    print(f"\n({time.time() - t0:.0f} s)  wrote {cfg.out}", flush=True)


if __name__ == "__main__":
    main()
