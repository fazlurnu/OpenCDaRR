"""Flat-ENU prototype: accuracy vs spherical, per-call speed, and a monkeypatched end-to-end run.

The candidate replaces qdrdist's polar round-trip (haversine + bearing, then back to Cartesian)
with a local tangent plane at ownship: ry = dlat*R, rx = dlon*R*cos(lat_own). Same WGS84 radius
model. Applied to relative_enu (detect / MVP / PastCPA read it) and to the pairwise sweep, via
monkeypatching only — the repo is untouched.
"""
from __future__ import annotations

import math
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path("/Users/mfrahman/Projects/OpenCDaRR")
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "scripts" / "validation"))
sys.path.insert(0, str(pathlib.Path(__file__).parent))

from opencdarr import geo  # noqa: E402
from opencdarr.relative import Relative, relative_enu, velocity_enu  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402


def relative_enu_flat(own: AircraftState, intr: AircraftState) -> Relative:
    radius = geo.earth_radius(own.lat)
    ry = math.radians(intr.lat - own.lat) * radius
    rx = math.radians(intr.lon - own.lon) * radius * math.cos(math.radians(own.lat))
    vox, voy = velocity_enu(own)
    vix, viy = velocity_enu(intr)
    return Relative(rx=rx, ry=ry, vx=vix - vox, vy=viy - voy)


def pairwise_relative_flat(states) -> tuple[Relative, ...]:
    vels = [velocity_enu(s) for s in states]
    out: list[Relative] = []
    for i in range(len(states)):
        own = states[i]
        radius = geo.earth_radius(own.lat)
        coslat = math.cos(math.radians(own.lat))
        vox, voy = vels[i]
        for j in range(i + 1, len(states)):
            intr = states[j]
            ry = math.radians(intr.lat - own.lat) * radius
            rx = math.radians(intr.lon - own.lon) * radius * coslat
            vix, viy = vels[j]
            out.append(Relative(rx=rx, ry=ry, vx=vix - vox, vy=viy - voy))
    return tuple(out)


def accuracy() -> None:
    rng = np.random.default_rng(2)
    print("accuracy vs spherical (1000 random pairs per row, lat ~52 N):")
    print(f"{'separation':>11} {'max |d dist|':>13} {'max |d rx|':>11} {'max |d ry|':>11}")
    for d in (50.0, 100.0, 300.0, 1000.0, 3000.0):
        worst_d = worst_x = worst_y = 0.0
        for _ in range(1000):
            own = AircraftState(id="O", lat=52.0 + rng.uniform(-0.02, 0.02),
                                lon=4.0 + rng.uniform(-0.03, 0.03),
                                trk=rng.uniform(0, 360), gs=10.0)
            blat, blon = geo.forward(own.lat, own.lon, rng.uniform(0, 360), d)
            intr = AircraftState(id="I", lat=blat, lon=blon, trk=rng.uniform(0, 360), gs=10.0)
            s = relative_enu(own, intr)
            f = relative_enu_flat(own, intr)
            worst_d = max(worst_d, abs(math.hypot(f.rx, f.ry) - math.hypot(s.rx, s.ry)))
            worst_x = max(worst_x, abs(f.rx - s.rx))
            worst_y = max(worst_y, abs(f.ry - s.ry))
        print(f"{d:>9.0f} m {worst_d:>12.2e} m {worst_x:>10.2e} m {worst_y:>10.2e} m")


def per_call() -> None:
    rng = np.random.default_rng(3)
    own = AircraftState(id="O", lat=52.001, lon=4.002, trk=45.0, gs=10.0)
    intr = AircraftState(id="I", lat=52.003, lon=4.004, trk=200.0, gs=11.0)
    for name, fn in (("spherical relative_enu", relative_enu),
                     ("flat relative_enu", relative_enu_flat)):
        fn(own, intr)
        t0 = time.perf_counter()
        for _ in range(200_000):
            fn(own, intr)
        us = (time.perf_counter() - t0) / 200_000 * 1e6
        print(f"{name}: {us:.2f} us/call")
    _ = rng


def end_to_end() -> None:
    """Patch every campaign-stack consumer, then measure us/step on real IPS evolution."""
    from campaign import base_config, methods_for
    from profile_ips import CASES, LADDERS, make_build_initial

    import opencdarr.cd.statebased as statebased
    import opencdarr.cr.mvp as mvp
    import opencdarr.crr.pastcpa as pastcpa
    import opencdarr.fleet as fleet
    import opencdarr.relative as relative
    from opencdarr.estimate.ips import evolve_shard, resample_level
    from opencdarr.rng import children, root_seed_sequence

    def run_case(name, n_particles):
        scenario_fn, pos, ladder_key, _, _ = CASES[name]
        scenario = scenario_fn()
        ladder = LADDERS[ladder_key]
        m = methods_for(scenario)
        cfg = base_config(n_particles, pos)
        build_initial = make_build_initial(scenario, m, cfg)
        seq = root_seed_sequence(0)
        init_seq, evolve_seq, _tail = children(seq, 0, 3)
        particles = [build_initial(s) for s in children(init_seq, 0, n_particles)]
        level_seqs = children(evolve_seq, 0, len(ladder))
        t0 = time.perf_counter()
        steps = 0.0
        for k, target in enumerate(ladder):
            sub = children(level_seqs[k], 0, n_particles + 1)
            before = [p.state.t for p in particles]
            evolved = evolve_shard(particles, target, sub[:n_particles])
            steps += sum((p.state.t - t) / 0.5 for p, t in zip(evolved, before, strict=True))
            _, particles, _ = resample_level(evolved, target, n_particles, sub[n_particles])
            if not particles:
                break
        return (time.perf_counter() - t0) / steps * 1e6, int(steps)

    patches = [
        (relative, "relative_enu"), (statebased, "relative_enu"),
        (mvp, "relative_enu"), (pastcpa, "relative_enu"),
    ]
    originals = [(mod, attr, getattr(mod, attr)) for mod, attr in patches]
    orig_pairwise = (fleet.pairwise_relative, relative.pairwise_relative)

    for case, n in (("ring8_40m", 40), ("random5_40m", 30)):
        base_us, base_steps = run_case(case, n)
        for mod, attr in patches:
            setattr(mod, attr, relative_enu_flat)
        fleet.pairwise_relative = pairwise_relative_flat
        relative.pairwise_relative = pairwise_relative_flat
        try:
            flat_us, flat_steps = run_case(case, n)
        finally:
            for mod, attr, fn in originals:
                setattr(mod, attr, fn)
            fleet.pairwise_relative, relative.pairwise_relative = orig_pairwise
        print(f"{case}: spherical {base_us:.0f} us/step ({base_steps} steps)  "
              f"flat {flat_us:.0f} us/step ({flat_steps} steps)  "
              f"-> {100 * (1 - flat_us / base_us):.1f}% faster")


if __name__ == "__main__":
    accuracy()
    print()
    per_call()
    print()
    end_to_end()
