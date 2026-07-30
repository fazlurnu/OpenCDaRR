"""Grade the separation measurement against an analytic reference — the probe behind
``vault/observations/segment-min-separation.md`` ([[todo-might-be-a-bug]] entry 1).

``FleetEnv.advance`` used to sample separation once per ``dt`` and set ``los = cur < rpz``, so a
pass that dipped inside a threshold and back out within one step left no sampled point inside. It
now takes the minimum over each whole step (:func:`opencdarr.kinematics.segment_min_range`). This
script measures what that is worth, and it exists because the claim cannot be checked from inside
the simulation: every earlier test compared ``los`` against quantities read off the *same* sampled
trajectory, which cannot detect a measurement that misses part of that trajectory.

**The reference is analytic.** :func:`opencdarr.scenario.create_conflict` builds a straight-line
encounter whose true minimum separation *is* the requested ``dcpa``. So on a uniform ``dcpa`` grid
the exact level-crossing curve is ``P(min_sep <= d) = d / dcpa_max`` in closed form, with no
simulation involved — the runs are purely what is being graded.

**The "before" column** is reconstructed from ``record=True`` frames of the very same run: the old
reading was taken at the top of each ``advance``, i.e. on every frame *except* the terminal one,
hence ``frames[:-1]``. So both columns come from one trajectory and cannot differ by anything but
the measurement.

Four modes:

    python scripts/segment_min_sep_probe.py plos      # time + accuracy of P(LoS) vs exact, by dt
    python scripts/segment_min_sep_probe.py shells    # the level-crossing curve vs exact, by dt
    python scripts/segment_min_sep_probe.py cost      # marginal cost of refining, per step
    python scripts/segment_min_sep_probe.py compare   # interpolate position vs extrapolate vel.

``shells`` is the one that matters: ``P(LoS)`` at ``rpz`` was never off by more than 0.6%, while
``P(min_sep <= 5 m)`` was under-counted by 44% at ``dt=1.0``. The relative error goes as
``(v_rel*dt)^2 / (24 d^2)``, so it grows as the target tightens — which is why refining ``dt`` was
never the fix.
"""

from __future__ import annotations

import argparse
import math
import time

from opencdarr.cd.statebased import StateBased
from opencdarr.cns.navigation import GnssNavigation
from opencdarr.cns.stack import CnsStreams
from opencdarr.cr.mvp import MVP
from opencdarr.crr.pastcpa import PastCPA
from opencdarr.fleet import Agent, FleetStreams, build_env, run_fleet
from opencdarr.kinematics import Relative, relative_enu, segment_min_range
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict, sample_pairwise

RPZ = 50.0
SPEED = 10.2889  # m/s (~20 kts), the configs/pairwise.yaml cruise
TLOS = 60.0
LOOKAHEAD = 120.0
DTS = (1.0, 0.5, 0.2)


def relative_speed(dpsi: float, gs: float = SPEED) -> float:
    """|v_intr - v_own| for a crossing at ``dpsi`` with both aircraft at ``gs``."""
    return 2.0 * gs * abs(math.sin(math.radians(dpsi) / 2.0))


def straight_run(dpsi: float, dcpa: float, dt: float, *, tlos: float = TLOS, record: bool = False):
    """One non-manoeuvring encounter, whose true minimum separation is exactly ``dcpa``."""
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=dcpa, tlos=tlos, rpz=RPZ)
    return run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=RPZ, t_lookahead=LOOKAHEAD, dt=dt, detector=StateBased(),
        t_max=600.0, record=record,
    )


def _own():
    from opencdarr.state import AircraftState
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED)


def endpoint_readings(frames) -> tuple[float, bool]:
    """What the pre-fix, per-``dt`` reading would have returned: (min_sep, los).

    ``frames[:-1]`` because the old measurement ran at the *top* of ``advance``, so the terminal
    frame was never measured at all.
    """
    dists = [relative_enu(f.states[0], f.states[1]).dist for f in list(frames)[:-1]]
    return min(dists), any(d < RPZ for d in dists)


# --- mode: plos ---------------------------------------------------------------------------------


def mode_plos(args: argparse.Namespace) -> None:
    """Time and accuracy of the counted P(LoS) against an exact reference, per ``dt``.

    ``dcpa`` uniform on ``[0, 2*rpz]`` makes exact ``P(LoS) = P(dcpa < rpz) = 0.5`` by
    construction — a non-degenerate reference, unlike the shipped ``dcpa_max = rpz`` where every
    encounter is a LoS.
    """
    dpsis = (45.0, 90.0, 135.0, 180.0)
    geoms = [
        (dpsi, 2.0 * RPZ * (i + 0.5) / args.n)
        for dpsi in dpsis
        for i in range(args.n)
    ]
    n = len(geoms)
    exact = sum(1 for _, dcpa in geoms if dcpa < RPZ)
    print(f"{n} straight-line encounters ({len(dpsis)} crossing angles x {args.n} miss distances)")
    print(f"dcpa uniform on [0, {2 * RPZ:.0f}] m, rpz={RPZ} -> exact P(LoS) = {exact / n:.4f} "
          f"by construction\n")
    print(f"{'dt':>5} {'steps/enc':>10} {'wall':>9} {'us/enc':>8} "
          f"{'P(LoS) segment':>15} {'err':>8} {'P(LoS) endpoint':>16} {'err':>8}")
    for dt in DTS:
        # timed pass in the production configuration (no recording)
        t0 = time.perf_counter()
        segment = sum(int(straight_run(dpsi, dcpa, dt).los) for dpsi, dcpa in geoms)
        elapsed = time.perf_counter() - t0
        # untimed pass, recorded, to reconstruct the old reading from the same trajectories
        endpoint = steps = 0
        for dpsi, dcpa in geoms:
            out = straight_run(dpsi, dcpa, dt, record=True)
            steps += len(list(out.frames)) - 1
            if endpoint_readings(out.frames)[1]:
                endpoint += 1
        print(f"{dt:5.2f} {steps / n:10.1f} {elapsed:8.2f}s {1e6 * elapsed / n:8.0f} "
              f"{segment / n:15.4f} {(segment - exact) / exact:+8.2%} "
              f"{endpoint / n:16.4f} {(endpoint - exact) / exact:+8.2%}")


# --- mode: shells -------------------------------------------------------------------------------


def mode_shells(args: argparse.Namespace) -> None:
    """The level-crossing curve against exact ``d/dcpa_max``, at each shell radius and ``dt``.

    ``ips.py`` crosses shells on ``state.min_sep`` — the same running minimum — so this is the
    error a rare-event estimate inherits directly. Per-shell survivals telescope, so an IPS
    estimate of ``P(min_sep <= d_m)`` inherits exactly the innermost row.
    """
    shells = [50.0, 25.0, 10.0, 5.0, 2.0, 1.0]
    dpsi = args.dpsi
    vrel = relative_speed(dpsi)
    print(f"straight-line, dpsi={dpsi} (v_rel={vrel:.2f} m/s), dcpa on a uniform grid of {args.n} "
          f"over [0, {RPZ}]")
    print("exact P(min_sep <= d) = d/50, because the true minimum separation IS dcpa.")
    print("tlos is phase-mixed over [0, dt) to break the tlos-is-a-multiple-of-dt alignment.\n")
    print(f"{'dt':>5} {'shell':>6} {'exact':>8} {'segment':>8} {'endpoint':>9} "
          f"{'seg err':>9} {'end err':>9} {'law':>9} {'steps_in':>9}")
    for dt in DTS:
        exact = [0] * len(shells)
        seg = [0] * len(shells)
        end = [0] * len(shells)
        for i in range(args.n):
            dcpa = RPZ * (i + 0.5) / args.n
            out = straight_run(dpsi, dcpa, dt, tlos=TLOS + dt * (i % 8) / 8.0, record=True)
            ep, _ = endpoint_readings(out.frames)
            for k, d in enumerate(shells):
                exact[k] += dcpa <= d
                seg[k] += out.min_sep <= d
                end[k] += ep <= d
        for k, d in enumerate(shells):
            law = (vrel * dt) ** 2 / (24.0 * d * d)  # predicted relative error
            steps_in = 2.0 * d / vrel / dt  # samples inside a central pass through this shell
            e_s = (seg[k] - exact[k]) / exact[k] if exact[k] else float("nan")
            e_e = (end[k] - exact[k]) / exact[k] if exact[k] else float("nan")
            print(f"{dt:5.2f} {d:6.1f} {exact[k] / args.n:8.4f} {seg[k] / args.n:8.4f} "
                  f"{end[k] / args.n:9.4f} {e_s:+9.2%} {e_e:+9.2%} {law:9.2e} {steps_in:9.2f}")
        print()


# --- mode: cost ---------------------------------------------------------------------------------


def mode_cost(args: argparse.Namespace) -> None:
    """Marginal cost of refining, against the cost of a whole ``advance``.

    The refinement needs the relative *vector* at both ends of a step, and the old measurement
    already paid a ``geo.qdrdist`` per pair per step for a bare scalar. Swapping that for
    ``relative_enu`` buys the vector nearly free, and each step's post-step vector *is* the next
    step's pre-step vector, so there is no second call.
    """
    import timeit

    from opencdarr import geo

    geom_seq, nav_seq, _ = spawn(spawn(root_seed_sequence(42), 1)[0], 3)
    own, intr = sample_pairwise(
        generator(geom_seq), speed=SPEED, dcpa_max=RPZ, tlos=TLOS, rpz=RPZ,
        pos_ci95=60.0, vel_ci95=6.0,
    )
    agents = [Agent(own, M600), Agent(intr, M600)]
    env = build_env(
        agents, rpz=RPZ, t_lookahead=LOOKAHEAD, dt=0.5, detector=StateBased(),
        resolver=MVP(margin=1.05), recovery=PastCPA(), navigation=GnssNavigation(),
    )
    streams = FleetStreams(cns=CnsStreams(nav=generator(nav_seq)))
    state = env.initial_state(agents)
    for _ in range(100):  # get mid-encounter, with the resolver engaged
        state = env.advance(state, streams)
    a, b = state.states[0], state.states[1]
    r0 = relative_enu(a, b)

    reps = args.n
    t_adv = timeit.timeit(lambda: env.advance(state, streams), number=max(reps // 10, 200))
    t_adv /= max(reps // 10, 200)
    t_qdr = timeit.timeit(lambda: geo.qdrdist(a.lat, a.lon, b.lat, b.lon), number=reps) / reps
    t_rel = timeit.timeit(lambda: relative_enu(a, b), number=reps) / reps
    t_seg = timeit.timeit(lambda: segment_min_range(r0, r0), number=reps) / reps

    print("marginal cost per pair per step (n=2, nav noise + MVP, dt=0.5)\n")
    print(f"  advance()                          {t_adv * 1e6:8.2f} us   (100%)")
    for label, t in (("geo.qdrdist      (paid before)    ", t_qdr),
                     ("relative_enu     (paid now)       ", t_rel),
                     ("segment_min_range(new arithmetic) ", t_seg)):
        print(f"  {label} {t * 1e6:8.2f} us   {100 * t / t_adv:5.2f}%")
    extra = (t_rel - t_qdr) + t_seg
    print(f"\n  => marginal {extra * 1e6:.2f} us = +{100 * extra / t_adv:.2f}% of a step, "
          f"against +150% for dropping dt 0.5 -> 0.2")


# --- mode: compare ------------------------------------------------------------------------------


def _velocity_cpa(r: Relative, dt: float) -> float:
    """The rejected alternative: extrapolate the pre-step relative velocity, clamped to the step.

    Kept here, not in the package, precisely because it is wrong: this script is what shows it.
    """
    v2 = r.vx * r.vx + r.vy * r.vy
    if v2 <= 0.0:
        return r.dist
    t = -(r.rx * r.vx + r.ry * r.vy) / v2
    if t <= 0.0:
        return r.dist
    return math.hypot(r.rx + r.vx * min(t, dt), r.ry + r.vy * min(t, dt))


def mode_compare(args: argparse.Namespace) -> None:
    """Interpolating positions vs extrapolating velocity, on manoeuvring runs.

    Both are "closed-form CPA between the pre- and post-step states", and they are not the same
    thing. A velocity extrapolation leaves the flown path whenever an aircraft is turning, and
    reports a range at a point never occupied — measured *inventing* losses of separation. They
    agree exactly on straight lines, so the whole gap is curvature.
    """
    print(f"sampled encounters, seed 42, n={args.n}, dcpa_max=rpz={RPZ}\n")
    for dt in (1.0, 0.5):
        for pos_ci95, resolver in ((0.0, False), (60.0, True)):
            spread = 0.0
            seg_los = vel_los = end_los = 0
            for seq in spawn(root_seed_sequence(42), args.n):
                geom_seq, nav_seq, _ = spawn(seq, 3)
                own, intr = sample_pairwise(
                    generator(geom_seq), speed=SPEED, dcpa_max=RPZ, tlos=TLOS, rpz=RPZ,
                    pos_ci95=pos_ci95, vel_ci95=pos_ci95 / 10.0,
                )
                out = run_fleet(
                    [Agent(own, M600), Agent(intr, M600)],
                    rpz=RPZ, t_lookahead=LOOKAHEAD, dt=dt, detector=StateBased(),
                    resolver=MVP(margin=1.05) if resolver else None,
                    recovery=PastCPA() if resolver else None,
                    navigation=GnssNavigation() if pos_ci95 else None,
                    rng=generator(nav_seq) if pos_ci95 else None,
                    t_max=600.0, record=True,
                )
                rels = [relative_enu(f.states[0], f.states[1]) for f in out.frames]
                vel = min(_velocity_cpa(r, dt) for r in rels)
                spread = max(spread, abs(out.min_sep - vel))
                seg_los += int(out.los)
                vel_los += int(vel < RPZ)
                end_los += int(endpoint_readings(out.frames)[1])
            tag = f"dt={dt:<4} pos_ci95={pos_ci95:<5} resolver={'MVP ' if resolver else 'none'}"
            print(f"{tag}  max|segment - velocity| {spread:6.3f} m")
            print(f"    P(LoS)  endpoint {end_los / args.n:.4f}   segment "
                  f"{seg_los / args.n:.4f}   velocity-extrapolated {vel_los / args.n:.4f}"
                  f"{'   <-- invented losses' if vel_los > seg_los else ''}")
        print()


_MODES = {"plos": mode_plos, "shells": mode_shells, "cost": mode_cost, "compare": mode_compare}
_DEFAULT_N = {"plos": 250, "shells": 6000, "cost": 20000, "compare": 400}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("mode", choices=sorted(_MODES))
    parser.add_argument("-n", type=int, default=None,
                        help="grid points / encounters / timing reps, per mode")
    parser.add_argument("--dpsi", type=float, default=90.0, help="crossing angle for `shells`")
    args = parser.parse_args()
    if args.n is None:
        args.n = _DEFAULT_N[args.mode]
    _MODES[args.mode](args)


if __name__ == "__main__":
    main()
