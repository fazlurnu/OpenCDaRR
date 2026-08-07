"""Run the whole validation campaign and print a paste-able summary.

One command, one output file, one table. Every condition is run at two GNSS-noise rungs: 40 m,
where plain Monte Carlo sees hundreds of events and agreement is testable, and 10 m, where MC
starves and the splitting estimator has to carry it.

    PYTHONPATH=. python scripts/validation/run_campaign.py --jobs -1

On a large machine the defaults below are the intended sizes. To see it work end to end first:

    PYTHONPATH=. python scripts/validation/run_campaign.py --quick

Results land in ``scripts/validation/out/`` — one JSON per part plus ``summary.md``. Both are
written after every condition, so an interrupted run keeps whatever it has already paid for, and
re-running skips those conditions instead of re-timing a file read.
"""

from __future__ import annotations

import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import campaign  # noqa: E402
import pairwise  # noqa: E402
import random_traffic  # noqa: E402
import ring  # noqa: E402

from opencdarr.estimate.parallel import resolve_jobs  # noqa: E402

PARTS = {
    "pairwise": (pairwise.ANGLES, lambda: [
        campaign.Cell(label={"dpsi": a}, scenario=pairwise.PairwiseEncounter(dpsi=a, dcpa=0.0))
        for a in pairwise.ANGLES]),
    "ring": (ring.SIZES, lambda: [
        campaign.Cell(label={"n": n}, scenario=ring.CrossingRing(n=n, radius=ring.RADIUS))
        for n in ring.SIZES]),
    "random_traffic": (random_traffic.DENSITIES, lambda: [
        campaign.Cell(
            label={"density": d},
            scenario=random_traffic.RandomTraffic(density=d, radius=random_traffic.RADIUS))
        for d in random_traffic.DENSITIES]),
}


def summarise(paths: list[pathlib.Path]) -> str:
    """One markdown table per part — the thing to paste back."""
    out: list[str] = ["# Validation campaign", ""]
    for path in paths:
        payload = json.loads(path.read_text())
        design, timing = payload["design"], payload["timing"]
        out += [
            f"## {payload['part']}", "",
            f"- MC anchor target: {design['target_events']} events "
            f"(min {design['mc_min']:,}, cap {design['mc_max']:,} encounters)",
            f"- IPS: {design['ips_particles']} particles x {design['ips_reps']} reps, "
            f"shells calibrated per condition from a "
            f"{design['pilot_encounters']}-encounter pilot",
            f"- workers {timing['workers']}, "
            f"MC {timing['mc_seconds_total']:,.0f} s, IPS {timing['ips_seconds_total']:,.0f} s",
            "",
            "| condition | pos_ci95 | N | MC P(LoS) | events | runs | anchored |"
            " IPS P(LoS) | collapsed | ratio | MC s | IPS s | gain |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for r in payload["rows"]:
            label = ", ".join(f"{k}={v}" for k, v in r["_label"].items() if k != "pos_ci95")
            out.append(
                f"| {label} | {r['_label']['pos_ci95']:.0f} m | {r['n_aircraft']} "
                f"| {r['mc_p_los_run']:.3e} | {r['mc_events']} | {r['mc_encounters']:,} "
                f"| {'yes' if r['mc_anchored'] else '**no**'} "
                f"| {r['ips_p_los_run']:.3e} | {r['ips_collapsed']} | {r['ratio_run']} "
                f"| {r['mc_seconds']:.0f} | {r['ips_seconds']:.0f} | {r['gain']} |"
            )
        out.append("")
    return "\n".join(out)


def main() -> None:
    p = campaign.parser("all three parts")
    p.add_argument("--quick", action="store_true",
                   help="tiny sizes, to check the machinery end to end")
    p.add_argument("--parts", nargs="*", default=list(PARTS), choices=list(PARTS))
    args = p.parse_args()

    if args.quick:
        args.events, args.mc_min, args.mc_max = 5, 200, 4_000
        args.particles, args.reps = 200, 4

    workers = resolve_jobs(args.jobs)
    print(f"validation campaign — {len(args.parts)} parts, {workers} workers")
    print(f"  rungs (pos_ci95): {', '.join(f'{r:.0f} m' for r in args.rungs)}")
    print(f"  MC: grow to {args.events} events, {args.mc_min:,}..{args.mc_max:,} encounters")
    print(f"  IPS: {args.particles} particles x {args.reps} reps\n")

    started = time.perf_counter()
    paths: list[pathlib.Path] = []
    for part in args.parts:
        _, build = PARTS[part]
        print(f"=== {part} ===", flush=True)
        paths.append(campaign.run_part(part, campaign.over_rungs(build(), args.rungs), args))

    summary = campaign.OUT_DIR / "summary.md"
    summary.write_text(summarise(paths))
    print(f"\nfinished in {time.perf_counter() - started:,.0f} s")
    print(f"wrote {summary}  <- paste this back")
    print("\n" + "=" * 70 + "\n")
    print(summary.read_text())


if __name__ == "__main__":
    main()
