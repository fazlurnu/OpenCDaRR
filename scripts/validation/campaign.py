"""The validation campaign's shared machinery: one condition, both backends, timed.

The campaign asks one question in three settings — **do plain Monte Carlo and interacting particle
splitting agree, and what does each cost?** Agreement without cost is half the answer: IPS earns
its place only where MC is expensive, so wall time is part of the result and is recorded with it.

**One condition at a time, both backends back to back.** Timing a whole sweep and dividing gives a
number that belongs to no cell in particular. Running the two backends on one condition, one after
the other, makes the seconds attributable to it — which is what makes the ``gain`` column mean
anything.

**The Monte-Carlo anchor sizes itself.** An anchor built on two events is not an anchor, and how
many encounters it takes to get fifty of them varies by three orders of magnitude across these
conditions. So IPS runs first and its estimate buys the MC sample: ``n = target_events / p``,
floored, and capped by ``--mc-max``. Sizing a run from a prior estimate is a design decision and
not an inference: the MC estimate is unbiased at whatever ``n`` it is given, so it stays an
independent check on the number that sized it.

Where even the cap cannot reach fifty events the row says so (``mc_anchored: false``) instead of
reporting a ratio resting on one or two. That is the honest reading of the rare regime: MC has
no answer there at any budget anyone will pay, which is the reason splitting exists.

**A cached condition is skipped, not re-timed.** Reading a stored row takes milliseconds. Writing
that down as the cost of a 50 000-encounter batch would be a wrong number rather than a fast one,
so a cached row keeps its original seconds and stays out of the totals. ``--no-cache`` re-runs and
re-times conditions that are already stored.

**Two noise rungs, and why.** Every condition is run at two GNSS self-noise levels, and the pair is
the point. At 40 m the loss is common enough that MC sees hundreds of events, so asking whether
the two agree is a fair question with a real answer — that is the *anchor*. At 10 m the loss is
rare enough that MC needs millions of encounters and often cannot reach fifty at all — that is the
*target*, the regime splitting exists for. An IPS number at the target rung is worth believing
only because the same code agreed with MC at the anchor rung.

Apart from that one knob the three parts differ by geometry and by nothing else.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Any

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import GnssNavigation  # noqa: E402
from opencdarr.config import (  # noqa: E402
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.estimate.parallel import resolve_jobs  # noqa: E402
from opencdarr.experiment import (  # noqa: E402
    IPS,
    MC,
    CacheConfig,
    Fixed,
    Methods,
    run_experiment,
)
from opencdarr.performance import M600  # noqa: E402
from opencdarr.scenario import Scenario  # noqa: E402

# The two rungs of the noise dial, in metres of 95% radial position accuracy. Velocity accuracy is
# a tenth of it, as in vault/observations/rare-event-validation-ladder.md.
ANCHOR_POS_CI95 = 40.0   # P(LoS) ~ 3e-2 — MC is solid here, so agreement is testable
TARGET_POS_CI95 = 10.0   # P(LoS) ~ 1e-5 and below — MC starves, IPS carries it
RUNGS = (ANCHOR_POS_CI95, TARGET_POS_CI95)
RPZ, LOOKAHEAD, SPEED = 50.0, 120.0, 10.2889
DT, T_MAX, DONE_TIMEOUT = 0.5, 250.0, 10.0

# The shells are not fixed. They are derived per condition from a short pilot — see
# ``_ladder_for``. A single hand-written ladder cannot serve these geometries: the median
# separation an encounter reaches ranges from 46 m for a converging ring to 290 m for a head-on
# pair, so a ladder starting at 100 m is either unreachable at one end or trivially crossed at the
# other. Measured with the fixed ladder, every ring condition collapsed 4 replications out of 4.
# The *ratio* is fixed and the *number* of shells adapts to the span, rather than the other way
# round. A fixed count over a wide span leaves a final leap to rpz far larger than every step
# before it, and that is the step the cloud fails: measured on a head-on pair, survival ran
# 0.44, 0.59, 0.55, 0.39, 0.45, 0.41 and then 0.0, collapsing every replication at the last shell
# while the ring geometries — whose span is a few metres — were fine with the same ladder shape.
SHELL_RATIO = 0.62      # each shell closes this fraction of the remaining distance to rpz
FINAL_GAP = 1.5         # m — how close the last shell gets before the step onto rpz itself
MAX_SHELLS = 20
PILOT_ENCOUNTERS = 200

OUT_DIR = pathlib.Path("scripts/validation/out")
CACHE_DIR = pathlib.Path(".opencdarr_cache/validation")


@dataclass(frozen=True)
class Cell:
    """One condition of a part: what identifies it, and the geometry it runs."""

    label: dict[str, Any]  # the identifying columns, e.g. {"dpsi": 45.0, "pos_ci95": 40.0}
    scenario: Scenario
    pos_ci95: float = TARGET_POS_CI95

    @property
    def vel_ci95(self) -> float:
        return self.pos_ci95 / 10.0

    @property
    def key(self) -> str:
        return json.dumps(self.label, sort_keys=True)


def over_rungs(cells: list[Cell], rungs: Sequence[float] = RUNGS) -> list[Cell]:
    """Each geometry at every rung, anchor first — so a part fails fast if it disagrees.

    How rare a rung is depends on the geometry as well as the noise: at 10 m a crossing pair sits
    near 1e-5 while an eight-aircraft converging ring is still a near-certainty. Reaching a given
    probability across all three parts therefore takes different rungs, which is why they are an
    argument rather than a constant.
    """
    return [
        Cell(label={**c.label, "pos_ci95": pos}, scenario=c.scenario, pos_ci95=pos)
        for pos in rungs
        for c in cells
    ]


def base_config(n: int, pos_ci95: float = TARGET_POS_CI95) -> Config:
    return Config(
        seed=0, n_encounters=n,
        scenario=ScenarioConfig("M600", SPEED, RPZ, 70.0, pos_ci95, pos_ci95 / 10.0),
        conflict=ConflictConfig(RPZ, LOOKAHEAD),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(DT, T_MAX, DONE_TIMEOUT),
    )


def methods_for(scenario: Scenario) -> Methods:
    """The CDR stack, identical in every part — only ``scenario`` differs."""
    return Methods(
        detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(bouncing_guard=True),
        navigation=GnssNavigation(), perf=M600, scenario=scenario,
    )


def _ladder_for(cell: Cell, args: argparse.Namespace) -> tuple[list[float], float]:
    """Shells for this condition, from a short Monte-Carlo pilot of the same geometry.

    The ladder has to span from where encounters ordinarily end up down to ``rpz``, in steps a
    cloud can actually cross. That upper range is exactly what a couple of hundred encounters
    resolves; the rare tail below it is what splitting is for and needs no pilot.

    Shells approach ``rpz`` geometrically from the pilot's median, so roughly half the cloud
    survives the first shell and the later ones do not bunch against the boundary. A geometry whose
    median is already inside ``rpz`` — a converging ring is one — has no upper range to start from,
    so the ladder starts above the boundary at the pilot's 90th percentile instead.

    How *many* shells is decided by the span rather than fixed, which keeps every step the same
    relative size: a head-on pair spanning 127 m above ``rpz`` needs about twelve, a converging
    ring spanning one metre needs two. Fixing the count instead leaves the final step onto ``rpz``
    far larger than the rest, and that is precisely the step a cloud fails.
    """
    n = args.pilot
    pilot = run_experiment(
        {"pos_ci95": Fixed(cell.pos_ci95), "vel_ci95": Fixed(cell.vel_ci95)},
        methods=methods_for(cell.scenario), backend=MC(n_encounters=n),
        base_config=base_config(n, cell.pos_ci95), seed=args.seed + 1, n_jobs=args.jobs,
    ).cell()
    seps = pilot.min_seps
    start = float(statistics.median(seps))
    if start <= RPZ:
        upper = sorted(seps)[int(0.9 * (len(seps) - 1))]
        start = RPZ + max(10.0, upper - RPZ)
    span = start - RPZ
    # enough shells that the last one sits ~FINAL_GAP above rpz, so the step onto it is no bigger
    # than the steps before it
    count = max(1, math.ceil(1 + math.log(FINAL_GAP / span) / math.log(SHELL_RATIO)))
    count = min(count, MAX_SHELLS - 1)
    shells = [round(RPZ + span * SHELL_RATIO**k, 1) for k in range(count)]
    return shells + [RPZ], start


def _mc_sample_size(p_hat: float, args: argparse.Namespace) -> tuple[int, str]:
    """How many encounters to buy so the anchor rests on ``--events`` losses, and why that many.

    ``p_hat`` comes from the splitting estimator, which has already run. A probability of zero or a
    collapsed ladder leaves nothing to size from, so the budget cap is spent and the row will show
    whether that was enough.
    """
    if not (p_hat > 0.0):
        return args.mc_max, "cap (no usable estimate)"
    wanted = math.ceil(args.events / p_hat)
    if wanted > args.mc_max:
        return args.mc_max, "cap"
    return max(args.mc_min, wanted), "ips estimate"


def _row(cell: Cell, args: argparse.Namespace) -> dict[str, Any]:
    """Run both backends on one condition, back to back, and time each."""
    methods = methods_for(cell.scenario)
    declaration = {"pos_ci95": Fixed(cell.pos_ci95), "vel_ci95": Fixed(cell.vel_ci95)}
    cache = CacheConfig(dir=CACHE_DIR)

    # The ladder is calibrated to this geometry before anything is split over it.
    t0 = time.perf_counter()
    shells, pilot_median = _ladder_for(cell, args)
    pilot_seconds = time.perf_counter() - t0

    # IPS next: it is the cheap one, and its estimate is what sizes the anchor below.
    t0 = time.perf_counter()
    ips = run_experiment(declaration, methods=methods,
                         backend=IPS(shells=shells, n_particles=args.particles, reps=args.reps),
                         base_config=base_config(args.particles, cell.pos_ci95), seed=args.seed,
                         n_jobs=args.jobs, cache=cache).cell()
    ips_seconds = time.perf_counter() - t0

    n_mc, sized_by = _mc_sample_size(ips.p_los_run, args)
    t0 = time.perf_counter()
    mc = run_experiment(declaration, methods=methods, backend=MC(n_encounters=n_mc),
                        base_config=base_config(n_mc, cell.pos_ci95), seed=args.seed,
                        n_jobs=args.jobs, cache=cache).cell()
    mc_seconds = time.perf_counter() - t0

    return {
        **cell.label,
        "n_aircraft": cell.scenario.size(),
        "mc_p_los_run": mc.p_los_run, "mc_p_los_ac": mc.p_los_ac, "mc_mean_k": mc.mean_k,
        "mc_events": mc.n_los, "mc_encounters": mc.n_encounters,
        "mc_sized_by": sized_by,
        "mc_anchored": mc.n_los >= args.events,
        "ips_p_los_run": ips.p_los_run, "ips_p_los_ac": ips.p_los_ac, "ips_mean_k": ips.mean_k,
        "ips_collapsed": ips.n_collapsed, "ips_lineages": ips.n_lineages,
        "shells": shells, "pilot_median_min_sep": round(pilot_median, 1),
        "pilot_seconds": round(pilot_seconds, 2),
        "survival": [round(s, 3) for s in ips.reps[0].survival],
        "ratio_run": _ratio(mc.p_los_run, ips.p_los_run),
        "ratio_ac": _ratio(mc.p_los_ac, ips.p_los_ac),
        "mc_seconds": round(mc_seconds, 2), "ips_seconds": round(ips_seconds, 2),
        "gain": round(mc_seconds / ips_seconds, 2) if ips_seconds > 0 else None,
        "cached": False,
    }


def _ratio(a: float, b: float) -> float | None:
    """How far apart the two estimates are, as a factor. ``None`` when one of them is zero."""
    lo, hi = min(a, b), max(a, b)
    return round(hi / lo, 3) if lo > 0 else None


def parser(part: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=f"validation campaign — {part}")
    p.add_argument("--events", type=int, default=50,
                   help="LoS events the MC anchor should rest on")
    p.add_argument("--mc-min", type=int, default=2_000, help="fewest MC encounters per condition")
    p.add_argument("--mc-max", type=int, default=200_000,
                   help="most MC encounters per condition — the budget ceiling")
    p.add_argument("--particles", type=int, default=800, help="IPS particles per shell")
    p.add_argument("--reps", type=int, default=8, help="IPS replications")
    p.add_argument("--jobs", type=int, default=-1, help="workers (-1 = every core)")
    p.add_argument("--rungs", nargs="+", type=float, default=list(RUNGS), metavar="POS_CI95",
                   help="GNSS noise levels to run each geometry at [m], rarest last")
    p.add_argument("--pilot", type=int, default=PILOT_ENCOUNTERS,
                   help="encounters used to calibrate this condition's shell ladder")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-cache", action="store_true",
                   help="re-run and re-time conditions already stored")
    return p


def run_part(part: str, cells: list[Cell], args: argparse.Namespace) -> pathlib.Path:
    """Run every condition of one part, and write the rows plus a timing block."""
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / f"{part}.json"

    stored: dict[str, dict[str, Any]] = {}
    if out_path.exists() and not args.no_cache:
        for row in json.loads(out_path.read_text())["rows"]:
            stored[json.dumps(row["_label"], sort_keys=True)] = row

    started = datetime.now(UTC)
    rows: list[dict[str, Any]] = []
    for cell in cells:
        previous = stored.get(cell.key)
        if previous is not None:
            print(f"  {cell.label}  cached")
            rows.append({**previous, "cached": True})
            continue
        print(f"  {cell.label}  running ...", flush=True)
        row = _row(cell, args)
        row["_label"] = cell.label
        rows.append(row)
        anchored = "anchored" if row["mc_anchored"] else "NOT anchored"
        print(f"    MC  {row['mc_p_los_run']:.3e}  {row['mc_events']:>4} events /"
              f" {row['mc_encounters']:,} runs  ({row['mc_seconds']}s)  {anchored}")
        print(f"    ladder {row['shells']}  survival {row['survival']}")
        print(f"    IPS {row['ips_p_los_run']:.3e}  collapsed {row['ips_collapsed']}"
              f"  ({row['ips_seconds']}s)   ratio {row['ratio_run']}   gain {row['gain']}")
        # write after every condition, so a crash keeps the cells already paid for
        _write(out_path, part, rows, args, started)

    _write(out_path, part, rows, args, started)
    return out_path


def _write(path: pathlib.Path, part: str, rows: list[dict[str, Any]],
           args: argparse.Namespace, started: datetime) -> None:
    fresh = [r for r in rows if not r["cached"]]
    payload = {
        "part": part,
        "rows": rows,
        "design": {
            "rungs_pos_ci95": list(args.rungs), "rpz": RPZ, "dt": DT,
            "target_events": args.events, "mc_min": args.mc_min, "mc_max": args.mc_max,
            "ips_particles": args.particles, "ips_reps": args.reps,
            "shell_ratio": SHELL_RATIO, "final_gap": FINAL_GAP,
            "pilot_encounters": args.pilot, "seed": args.seed,
        },
        "timing": {
            # Only the conditions actually computed in this run. A cached row costs a file read,
            # and counting that as the cost of the batch it stands for would be a wrong number.
            "started_utc": started.isoformat(timespec="seconds"),
            "finished_utc": datetime.now(UTC).isoformat(timespec="seconds"),
            "workers": resolve_jobs(args.jobs),
            "conditions_run": len(fresh),
            "conditions_cached": len(rows) - len(fresh),
            "mc_seconds_total": round(sum(r["mc_seconds"] for r in fresh), 2),
            "ips_seconds_total": round(sum(r["ips_seconds"] for r in fresh), 2),
        },
    }
    path.write_text(json.dumps(payload, indent=2))
