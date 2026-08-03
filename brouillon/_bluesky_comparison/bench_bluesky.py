"""BlueSky half of the wallclock comparison. Run with the cdarr env's python:

    python bench_bluesky.py <scenario.json> <results_out.json>

Reads the scenario written by the notebook so both simulators fly identical geometry.
"""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import bluesky as bs

scenario_path, results_path = Path(sys.argv[1]), Path(sys.argv[2])
scenario = json.loads(scenario_path.read_text())
cfg = scenario["config"]

SPEED_KT = cfg["speed"] / 0.514444     # BlueSky's CRE/SPD take knots
ALT_FT = 164.0                         # ~50 m, level throughout
CAPTURE = cfg["capture_m"]
RPZ_NM = cfg["rpz_m"] / 1852.0
T_MAX = cfg["t_max"]

bs.init(mode="sim", detached=True)
results = []

for key in sorted(scenario["fleets"], key=int):
    fleet = scenario["fleets"][key]
    for rep in range(cfg["reps"][key]):

        bs.sim.reset()
        bs.stack.stack("CDMETHOD STATEBASED")     # detection on, resolution off
        bs.stack.stack("RESO OFF")
        bs.stack.stack(f"ZONER {RPZ_NM}")
        bs.stack.stack(f"DTLOOK {cfg['t_lookahead']}")
        for ac in fleet:
            bs.stack.stack(
                f"CRE {ac['id']},M600,{ac['lat']},{ac['lon']},{ac['trk']},{ALT_FT},{SPEED_KT}")
            bs.stack.stack(f"ADDWPT {ac['id']},{ac['wp_lat']},{ac['wp_lon']}")
            bs.stack.stack(f"LNAV {ac['id']},ON")
            bs.stack.stack(f"VNAV {ac['id']},OFF")
            bs.stack.stack(f"SPD {ac['id']},{SPEED_KT}")
        bs.sim.step()   # let the stack commands land before the clock starts

        wp_lat = np.array([ac["wp_lat"] for ac in fleet])
        wp_lon = np.array([ac["wp_lon"] for ac in fleet])

        def all_arrived() -> bool:
            """Every drone within CAPTURE m of its target (equirectangular; fine at 30 m)."""
            dlat = np.radians(wp_lat - bs.traf.lat)
            dlon = np.radians(wp_lon - bs.traf.lon) * np.cos(np.radians(bs.traf.lat))
            return bool(np.all(np.hypot(dlat, dlon) * 6371000.0 <= CAPTURE))

        ntraf, steps, max_conf = int(bs.traf.ntraf), 0, 0
        started = datetime.now().astimezone()
        t_start = bs.sim.simt
        t0 = time.perf_counter()
        while not all_arrived() and bs.sim.simt - t_start < T_MAX:
            bs.sim.step()
            steps += 1
            max_conf = max(max_conf, len(bs.traf.cd.confpairs))
        wall = time.perf_counter() - t0

        results.append({
            "n": len(fleet), "rep": rep, "started": started.isoformat(timespec="seconds"),
            "wallclock_s": wall, "sim_t_s": bs.sim.simt - t_start, "steps": steps,
            "ntraf": ntraf, "max_confpairs": max_conf, "simdt": float(bs.settings.simdt),
        })
        print(f"bluesky n={len(fleet):2d} rep {rep}  sim={bs.sim.simt - t_start:6.2f}s  "
              f"wall={wall:8.2f}s  steps={steps}  max_confpairs={max_conf}", flush=True)

results_path.write_text(json.dumps(results, indent=1))
