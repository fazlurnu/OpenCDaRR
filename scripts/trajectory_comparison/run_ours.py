"""Record OpenCDaRR's ownship trajectory for one no-noise conflict pair.

Usage:  python scripts/trajectory_comparison/run_ours.py <dpsi_deg>

Writes ``_out/ours_<dpsi>.npz`` (t, gs, trk, lat, lon, active, cmdgs) for the ownship
(init hdg 0). Pairs with ``run_reference.py`` (BlueSky/CDaRR_git) and ``plot.py``.
See ``README.md``.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.dynamics import Command, step_dynamics
from opencdarr.loop import _decide
from opencdarr.performance import M600
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

SPEED, RPZ, MARGIN, DT, BCAST = 10.2889, 50.0, 1.05, 0.2, 1.0
LOOKAHEAD, TLOS, TMAX = 120.0, 180.0, 250.0


def run(dpsi: float) -> dict[str, np.ndarray]:
    det, res, rec = StateBased(), MVP(MARGIN), PastCPA(bouncing_guard=True)
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED)
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=0.0, tlos=TLOS, rpz=RPZ, side=1)
    nom_own = Command(hdg=own.trk, spd=own.gs)
    nom_intr = Command(hdg=intr.trk, spd=intr.gs)
    cmd_own, cmd_intr = nom_own, nom_intr
    ro = ri = False
    t, nb = 0.0, 0.0
    rows = []
    while t < TMAX + 1e-9:
        if t + 1e-9 >= nb:  # no noise: decide on the true states, once per broadcast interval
            cmd_own, ro = _decide(own, intr, nom_own, ro, RPZ, LOOKAHEAD, det, res, rec)
            cmd_intr, ri = _decide(intr, own, nom_intr, ri, RPZ, LOOKAHEAD, det, res, rec)
            nb += BCAST
        rows.append((t, own.gs, own.trk, own.lat, own.lon, float(ro), cmd_own.spd))
        own = step_dynamics(own, cmd_own, M600, DT)
        intr = step_dynamics(intr, cmd_intr, M600, DT)
        t += DT
    a = np.array(rows)
    keys = ("t", "gs", "trk", "lat", "lon", "active", "cmdgs")
    return {k: a[:, i] for i, k in enumerate(keys)}


if __name__ == "__main__":
    dpsi = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    out = Path(__file__).parent / "_out"
    out.mkdir(exist_ok=True)
    path = out / f"ours_{int(dpsi)}.npz"
    np.savez(path, **run(dpsi))
    print(f"wrote {path}")
