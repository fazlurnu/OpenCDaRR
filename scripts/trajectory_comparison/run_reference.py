"""Record CDaRR_git's (BlueSky) ownship trajectory for one no-noise conflict pair.

Runs the *reference* stack, so it must be executed with the reference's Python environment
(the ``cdarr`` conda env, which has BlueSky) and needs the CDaRR_git repo importable:

    CDARR_GIT=/path/to/CDaRR_git \\
        /path/to/envs/cdarr/bin/python scripts/trajectory_comparison/run_reference.py <dpsi_deg>

Writes ``_out/ref_<dpsi>.npz`` (t, gs, trk, lat, lon, active, cmdgs) for the ownship (DRO000).
CDARR_GIT defaults to ``~/Projects/CDaRR_git``. See ``README.md``.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

_CDARR_GIT = os.environ.get("CDARR_GIT", str(Path.home() / "Projects" / "CDaRR_git"))
sys.path.insert(0, _CDARR_GIT)
os.chdir(_CDARR_GIT)  # the pairwise env reads envs/pairwise_params.json relative to cwd

import bluesky as bs  # noqa: E402
from envs.pairwise_conflict import PairwiseHorConflict  # noqa: E402
from sim_models.adsl_module import ADSL  # noqa: E402
from sim_models.cd_statebased import StateBased  # noqa: E402
from sim_models.cr_mvp import MVP  # noqa: E402
from sim_models.crr_resumenav_cpa import resumenav as resumenav_cpa  # noqa: E402
from sim.utils import suppress_output  # noqa: E402

SPEED, RPZ, MARGIN, LOOKAHEAD, TMAX = 10.2889, 50.0, 1.05, 120.0, 250.0


def run(dpsi: float) -> dict[str, np.ndarray]:
    if not getattr(bs, "_inited", False):
        with suppress_output():
            bs.init(mode="sim", detached=True)
            bs._inited = True
    bs.settings.asas_marh = MARGIN

    detection, detection_gt = StateBased(), StateBased()
    resolution, recovery = MVP(), resumenav_cpa
    pairwise = PairwiseHorConflict(
        pair_width=1, pair_height=1, asas_pzr_m=RPZ, dtlookahead=LOOKAHEAD * 1.5,
        init_speed_ownship=SPEED, init_speed_intruder=SPEED,
        init_dpsi=dpsi, aircraft_type_ownship="M600", simdt_factor=4)
    # zero-noise ADSL nodes -> the perceived state is the truth
    bus = ADSL(0.0, 0.0, reception_prob=1.0, seed=1)
    own_adsl = ADSL(0.0, 0.0, reception_prob=1.0, seed=2, latency_s=0.0)
    intr_adsl = ADSL(0.0, 0.0, reception_prob=1.0, seed=3)
    prev_adsl = ADSL(0.0, 0.0, reception_prob=1.0, seed=4)

    simdt = bs.settings.simdt * 4
    event_dt = float(bs.settings.asas_dt)
    eps = np.finfo(float).eps * 100
    sim_t, next_event, inited, reso = 0.0, 0.0, False, None
    own_id = pairwise.ownship_ids[0]
    active, cmdgs = 0.0, SPEED
    rows = []
    while sim_t < TMAX + 1e-9:
        states = pairwise._get_states()
        if not inited:
            own_adsl.update_from_truth(states)
            bus.send_data(intr_adsl, own_adsl, indices=None)
            bus.send_data(prev_adsl, intr_adsl, indices=None)
            inited = True
        if sim_t + eps >= next_event:
            own_adsl.update_from_truth(states)
            bus.send_data(intr_adsl, own_adsl, indices=None)
            bus.send_data(prev_adsl, intr_adsl, indices=None)
            detection.detect(own_adsl, intr_adsl, RPZ, 100.0, LOOKAHEAD)
            detection_gt.detect(states, states, RPZ, 100.0, LOOKAHEAD)
            reso = resolution.resolve(detection, own_adsl, intr_adsl, MARGIN)
            recovery(resolution, detection, own_adsl, intr_adsl)
            active = 1.0 if any(own_id in pair for pair in resolution.resopairs) else 0.0
            cmdgs = float(reso[1][0]) if active > 0.5 else SPEED  # commanded gs (pre CAS/TAS)
            next_event += event_dt
        rows.append((sim_t, float(states.gs[0]), float(states.trk[0]),
                     float(states.lat[0]), float(states.lon[0]), active, cmdgs))
        pairwise.step(reso)
        sim_t += simdt
    pairwise.reset()
    a = np.array(rows)
    keys = ("t", "gs", "trk", "lat", "lon", "active", "cmdgs")
    return {k: a[:, i] for i, k in enumerate(keys)}


if __name__ == "__main__":
    dpsi = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    out = Path(__file__).parent / "_out"
    out.mkdir(exist_ok=True)
    path = out / f"ref_{int(dpsi)}.npz"
    np.savez(path, **run(dpsi))
    print(f"wrote {path}")
