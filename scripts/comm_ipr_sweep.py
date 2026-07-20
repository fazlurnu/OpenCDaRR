"""IPR vs. communication quality — reception loss and latency, through the real loop.

Two sweeps, both via the actual ``estimate_ipr`` / ``run_encounter`` path (not a standalone
simulation): (1) IPR vs. ``reception_prob`` at zero latency, (2) IPR vs. latency spread at
perfect reception. Same random-angle scenario as ``configs/pairwise.yaml`` (``dcpa_max=50``,
``tlos=60``, ``rpz=50``, ``lookahead=120``), no GPS noise, so communication is the only
stochastic driver of the outcome — isolates the effect this script is about.

Usage:  python scripts/comm_ipr_sweep.py

Writes ``vault/observations/img/loop-communication-ipr.png``. Backs
``vault/observations/loop-communication-integration.md``.
"""

from __future__ import annotations

import time
from pathlib import Path

import matplotlib
import numpy as np
from joblib import Parallel, delayed

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import Comm, uniform_latency  # noqa: E402
from opencdarr.config import (  # noqa: E402
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.estimator import estimate_ipr  # noqa: E402
from opencdarr.performance import M600  # noqa: E402

SPEED, DCPA_MAX, TLOS, RPZ, LOOKAHEAD = 10.2889, 50.0, 60.0, 50.0, 120.0
N_ENCOUNTERS = 400
SEED = 1
BROADCAST_INTERVAL = 1.0
# average wait for a first success at rate p, one broadcast/s: E[wait] = interval / p
EXPECTED_WAIT_EQUALS_TLOS_P = BROADCAST_INTERVAL / TLOS


def _config() -> Config:
    return Config(
        seed=SEED, n_encounters=N_ENCOUNTERS,
        scenario=ScenarioConfig("M600", SPEED, DCPA_MAX, TLOS),
        conflict=ConflictConfig(RPZ, LOOKAHEAD),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0, BROADCAST_INTERVAL),
    )


def _ipr(communication) -> float:
    return estimate_ipr(
        _config(), M600, StateBased(), MVP(1.05), PastCPA(), communication=communication
    ).ipr


def run() -> dict:
    reception_ps = np.geomspace(1.0, 0.002, 16)
    latencies = np.linspace(0.0, 90.0, 13)

    t0 = time.time()
    ipr_vs_reception = Parallel(n_jobs=4)(
        delayed(_ipr)(Comm(reception_prob=float(p), latency=0.0)) for p in reception_ps
    )
    ipr_vs_latency = Parallel(n_jobs=4)(
        delayed(_ipr)(Comm(reception_prob=1.0, latency=uniform_latency(0.0, float(lat))))
        for lat in latencies
    )
    baseline = _ipr(None)
    print(f"(elapsed {time.time() - t0:.1f}s)")

    return {
        "reception_ps": reception_ps, "ipr_vs_reception": np.array(ipr_vs_reception),
        "latencies": latencies, "ipr_vs_latency": np.array(ipr_vs_latency),
        "baseline": baseline,
    }


def plot(data: dict, out: Path) -> None:
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))

    baseline_label = f"no communication ({data['baseline']:.2f})"

    a = ax[0]
    a.plot(data["reception_ps"], data["ipr_vs_reception"], "o-", color="tab:blue",
           label="IPR (through the loop)")
    a.axhline(data["baseline"], color="grey", ls="--", lw=1, label=baseline_label)
    p_star = 1.0 / TLOS  # E[wait for first delivery] = TLOS at this rate
    a.axvline(p_star, color="tab:red", ls=":", lw=1.3,
              label=f"E[wait]=tlos={TLOS:.0f}s at p={p_star:.3f}")
    a.set_xscale("log")
    a.invert_xaxis()  # worse links (smaller p) to the right, matching panel 2's "worse -> right"
    a.set_xlabel("reception_prob (log scale, latency=0)")
    a.set_ylabel("IPR")
    a.set_title(f"1. IPR vs. reception loss  ({N_ENCOUNTERS} encounters/point)")
    a.legend(fontsize=8, loc="lower left")
    a.grid(True, alpha=0.3, which="both")

    a = ax[1]
    a.plot(data["latencies"], data["ipr_vs_latency"], "o-", color="tab:orange",
           label="IPR (through the loop)")
    a.axhline(data["baseline"], color="grey", ls="--", lw=1, label=baseline_label)
    a.axvline(TLOS, color="tab:red", ls=":", lw=1.3, label=f"latency spread = tlos = {TLOS:.0f}s")
    a.set_xlabel("latency ~ Uniform(0, x) [s]  (reception_prob=1.0)")
    a.set_ylabel("IPR")
    a.set_title(f"2. IPR vs. latency  ({N_ENCOUNTERS} encounters/point)")
    a.legend(fontsize=8, loc="lower left")
    a.grid(True, alpha=0.3)

    fig.suptitle("IPR vs. communication quality, through the real encounter loop "
                 f"(dcpa_max={DCPA_MAX:.0f}m, tlos={TLOS:.0f}s, rpz={RPZ:.0f}m)", fontsize=12)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=105)
    print(f"wrote {out}")


def main() -> None:
    data = run()
    print(f"baseline (no communication): {data['baseline']:.4f}")
    print("reception sweep:")
    for p, ipr in zip(data["reception_ps"], data["ipr_vs_reception"], strict=True):
        print(f"  p={p:7.4f}  IPR={ipr:.4f}")
    print("latency sweep:")
    for lat, ipr in zip(data["latencies"], data["ipr_vs_latency"], strict=True):
        print(f"  latency~U(0,{lat:5.1f})  IPR={ipr:.4f}")
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/loop-communication-ipr.png"
    plot(data, out)


if __name__ == "__main__":
    main()
