"""Compare DubinsDynamics (turn-rate-limited) vs HolonomicDynamics (ADR 0009) on the same command.

Two scenarios, same initial state and same M600 Performance for both models:

- **turn**   — flying north, then commanded to fly east (a 90 deg direction change).
- **reverse** — flying north, then commanded to fly south (a 180 deg reversal).

Both models get the exact same `Command` sequence; only the `Dynamics` differs. This is the
"how to control both models" example: identical control code, different physics.

Usage:  python scripts/dynamics_comparison_demo.py

Writes ``vault/observations/img/dubins-vs-holonomic.png``. Backs
``vault/observations/controlling-dubins-vs-holonomic.md``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.dynamics import (  # noqa: E402
    Command,
    DubinsDynamics,
    Dynamics,
    HolonomicDynamics,
)
from opencdarr.performance import M600  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

DT = 0.1
T_MAX = 12.0
SPEED = 10.0
M_PER_DEG_LAT = 111320.0


def _start() -> AircraftState:
    return AircraftState(id="D0", lat=52.0, lon=4.0, trk=0.0, gs=SPEED)  # flying north


def run(dynamics: Dynamics, new_heading: float) -> dict[str, np.ndarray]:
    """Fly north for 2 s (settled cruise), then hold a command to `new_heading` for the rest."""
    s = _start()
    cmd_cruise = Command.from_track_speed(0.0, SPEED)
    cmd_turn = Command.from_track_speed(new_heading, SPEED)
    m_lon = M_PER_DEG_LAT * np.cos(np.radians(s.lat))
    rows = []
    t = 0.0
    while t < T_MAX + 1e-9:
        cmd = cmd_cruise if t < 2.0 else cmd_turn
        rows.append((t, s.gs, s.trk, (s.lat - 52.0) * M_PER_DEG_LAT, (s.lon - 4.0) * m_lon))
        s = dynamics.step(s, cmd, M600, DT)
        t += DT
    a = np.array(rows)
    return {"t": a[:, 0], "gs": a[:, 1], "trk": a[:, 2], "north": a[:, 3], "east": a[:, 4]}


def plot(data: dict[str, dict[str, np.ndarray]], out: Path) -> None:
    fig, ax = plt.subplots(2, 2, figsize=(14, 10))

    scenarios = [
        ("turn", "commanded 90 deg turn (N -> E)"),
        ("reverse", "commanded 180 deg reversal (N -> S)"),
    ]
    for col, (scenario, title) in enumerate(scenarios):
        dub, holo = data[f"{scenario}_dubins"], data[f"{scenario}_holonomic"]

        a = ax[0, col]
        a.plot(dub["east"], dub["north"], color="tab:orange", lw=2.0,
               label="DubinsDynamics (turn-rate-limited)")
        a.plot(holo["east"], holo["north"], color="tab:blue", lw=2.0, label="HolonomicDynamics")
        a.scatter([0], [0], color="k", zorder=5, s=25, label="command issued here (t=2s)")
        a.set_xlabel("East [m]")
        a.set_ylabel("North [m]")
        a.set_title(f"Ground track — {title}")
        a.axis("equal")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8, loc="best")

        a = ax[1, col]
        a.plot(dub["t"], dub["gs"], color="tab:orange", lw=1.8, label="gs — Dubins")
        a.plot(holo["t"], holo["gs"], color="tab:blue", lw=1.8, label="gs — Holonomic")
        a.axvline(2.0, color="k", lw=0.8, ls=":")
        a.set_xlabel("t [s]")
        a.set_ylabel("ground speed [m/s]")
        a2 = a.twinx()
        a2.plot(dub["t"], dub["trk"], color="tab:orange", lw=1.0, ls="--", alpha=0.6,
                label="trk — Dubins")
        a2.plot(holo["t"], holo["trk"], color="tab:blue", lw=1.0, ls="--", alpha=0.6,
                label="trk — Holonomic")
        a2.set_ylabel("track [deg]")
        a.set_title(f"Speed / track over time — {title}")
        lines1, labels1 = a.get_legend_handles_labels()
        lines2, labels2 = a2.get_legend_handles_labels()
        a.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc="center right")
        a.grid(True, alpha=0.3)

    fig.suptitle(
        "DubinsDynamics vs HolonomicDynamics: identical Command sequence, identical M600 "
        "Performance, different physics (ADR 0009)", fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=105)
    print(f"wrote {out}")


def main() -> None:
    dub, holo = DubinsDynamics(), HolonomicDynamics()
    data = {
        "turn_dubins": run(dub, 90.0),
        "turn_holonomic": run(holo, 90.0),
        "reverse_dubins": run(dub, 180.0),
        "reverse_holonomic": run(holo, 180.0),
    }

    # a few numbers worth printing: path length and max lateral deviation, since "cuts the corner"
    # is a visual claim that should also be a checkable one
    for scenario in ("turn", "reverse"):
        for model in ("dubins", "holonomic"):
            d = data[f"{scenario}_{model}"]
            path_len = float(np.sum(np.hypot(np.diff(d["east"]), np.diff(d["north"]))))
            max_east = float(np.max(np.abs(d["east"])))
            print(f"{scenario:8s} {model:10s}  path_length={path_len:7.2f} m  "
                  f"max_east_excursion={max_east:6.3f} m")

    out = Path(__file__).resolve().parents[1] / "vault/observations/img/dubins-vs-holonomic.png"
    plot(data, out)


if __name__ == "__main__":
    main()
