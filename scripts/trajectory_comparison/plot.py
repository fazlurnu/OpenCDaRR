"""Plot the ours-vs-reference ownship trajectory comparison for one crossing angle.

Usage:  python scripts/trajectory_comparison/plot.py <dpsi_deg>

Reads ``_out/ours_<dpsi>.npz`` and ``_out/ref_<dpsi>.npz`` (from run_ours.py / run_reference.py)
and writes ``vault/observations/img/trajectory-<dpsi>deg.png``. Four panels: ground speed
(achieved + commanded), heading/track, North offset, East offset — both start points normalised
to (0,0); shaded spans mark when avoidance is active (blue = ours, orange = reference).
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

M_PER_DEG_LAT = 111320.0


def _spans(t: np.ndarray, active: np.ndarray) -> list[tuple[float, float]]:
    out, start = [], None
    for i, a in enumerate(active):
        if a > 0.5 and start is None:
            start = t[i]
        elif a <= 0.5 and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def main(dpsi: float) -> None:
    d = Path(__file__).parent / "_out"
    o = np.load(d / f"ours_{int(dpsi)}.npz")
    r = np.load(d / f"ref_{int(dpsi)}.npz")
    m_lon = M_PER_DEG_LAT * np.cos(np.radians(52.0))
    os_, rs_ = _spans(o["t"], o["active"]), _spans(r["t"], r["active"])

    fig, ax = plt.subplots(2, 2, figsize=(15, 9))
    a = ax[0, 0]
    for s, e in os_:
        a.axvspan(s, e, color="tab:blue", alpha=0.10, lw=0)
    for s, e in rs_:
        a.axvspan(s, e, color="tab:orange", alpha=0.10, lw=0)
    a.plot(o["t"], o["gs"], "tab:blue", lw=1.8, label="ours gs (achieved)")
    a.plot(o["t"], o["cmdgs"], "tab:blue", lw=1.2, ls=":", label="ours gs (commanded)")
    a.plot(r["t"], r["gs"], "tab:orange", lw=1.6, ls="--", label="ref gs (achieved)")
    a.plot(r["t"], r["cmdgs"], "tab:red", lw=1.2, ls=":", label="ref gs (commanded)")
    a.set_title("ground speed [m/s] — achieved vs commanded")
    a.set_xlabel("t [s]"); a.set_ylabel("gs [m/s]"); a.grid(True, alpha=0.3); a.legend(fontsize=8)

    rest = [
        (o["trk"], r["trk"], "heading/track [deg]"),
        ((o["lat"] - o["lat"][0]) * M_PER_DEG_LAT, (r["lat"] - r["lat"][0]) * M_PER_DEG_LAT,
         "North offset from start [m]"),
        ((o["lon"] - o["lon"][0]) * m_lon, (r["lon"] - r["lon"][0]) * m_lon,
         "East offset from start [m]"),
    ]
    for a, (oy, ry, lbl) in zip((ax[0, 1], ax[1, 0], ax[1, 1]), rest):
        for s, e in os_:
            a.axvspan(s, e, color="tab:blue", alpha=0.10, lw=0)
        for s, e in rs_:
            a.axvspan(s, e, color="tab:orange", alpha=0.10, lw=0)
        a.plot(o["t"], oy, "tab:blue", lw=1.8, label="OURS")
        a.plot(r["t"], ry, "tab:orange", lw=1.5, ls="--", label="CDaRR_git")
        a.set_title(lbl); a.set_xlabel("t [s]"); a.set_ylabel(lbl)
        a.grid(True, alpha=0.3); a.legend(loc="best")

    fig.suptitle(
        f"Ownship (init hdg 0) — no-noise {int(dpsi)} deg conflict, lookahead 120s, rpz 50m, 250s"
        "   (shaded = avoidance active: blue = ours, orange = ref)", fontsize=12)
    fig.tight_layout()
    img_dir = Path(__file__).resolve().parents[2] / "vault" / "observations" / "img"
    img_dir.mkdir(parents=True, exist_ok=True)
    out = img_dir / f"trajectory-{int(dpsi)}deg.png"
    fig.savefig(out, dpi=105)
    print(f"wrote {out}")


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 2.0)
