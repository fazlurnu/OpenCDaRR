"""Prototype benchmark: vectorized pairwise_relative vs the scalar path, plus a ulp audit.

Go/no-go for geodesy phase 2: at which fleet size does the numpy condensed-upper-triangle
sweep beat the per-pair scalar loop, by how much, and how far do the floats move?
"""
from __future__ import annotations

import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path("/Users/mfrahman/Projects/OpenCDaRR")
sys.path.insert(0, str(REPO))

from opencdarr import geo  # noqa: E402
from opencdarr.relative import Relative, pairwise_relative  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

_TRIU: dict[int, tuple[np.ndarray, np.ndarray]] = {}


def _triu(n: int) -> tuple[np.ndarray, np.ndarray]:
    if n not in _TRIU:
        _TRIU[n] = np.triu_indices(n, k=1)
    return _TRIU[n]


def qdrdist_many(lat1, lon1, lat2, lon2):
    """geo.qdrdist, formula for formula, over arrays (degrees in, deg/metres out)."""
    radius = geo.earth_radius_many(lat1) if hasattr(geo, "earth_radius_many") else None
    if radius is None:
        # inline the earth_radius formula vectorized (mirrors geo.earth_radius)
        radius = _earth_radius_many(lat1)
    p1 = np.radians(lat1)
    p2 = np.radians(lat2)
    dlat = p2 - p1
    dlon = np.radians(lon2 - lon1)
    a = np.sin(dlat / 2.0) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dlon / 2.0) ** 2
    dist = 2.0 * radius * np.arcsin(np.sqrt(a))
    y = np.sin(dlon) * np.cos(p2)
    x = np.cos(p1) * np.sin(p2) - np.sin(p1) * np.cos(p2) * np.cos(dlon)
    qdr = np.degrees(np.arctan2(y, x)) % 360.0
    return qdr, dist


def _earth_radius_many(lat_deg):
    """geo.earth_radius (WGS84, matches BlueSky rwgs84), formula for formula, over arrays."""
    a_ = geo._WGS84_A
    b_ = geo._WGS84_B
    lat = np.radians(lat_deg)
    cos_lat = np.cos(lat)
    sin_lat = np.sin(lat)
    an = a_ * a_ * cos_lat
    bn = b_ * b_ * sin_lat
    ad = a_ * cos_lat
    bd = b_ * sin_lat
    return np.sqrt((an * an + bn * bn) / (ad * ad + bd * bd))


def pairwise_relative_np(states) -> tuple[Relative, ...]:
    n = len(states)
    lat = np.fromiter((s.lat for s in states), dtype=float, count=n)
    lon = np.fromiter((s.lon for s in states), dtype=float, count=n)
    trk = np.fromiter((s.trk for s in states), dtype=float, count=n)
    gs = np.fromiter((s.gs for s in states), dtype=float, count=n)
    i, j = _triu(n)
    qdr, dist = qdrdist_many(lat[i], lon[i], lat[j], lon[j])
    q = np.radians(qdr)
    r = np.radians(trk)
    ve = gs * np.sin(r)
    vn = gs * np.cos(r)
    rx = dist * np.sin(q)
    ry = dist * np.cos(q)
    vx = ve[j] - ve[i]
    vy = vn[j] - vn[i]
    columns = zip(rx.tolist(), ry.tolist(), vx.tolist(), vy.tolist(), strict=True)
    return tuple(Relative(rx=a, ry=b, vx=c, vy=d) for a, b, c, d in columns)


def make_states(n: int, rng: np.random.Generator) -> list[AircraftState]:
    return [
        AircraftState(id=f"T{k}", lat=52.0 + rng.uniform(-0.02, 0.02),
                      lon=4.0 + rng.uniform(-0.03, 0.03),
                      trk=rng.uniform(0, 360), gs=rng.uniform(8, 13))
        for k in range(n)
    ]


def bench(fn, states, reps: int) -> float:
    fn(states)  # warm
    t0 = time.perf_counter()
    for _ in range(reps):
        fn(states)
    return (time.perf_counter() - t0) / reps * 1e6  # us/call


def main() -> None:
    rng = np.random.default_rng(1)
    step_cost = {2: 41.0, 8: 244.0, 13: 526.0}  # measured after phase 1, us/step
    print(f"{'n':>3} {'pairs':>6} {'scalar us':>10} {'numpy us':>9} {'speedup':>8} "
          f"{'max rel diff':>13} {'proj step':>10}")
    for n in (2, 4, 6, 8, 13, 20, 28):
        worst = 0.0
        for _ in range(200):
            states = make_states(n, rng)
            a = pairwise_relative(states)
            b = pairwise_relative_np(states)
            for ra, rb in zip(a, b, strict=True):
                for fa, fb in ((ra.rx, rb.rx), (ra.ry, rb.ry), (ra.vx, rb.vx), (ra.vy, rb.vy)):
                    scale = max(abs(fa), abs(fb), 1e-30)
                    worst = max(worst, abs(fa - fb) / scale)
        states = make_states(n, rng)
        reps = 4000 if n <= 8 else 1500
        us_s = bench(pairwise_relative, states, reps)
        us_n = bench(pairwise_relative_np, states, reps)
        proj = ""
        if n in step_cost:
            proj = f"-{100 * (us_s - us_n) / step_cost[n]:.0f}%" if us_n < us_s else "slower"
        print(f"{n:>3} {n*(n-1)//2:>6} {us_s:>10.1f} {us_n:>9.1f} {us_s/us_n:>7.2f}x "
              f"{worst:>13.2e} {proj:>10}")


if __name__ == "__main__":
    main()
