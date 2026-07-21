"""Offline anchor — `step_dynamics` vs BlueSky's own M600 integrator (ADR 0002/0003).

This is NOT part of the core gate: BlueSky is a heavy, boot-fragile fork. The test skips
unless BlueSky both imports and initialises, so `pytest` stays green everywhere while this
still runs as the periodic equivalence check where BlueSky is available (e.g. the `cdarr`
conda env). It validates the *geometry* our integrator owns — the turn-rate law and the
geodesy. We do model a speed ramp now, but its `ax` is assumed rather than BlueSky-sourced,
so to isolate geometry from that speed-value mismatch we feed BlueSky's own ground speed into
our step each tick; any residual is then real turn-law/geodesy disagreement.
"""

from __future__ import annotations

import numpy as np
import pytest

bluesky = pytest.importorskip("bluesky")

from opencdarr.dynamics import Command, step_dynamics  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402


@pytest.fixture(scope="module")
def bs():  # type: ignore[no-untyped-def]
    import bluesky as _bs

    try:
        _bs.init(mode="sim", detached=True)
    except Exception as exc:  # noqa: BLE001 - any boot failure -> skip, not fail
        pytest.skip(f"BlueSky failed to initialise: {exc}")
    return _bs


def _max_errors(bs, cmd_hdg: float, spd_ms: float, n_steps: int) -> tuple[float, float]:  # type: ignore[no-untyped-def]
    from bluesky.tools.aero import kts  # m/s per knot

    dt = float(bs.sim.simdt)
    spd_kts = spd_ms / kts

    bs.traf.reset()
    bs.traf.cre(
        acid="D0", actype="M600", aclat=52.0, aclon=4.0, achdg=0.0, acalt=100.0, acspd=spd_kts
    )

    ours = AircraftState(id="D0", lat=52.0, lon=4.0, trk=0.0, gs=float(bs.traf.gs[0]))

    max_pos = 0.0
    max_trk = 0.0
    for _ in range(n_steps):
        bs.stack.stack(f"HDG D0 {cmd_hdg}")
        bs.stack.stack(f"SPD D0 {spd_kts}")
        bs.sim.step()
        # feed BlueSky's own ground speed into our step, so speed matches by construction
        # and the comparison isolates the turn-rate law + geodesy (the deferred speed-accel
        # ramp is thereby excluded from the residual)
        bgs = float(bs.traf.gs[0])
        ours = step_dynamics(ours, Command.from_track_speed(cmd_hdg, bgs), M600, dt)

        blat, blon, btrk = float(bs.traf.lat[0]), float(bs.traf.lon[0]), float(bs.traf.trk[0])
        mlat = (ours.lat - blat) * 111320.0
        mlon = (ours.lon - blon) * 111320.0 * np.cos(np.radians(blat))
        max_pos = max(max_pos, float(np.hypot(mlat, mlon)))
        max_trk = max(max_trk, abs(((ours.trk - btrk + 180.0) % 360.0) - 180.0))
    return max_pos, max_trk


def test_straight_line_matches_bluesky(bs) -> None:  # type: ignore[no-untyped-def]
    pos_err, trk_err = _max_errors(bs, cmd_hdg=0.0, spd_ms=10.0, n_steps=200)
    assert trk_err < 1e-6  # heading matches to machine precision
    assert pos_err < 1.0  # sub-metre; residual is integration-order, not geometry


def test_turn_matches_bluesky(bs) -> None:  # type: ignore[no-untyped-def]
    pos_err, trk_err = _max_errors(bs, cmd_hdg=90.0, spd_ms=10.0, n_steps=400)
    assert trk_err < 1e-6  # turn-rate law matches BlueSky bit-for-bit
    assert pos_err < 1.0
