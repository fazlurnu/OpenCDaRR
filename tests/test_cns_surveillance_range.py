"""A hard surveillance range: beyond ``max_range`` no message is ever received.

The tests split the way ``test_cns_transceiver.py`` splits, because the two halves fail
independently: the **geometry** (where the cutoff actually falls, and that it is measured between
true positions) and the **stream** (that a closed link spends no draw, which is what makes this a
veto rather than ``reception_prob = 0`` — ADR 0019 §4).
"""

from __future__ import annotations

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import (
    BroadcastSchedule,
    Comm,
    CommState,
    Message,
    RadioHealth,
    RadioHealthState,
    SurveillanceRange,
    SurveillanceRangeState,
    constant_latency,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.performance import M600
from opencdarr.state import AircraftState

LAT0, LON0 = 52.0, 4.0
RANGE = 5000.0


def _at(aid: str, north_m: float = 0.0, gs: float = 10.0) -> AircraftState:
    """``aid`` placed ``north_m`` metres due north of the reference point."""
    lat, lon = geo.forward(LAT0, LON0, 0.0, north_m)
    return AircraftState(id=aid, lat=lat, lon=lon, trk=0.0, gs=gs)


def _msg(source: str, t_meas: float = 0.0, north_m: float = 0.0) -> Message:
    return Message(source=source, state=_at(source, north_m), t_meas=t_meas)


def _rng() -> np.random.Generator:
    return np.random.default_rng(0)


def _held(separation: float, model: Comm | None = None) -> set[tuple[str, str]]:
    """Which ``(receiver, source)`` slots fill when OWN and INT are ``separation`` metres apart."""
    comm = model or Comm(reception_prob=1.0, latency=0.0,
                         gates=(SurveillanceRange(max_range=RANGE),))
    roster = (_at("OWN"), _at("INT", separation))
    state = comm.step(comm.initial_state(), [_msg("OWN"), _msg("INT", north_m=separation)],
                      roster, 0.0, _rng())
    return set(state.held)


# --- the geometry: where the cutoff falls -------------------------------------------------------


def test_a_pair_inside_the_range_communicates_normally() -> None:
    assert _held(RANGE - 1.0) == {("INT", "OWN"), ("OWN", "INT")}


def test_a_pair_beyond_the_range_exchanges_nothing() -> None:
    """Both directions close: the geometry is symmetric even though the links are not."""
    assert _held(RANGE + 1.0) == set()


def test_the_link_is_admitted_at_exactly_the_range() -> None:
    """``d <= max_range``: a system required to see out to ``d_surv_min`` works *at* that distance.

    The boundary is placed deliberately rather than left to whichever comparison got written, so
    the two aircraft are constructed exactly ``RANGE`` apart and the assertion is on that pair.
    """
    lat, lon = geo.forward(LAT0, LON0, 0.0, RANGE)
    assert geo.qdrdist(LAT0, LON0, lat, lon)[1] == pytest.approx(RANGE, abs=1e-6)
    assert _held(RANGE) == {("INT", "OWN"), ("OWN", "INT")}


def test_the_cutoff_is_where_max_range_says_it_is() -> None:
    """Sweep the separation: every pair below the range talks, every pair above is silent."""
    comm = Comm(reception_prob=1.0, latency=0.0,
                gates=(SurveillanceRange(max_range=RANGE),))
    delivered = {d: bool(_held(d, comm)) for d in (100.0, 2500.0, 4999.0, 5001.0, 9000.0, 5e4)}
    assert delivered == {100.0: True, 2500.0: True, 4999.0: True,
                         5001.0: False, 9000.0: False, 5e4: False}


def test_range_is_measured_between_true_positions_not_broadcast_ones() -> None:
    """A wildly wrong self-fix on the air cannot open or close a link.

    The gate reads the roster (truth); the broadcast carries the sender's *measured* state. Here
    the two disagree by 100 km — if the gate were reading the message, the in-range pair would go
    silent.
    """
    comm = Comm(reception_prob=1.0, latency=0.0,
                gates=(SurveillanceRange(max_range=RANGE),))
    roster = (_at("OWN"), _at("INT", 1000.0))  # truly 1 km apart: well inside the range
    liar = Message(source="INT", state=_at("INT", 100_000.0), t_meas=0.0)  # claims 100 km north
    state = comm.step(comm.initial_state(), [liar], roster, 0.0, _rng())
    assert ("OWN", "INT") in state.held


def test_an_aircraft_off_the_roster_raises_rather_than_being_let_through() -> None:
    """Admitting an unknown aircraft would silently mean "always in range" — fail loudly."""
    comm = Comm(gates=(SurveillanceRange(max_range=RANGE),))
    with pytest.raises(ValueError, match="no position for 'GHOST'"):
        comm.step(comm.initial_state(), [_msg("GHOST")], (_at("OWN"),), 0.0, _rng())


def test_a_message_already_in_flight_still_arrives() -> None:
    """The veto applies when a broadcast is *offered*, not when delivered (as RadioHealth)."""
    comm = Comm(reception_prob=1.0, latency=constant_latency(2.0),
                gates=(SurveillanceRange(max_range=RANGE),))
    near = (_at("OWN"), _at("INT", 1000.0))
    sent = comm.step(comm.initial_state(), [_msg("INT", north_m=1000.0)], near, 0.0, _rng())
    assert len(sent.in_flight) == 1  # accepted while in range, due at t = 2

    far = (_at("OWN"), _at("INT", 50_000.0))  # flew far out of range in the meantime
    later = comm.step(sent, [], far, 2.0, _rng())
    assert ("OWN", "INT") in later.held
    assert later.in_flight == ()


def test_invalid_ranges_raise() -> None:
    for bad in (0.0, -1.0, float("nan")):
        with pytest.raises(ValueError, match="max_range"):
            SurveillanceRange(max_range=bad)
    SurveillanceRange(max_range=float("inf"))  # a gate that never vetoes is legitimate


# --- the stream: a veto spends no randomness ----------------------------------------------------


def _delivery_trace(comm: Comm, separations: list[float], seed: int) -> list[set[str]]:
    """Which links delivered a *fresh* message each tick, stepping through ``separations``."""
    rng = np.random.default_rng(seed)
    state = comm.initial_state()
    trace: list[set[str]] = []
    for k, separation in enumerate(separations):
        t = float(k)
        roster = (_at("OWN"), _at("INT", separation))
        state = comm.step(state, [_msg("OWN", t), _msg("INT", t, north_m=separation)],
                          roster, t, rng)
        trace.append({f"{src}->{rcv}" for (rcv, src), m in state.held.items() if m.t_meas == t})
    return trace


def test_an_all_in_range_run_is_bit_for_bit_the_ungated_channel() -> None:
    """The property that makes this a veto: an admitted link consumes exactly what ``Comm`` does.

    The gate draws nothing in ``evolve`` and skips nothing here, so a run that never leaves the
    range must reproduce the plain channel's stream *exactly* — not merely its statistics. Spelled
    as ``reception_prob = 0`` beyond the range this would fail, because the suppressed links would
    still have spent their draws.
    """
    near = [1000.0] * 40
    gated = _delivery_trace(
        Comm(reception_prob=0.5, gates=(SurveillanceRange(max_range=RANGE),)), near, seed=3
    )
    plain = _delivery_trace(Comm(reception_prob=0.5), near, seed=3)
    assert gated == plain
    assert any(links for links in gated)  # the trace really has deliveries to compare


def test_a_closed_link_spends_no_reception_draw() -> None:
    """Out of range for a stretch, then back: the deliveries afterwards are the ones a run that
    never offered those links at all would have made.

    The reference run is the same channel with the same seed, stepped only over the ticks the gate
    admits. If a closed link burned a draw, the two would diverge from the first re-entry onwards.
    """
    comm = Comm(reception_prob=0.5, gates=(SurveillanceRange(max_range=RANGE),))
    separations = [1000.0] * 10 + [50_000.0] * 10 + [1000.0] * 10
    trace = _delivery_trace(comm, separations, seed=5)
    assert trace[10:20] == [set()] * 10  # nothing at all while out of range

    # the same channel, ungated, stepped over just the in-range ticks
    plain = _delivery_trace(Comm(reception_prob=0.5), [1000.0] * 20, seed=5)
    assert trace[:10] + trace[20:] == plain


def test_it_composes_with_radio_health_without_either_gate_seeing_the_other() -> None:
    """Two gates, two independent states: a link is offered only if both admit it (ADR 0019 §3)."""
    comm = Comm(reception_prob=1.0, latency=0.0,
                gates=(SurveillanceRange(max_range=RANGE), RadioHealth()))
    roster = (_at("OWN"), _at("INT", 1000.0))
    broadcasts = [_msg("OWN"), _msg("INT", north_m=1000.0)]

    # in range, every radio working: both links deliver
    both_fine = comm.step(comm.initial_state(), broadcasts, roster, 0.0, _rng())
    assert set(both_fine.held) == {("INT", "OWN"), ("OWN", "INT")}
    assert len(both_fine.gates) == 2
    assert isinstance(both_fine.gates[0], SurveillanceRangeState)
    assert isinstance(both_fine.gates[1], RadioHealthState)

    # in range, but OWN's transmitter is out: the range gate admits, the radio gate does not
    silent = comm.step(
        CommState(gates=(SurveillanceRangeState(), RadioHealthState(tx_down=frozenset({"OWN"})))),
        broadcasts, roster, 0.0, _rng(),
    )
    assert set(silent.held) == {("OWN", "INT")}  # only INT -> OWN survives


# --- through the real loop ----------------------------------------------------------------------


def test_a_range_too_short_for_the_encounter_leaves_the_pair_blind() -> None:
    """End to end: a range the pair never satisfies in time means they never hear each other, so no
    resolution is commanded and a head-on runs all the way to 0 m.

    ``run_fleet`` is what threads the true roster down to the gate, so this is the test that would
    fail if that plumbing regressed — every test above calls ``Comm.step`` directly.

    Only the two extremes are asserted. In between, min_sep does **not** fall monotonically with
    the range: a link that opens late gives a resolution that is both later and geometrically
    different, and 2 km happens to come out worse than 1 km here. That is a real effect worth
    studying, not a property to pin in a test.
    """
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=90.0, gs=15.0)
    intr = AircraftState(id="INT", lat=LAT0, lon=LON0 + 0.2, trk=270.0, gs=15.0)  # ~13.7 km apart

    def once(max_range: float) -> float:
        out = run_fleet(
            [Agent(own, M600), Agent(intr, M600)],
            rpz=50.0, t_lookahead=300.0, dt=1.0, detector=StateBased(),
            resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
            communication=Comm(reception_prob=1.0, latency=0.0,
                               gates=(SurveillanceRange(max_range=max_range),)),
            comm_rng=np.random.default_rng(0),
            schedule=BroadcastSchedule(interval=1.0),
        )
        return out.min_sep

    # a range covering the whole encounter: in contact from the first tick, so they resolve wide
    assert once(100_000.0) > 1000.0
    # 20 m: in range only once they are already on top of each other — an unresolved head-on
    assert once(20.0) < 1.0
