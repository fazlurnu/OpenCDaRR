"""The random-traffic environment: the entry rule, the ring, and the measurement area.

``random_traffic`` + :class:`MeasurementArea` are the pieces a traffic study is built from, and
both carry properties that are easy to break silently. The entry rule has to spread the miss
distance uniformly *across the diameter* — a uniform entry bearing would look almost identical and
crowd the traffic to the edge. The measurement area has to drop a pair the moment either aircraft
leaves the disc, because that gated running minimum is what IPS splits on; and its departure stop
has to be a pure saving, changing when a run ends but never what it measured.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from opencdarr import MeasurementArea, geo
from opencdarr import scenario as sc
from opencdarr.autopilot import WaypointAutopilot
from opencdarr.cd import StateBased
from opencdarr.cns.navigation import GnssNavigation
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.mission import Mission
from opencdarr.performance import M600
from opencdarr.rng import children, generator, root_seed_sequence
from opencdarr.state import AircraftState

_CENTRE = (52.0, 4.0)
_R_INNER = 1000.0
_R_OUTER = 1200.0
_RPZ = 50.0


def _offset(state: AircraftState, centre: tuple[float, float] = _CENTRE) -> float:
    """The signed perpendicular distance from ``centre`` to this aircraft's track [m]."""
    qdr, dist = geo.qdrdist(state.lat, state.lon, centre[0], centre[1])
    return dist * math.sin(math.radians(state.trk - qdr))


def _agents(fleet: sc.FleetScenario) -> list[Agent]:
    return [
        Agent(state, M600, autopilot=WaypointAutopilot(Mission(goto=target), capture_radius=30.0))
        for state, target in fleet
    ]


# --- the entry rule ------------------------------------------------------------------------------
def test_random_traffic_offsets_are_uniform_across_the_diameter() -> None:
    """The miss distance from the centre is uniform on [-r_inner, r_inner], not the bearing.

    A uniform *bearing* would put ~29% of the offsets in the outer fifth of the diameter instead
    of the 20% a uniform offset gives, which is the difference this rule exists to make.
    """
    rng = generator(root_seed_sequence(0))
    offsets = np.array([_offset(s) for s, _ in sc.random_traffic(4000, rng, r_inner=_R_INNER,
                                                                 r_outer=_R_OUTER)])
    assert np.all(np.abs(offsets) <= _R_INNER + 1.0)

    counts, _ = np.histogram(offsets, bins=10, range=(-_R_INNER, _R_INNER))
    expected = offsets.size / 10
    assert np.all(np.abs(counts - expected) < 4 * math.sqrt(expected))  # ~4 sigma per bin


def test_random_traffic_releases_outside_and_crosses_the_disc() -> None:
    """Every aircraft starts on the release circle and its track cuts the measured disc."""
    rng = generator(root_seed_sequence(1))
    for state, target in sc.random_traffic(200, rng, r_inner=_R_INNER, r_outer=_R_OUTER):
        _, from_centre = geo.qdrdist(_CENTRE[0], _CENTRE[1], state.lat, state.lon)
        assert from_centre == pytest.approx(_R_OUTER, abs=1.0)
        _, to_target = geo.qdrdist(state.lat, state.lon, target[0], target[1])
        # the chord across the release circle, which the offset shortens
        assert to_target == pytest.approx(2 * math.sqrt(_R_OUTER**2 - _offset(state) ** 2),
                                          rel=1e-3)


def test_random_traffic_rejects_a_release_circle_inside_the_disc() -> None:
    with pytest.raises(ValueError, match="r_outer"):
        sc.random_traffic(2, generator(root_seed_sequence(2)), r_inner=1000.0, r_outer=500.0)


# --- the ring ------------------------------------------------------------------------------------
def test_crossing_ring_is_antipodal_at_every_n() -> None:
    """Each aircraft flies a full diameter, odd fleet sizes included."""
    for n in (2, 3, 4, 5, 6):
        for state, target in sc.crossing_ring(n, radius=500.0):
            _, from_centre = geo.qdrdist(_CENTRE[0], _CENTRE[1], state.lat, state.lon)
            _, span = geo.qdrdist(state.lat, state.lon, target[0], target[1])
            assert from_centre == pytest.approx(500.0, abs=1.0)
            assert span == pytest.approx(1000.0, abs=1.0)


def test_crossing_ring_matches_swap_ring_when_n_is_even() -> None:
    """The two builders differ only at odd n, where swap_ring aims at another aircraft's start."""
    for n in (2, 4, 8):
        for (_, a), (_, b) in zip(sc.crossing_ring(n, radius=1500.0),
                                  sc.swap_ring(n, radius=1500.0), strict=True):
            assert a == pytest.approx(b, abs=1e-6)
    # ... and at n = 3 they genuinely disagree: 120 deg round the ring is not the antipode
    _, odd_target = sc.swap_ring(3, radius=1500.0)[0]
    _, antipode = sc.crossing_ring(3, radius=1500.0)[0]
    _, apart = geo.qdrdist(odd_target[0], odd_target[1], antipode[0], antipode[1])
    assert apart > 1000.0


# --- the measurement area ------------------------------------------------------------------
def test_measurement_area_ignores_pairs_outside_the_disc() -> None:
    """Two aircraft in loss of separation *outside* the area are not measured as one."""
    far = geo.forward(_CENTRE[0], _CENTRE[1], 0.0, 5000.0)  # well beyond r_inner
    near = geo.forward(far[0], far[1], 90.0, 20.0)  # 20 m apart: an unmistakable LoS
    pair = [
        Agent(AircraftState(id="A", lat=far[0], lon=far[1], trk=0.0, gs=10.0), M600),
        Agent(AircraftState(id="B", lat=near[0], lon=near[1], trk=0.0, gs=10.0), M600),
    ]
    common = dict(rpz=_RPZ, t_lookahead=30.0, dt=0.5, detector=StateBased(), t_max=20.0)

    ungated = run_fleet(pair, **common)
    assert ungated.los is True and ungated.min_sep < _RPZ

    gated = run_fleet(pair, measure_within=MeasurementArea(_CENTRE, _R_INNER,
                                                           stop_when_departed=False), **common)
    assert gated.los is False and gated.min_sep == math.inf


def test_departure_stop_does_not_change_what_was_measured() -> None:
    """Stopping once everyone has left the disc is a saving, not a different measurement."""
    rules = dict(rpz=_RPZ, t_lookahead=30.0, dt=0.5, detector=StateBased(),
                 resolver=MVP(1.05), recovery=PastCPA(bouncing_guard=True),
                 t_max=400.0, done_timeout=10.0, stop_within=50.0)
    for seq in children(root_seed_sequence(3), 0, 5):
        geom, fwd = children(seq, 0, 2)
        fleet = sc.random_traffic(4, generator(geom), r_inner=_R_INNER, r_outer=_R_OUTER,
                                  pos_ci95=10.0, vel_ci95=1.0)
        runs = [
            run_fleet(_agents(fleet), navigation=GnssNavigation(), rng=generator(fwd),
                      measure_within=MeasurementArea(_CENTRE, _R_INNER, stop_when_departed=stop),
                      **rules)
            for stop in (False, True)
        ]
        assert runs[0].min_sep == runs[1].min_sep
        assert runs[0].los == runs[1].los


# --- per-aircraft speed ---------------------------------------------------------------------
def test_a_scalar_speed_applies_to_every_aircraft() -> None:
    fleet = sc.crossing_ring(4, speed=12.0, radius=500.0)
    assert [s.gs for s, _ in fleet] == [12.0] * 4


def test_a_sequence_sets_each_aircraft_speed_in_fleet_order() -> None:
    """A mixed fleet needs this: one shared speed either stalls a fixed-wing or over-flies a
    multirotor, because their envelopes overlap only in a narrow band."""
    speeds = (10.0, 20.0, 12.0, 15.0)
    for fleet in (sc.crossing_ring(4, speed=speeds, radius=500.0),
                  sc.swap_ring(4, speed=speeds, radius=1500.0),
                  sc.converging_ring(4, speed=speeds, radius=1500.0),
                  sc.random_traffic(4, generator(root_seed_sequence(0)), speed=speeds)):
        assert tuple(s.gs for s, _ in fleet) == speeds
    assert tuple(s.gs for s, _ in sc.swap_pair(speed=(10.0, 20.0))) == (10.0, 20.0)


def test_a_speed_list_that_does_not_match_the_fleet_is_refused() -> None:
    with pytest.raises(ValueError, match="places 4 aircraft"):
        sc.crossing_ring(4, speed=(10.0, 20.0), radius=500.0)


def test_a_mixed_fleet_flies_each_airframe_at_its_own_speed() -> None:
    """The three-type ring: without per-aircraft speeds the fixed-wing is below its stall."""
    from opencdarr.estimator import agents_for
    from opencdarr.fleet import Airframe
    from opencdarr.kinematics import FixedWing, Multirotor
    from opencdarr.performance import SMALL_FIXEDWING

    mixed = [Airframe(M600, Multirotor()), Airframe(SMALL_FIXEDWING, FixedWing())]
    shared = sc.crossing_ring(2, speed=10.0, radius=1500.0)
    with pytest.raises(ValueError, match="outside its envelope"):
        agents_for(shared, M600, airframes=mixed)  # 10 m/s is under the fixed-wing's 12 m/s stall

    each = sc.crossing_ring(2, speed=(10.0, 20.0), radius=1500.0)
    agents = agents_for(each, M600, airframes=mixed)
    assert [a.state.gs for a in agents] == [10.0, 20.0]
