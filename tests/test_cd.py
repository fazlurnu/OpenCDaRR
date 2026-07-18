"""Functional tests for state-based detection (`detect`, `is_los`)."""

from __future__ import annotations

from opencdarr.cd import StateBased, is_los
from opencdarr.state import AircraftState

_DET = StateBased()


def detect(own: AircraftState, intr: AircraftState, rpz: float, t_lookahead: float) -> bool:
    return _DET.detect(own, intr, rpz, t_lookahead)

# At lat 52, 1 deg lon ~ 68.5 km, 1 deg lat ~ 111.3 km. Offsets below are chosen so the
# geometry is unambiguously conflict / non-conflict, robust to the exact projection.
_LAT = 52.0
_LON = 4.0
_RPZ = 50.0
_LOOKAHEAD = 120.0


def _ac(id: str, lat: float, lon: float, trk: float, gs: float = 10.0) -> AircraftState:
    return AircraftState(id=id, lat=lat, lon=lon, trk=trk, gs=gs)


def test_head_on_is_conflict() -> None:
    """Two aircraft closing head-on (E vs W) within lookahead -> conflict."""
    own = _ac("A", _LAT, _LON, trk=90.0)  # heading East
    intr = _ac("B", _LAT, _LON + 0.009, trk=270.0)  # ~617 m East, heading West
    assert detect(own, intr, _RPZ, _LOOKAHEAD) is True
    assert is_los(own, intr, _RPZ) is False  # still ~617 m apart


def test_diverging_is_not_conflict() -> None:
    """Aircraft moving apart -> no conflict."""
    own = _ac("A", _LAT, _LON, trk=270.0)  # heading West (away)
    intr = _ac("B", _LAT, _LON + 0.009, trk=90.0)  # heading East (away)
    assert detect(own, intr, _RPZ, _LOOKAHEAD) is False


def test_clear_miss_is_not_conflict() -> None:
    """Large lateral offset (dcpa >> rpz) -> no conflict even while closing in longitude."""
    own = _ac("A", _LAT, _LON, trk=90.0)
    intr = _ac("B", _LAT + 0.02, _LON + 0.009, trk=270.0)  # ~2.2 km north
    assert detect(own, intr, _RPZ, _LOOKAHEAD) is False


def test_parallel_same_speed_is_not_conflict() -> None:
    """No relative motion -> no predicted conflict."""
    own = _ac("A", _LAT, _LON, trk=90.0, gs=10.0)
    intr = _ac("B", _LAT, _LON + 0.009, trk=90.0, gs=10.0)
    assert detect(own, intr, _RPZ, _LOOKAHEAD) is False


def test_conflict_only_within_lookahead() -> None:
    """A far, slow-closing conflict is seen with a long lookahead but not a short one."""
    own = _ac("A", _LAT, _LON, trk=90.0)
    intr = _ac("B", _LAT, _LON + 0.03, trk=270.0)  # ~2 km East, closing at 20 m/s (~100 s)
    assert detect(own, intr, _RPZ, t_lookahead=120.0) is True
    assert detect(own, intr, _RPZ, t_lookahead=60.0) is False


def test_detection_is_symmetric_in_truth() -> None:
    """With true (noise-free) states, the directed verdict is the same both ways."""
    a = _ac("A", _LAT, _LON, trk=90.0)
    b = _ac("B", _LAT, _LON + 0.009, trk=270.0)
    assert detect(a, b, _RPZ, _LOOKAHEAD) == detect(b, a, _RPZ, _LOOKAHEAD)


def test_is_los_true_when_inside_rpz() -> None:
    """Within rpz -> LoS true; well outside -> false."""
    own = _ac("A", _LAT, _LON, trk=90.0)
    near = _ac("B", _LAT, _LON + 0.0002, trk=270.0)  # ~14 m East
    far = _ac("C", _LAT, _LON + 0.009, trk=270.0)  # ~617 m East
    assert is_los(own, near, _RPZ) is True
    assert is_los(own, far, _RPZ) is False
