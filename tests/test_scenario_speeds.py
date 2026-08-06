"""Per-aircraft cruise speed on the fleet builders — what a mixed fleet needs.

One speed cannot serve a mixed fleet. ``SMALL_FIXEDWING`` stalls at 12 m/s, above the 10 m/s a
multirotor normally cruises at, so a single fleet speed either stalls one airframe or flies the
other well above the speed it would really fly. :class:`~opencdarr.fleet.Agent` already refuses an
out-of-envelope speed; what changes here is that the *scenario* can express the difference instead
of the caller working around it.

That also makes the speed difference a subject of study — a GA-versus-UAS encounter is exactly a
fast aircraft meeting a slow one — rather than an obstacle to setting one up.
"""

from __future__ import annotations

import pytest

from opencdarr.fleet import Agent
from opencdarr.performance import M600, SMALL_FIXEDWING
from opencdarr.scenario import converging_ring, swap_pair, swap_ring


def test_a_scalar_speed_applies_to_every_aircraft() -> None:
    """The ordinary case is unchanged: one number, one fleet speed."""
    fleet = swap_ring(4, speed=12.0)
    assert [state.gs for state, _ in fleet] == [12.0] * 4


def test_a_sequence_gives_each_aircraft_its_own_speed() -> None:
    """The mixed-fleet spelling: one speed per aircraft, in fleet order."""
    speeds = [10.0, 14.0, 11.0, 13.0]
    fleet = swap_ring(4, speed=speeds)
    assert [state.gs for state, _ in fleet] == speeds


def test_a_sequence_of_the_wrong_length_is_refused() -> None:
    """A length mismatch is a declaration error, so it fails where it is written.

    Silently recycling or truncating would fly a fleet nobody declared, and the run would look
    entirely normal afterwards.
    """
    with pytest.raises(ValueError, match="3 entries but the scenario places 4"):
        swap_ring(4, speed=[10.0, 11.0, 12.0])


def test_the_pairwise_and_converging_builders_take_it_too() -> None:
    """Every fleet builder accepts both spellings — the scalar path is not a special case."""
    assert [s.gs for s, _ in swap_pair(speed=[9.0, 13.0])] == [9.0, 13.0]
    assert [s.gs for s, _ in converging_ring(3, speed=[10.0, 12.0, 14.0])] == [10.0, 12.0, 14.0]


def test_a_mixed_fleet_can_now_be_placed_at_all() -> None:
    """The point of the change: a multirotor and a fixed-wing, each inside its own envelope.

    A shared speed cannot do this. 10 m/s stalls the fixed-wing (v_min = 12); 14 m/s is fine for
    the fixed-wing but is well above the multirotor's normal cruise. Only a per-aircraft speed
    places both aircraft honestly, and ``Agent`` accepting them is the proof.
    """
    assert SMALL_FIXEDWING.v_min > 10.0        # the stall speed that makes one number impossible

    fleet = swap_pair(speed=[10.0, 14.0])
    pairs = zip(fleet, (M600, SMALL_FIXEDWING), strict=True)
    agents = [Agent(state, perf) for (state, _), perf in pairs]
    assert [a.state.gs for a in agents] == [10.0, 14.0]

    # and the shared-speed version really is refused, rather than merely discouraged
    shared = swap_pair(speed=10.0)
    with pytest.raises(ValueError, match="outside its envelope"):
        Agent(shared[1][0], SMALL_FIXEDWING)
