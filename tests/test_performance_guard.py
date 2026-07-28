"""The performance-envelope guard (``Dynamics.validate_performance``).

:class:`~opencdarr.performance.Performance` is one flat bag of numbers shared across airframes, so
a field an airframe does not use sits at its ``0.0`` default — and ``0.0`` is also a value the
reading integrator interprets. A mismatched envelope therefore flies **silently wrong** rather than
failing: a fixed-wing handed a multirotor's ``phi_max == 0`` never banks, so it can never turn.

Each dynamics rejects an envelope it cannot fly, both at :class:`~opencdarr.fleet.Agent`
construction (the explicit case, failing at the line the mistake is written) and again at the
composition root (the ``dynamics=None`` default case, whose effective model is not known earlier).
"""

from __future__ import annotations

import pytest

from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.dynamics import FixedWing, Multirotor
from opencdarr.fleet import Agent, run_fleet
from opencdarr.performance import M600, SMALL_FIXEDWING
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_OWN = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=15.0, yaw=0.0)


def test_matched_envelopes_construct() -> None:
    """The two built-in airframes pair with their own dynamics without complaint."""
    Agent(_OWN, M600, Multirotor())
    Agent(_OWN, SMALL_FIXEDWING, FixedWing())


def test_fixedwing_rejects_multirotor_envelope() -> None:
    """A fixed-wing with a multirotor envelope (phi_max == 0) is caught, not flown straight."""
    with pytest.raises(ValueError, match="phi_max"):
        Agent(_OWN, M600, FixedWing())


def test_multirotor_rejects_fixedwing_envelope() -> None:
    """A multirotor with a banking (fixed-wing) envelope is caught as a misapplied envelope."""
    with pytest.raises(ValueError, match="fixed-wing"):
        Agent(_OWN, SMALL_FIXEDWING, Multirotor())


def test_default_dynamics_mismatch_caught_at_composition_root() -> None:
    """``dynamics=None`` defers to the Multirotor default, so the mismatch surfaces in run_fleet."""
    intr = create_conflict(_OWN, intr_id="INTR", dpsi=90.0, dcpa=0.0,
                           tlos=30.0, rpz=50.0, gs_intr=15.0, side=1)
    # Agent construction is fine — the effective dynamics is not known yet.
    agents = [Agent(_OWN, SMALL_FIXEDWING), Agent(intr, SMALL_FIXEDWING)]
    with pytest.raises(ValueError, match="Multirotor"):
        run_fleet(agents, rpz=50.0, t_lookahead=20.0, dt=0.5,
                  detector=StateBased(), resolver=MVP(), recovery=PastCPA())


def test_error_names_the_fix() -> None:
    """The message points at a working envelope so the reader knows what to pass instead."""
    with pytest.raises(ValueError, match="SMALL_FIXEDWING"):
        Agent(_OWN, M600, FixedWing())
