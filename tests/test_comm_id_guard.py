"""The communication-id guard (``CommunicationModel.validate_ids``).

A directed :class:`~opencdarr.cns.communication.Comm` keys ``reception_prob`` by aircraft id, and an
absent link defaults to ``1.0`` (perfect). A mistyped id therefore applies *no* loss on that link
silently, rather than the value written — the footgun that let a ``("COPTER", "PLANE")`` link do
nothing on a ``COPTER`` / ``CARGO`` fleet. The fleet composition root validates the model against
the real roster so the typo fails loudly instead.
"""

from __future__ import annotations

import pytest

from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GnssNavigation
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_OWN = AircraftState(id="COPTER", lat=52.0, lon=4.0, trk=0.0, gs=15.0, yaw=0.0)
_INTR = create_conflict(_OWN, intr_id="CARGO", dpsi=90.0, dcpa=0.0,
                        tlos=30.0, rpz=50.0, gs_intr=15.0, side=1)
_AGENTS = [Agent(_OWN, M600), Agent(_INTR, M600)]


def _run(comm: Comm):
    return run_fleet(
        _AGENTS, rpz=50.0, t_lookahead=20.0, dt=0.5,
        detector=StateBased(), resolver=MVP(), recovery=PastCPA(),
        navigation=GnssNavigation(), rng=generator(root_seed_sequence(1)),
        communication=comm, comm_rng=generator(root_seed_sequence(2)),
    )


def test_unknown_link_id_raises() -> None:
    """A reception_prob link naming an aircraft not in the fleet is caught, not ignored."""
    with pytest.raises(ValueError, match="PLANE"):
        _run(Comm(reception_prob={("COPTER", "PLANE"): 0.9}))


def test_error_lists_the_known_ids() -> None:
    """The message shows the actual roster so the fix is obvious."""
    with pytest.raises(ValueError, match="CARGO"):
        _run(Comm(reception_prob={("PLANE", "COPTER"): 0.6}))


def test_matching_ids_run() -> None:
    """Directed links that name real aircraft pass the guard."""
    _run(Comm(reception_prob={("COPTER", "CARGO"): 0.9, ("CARGO", "COPTER"): 0.6}))


def test_scalar_reception_prob_always_accepted() -> None:
    """A scalar reception_prob keys nothing, so there are no ids to check."""
    _run(Comm(reception_prob=0.9))


def test_validate_ids_unit() -> None:
    """The guard is callable directly on the model with an explicit roster."""
    Comm(reception_prob={("A", "B"): 0.5}).validate_ids(frozenset({"A", "B"}))
    with pytest.raises(ValueError, match=r"\['B'\]"):
        Comm(reception_prob={("A", "B"): 0.5}).validate_ids(frozenset({"A"}))
