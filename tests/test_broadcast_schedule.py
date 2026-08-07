"""Transmit timing: the Hz spelling, and reaching **both** backends from a declaration.

``BroadcastSchedule`` has owned the cadence since it landed, but only the ``interval`` was
reachable from a config file or a ``run_experiment`` declaration — and on the IPS path not even
that, because ``_run_ips`` built its environment without passing a schedule at all and silently
took the 1 s default. These tests pin the wiring rather than the timing arithmetic, which
``BroadcastSchedule`` already had.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from opencdarr import experiment
from opencdarr.cd import StateBased
from opencdarr.cns import BroadcastSchedule
from opencdarr.cns.broadcast import schedule_for
from opencdarr.config import load_config
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.estimate.ips import estimate_rare_prob
from opencdarr.experiment import IPS, Fixed, Methods, run_experiment
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence

_EXAMPLE = Path(__file__).resolve().parent.parent / "configs" / "pairwise.yaml"


# --- the Hz spelling ---------------------------------------------------------------------------


def test_a_rate_is_the_reciprocal_interval() -> None:
    assert BroadcastSchedule.at_rate(2.0).interval == 0.5  # 2 Hz -> every 0.5 s
    assert BroadcastSchedule.at_rate(1.0).interval == 1.0
    assert BroadcastSchedule.at_rate(0.5).interval == 2.0


def test_the_two_spellings_are_the_same_value() -> None:
    """One stored field, so a schedule cannot remember which way it was written."""
    assert BroadcastSchedule.at_rate(1.0) == BroadcastSchedule()
    assert BroadcastSchedule.at_rate(4.0) == BroadcastSchedule(interval=0.25)
    assert BroadcastSchedule.at_rate(2.0, jitter=0.1) == BroadcastSchedule(0.5, jitter=0.1)


def test_at_rate_keeps_the_other_options() -> None:
    sched = BroadcastSchedule.at_rate(2.0, phase=[0.0, 0.25], jitter=0.1)
    assert sched.initial(2) == [0.0, 0.25]
    assert sched.jitter == 0.1


def test_an_impossible_rate_raises() -> None:
    for rate in (0.0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            BroadcastSchedule.at_rate(rate)
    # and the inherited constraint still applies: a dither cannot reach the period
    with pytest.raises(ValueError):
        BroadcastSchedule.at_rate(2.0, jitter=0.5)


# --- schedule_for: the one builder both backends use --------------------------------------------


def test_a_fixed_phase_draws_nothing() -> None:
    """``random_phase=False`` must not touch the generator — that is what leaves every existing
    number unmoved when the option is merely *available*."""
    rng = np.random.default_rng(0)
    before = rng.bit_generator.state
    sched = schedule_for(2, 1.0, rng, jitter=0.0, random_phase=False)
    assert sched == BroadcastSchedule(interval=1.0)
    assert rng.bit_generator.state == before  # untouched


def test_a_random_phase_is_drawn_within_the_interval_and_is_seeded() -> None:
    a = schedule_for(4, 2.0, np.random.default_rng(3), random_phase=True)
    b = schedule_for(4, 2.0, np.random.default_rng(3), random_phase=True)
    assert a == b  # same seed, same offsets
    assert a.phase is not None
    assert len(a.phase) == 4
    assert all(0.0 <= p < 2.0 for p in a.phase)
    assert len(set(a.phase)) == 4  # independent per aircraft, not one shared offset
    assert a != schedule_for(4, 2.0, np.random.default_rng(4), random_phase=True)


# --- the declaration reaches both backends ------------------------------------------------------


def _captured_ips_schedule(
    monkeypatch: pytest.MonkeyPatch, **simulation: Any
) -> BroadcastSchedule:
    """The schedule the IPS backend actually builds for a condition, via its own code path.

    White-box on purpose: the defect was that ``_run_ips`` never passed ``schedule=`` to
    ``build_env``, and the only honest assertion is on the environment IPS constructs. Reading it
    off a statistical difference between two runs would be both slower and flakier.
    """
    captured: dict[str, BroadcastSchedule] = {}

    def spy(build_initial: Any, shells: Any, **kwargs: Any) -> Any:
        captured["schedule"] = build_initial(root_seed_sequence(0)).env.schedule
        return estimate_rare_prob(build_initial, shells, **kwargs)

    cfg = load_config(_EXAMPLE)
    cfg = dataclasses.replace(cfg, simulation=dataclasses.replace(cfg.simulation, **simulation))
    monkeypatch.setattr(experiment, "estimate_rare_prob", spy)
    run_experiment(
        {"dpsi": Fixed(90.0)},
        methods=Methods(detector=StateBased(), resolver=MVP(margin=1.05),
                        recovery=PastCPA(bouncing_guard=True), perf=M600),
        backend=IPS(shells=[60.0, 50.0], n_particles=8, reps=1),
        base_config=cfg, seed=1,
    )
    return captured["schedule"]


def test_the_broadcast_interval_reaches_the_ips_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    """The regression. ``_run_ips`` built its env with no ``schedule=``, so a declared interval
    reached MC and was silently replaced by the 1 s default on IPS — the same declaration meaning
    two different things depending on the backend."""
    assert _captured_ips_schedule(monkeypatch, broadcast_interval=4.0).interval == 4.0
    assert _captured_ips_schedule(monkeypatch, broadcast_interval=1.0).interval == 1.0


def test_jitter_and_random_phase_reach_the_ips_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    sched = _captured_ips_schedule(monkeypatch, broadcast_interval=2.0, broadcast_jitter=0.3,
                                   broadcast_random_phase=True)
    assert sched.interval == 2.0
    assert sched.jitter == 0.3
    assert sched.phase is not None and len(sched.phase) == 2
    assert all(0.0 <= p < 2.0 for p in sched.phase)


def test_the_transmit_timing_is_declarable() -> None:
    """All three keys are accepted by the declaration surface, so a typo is the only failure."""
    for key in ("broadcast_interval", "broadcast_jitter", "broadcast_random_phase"):
        assert key in experiment._SIMULATION_FIELDS
    with pytest.raises(ValueError, match="broadcast_interval"):
        experiment.expand({"broadcast_intervals": Fixed(1.0)})  # the plural typo lists the keys


# --- config validation --------------------------------------------------------------------------


def test_a_dither_that_reaches_the_period_is_rejected_at_load(tmp_path: Path) -> None:
    """A gap of ``interval + U(-j, +j)`` must stay positive, so the dither cannot reach the
    period. Checked in the config's own vocabulary, at load, rather than part-way into a run."""
    text = _EXAMPLE.read_text().replace(
        "simulation:", "simulation:\n  broadcast_jitter: 1.0", 1
    )
    bad = tmp_path / "bad.yaml"
    bad.write_text(text)
    with pytest.raises(ValueError, match="broadcast_jitter"):
        load_config(bad)


def test_a_declared_dither_is_still_caught_downstream() -> None:
    """...but only *loading* validates. A declaration reaches the config through
    ``dataclasses.replace``, which runs no constraint checks, so an out-of-range declared value
    survives to the point of use. Here that is still a loud failure — ``BroadcastSchedule`` rejects
    it — which is why this is a sharp edge for the transmit fields rather than a wrong number.
    It is *not* harmless for every field; see [[todo-might-be-a-bug]] on declared ``dcpa_max``."""
    cfg = load_config(_EXAMPLE)
    built = experiment._config_for(
        experiment.expand({"broadcast_jitter": Fixed(cfg.simulation.broadcast_interval)})[0],
        cfg, 1,
    )
    assert built.simulation.broadcast_jitter == built.simulation.broadcast_interval  # no check ran
    with pytest.raises(ValueError, match="jitter must be"):
        schedule_for(2, built.simulation.broadcast_interval,
                     generator(root_seed_sequence(0)),
                     jitter=built.simulation.broadcast_jitter)


def test_the_defaults_are_todays_behaviour() -> None:
    """Both new fields default to off, so an existing config file is unchanged by them."""
    sim = load_config(_EXAMPLE).simulation
    assert sim.broadcast_jitter == 0.0
    assert sim.broadcast_random_phase is False
    assert schedule_for(2, sim.broadcast_interval, generator(root_seed_sequence(0)),
                        jitter=sim.broadcast_jitter,
                        random_phase=sim.broadcast_random_phase) == BroadcastSchedule(interval=1.0)
