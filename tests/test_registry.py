"""The name → component registry (``opencdarr/registry.py``).

The config file's half of the contribution surface: a YAML file can hold a *name*, not an instance,
so something must map ``resolution: mvp`` onto ``MVP(margin=1.05)``. These lock what that mapping
resolves, and — just as importantly — that it **fails loudly** on a name it does not know, since a
silently-ignored component would produce a plausible number for the wrong model.
"""

from __future__ import annotations

import pytest

from opencdarr.cd import StateBased
from opencdarr.cr import MVP, VO
from opencdarr.crr import FTR, PastCPA
from opencdarr.experiment import registry
from opencdarr.performance import M600


def test_resolves_the_names_it_documents() -> None:
    assert registry.make_perf("M600") is M600
    assert isinstance(registry.make_detector("statebased"), StateBased)
    assert isinstance(registry.make_resolver("mvp", 1.05), MVP)
    assert isinstance(registry.make_resolver("vo", 1.05), VO)
    assert isinstance(registry.make_recovery("pastcpa", True), PastCPA)
    assert isinstance(registry.make_recovery("ftr", False), FTR)


def test_config_values_are_threaded_not_defaulted() -> None:
    """The margin and the bouncing guard come from the config, not from the class default."""
    assert registry.make_resolver("mvp", 1.4).margin == 1.4
    assert registry.make_resolver("vo", 1.2).margin == 1.2
    assert registry.make_recovery("pastcpa", True).bouncing_guard is True
    assert registry.make_recovery("pastcpa", False).bouncing_guard is False


def test_none_means_the_baseline_not_a_missing_name() -> None:
    """``resolution: null`` is the no-resolution baseline — a legitimate configuration."""
    assert registry.make_resolver(None, 1.05) is None
    assert registry.make_recovery(None, False) is None


def test_unknown_names_fail_loudly() -> None:
    """Each ladder names the thing it could not resolve, so a typo is obvious from the message."""
    with pytest.raises(ValueError, match="unknown aircraft_type"):
        registry.make_perf("A320")
    with pytest.raises(ValueError, match="unknown detector"):
        registry.make_detector("bogus")
    with pytest.raises(ValueError, match="unknown resolver"):
        registry.make_resolver("orca", 1.05)
    with pytest.raises(ValueError, match="unknown recovery"):
        registry.make_recovery("probabilistic_ftr", False)


def test_probabilistic_ftr_is_deliberately_unreachable_from_a_config() -> None:
    """The documented limit, pinned so it is a decision rather than a surprise.

    ``ProbabilisticFTR`` exists and takes ``prob_threshold`` / ``ktheta``, which ``MethodsConfig``
    has no fields for — so naming it here would only half-work. It is reachable by passing the
    instance to ``run_experiment``, which is the point of the instance-taking entry point. If a
    future registry gains parameterised names, this test is the one to delete.
    """
    with pytest.raises(ValueError, match="unknown recovery"):
        registry.make_recovery("probabilistic_ftr", False)
