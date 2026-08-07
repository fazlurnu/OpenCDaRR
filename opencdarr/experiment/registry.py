"""Name → component: the config file's half of the contribution surface.

Everything else in this package takes model *instances*, which is what makes a new algorithm "add a
file, not a fork" (``design_brief.md``). A **config file** cannot hold an instance, only a name —
so something has to turn ``resolution: mvp`` into an ``MVP(margin=1.05)``. That mapping is this
module, and it is the one thing a Python caller never needs: passing ``MVP(1.05)`` directly skips
it entirely.

That asymmetry is worth stating plainly, because it is a real limit rather than an oversight. The
ladders below know exactly six names, so a component that is not in them — including
:class:`~opencdarr.crr.ProbabilisticFTR`, which *exists*, and any resolver a contributor writes —
**cannot be reached from a YAML file at all**. A full registry (config-selectable plugins, so a
contributor can register their own name) is deferred to the first outside contribution; until then
the Python entry point (:func:`opencdarr.experiment.run_experiment`) is the complete surface and
this is the convenience path for a committable, diffable config.

The makers are public because that is now this module's whole job — it used to hide them behind a
single ``run_one_experiment``, which has moved to :mod:`opencdarr.experiment` where it belongs
alongside the estimator's other entry points.
"""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.cd.base import ConflictDetector
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import FTR, PastCPA
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.performance import M600, Performance

# name -> airframe limits. A mapping rather than an if-ladder because these are values, not
# constructions: there is nothing to parameterise.
_PERF = {"M600": M600}


def make_perf(name: str) -> Performance:
    """The named airframe's flight envelope."""
    try:
        return _PERF[name]
    except KeyError:
        raise ValueError(f"unknown aircraft_type {name!r}") from None


def make_detector(name: str) -> ConflictDetector:
    """The named conflict detector."""
    if name == "statebased":
        return StateBased()
    raise ValueError(f"unknown detector {name!r}")


def make_resolver(name: str | None, margin: float) -> ConflictResolver | None:
    """The named conflict resolver, or ``None`` for the no-resolution baseline."""
    if name is None:
        return None
    if name == "mvp":
        return MVP(margin=margin)
    if name == "vo":
        return VO(margin=margin)
    raise ValueError(f"unknown resolver {name!r}")


def make_recovery(name: str | None, bouncing_guard: bool) -> RecoveryCriterion | None:
    """The named recovery criterion, or ``None`` to keep resolving.

    ``probabilistic_ftr`` is deliberately **absent**: :class:`~opencdarr.crr.ProbabilisticFTR`
    takes a confidence threshold and an angular resolution, which ``MethodsConfig`` has no fields
    for, so naming it here would only half-work. Reach it by passing the instance instead.
    """
    if name is None:
        return None
    if name == "pastcpa":
        return PastCPA(bouncing_guard=bouncing_guard)
    if name == "ftr":
        return FTR()
    raise ValueError(f"unknown recovery {name!r}")
