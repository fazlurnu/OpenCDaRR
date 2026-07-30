"""OpenCDaRR — conflict detection, resolution & recovery research platform.

You own the state and the simulation loop; BlueSky is used as a library of stateless
math, not as the runtime. See ``docs/design_brief.md`` for the architecture and
``vault/phase-0-plan.md`` for the current build state.

**What is re-exported here, and why.** The package's promise is that a new algorithm is "add a
file, not a fork" (``design_brief.md``: the interface is the contribution surface), so the top
level exports what a contributor needs to *write* one and *run* it against the built-ins:

- **contribution surfaces** — ``ConflictDetector``, ``ConflictResolver``, ``RecoveryCriterion``,
  ``Dynamics``, ``NavigationModel``, ``CommunicationModel``, ``SurveillanceModel``,
  ``NoiseDistribution``;
- **reference implementations** to compare against — ``StateBased``, ``MVP``, ``VO``, ``PastCPA``,
  ``FTR``, ``ProbabilisticFTR``, ``Multirotor``, ``FixedWing``, ``GnssNavigation``, ``Comm``,
  ``LastKnown``;
- **runners and estimators** — ``run_encounter``, ``run_fleet``, ``estimate_ipr``,
  ``estimate_rare_prob``, ``run_one_experiment``;
- **the values you construct** — ``AircraftState``, ``MotionCommand``, ``Performance``, ``Agent``,
  ``Config``, plus the scenario builders and the seeded-RNG helpers.

Deliberately shallow: a submodule import still reaches everything (e.g.
``from opencdarr.cns.noise_distributions import make_mixture_gaussian``), and this list is the
short path for the common case rather than a mirror of the whole tree.

    from opencdarr import ConflictResolver, MotionCommand   # write your own ...
    from opencdarr import MVP, StateBased, PastCPA, M600    # ... against these
    from opencdarr import estimate_ipr, load_config         # ... and measure it

Both optional extras stay optional: ``matplotlib`` (via :mod:`opencdarr.viz`) and ``joblib`` (via
:mod:`opencdarr.parallel`) are imported lazily inside the functions that need them, so
``import opencdarr`` remains numpy + pyyaml only. :mod:`opencdarr.parallel` is deliberately *not*
re-exported — it is a scheduling concern with its own install extra, reached explicitly.

The import block below is sorted (ruff ``I001``) rather than grouped by role; the grouping above is
the map.
"""

from opencdarr import geo
from opencdarr.cd.base import ConflictDetector
from opencdarr.cd.statebased import StateBased
from opencdarr.cns.base import (
    CommunicationModel,
    NavigationModel,
    NoiseDistribution,
    SurveillanceModel,
)
from opencdarr.cns.communication import Comm
from opencdarr.cns.navigation import GnssNavigation
from opencdarr.cns.surveillance import LastKnown
from opencdarr.config import Config, load_config
from opencdarr.cr.base import ConflictResolver
from opencdarr.cr.mvp import MVP
from opencdarr.cr.vo import VO
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.crr.ftr import FTR
from opencdarr.crr.pastcpa import PastCPA
from opencdarr.crr.probabilistic_ftr import ProbabilisticFTR
from opencdarr.dynamics.base import Dynamics, MotionCommand
from opencdarr.dynamics.fixedwing import FixedWing
from opencdarr.dynamics.multirotor import Multirotor
from opencdarr.estimator import IPRResult, combine_ipr, estimate_ipr, wilson_interval
from opencdarr.experiment import (
    IPS,
    MC,
    ExperimentResult,
    Fixed,
    Methods,
    Sweep,
    run_experiment,
    run_one_experiment,
)
from opencdarr.fleet import Agent, run_fleet
from opencdarr.ips import estimate_rare_prob
from opencdarr.loop import run_encounter
from opencdarr.performance import M600, SMALL_FIXEDWING, Performance
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict, sample_pairwise
from opencdarr.state import AircraftState, create_aircraft
from opencdarr.viz import extract_tracks, plot_pairwise
from opencdarr.wind import NO_WIND, WindField

__version__ = "0.0.0"

__all__ = [
    "FTR",
    "IPS",
    "MC",
    "MVP",
    "NO_WIND",
    "SMALL_FIXEDWING",
    "VO",
    "AircraftState",
    "Agent",
    "Comm",
    "CommunicationModel",
    "Config",
    "ConflictDetector",
    "ConflictResolver",
    "Dynamics",
    "ExperimentResult",
    "Fixed",
    "FixedWing",
    "GnssNavigation",
    "IPRResult",
    "LastKnown",
    "M600",
    "Methods",
    "MotionCommand",
    "Multirotor",
    "NavigationModel",
    "NoiseDistribution",
    "PastCPA",
    "Performance",
    "ProbabilisticFTR",
    "RecoveryCriterion",
    "StateBased",
    "Sweep",
    "SurveillanceModel",
    "WindField",
    "__version__",
    "combine_ipr",
    "create_aircraft",
    "create_conflict",
    "estimate_ipr",
    "estimate_rare_prob",
    "extract_tracks",
    "generator",
    "geo",
    "load_config",
    "plot_pairwise",
    "root_seed_sequence",
    "run_encounter",
    "run_fleet",
    "run_experiment",
    "run_one_experiment",
    "sample_pairwise",
    "spawn",
    "wilson_interval",
]
