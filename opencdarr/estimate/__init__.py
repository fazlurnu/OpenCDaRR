"""The estimators: plain Monte Carlo, the serial rare-event IPS, and its parallel twin.

One family, one import surface (mirroring ``cd``/``cr``/``crr``): ``montecarlo`` is
:func:`estimate_p_los` over :func:`~opencdarr.fleet.run_fleet`; ``ips`` is the fixed-effort
multilevel splitting estimator (ADR 0017); ``parallel`` schedules the same IPS statistics over
joblib workers (ADR 0018). ``parallel`` is deliberately **not** re-exported here: its
``estimate_rare_prob`` shares the serial twin's name, so importing it as
:mod:`opencdarr.estimate.parallel` keeps which one you asked for spelled out. (Until Aug 2026 the
optional joblib extra was a second reason; joblib is a core dependency now and there is no extra,
so the name collision carries the decision on its own.)
"""

from opencdarr.estimate.ips import (
    BuildInitial,
    IPSResult,
    Particle,
    RareEventEstimate,
    combine_replications,
    estimate_rare_prob,
    evolve_shard,
    ips_once,
    level,
    replication_seeds,
    resample_level,
)
from opencdarr.estimate.montecarlo import MonteCarloEstimate, combine_p_los, estimate_p_los

__all__ = [
    "BuildInitial",
    "IPSResult",
    "MonteCarloEstimate",
    "Particle",
    "RareEventEstimate",
    "combine_p_los",
    "combine_replications",
    "estimate_p_los",
    "estimate_rare_prob",
    "evolve_shard",
    "ips_once",
    "level",
    "replication_seeds",
    "resample_level",
]
