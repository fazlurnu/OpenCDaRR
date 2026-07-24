"""Dynamics — the pluggable physics boundary (ADR 0007 / 0009 / 0010).

The control input (:class:`Command`), the contribution-surface ABC (:class:`Dynamics`), and the
implementations, one per file beside ``base.py`` (mirroring ``cd/``, ``cr/``, ``crr/``):

- :class:`DubinsDynamics` — turn-rate-limited, heading coupled to travel (``dubins.py``);
  wraps the raw integrator :func:`step_dynamics`.
- :class:`HolonomicDynamics` — isotropic acceleration limit, no coupled heading (``holonomic.py``).

The public surface is re-exported here, so ``from opencdarr.dynamics import Command, Dynamics,
DubinsDynamics, HolonomicDynamics, step_dynamics`` is unchanged by the package split.
"""

from opencdarr.dynamics.base import Command, Dynamics
from opencdarr.dynamics.dubins import DubinsDynamics, step_dynamics
from opencdarr.dynamics.holonomic import HolonomicDynamics

__all__ = [
    "Command",
    "Dynamics",
    "DubinsDynamics",
    "HolonomicDynamics",
    "step_dynamics",
]
