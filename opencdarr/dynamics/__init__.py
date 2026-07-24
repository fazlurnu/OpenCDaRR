"""Dynamics — the pluggable physics boundary (ADR 0007 / 0011 / 0012 / 0013).

The control input (:class:`MotionCommand`), the contribution-surface ABC (:class:`Dynamics`), and
the implementations, one per file beside ``base.py`` (mirroring ``cd/``, ``cr/``, ``crr/``):

- :class:`Multirotor` — isotropic acceleration limit, no coupled heading, independent yaw; consumes
  a PX4 ``TrajectorySetpoint``-shaped command (``multirotor.py``, ADR 0012). Superseded
  ``HolonomicDynamics``.
- :class:`FixedWing` — non-holonomic coordinated-turn point mass: bank-limited heading, stall/load
  envelope, finite roll, wind-ready; consumes PX4 ``FixedWing{Lateral,Longitudinal}Setpoint``
  channels (``fixedwing.py``, ADR 0013). Superseded ``DubinsDynamics``.

The public surface is re-exported here, so ``from opencdarr.dynamics import MotionCommand,
Dynamics, Multirotor, FixedWing`` is unchanged by the package split. ``Command`` remains a
backward-compatible alias of :class:`MotionCommand` during the Phase-4 migration.
"""

from opencdarr.dynamics.base import Command, Dynamics, MotionCommand
from opencdarr.dynamics.fixedwing import FixedWing
from opencdarr.dynamics.multirotor import Multirotor

__all__ = [
    "Command",
    "Dynamics",
    "FixedWing",
    "MotionCommand",
    "Multirotor",
]
