"""Kinematics — the pluggable motion-model boundary (ADR 0007 / 0011 / 0012 / 0013).

These are *kinematic* models: turn rate, speed and heading evolve under performance limits, with
no mass, thrust, drag or energy balance anywhere. Hence ``Kinematics``, not ``Dynamics`` — the
literature reserves the latter for force-and-mass models (ADR 0020).

The control input (:class:`MotionCommand`), the contribution-surface ABC (:class:`Kinematics`), and
the implementations, one per file beside ``base.py`` (mirroring ``cd/``, ``cr/``, ``crr/``):

- :class:`Multirotor` — isotropic acceleration limit, no coupled heading, independent yaw; consumes
  a PX4 ``TrajectorySetpoint``-shaped command (``multirotor.py``, ADR 0012). Superseded
  ``HolonomicDynamics``.
- :class:`FixedWing` — non-holonomic coordinated-turn point-mass kinematics: bank-limited heading,
  stall/load envelope, finite roll, wind-ready; consumes PX4
  ``FixedWing{Lateral,Longitudinal}Setpoint`` channels (``fixedwing.py``, ADR 0013). Superseded
  ``DubinsDynamics``.

The public surface is re-exported here, so ``from opencdarr.kinematics import MotionCommand,
Kinematics, Multirotor, FixedWing`` reaches everything without knowing the file layout.
"""

from opencdarr.kinematics.base import Kinematics, MotionCommand
from opencdarr.kinematics.fixedwing import FixedWing
from opencdarr.kinematics.multirotor import Multirotor

__all__ = [
    "FixedWing",
    "Kinematics",
    "MotionCommand",
    "Multirotor",
]
