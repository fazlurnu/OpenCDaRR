"""Autopilot — the guidance / mission-executor layer (ADR 0011).

The setpoint **producer**: turns intent into a :class:`~opencdarr.kinematics.MotionCommand` at the
decision cadence, vehicle-aware. One implementation per file beside ``base.py`` (mirroring ``cd/``
/ ``cr/`` / ``crr/`` / ``cns/``). The low-level setpoint tracker lives in ``kinematics/``, the
safety overlay in ``separation.py``.
"""

from opencdarr.autopilot.base import Autopilot, GuidanceMemory, nominal_velocity
from opencdarr.autopilot.cruise import CruiseAutopilot
from opencdarr.autopilot.waypoint import WaypointAutopilot

__all__ = [
    "Autopilot",
    "CruiseAutopilot",
    "GuidanceMemory",
    "WaypointAutopilot",
    "nominal_velocity",
]
