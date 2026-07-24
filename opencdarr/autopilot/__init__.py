"""Autopilot — the guidance / mission-executor layer (ADR 0011).

The setpoint **producer**: turns intent into a :class:`~opencdarr.dynamics.MotionCommand` at the
decision cadence, vehicle-aware. One implementation per file beside ``base.py`` (mirroring ``cd/``
/ ``cr/`` / ``crr/`` / ``cns/``). The low-level setpoint tracker lives in ``dynamics/``, the safety
overlay in ``separation.py``.
"""

from opencdarr.autopilot.base import Autopilot, GuidanceMemory
from opencdarr.autopilot.cruise import CruiseAutopilot
from opencdarr.autopilot.waypoint import WaypointAutopilot

__all__ = [
    "Autopilot",
    "CruiseAutopilot",
    "GuidanceMemory",
    "WaypointAutopilot",
]
