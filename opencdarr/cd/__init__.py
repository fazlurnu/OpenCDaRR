"""Conflict detection — interface (`ConflictDetector`, `is_los`) and implementations."""

from opencdarr.cd.base import ConflictDetector, is_los
from opencdarr.cd.statebased import StateBased

__all__ = ["ConflictDetector", "StateBased", "is_los"]
