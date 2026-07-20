"""Conflict resolution — interface (`ConflictResolver`) and implementations."""

from opencdarr.cr.base import ConflictResolver
from opencdarr.cr.mvp import MVP
from opencdarr.cr.vo import VO

__all__ = ["MVP", "VO", "ConflictResolver"]
