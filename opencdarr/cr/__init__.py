"""Conflict resolution — interface (`ConflictResolver`) and implementations."""

from opencdarr.cr.base import ConflictResolver
from opencdarr.cr.mvp import MVP

__all__ = ["MVP", "ConflictResolver"]
