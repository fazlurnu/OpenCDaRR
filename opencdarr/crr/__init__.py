"""Conflict recovery — interface (`RecoveryCriterion`) and implementations."""

from opencdarr.crr.base import RecoveryCriterion
from opencdarr.crr.pastcpa import PastCPA

__all__ = ["PastCPA", "RecoveryCriterion"]
