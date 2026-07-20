"""Conflict recovery — interface (`RecoveryCriterion`) and implementations."""

from opencdarr.crr.base import RecoveryCriterion
from opencdarr.crr.ftr import FTR
from opencdarr.crr.pastcpa import PastCPA

__all__ = ["FTR", "PastCPA", "RecoveryCriterion"]
