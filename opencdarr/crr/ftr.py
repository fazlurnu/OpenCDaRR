"""Free-To-Revert (FTR) conflict recovery — intent-based (2D, directed).

Implements :class:`~opencdarr.crr.base.RecoveryCriterion`. Governing equations:
``vault/derivations/ftr-recovery.md``.

FTR resumes **proactively**: as soon as reverting to the ownship's *desired* (nominal, pre-
resolution) velocity would keep the pair's closest approach beyond ``rpz`` — unlike
:class:`~opencdarr.crr.PastCPA`, which waits until the pair is already diverging. Re-derived from
``CDaRR_git/sim_models/crr_resumenav_ftr.py`` (double criteria); the reference's second criterion
("intruder reverts to its start-of-conflict velocity", which needs per-pair logged memory) becomes
"intruder reverts to its **shared** desired velocity", available only when intent-sharing is on —
otherwise FTR uses the single criterion (intruder holds its current velocity).
"""

from __future__ import annotations

from opencdarr.crr.base import RecoveryCriterion
from opencdarr.kinematics import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_STATIONARY_EPS = 1e-9  # (m/s)^2: no relative motion


def _clears(rx: float, ry: float, du: float, dv: float, rpz: float) -> bool:
    """Does relative motion (position ``rx,ry``; velocity ``du,dv``) keep CPA beyond ``rpz``?

    Uses the *forward* closest approach: if the pair is already diverging (t_cpa ≤ 0) or has no
    relative motion, the minimum future separation is the current distance.
    """
    dv2 = du * du + dv * dv
    dist2 = rx * rx + ry * ry
    if dv2 < _STATIONARY_EPS:
        return dist2 > rpz * rpz
    t_cpa = -(rx * du + ry * dv) / dv2
    if t_cpa <= 0.0:
        return dist2 > rpz * rpz
    dcpa2 = max(0.0, dist2 - t_cpa * t_cpa * dv2)
    return dcpa2 > rpz * rpz


class FTR(RecoveryCriterion):
    """Resume once reverting to the ownship's desired velocity keeps the pair clear of ``rpz``.

    Reads the ownship's own ``desired`` velocity (its intent — always known to itself). The second
    (intent-based) criterion additionally checks the intruder's ``desired`` velocity, which is
    present only when intent-sharing is enabled; without it, FTR falls back to the single
    criterion.
    """

    def should_resume(self, own: AircraftState, intr: AircraftState, rpz: float) -> bool:
        if own.desired is None:
            raise ValueError(
                "FTR needs the ownship's desired (nominal) velocity; run_encounter sets it, or "
                "set AircraftState.desired explicitly."
            )
        rel = relative_enu(own, intr)  # rx,ry = intr − own position

        vo_e, vo_n = own.desired.v_east, own.desired.v_north  # desired velocity, read directly

        # criterion 1: the intruder holds its current (observed) velocity
        vi_e, vi_n = velocity_enu(intr)
        if not _clears(rel.rx, rel.ry, vi_e - vo_e, vi_n - vo_n, rpz):
            return False

        # criterion 2 (intent-based): the intruder reverts to its own desired velocity too —
        # only if it shared it
        if intr.desired is not None:
            vir_e, vir_n = intr.desired.v_east, intr.desired.v_north
            if not _clears(rel.rx, rel.ry, vir_e - vo_e, vir_n - vo_n, rpz):
                return False

        return True
