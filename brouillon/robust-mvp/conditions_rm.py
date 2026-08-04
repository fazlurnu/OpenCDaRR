"""The experiment's fixed settings and its grid, shared by the probe and the IPS run.

The fixed settings are **imported** from
[`finding-best-cr/conditions.py`](../finding-best-cr/conditions.py) rather than restated, the same
way ``uncertainty-aware-mvp`` does, so "the same experiment at the same settings" is a structural
fact rather than a claim: both aircraft at 20 kt, ``dcpa`` 0, ``rpz`` 50 m, ``margin`` 1.05,
``t_lookahead`` 120 s, ``tlos`` 180 s, ``dt`` 0.2 s, ``t_max`` 600 s, 1 Hz broadcast, M600
multirotor, GNSS navigation noise, perfect datalink.

Two settings are this study's own:

- ``PROB_THRESHOLD`` is **0.95**, not ``finding-best-cr``'s 0.999. At 0.999 and a shallow crossing
  the recovery criterion is unreachable — the offset spread makes ``P(||d|| > rpz)`` saturate below
  the threshold, so the pair never resumes and 70% of encounters run to ``t_max`` still avoiding.
  Those runs measure the clock. This choice means the MVP rows here are **not** directly comparable
  to ``finding-best-cr/ips_results.json``, which is the 0.999 arm.
- ``RESOLVERS`` is MVP against RMVP alone, the controlled pair: RMVP at ``confidence = 0.5`` *is*
  MVP, so the two differ in one number.
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # make ``rmvp`` importable in joblib workers
sys.path.insert(0, str(HERE.parent / "finding-best-cr"))  # the shared fixed settings

from conditions import (  # noqa: E402
    BROADCAST_INTERVAL,
    DCPA,
    DONE_TIMEOUT,
    DT,
    KTHETA,
    MARGIN,
    RPZ,
    SPEED,
    T_LOOKAHEAD,
    T_MAX,
    TLOS,
)
from rmvp import RMVP  # noqa: E402
from rmvp_exact import RMVPExact  # noqa: E402

from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns import GnssNavigation  # noqa: E402
from opencdarr.cns.broadcast import BroadcastSchedule  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.cr.base import ConflictResolver  # noqa: E402
from opencdarr.crr import ProbabilisticFTR  # noqa: E402
from opencdarr.fleet import Agent, build_env  # noqa: E402
from opencdarr.ips import Particle  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

PROB_THRESHOLD = 0.95  # recovery confidence; see the module docstring on why not 0.999
CONFIDENCE = 0.95  # RMVP's design confidence


def recovery() -> ProbabilisticFTR:
    """The recovery criterion for every cell: ``Sigma_Vrel = Sigma_Vo + Sigma_Vi``, both aircraft's
    declared velocity uncertainty.

    ``velocity_uncertainty="both"`` is the reading this study runs throughout, so the resolver and
    the recovery criterion size the *same* relative-velocity uncertainty from the same two declared
    numbers — one constrains it before the manoeuvre, the other tests it before resuming.
    """
    return ProbabilisticFTR(
        prob_threshold=PROB_THRESHOLD, ktheta=KTHETA, velocity_uncertainty="both"
    )


# --- the grid -------------------------------------------------------------------------------

DPSI = (2.0, 30.0)  # deg
NOISE = ((10.0, 1.0), (10.0, 3.0))  # (pos_ci95 [m], vel_ci95 [m/s])

RESOLVERS: dict[str, ConflictResolver] = {
    "MVP": MVP(margin=MARGIN),
    f"RMVP{CONFIDENCE:.2f}": RMVP(
        margin=MARGIN, confidence=CONFIDENCE, velocity_uncertainty="both"
    ),
    # The same rule with the projected-normal quantile in place of the Gaussian one, to price the
    # §9.1 limitation. Not part of the headline grid — select it with --resolvers.
    f"RMVPExact{CONFIDENCE:.2f}": RMVPExact(
        margin=MARGIN, confidence=CONFIDENCE, velocity_uncertainty="both"
    ),
}


@dataclass(frozen=True)
class Cell:
    """One cell of the campaign — a crossing angle, a declared accuracy pair, and a resolver."""

    dpsi: float
    pos_ci95: float
    vel_ci95: float
    resolver: str

    @property
    def key(self) -> str:
        """Stable identifier, so probe and campaign results join on it across runs."""
        return f"dpsi{self.dpsi:g}_pos{self.pos_ci95:g}_vel{self.vel_ci95:g}_{self.resolver}"

    @property
    def label(self) -> str:
        return f"{self.dpsi:g}deg  ci95=({self.pos_ci95:g}m,{self.vel_ci95:g}m/s)  {self.resolver}"

    def build_particle(self) -> Particle:
        """The one starting particle for this cell — pinned geometry, so it is built once and
        shared.

        Geometry is **pinned** (one Δψ, ``dcpa`` = 0), as in the handbook's
        ``rare_event_ips.ipynb``: every particle starts from the same world under the same
        rules, so the splitting acts purely on the forward CNS noise and the estimate answers
        "given this encounter, how often does this stack lose separation?".
        :class:`~opencdarr.fleet.FleetEnv` and :class:`~opencdarr.fleet.FleetState` are deeply
        immutable, so every particle can share this value; only the forward CNS streams differ,
        and those are spawned per particle per level.

        The probe uses this same construction, so the distribution being laddered is the
        distribution being split.
        """
        own = AircraftState(
            id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED,
            pos_ci95=self.pos_ci95, vel_ci95=self.vel_ci95,
        )
        intr = create_conflict(
            own, intr_id="INT", dpsi=self.dpsi, dcpa=DCPA, tlos=TLOS, rpz=RPZ, side=1
        )
        agents = [Agent(own, M600), Agent(intr, M600)]
        env = build_env(
            agents, rpz=RPZ, t_lookahead=T_LOOKAHEAD, dt=DT,
            detector=StateBased(), resolver=RESOLVERS[self.resolver], recovery=recovery(),
            navigation=GnssNavigation(), t_max=T_MAX, done_timeout=DONE_TIMEOUT,
            # phase=None aligns both transmit clocks at t=0, one timing model for every backend
            schedule=BroadcastSchedule(interval=BROADCAST_INTERVAL),
        )
        return Particle(env=env, state=env.initial_state(agents))


def cells(
    resolvers: tuple[str, ...] | None = None,
    vel_ci95: tuple[float, ...] | None = None,
    dpsi: tuple[float, ...] | None = None,
) -> list[Cell]:
    """The full grid, or the subset matching any of ``resolvers`` / ``vel_ci95`` / ``dpsi``."""
    names = resolvers if resolvers is not None else tuple(RESOLVERS)
    noise = [(p, v) for p, v in NOISE if vel_ci95 is None or v in vel_ci95]
    angles = [d for d in DPSI if dpsi is None or d in dpsi]
    return [
        Cell(dpsi=d, pos_ci95=p, vel_ci95=v, resolver=r)
        for d in angles
        for p, v in noise
        for r in names
    ]


def ladder(
    min_sep: Sequence[float],
    tail_levels: int = 12,
    first_q: float = 0.4,
    survival: float = 0.6,
    final_excess: float = 0.25,
) -> tuple[float, ...]:
    """The IPS shell sequence for a cell, built from that cell's own probe distribution.

    Two regimes, because the evidence runs out partway down:

    - **Where the probe saw data**, shells sit at *quantiles* of the observed minimum separation —
      ``first_q``, then shrinking by ``survival`` each level, down to the second-smallest run.
      Every level then costs about the same fraction of the cloud, which is what a splitting
      ladder wants. A ladder spaced uniformly in separation or in log-excess does not, and the
      failure it produces is not a wide interval but a **collapse**: at Δψ = 30° with ``vel_ci95``
      1 m/s a log-excess ladder stepped 139 → 113 → 95 → 82 → 72 → 66 m and lost every particle at
      the sixth shell, because a step that large lets the survivors pass CPA before the next split.
      Past CPA the separation can only grow, so a clone of such a particle can never cross a lower
      shell however the noise falls, and the level reads zero. That is the mechanism behind ADR
      0017 §2's "a collapsed level is not a real zero".
    - **Below the smallest observed run**, there is nothing to place shells on, so the remaining
      ``tail_levels`` descend geometrically in the excess over ``rpz`` to ``final_excess`` and
      then to ``rpz`` itself.

    The ladder length is therefore not fixed: a rarer cell earns more shells, which is the right
    way round. One sequence cannot serve both crossing angles anyway — at Δψ = 2° the pair starts
    114.6 m apart and at 30° it starts 1008.7 m apart.
    """
    sample = np.asarray(min_sep, dtype=float)
    q_min = 1.5 / len(sample)  # just above the smallest observed run: below it we have no evidence
    quantiles: list[float] = []
    q = first_q
    while q > q_min:
        quantiles.append(q)
        q *= survival
    observed = [float(v) for v in np.quantile(sample, quantiles)]

    shells = [d for d in observed if d > RPZ + final_excess]
    excess = max((shells[-1] if shells else float(np.quantile(sample, first_q))) - RPZ,
                 2.0 * final_excess)
    ratio = (final_excess / excess) ** (1.0 / tail_levels)
    shells += [RPZ + excess * ratio**k for k in range(1, tail_levels + 1)]
    shells.append(RPZ)
    # strictly decreasing: two quantiles can land on the same run in a cell with few distinct
    # values
    out = [shells[0]]
    for d in shells[1:]:
        if d < out[-1]:
            out.append(d)
    return tuple(out)
