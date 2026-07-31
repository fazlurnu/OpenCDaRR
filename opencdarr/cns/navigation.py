"""GNSS self-measurement (navigation noise).

Implements :class:`~opencdarr.cns.base.NavigationModel`. Governing equations:
``vault/derivations/gps-noise.md``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from opencdarr import geo
from opencdarr.cns.base import (
    Message,
    NavEffect,
    NavigationModel,
    NavQuality,
    NavState,
    NoiseDistribution,
)
from opencdarr.cns.hazard import hazard, toggle
from opencdarr.cns.noise_distributions import gaussian
from opencdarr.relative import velocity_enu
from opencdarr.state import AircraftState


class GnssNavigation(NavigationModel):
    """Measure own position and velocity, each with a pluggable 2D error distribution.

    The noise magnitude is **not** a constructor parameter here: it is read from the aircraft
    being measured (``true.pos_ci95``, ``true.vel_ci95`` — 95% radial position [m] / velocity
    [m/s] accuracy), since accuracy is a property of *that* aircraft's sensor, can differ per
    aircraft, and may evolve over a run (``AircraftState``'s docstring). ``pos_distribution`` and
    ``vel_distribution`` are the position- and velocity-error models (default isotropic Gaussian
    for both); each is a :class:`~opencdarr.cns.base.NoiseDistribution` mapping a CI95 to a 2D
    ``(east, north)`` error. Position and velocity are separate quantities from the same receiver
    (pseudorange vs Doppler), so they take independent distributions
    (see ``vault/derivations/gps-noise.md``).

    The error is drawn from the aircraft's **actual** accuracy and the broadcast carries its
    **declared** one. They are the same number unless the aircraft sets
    ``pos_ci95_declared``/``vel_ci95_declared``, which is how a mismatch between what a sensor
    delivers and what its transponder claims is expressed — see :meth:`measure`.
    """

    def __init__(
        self,
        pos_distribution: NoiseDistribution = gaussian,
        vel_distribution: NoiseDistribution = gaussian,
        *,
        effects: Sequence[NavEffect] = (),
    ) -> None:
        self.pos_distribution = pos_distribution
        self.vel_distribution = vel_distribution
        self.effects = tuple(effects)

    def initial_state(self) -> NavState:
        return NavState(effects=tuple(e.initial() for e in self.effects))

    def evolve(
        self,
        state: NavState,
        aircraft: Sequence[AircraftState],
        t: float,
        rng: np.random.Generator,
    ) -> NavState:
        if len(state.effects) != len(self.effects):
            raise ValueError(
                f"nav state carries {len(state.effects)} effect state(s) but this model has "
                f"{len(self.effects)}; build it with GnssNavigation.initial_state()"
            )
        if not self.effects:
            return replace(state, t_prev=t)
        elapsed = t if state.t_prev is None else t - state.t_prev
        advanced = tuple(
            effect.evolve(own, aircraft, elapsed, rng)
            for effect, own in zip(self.effects, state.effects, strict=True)
        )
        return NavState(effects=advanced, t_prev=t)

    def _quality_for(self, state: NavState, aircraft_id: str) -> NavQuality:
        """This aircraft's combined degradation: every effect's, multiplied (ADR 0021 §1)."""
        pos_scale = vel_scale = pos_declared = vel_declared = 1.0
        for effect, own in zip(self.effects, state.effects, strict=True):
            q = effect.quality(own, aircraft_id)
            pos_scale *= q.pos_scale
            vel_scale *= q.vel_scale
            pos_declared *= q.pos_declared
            vel_declared *= q.vel_declared
        return NavQuality(pos_scale, vel_scale, pos_declared, vel_declared)

    def measure(
        self, state: NavState, true: AircraftState, t: float, rng: np.random.Generator
    ) -> Message:
        # every scale is exactly 1.0 with no effects, and x * 1.0 == x for every finite float,
        # so an unaffected fix is bit-for-bit the pre-seam one
        quality = self._quality_for(state, true.id)

        # position error (East, North) -> offset the true position via our geodesy
        err_e, err_n = self.pos_distribution(rng, true.pos_ci95 * quality.pos_scale)
        bearing = math.degrees(math.atan2(err_e, err_n)) % 360.0
        lat, lon = geo.forward(true.lat, true.lon, bearing, math.hypot(err_e, err_n))

        # velocity error (East, North) -> measured track and ground speed
        verr_e, verr_n = self.vel_distribution(rng, true.vel_ci95 * quality.vel_scale)
        ve, vn = velocity_enu(true)
        ve += verr_e
        vn += verr_n
        trk = math.degrees(math.atan2(ve, vn)) % 360.0
        gs = math.hypot(ve, vn)

        # The broadcast declares *one* accuracy -- the sender's claim, which is what a receiver
        # reads. That is the accuracy the sensor actually had unless the aircraft says otherwise
        # (`pos_ci95_declared`/`vel_ci95_declared`, `None` = honest), so the error above is drawn
        # from the truth while the message carries the claim. The two agreeing is the default and
        # the ordinary case; them disagreeing is the integrity failure RAIM exists to catch.
        # The scenario sets the *static* claim; an effect scales whatever that came out as, so a
        # degrading sensor can either admit it or keep claiming nominal (ADR 0021 §2).
        pos_claim = true.pos_ci95 if true.pos_ci95_declared is None else true.pos_ci95_declared
        vel_claim = true.vel_ci95 if true.vel_ci95_declared is None else true.vel_ci95_declared
        measured = AircraftState(
            id=true.id, lat=lat, lon=lon, trk=trk, gs=gs,
            pos_ci95=pos_claim * quality.pos_declared,
            vel_ci95=vel_claim * quality.vel_declared,
        )
        return Message(source=true.id, state=measured, t_meas=t)


@dataclass(frozen=True)
class GnssOutageState:
    """Which aircraft currently have a degraded GNSS fix — :class:`GnssOutage`'s state.

    Absent from the set means the receiver is nominal — the same "absent ⇒ nothing has happened
    yet" reading :attr:`~opencdarr.cns.base.CommState.held` uses, which is why
    :meth:`GnssOutage.initial` needs no roster.
    """

    out: frozenset[str] = frozenset()


@dataclass(frozen=True)
class GnssOutage(NavEffect):
    """A GNSS receiver that degrades and recovers, as a hazard process over elapsed time.

    The N-side twin of :class:`~opencdarr.cns.communication.RadioHealth`, and deliberately a
    *degradation* rather than a silence: a receiver that loses satellites reports a worse
    position, not no position, so this scales the accuracy rather than vetoing the broadcast. An
    aircraft that stops transmitting altogether is ``RadioHealth(tx_fail_rate=...)`` — the same
    physical event has one spelling, on the C side where the link lives (ADR 0021 §1).

    ``pos_factor``/``vel_factor`` are how many times worse the fix is while degraded. ``declare``
    is the fork ADR 0021 §2 exists for: ``True`` broadcasts the degraded accuracy (an honest
    transponder derating its NACp/NIC, so receivers widen their uncertainty and act accordingly),
    ``False`` keeps claiming nominal while the fix is bad — the *integrity failure* RAIM exists to
    catch, and the only case where a receiver acts confidently on a wrong number.

    Rates are per **hour** and applied over the elapsed time
    (:func:`~opencdarr.cns.hazard.hazard`), so the mean time to an outage is ``1 / fail_rate``
    hours whatever the broadcast cadence — a cadence sweep moves one thing, not two. A zero
    ``recover_rate`` (the default) means an outage **latches** for the rest of the encounter.

    Draws come from the **existing** nav substream: a fourth stream would break the
    config-invariant tree (ADR 0006 §6). Exactly one draw per aircraft per tick, whatever the
    health and whatever the rates including zero, so sweeping a rate moves the outages without
    shifting the measurement draws underneath them.

    .. warning::
       IPS will not reach a small ``fail_rate``. A latching outage is a discrete jump that
       ``min_sep`` carries no information about, so the shells cannot steer toward it — the pathway
       measured collapsing 8/8 replications in ``vault/observations/important-ips-gap.md``. Note
       this is a property of the *jump*, not of navigation effects generally: a **continuous**
       accuracy degradation is coupled to ``min_sep`` (a bigger position error gives worse geometry
       gives less separation) and IPS reaches it fine, and a permanently degraded sensor needs no
       effect at all — it is just a larger ``pos_ci95``. The IPS-blind set is exactly
       ``fail_rate > 0``. Estimate an outage study by plain MC, or condition on the failure time
       and reweight.
    """

    fail_rate: float = 0.0
    recover_rate: float = 0.0
    pos_factor: float = 20.0
    vel_factor: float = 20.0
    declare: bool = True

    def __post_init__(self) -> None:
        for name in ("fail_rate", "recover_rate"):
            rate = getattr(self, name)
            if rate < 0.0 or not math.isfinite(rate):
                raise ValueError(f"{name} must be a finite rate >= 0 [1/h], got {rate}")
        for name in ("pos_factor", "vel_factor"):
            factor = getattr(self, name)
            if factor < 0.0 or not math.isfinite(factor):
                raise ValueError(f"{name} must be a finite factor >= 0, got {factor}")

    def initial(self) -> GnssOutageState:
        """Every receiver nominal."""
        return GnssOutageState()

    def evolve(
        self,
        own: object,
        aircraft: Sequence[AircraftState],
        elapsed: float,
        rng: np.random.Generator,
    ) -> GnssOutageState:
        """Age every receiver by ``elapsed`` seconds — one draw per aircraft, always.

        The hazard is applied over the *elapsed* time rather than per call, so offset broadcast
        phases and jitter — which make the gap between calls something other than the nominal
        interval — come out right without the effect being told the cadence at all.
        """
        assert isinstance(own, GnssOutageState)
        out = set(own.out)
        p_fail = hazard(self.fail_rate, elapsed)
        p_recover = hazard(self.recover_rate, elapsed)
        for ac in aircraft:  # agent order: the fleet's, so the pairwise runner's at n = 2
            toggle(out, ac.id, p_fail, p_recover, rng)
        return GnssOutageState(out=frozenset(out))

    def quality(self, own: object, aircraft_id: str) -> NavQuality:
        """A degraded receiver's fix is ``*_factor`` times worse; a nominal one is untouched."""
        assert isinstance(own, GnssOutageState)
        if aircraft_id not in own.out:
            return NavQuality()
        declared = (self.pos_factor, self.vel_factor) if self.declare else (1.0, 1.0)
        return NavQuality(
            pos_scale=self.pos_factor,
            vel_scale=self.vel_factor,
            pos_declared=declared[0],
            vel_declared=declared[1],
        )


def gnss_outage(state: NavState) -> GnssOutageState:
    """The :class:`GnssOutage` state inside a :class:`~opencdarr.cns.base.NavState`.

    Instrumentation, mirroring :func:`~opencdarr.cns.communication.radio_health`: effect states
    are positional and opaque on :attr:`NavState.effects`, so finding one means matching by type.
    Raises if the stack has no :class:`GnssOutage`, since silently reporting "nothing is degraded"
    would read the same as a working receiver.
    """
    for own in state.effects:
        if isinstance(own, GnssOutageState):
            return own
    raise ValueError("this navigation stack has no GnssOutage effect")
