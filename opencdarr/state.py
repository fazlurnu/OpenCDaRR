"""The owner of state: plain, clonable data.

This is the spine of the design (``docs/design_brief.md``): *you* own the state, and
BlueSky is a library of stateless math, not the runtime. State is therefore an ordinary,
copyable value — not something hidden inside a global (``bs.traf``) or a singleton.

Invariant (load-bearing for rare-event simulation)
--------------------------------------------------
**Everything that influences an aircraft's future must live in its state object; nothing
in a global, a singleton, a module variable, or a closure.** Cloning a particle for the
interacting particle system (IPS, roadmap v0.4) means copying its state; any future-
affecting value kept *outside* the state would be silently shared between clones — which
is exactly the KI-1 bug (``docs/lesson-learnt.md``), and at 1e-9 it would corrupt the
estimate invisibly. Holding this invariant is what lets the state grow field-by-field,
step by step, and stay correct to clone.

Scope
-----
``AircraftState`` is the *certain kinematic core* — a single aircraft's 2D horizontal
point-mass state. It is deliberately not the whole IPS particle: the particle will also
carry per-aircraft CDR / recovery memory (e.g. ``resopairs``, the initial intruder
velocity a recovery criterion compares against) and an RNG substream (``ADR 0001``). Those
are added by the steps that introduce them (CDR: Steps 2-3; estimator: Steps 5-6), each
inside the clonable state, never outside it.

The model is horizontal at fixed altitude, matching every experiment on the roadmap
(recovery criteria, multi-aircraft conflict, rare events). A future 3D extension would add
``alt`` / vertical rate here *and* vertical dynamics, detection, and a 3D level function —
a deliberate, re-validated change recorded as its own ADR, not a set of dead fields now.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AircraftState:
    """One aircraft's 2D horizontal point-mass kinematics.

    Frozen (immutable): a copy can never alias its source, and no attribute — declared
    field or stray — can be assigned after construction, so nothing can smuggle hidden
    state onto an instance. Both serve the no-hidden-state invariant above. Evolve it
    functionally with :func:`dataclasses.replace`, e.g. inside ``step_dynamics`` (Step 1).

    ``slots`` / a NumPy-backed layout is deliberately *not* used yet: it interacts badly
    with ``frozen`` (a known CPython class-recreation wart) and is a memory optimisation we
    take only when IPS profiling shows per-particle object overhead matters — measured, not
    assumed (``design-philosophy.md`` #12).

    Attributes
    ----------
    id:
        Aircraft identifier (e.g. ``"DRO000"``), unique within a scenario.
    lat, lon:
        Position in decimal degrees (WGS84).
    trk:
        Track over ground in degrees, aviation convention (0 = North, increasing
        clockwise). Detection/resolution math converts to radians at its edge.
    gs:
        Ground speed in metres per second (SI internally; unit conversions live at the
        BlueSky boundary, not here).
    """

    id: str
    lat: float
    lon: float
    trk: float
    gs: float
