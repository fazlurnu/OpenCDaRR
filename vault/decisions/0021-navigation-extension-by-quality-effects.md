# ADR 0021 — Navigation extends by quality-modulating effects, and never by a veto

- Status: accepted
- Date: 2026-07-31
- Deciders: Fazlur Rahman
- Extends: [[0019-channel-extension-by-link-gates]] — the same "compose an effect, don't subclass
  the model" shape, applied to N, with the one place it deliberately diverges recorded in §1.

## Context

The C layer has two extension seams and the N layer has none. `CommunicationModel` carries a
threaded `CommState`, an `initial_state()` hook for replacing the channel wholesale, and a
`LinkGate` tuple for adding one effect to it. `NavigationModel.measure(true, t, rng)` is a pure
per-tick function: nothing it computes survives to the next tick.

| | C (communication) | N (navigation), before this ADR |
|---|---|---|
| threaded state | `CommState` in `CnsState.comm` | none |
| replace-the-model seam | `initial_state()` | none |
| add-one-effect seam | `LinkGate` + `CommState.gates` | none |
| elapsed time | `CommState.t_prev` | none |
| roster validation | `validate_ids()` | none |

So an error model with **memory** has nowhere to live: a GNSS outage that latches, a slowly
drifting bias, an urban-canyon corridor, RAIM-triggered degradation. Each is a per-aircraft value
that has to persist across ticks and clone with an IPS particle, which is precisely what
`CommState` is for on the C side.

Normally `design-philosophy.md` #17 would defer this until a second implementation exists — the
argument that kept `Coordination` out of the tree. It does not apply here: a concrete stateful
effect is wanted now (a latching outage, ADR §6), and the layer is being prepared for outside
contributions, where the interface *is* the contribution surface.

## Decision

### 1. The nav effect modulates quality; it is never a veto

[[0019-channel-extension-by-link-gates]] §4 does not merely draw a veto/modulation line, it gives a
test: **does a draw still happen?** If the effect removes the draw it is a veto and must `continue`
ahead of it; if the draw survives, the effect belongs in the model's arguments instead.

Navigation fails the veto test twice:

1. **There is no link to veto.** A gate's unit of work is a directed link offer. Navigation's unit
   of work is "produce a fix", and an aircraft always produces one.
2. **A degraded receiver still draws.** Real ADS-B/ADS-L transmitters degrade their NACp/NIC rather
   than going silent, so the two position draws and two velocity draws still happen.

And the one nav-shaped effect that *would* be a veto — "this aircraft does not transmit" — already
exists as `RadioHealth(tx_fail_rate=...)` on the C side. Building a second spelling for one physical
event is what #17 forbids.

So `NavEffect` has **no `admits()`**. Effects compose by multiplication over positive reals with
identity `1.0`, where `LinkGate` composed by `all()` over booleans with identity `True` — the same
"several effects compose" property, on the other side of §4's line.

> Obligation: `NavQuality` carries scales and no additive offset. A *static* bias needs nothing from
> the library — it is a five-line `NoiseDistribution` closure over the existing `(rng, ci95)`
> protocol, and belongs in the user's own file because it breaks the containment guarantee
> `noise_distributions.py` promises. Only a **drifting** bias, which needs memory, would force
> `NavQuality` to grow offset fields. Revisit then, not before.

### 2. Sensed and declared degrade independently

`NavQuality` carries four floats: `pos_scale`/`vel_scale` multiply the accuracy the error is
actually drawn from, `pos_declared`/`vel_declared` multiply the accuracy stamped on the broadcast.

- Honest degradation is `declared == scale` — the transmitter admits what its sensor is doing.
- An **integrity failure** is `declared = 1.0, scale = 20.0` — the fix degrades and the transponder
  keeps claiming nominal. This is the case RAIM exists to catch, and the only one where a receiver
  acts on a confident number that is wrong.

Four fields rather than a shared one follows `RadioHealth`'s own precedent, which gives transmitter
and receiver separate rates "because a transmitter and a receiver are separate hardware with no
reason to share a reliability figure". Pseudorange position and Doppler velocity are the same
argument.

This is the **dynamic** half of the mismatch story. The **static** half is
`AircraftState.pos_ci95_declared`/`vel_ci95_declared` (a scenario property, needing no effect at
all). They multiply rather than overlap: an outage can degrade the real fix while the transponder
keeps claiming whatever the scenario set.

The degraded value is stamped on the **measured** state, never on truth — `fleet.advance` must not
write accuracy back into the truth record. That falls out correctly for the ownship too, since
`CNS.sense` builds `Perception.own` from the same fix, so an aircraft's own `ProbabilisticFTR` sees
its own degradation with no extra plumbing.

### 3. Empty effects draw nothing, so nothing re-bases

The stream tree is closed: `estimator.estimate_ipr` pins exactly four children per encounter,
`ips._streams` exactly three. No new substream, ever ([[0006-communication-model-design]] §6).

The append-only trick `schedule_for` uses — draw the broadcast phase from `geom_rng` *after* the
geometry has finished with it, so switching it on appends rather than shifts — **does not
transfer**. `geom_rng` has a tail because it is used once per encounter and then abandoned;
`streams.nav` is drawn from every tick, so anything appended at the end of tick *k* shifts tick
*k+1*. There is no append-only position in the nav stream and looking for one is wasted effort.

What is available instead is stronger than what C got: **with `effects=()` the layer draws nothing
new at all**, so every existing number is bit-for-bit unmoved. `TransceiverComm` could never claim
this — it drew two per aircraft per tick unconditionally and so was never stream-identical to
`Comm`. The nav layer has no unconditional per-tick state today, so the empty case really is free.

The constant-draw rule is therefore preserved in the form it is actually stated — constant across
*parameter values* for a *given* model configuration:

- `GnssOutage(fail_rate=0.0)` vs `GnssOutage(fail_rate=1e-15)` — **must** be stream-identical. Every
  cell of a rate sweep shares one measurement stream. This is what the unconditional draw buys.
- `GnssNavigation()` vs `GnssNavigation(effects=(GnssOutage(0.0),))` — a different model, a
  different cache key, and allowed to differ.

Effects evolve once per tick over the **whole roster in agent order**, before any aircraft measures
— mirroring `Comm.step`'s gates-first ordering. Per-tick consumption is then a function of tick
count and fleet size alone, so the measurement draws sit at a fixed offset behind them and do not
move with which aircraft happened to fire.

### 4. `NavEffect.evolve` receives states, not ids

A deliberate divergence from `LinkGate.evolve(own, receivers: Sequence[str], ...)`. An aircraft's
GNSS environment depends on *where it is* — urban canyon, terrain masking — which is the navigation
analogue of the range dependence §4 of ADR 0019 left open on the comm side. Ids-only is the tighter
contract and would stop an effect reading truth, but it also makes the single most obvious second
effect unbuildable.

> Obligation: an effect must not read `desired`. Intent is private (`DesiredVelocity`'s docstring),
> and an effect that steered on it would be reading another aircraft's intentions through a side
> channel.

### 5. Two seams at two levels, as on the C side

`NavigationModel.initial_state()` returning a `NavState` subclass is the seam for **replacing the
model wholesale**; `effects` is the seam for **adding one effect to the standard one**. Both are
kept for the reason ADR 0019 §5 keeps them: a model that must remember something the effect
contract cannot express should not have to force it into that contract.

`NavigationModel.evolve` is non-abstract with a safe default that draws nothing and only stamps
`t_prev`, so every existing implementation keeps working — the same "abstract core method plus
optional hooks" shape `Kinematics.validate_performance` and `CommunicationModel.validate_ids` use.

### 6. `GnssOutage` is the reference implementation, and it is IPS-blind

A latching outage with hazard-rate transitions, the N-side twin of `RadioHealth`, living beside
`GnssNavigation` as `RadioHealth` lives beside `Comm`. It reuses the shared `hazard`/`toggle`
helpers verbatim, so rates are per **hour** — the unit reliability is actually quoted in — and the
mean time to outage is `1/rate` hours *independent of broadcast cadence*. Quoting a probability per
broadcast instead would tie that mean to the interval, and a cadence sweep would then move two
things at once.

It carries `RadioHealth`'s IPS warning, plus a distinction that is new on the N side and worth
stating: a **continuous** accuracy degradation *is* coupled to `min_sep` — a bigger position error
gives worse CDR geometry gives smaller separation — so IPS reaches it fine. Only the discrete
**jump** is unreachable, because `min_sep` carries no information about it and the shells cannot
steer toward it. The IPS-blind set is therefore exactly `fail_rate > 0`, not "navigation effects";
and a permanently degraded sensor needs no effect at all, being just a larger `pos_ci95`.

## Alternatives rejected

- **A `NavGate` with `admits()`, mirroring `LinkGate` exactly.** Symmetry is not a reason: §1's test
  classifies the nav effect as modulation, and the veto case is already spelled `RadioHealth`.
- **Two `NavQuality` fields (scale only, always honest).** The #17 answer, and rejected because the
  honest and dishonest arms give *opposite* CDR conclusions — neither can be a silent default.
- **An additive bias term now.** A drifting bias is physically additive, but an offset needs a frame
  convention (ENU vs along/cross-track), which is the open anisotropy-orientation question. Deferred
  to §1's obligation rather than guessed at.
- **A fifth RNG substream for effects.** Forbidden by [[0001-rng-per-particle-spawn]] and
  [[0006-communication-model-design]] §6; the tree is pinned at four children and three in IPS.
- **Extending `NoiseDistribution` to see heading and ground speed** (the open item this work
  replaces). Rejected as double-counting: the along-track displacement of a stale fix is already
  produced by `LatencyDistribution` plus `LastKnown` hold-as-is, so folding `−ℓ·g` into the error
  would apply it twice. Recorded in full where that item is closed.

## Consequences

- `NavigationModel.measure` gains a `NavState` argument. One in-package call site (`CNS.sense`) and
  a handful of scripts; there is no deprecation path because the API is not frozen.
- `CnsState` gains a `nav` field and `CNS` gains `initial_state(n)`. The two composition roots
  (`fleet.build_env`, `loop.run_encounter`) switch to it. Worth doing on its own merit: the two most
  expensive recent defects were both a composition root forgetting to pass a model.
- `_hazard`/`_toggle` move out of `communication.py` into `cns/hazard.py` as public `hazard`/
  `toggle`. A pure move, so both channel golden traces must hold unchanged; duplicating them would
  also duplicate the docstring where the constant-draw-count rule is written down.
- Adding a public `effects` attribute to `GnssNavigation` changes every `experiment.identity` cache
  key that includes a navigation model. **Entries invalidate and must be recomputed; no number
  inside them changes.** It cannot be dodged by naming it `_effects`: `identity` includes private
  *names* but not their *values*, so two effect configurations would collide on one key — a wrong
  key, which is worse than no cache.
- **Obligation:** effects are evaluated independently and their qualities multiply. If two effects
  must observe each other (a RAIM effect that should not fire while GNSS is already out), revisit —
  the same obligation ADR 0019 records for two gates.

## Relations

- Extends [[0019-channel-extension-by-link-gates]] — same composition-over-subclassing shape; §1
  records where and why N diverges from the gate contract.
- Rests on [[0001-rng-per-particle-spawn]] and [[0006-communication-model-design]] §6 for the closed
  stream tree that §3 works within.
- Rests on the clonable-value discipline of
  [[0004-layered-directed-design-for-multiaircraft-and-ips]]: `NavState` clones with the particle,
  the effect objects are shared immutable config.
- The static half of §2's mismatch lives on `AircraftState`, not here; see
  [[gps-noise]] for where declared accuracy lives and why.
