# ADR 0006 — Communication model design (Phase 3b: reception + latency)

- Status: accepted
- Date: 2026-07-20
- Deciders: Fazlur Rahman

## Context

Phase 3a (navigation) made each aircraft measure its **own** state with GPS error and broadcast
it; the receiver saw that broadcast instantly and perfectly. Phase 3b adds the two things a real
link actually does: it can **drop** a message (reception `< 1`), and a delivered message is
**late** (latency), so the receiver acts on where the source *was*, not where it is now
(`vault/phase-3-plan.md`). This needs its own pure, clonable state — `CommState` — threaded
through the encounter the same way `AircraftState` is (`state.py`'s no-hidden-state invariant),
because a dropped/delayed message *is* exactly the kind of discrete stochastic event the IPS
(roadmap v0.4) will need to branch on later (`vault/phase-3-plan.md`, "why latency is
first-class").

Several shapes and edge cases were open before writing code — settled here, the same way cd/cr/crr
were nailed down before their first line (`vault/phase-3-plan.md`, "decisions to settle first").

## Decision

### 1. `CommState` shape

```python
@dataclass(frozen=True)
class InFlight:
    message: Message     # source, state (noisy self-measurement), t_meas (send time)
    receiver: str
    deliver_t: float     # t_meas + latency for this specific link

@dataclass(frozen=True)
class CommState:
    held: dict[tuple[str, str], Message]   # (receiver, source) -> latest delivered Message
    in_flight: tuple[InFlight, ...]
```

`held` is keyed by **(receiver, source)**, not just `source`, because surveillance is directed
(ADR 0004): B's view of A and A's view of B are independent draws. `in_flight` is a tuple so the
whole state stays an ordinary frozen value — clonable for IPS with no aliasing risk, exactly like
`AircraftState`.

`Comm.step(comm_state, broadcasts, t, rng) -> CommState` is pure: for each new broadcast, draw
Bernoulli reception; if received, draw latency and enqueue an `InFlight` with
`deliver_t = t_meas + latency`; then move any `in_flight` entries with `deliver_t <= t` into
`held`.

`latency` is a **pluggable `LatencyDistribution`** (`(rng) -> delay`, mirroring `NoiseDistribution`
for position error) — a constant, `uniform_latency`, `lognormal_latency`, or a custom callable, not
a fixed shape. `reception_prob` is *not* similarly pluggable over time (fixed per link for the
whole run, `Comm._reception_for` has no `t`) — a real gap, tracked as
[[time-varying-reception-probability]] rather than designed here.

### 2. Hold-as-is, not dead-reckoning

A receiver holding a stale message uses it **unchanged** — it does not extrapolate the sender's
position forward using the held velocity. Two reasons: (a) it's the honest representation of what
the receiver actually has (no invented information), and (b) dead-reckoning assumes the sender
kept flying straight during the staleness window, which is wrong exactly when it matters most (the
sender just started maneuvering). `perceived(held, t_now)` still takes `t_now` — not to move the
state, but to report **age** (`t_now - t_meas`) for instrumentation (the "how stale was the info at
the near-miss" story the plan wants visible). All times are the sim's single global clock — no
cross-machine clock-skew modelling; that's a real ADS-B concern, orthogonal to what we model here.

### 3. Delivery is swept at the decision tick, not every integration step

Deliveries are drained once per CDR decision tick (`deliver_t <= t_now`), not every `dt` step.
Since CDR only *acts* at decision ticks anyway, a message's exact sub-tick arrival time cannot
change behaviour — only how stale it reads when finally used, which the stored `deliver_t` still
lets us report at full precision. Sweeping every `dt` step would be strictly more code for no
behavioural difference.

### 4. `held` keeps the freshest message **by `t_meas`**, not by arrival order

Latency is not required to be smaller than the broadcast interval, so two messages from the same
source can arrive **out of order** (a low-latency later message can beat a high-latency earlier
one). Overwriting `held` on every delivery would let a stale message clobber a fresher one already
held. The guard:

```python
if key not in held or delivered.message.t_meas > held[key].t_meas:
    held[key] = delivered.message
```

The alternative — forbidding latency ≥ broadcast interval — was rejected: it's a real, interesting
regime (a badly delayed link) and the guard to handle it correctly is one comparison.

### 5. No held message ⇒ that directed pair is nominal

Before a receiver has ever received *anything* from a source (dropped at first contact, or just
started), `perceived` returns no state for that link, and the loop treats the pair as no conflict
(fly nominal) — an aircraft cannot avoid a threat it has never heard of. This is a real degradation
mode Phase 3b is meant to expose (flying blind for a stretch after a drop), not an edge case to
paper over with a synthetic bootstrap message.

### 6. RNG substream layout extends ADR 0001

3b adds two new independent stochastic draws per directed link per broadcast tick (Bernoulli
reception, latency) that must not share a stream with GPS-noise draws — sharing one is exactly the
old ADSL bug (`vault/decisions/0001-rng-per-particle-spawn.md`). The per-encounter tree grows from
`spawn(2)` (`geom_seq, sim_seq`) to `spawn(3)`:

```
root (seed)
└── per encounter: spawn(3) -> geom_seq, nav_seq, comm_seq
                                          └── comm_seq feeds Comm.step() (reception + latency
                                              draws for both directed links)
```

`nav_seq` replaces the old `sim_seq` name (it now feeds only `GpsNavigation.measure`); `comm_seq`
is new. Documented here per ADR 0001's obligation that the stream layout be written down, not
implicit.

## Alternatives rejected

- **Dead-reckoning the held estimate.** Rejected (see #2) — invents information and is wrong
  exactly when the sender maneuvers.
- **Sweeping deliveries every `dt` step.** Rejected (see #3) — no behavioural difference from
  sweeping at decision ticks, only more code.
- **Overwrite `held` on arrival order.** Rejected (see #4) — silently regresses a receiver's view
  to a staler message when latency exceeds the broadcast interval.
- **Forbidding latency ≥ broadcast interval.** Rejected (see #4) — throws away a real, interesting
  regime for a modelling convenience.
- **Seeding every receiver with a synthetic first message.** Rejected (see #5) — masks a real
  degradation mode (flying blind after an early drop) that the plan explicitly wants visible.

## Consequences

- **Good:** `CommState` is a plain frozen value, clonable exactly like `AircraftState` — ready for
  IPS to branch on a specific drop/delay event later. The design handles the out-of-order-arrival
  edge case correctly instead of by assumption. The cold-start/no-held-message case is a feature of
  the model, not a gap.
- **Cost:** every directed pair now needs a `held.get((receiver, source))` presence check before
  perceiving, and the freshest-by-`t_meas` guard is one more thing a new communication model
  implementation must get right (documented here so it isn't rediscovered).
- **Obligation:** `comm_seq`'s two draws (reception, latency) per link per tick must be drawn in a
  fixed, documented order (reception first, then latency only if received) so the stream is
  reproducible and independent of unrelated changes elsewhere in the tick.

## Relations

- Implements the "decisions to settle before building" list in `vault/phase-3-plan.md` (3b
  section).
- Extends [[0001-rng-per-particle-spawn]] to the new `comm_seq` branch.
- Builds on the directed-pair design from [[0004-layered-directed-design-for-multiaircraft-and-ips]]
  (`held` keyed by `(receiver, source)`).
- Precedes the still-open "latency is its own layer, not a noise bias" ADR (`phase-3-plan.md`'s
  0006/0007 naming — renumbered here as this ADR absorbs both topics; a separate ADR for that
  framing point can still be split out if warranted once 3b lands).
