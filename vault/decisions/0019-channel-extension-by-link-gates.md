# ADR 0019 — The channel is already publish–subscribe; extend it with link gates, not subclasses

- Status: accepted
- Date: 2026-07-30
- Deciders: Fazlur Rahman
- Supersedes: [[0006-communication-model-design]] §1's deferral of the extension question — the
  "`Comm._reception_for` has no `t`" gap is designed here rather than only tracked.

## Context

The question that started this was whether restructuring the CNS layer as a publish–subscribe
system would improve its readability and modularity, keeping all six modelled effects (jitter,
latency, off-phasing, reception probability, transmitter down, receiver down).

The answer to the question as asked is that **there is nothing to switch to**. The layer is already
publish–subscribe, and the correspondence is exact:

| Pub/sub concept | CNS today |
|---|---|
| Publisher | the firing aircraft (`BroadcastSchedule.due`) |
| Topic | source aircraft id |
| Publish cadence | `BroadcastSchedule` — interval, phase, jitter |
| Broker | `Comm.step` |
| Delivery QoS | `reception_prob`, `latency` |
| In-flight queue | `CommState.in_flight` |
| Subscriber inbox | `held[(receiver, source)]` — depth-1, latest-wins |
| Subscriber read | `LastKnown.perceived` |

That inbox is precisely MQTT's *retained message* / DDS's *keep-last-1* durability. So adopting
broker/topic/subscriber vocabulary would rename working code and buy nothing — and it would cost
something real. `docs/lesson-learnt.md` records the prior system's "4-node message-passing dance"
as part of the architecture that had to be discarded; `vault/phase-3-plan.md` scopes this layer to
the *effect* of ADS-L rather than its protocol. The ADS-B domain names are what tie the code to the
literature and to the eight vault notes that cite them.

**The defect the question was pointing at is real, but it is the extension axis, not the naming.**
`TransceiverComm(Comm)` has no way to influence delivery except by pre-filtering the arguments it
hands to `super().step()`. That mechanism supports exactly one shape of effect — *remove some
publishers, remove some subscribers*. The next effect on the roadmap is not that shape:
[[time-varying-reception-probability]] wants reception to depend on time and range, and
`_reception_for(source, receiver)` sees only ids. Each further effect therefore forces a new `Comm`
subclass **and** a parallel new `CommState` subclass, with the `isinstance` guard in
`TransceiverComm.step` growing to match — the classic parallel-hierarchy trap.

## Decision

### 1. Keep the vocabulary; record the correspondence

No rename. The table above is the mapping for anyone who arrives expecting a message bus; it lives
here so the next person does not re-open the question. `docs/design-philosophy.md` #17 ("No
unrequested generality") is the standing reason not to build a broker abstraction over a system
that already has one.

### 2. One transmit-timing model for both runners

`run_encounter` advanced a single global `next_broadcast` by a scalar interval, so the pairwise
runner could express neither jitter nor phase — and the n = 2 bit-for-bit reduction with `run_fleet`
therefore only held at the aligned default. Both runners now thread the same `BroadcastSchedule`,
with the tick restructured over a *firing subset* the way `FleetEnv.advance` already did.
`broadcast_interval` remains as the scalar spelling, so existing callers are untouched.

The reduction is now unconditional, and pinned by a test that could not previously be written:
`run_encounter == run_fleet` under `phase=[0.0, 0.37], jitter=0.2`.

### 3. Extension is by composition — a `LinkGate` that vetoes a directed link

```python
class LinkGate(ABC):
    def initial(self) -> object: ...
    def evolve(self, own, receivers, elapsed, rng) -> object: ...
    def admits(self, own, source, receiver) -> bool: ...
```

`CommState` gains one additive field, `gates: tuple[object, ...]`, so each gate owns its own state
and the `initial_state()` seam for user-written whole-model replacements (§5 below) keeps working.
`Comm.__init__` gains a keyword-only `gates`. `Comm.step` evolves each gate in registration order
*before* the channel, then `_offer` skips any link a gate denies.

Radio health becomes `RadioHealth(LinkGate)`, and `TransceiverComm` becomes a thin façade over
`Comm(..., gates=(RadioHealth(...),))` with its public constructor unchanged. Gates compose where
subclasses did not: radio failure *and* terrain masking is two gates, not a fourth class.

### 4. A veto is not `p = 0`, and that is a stream fact, not a style choice

The gate admits or denies; it does not modulate the probability. This looks like a limitation and
is actually forced. Today a down radio is filtered out **before** the offer loop, so it consumes
**zero** reception draws. A gate expressed as "return `p = 0.0`, then draw" would consume one draw
per suppressed link and shift every subsequent number. The veto must therefore `continue` ahead of
`rng.random()`, which is exactly what preserves the stream.

The consequence for [[time-varying-reception-probability]] is that the gap splits in two:

- **Hard availability** — out of range, terrain-masked, radio down. A veto. This seam covers it.
- **Continuous modulation** — a link budget where `p` falls off with range. A *draw still happens*,
  so this is a widening of `_reception_for`'s arguments, not a gate. Smaller, separate, and safe to
  do later precisely because it does not change the draw count.

Conflating the two is what would have made this seam wrong.

### 5. `initial_state()` survives as the whole-model seam

Gates are for adding an effect to the standard channel. Replacing the channel entirely is still
done by subclassing `CommunicationModel` and returning your own `CommState` subclass from
`initial_state()` — the seam `test_cns_communication.py` pins end-to-end through `run_fleet`. Two
mechanisms at two levels, which is the cost of this decision and is accepted: "add an effect" and
"replace the channel" are genuinely different asks.

### 6. Bit-for-bit is the binding constraint, and is now tested rather than asserted

ADR 0006 §6 pins draw order, and the existing tests would all still pass if the order moved — a
different but equally-Bernoulli stream is still 70 % reception. Two golden traces now pin the
*realised* sequence:

- `test_cns_communication.py` over plain `Comm` — the ungated channel core, the path almost every
  published number runs through. Encodes held-message age (moves with the reception draw) and the
  in-flight queue's latency to 0.1 s (moves with the latency draw).
- `test_cns_transceiver.py` over `TransceiverComm` — the gated path, across a latching transmitter
  and two receiver failures.

Both were captured from the pre-refactor implementation. They are the check that §3 is a
restructuring and not a re-basing.

## Alternatives rejected

- **Rename to broker/topic/subscriber.** ~20 scripts, 6 test files and 8 vault notes of churn for
  no structural gain, against the explicit lesson of `docs/lesson-learnt.md`.
- **Readability only** — split `Comm.step` into named stages, stop there. Done as part of this work
  (`_offer` / `_deliver`), but insufficient on its own: it improves what reads badly and leaves the
  thing that *is* badly factored.
- **A general middleware/plugin pipeline** (ordered handlers, each free to transform the message,
  duplicate it, or re-time it). Rejected under #17: one filed future feature does not justify a
  framework, and the wider the handler contract the harder §4's stream guarantee becomes.

## Consequences

- Adding a public `gates` attribute to `Comm` changes every `experiment.identity()` cache key
  (`experiment.py`'s reflection treats leading-underscore names as derived, public ones as
  identity). Cached results invalidate and must be recomputed; **no number inside them changes**.
- `RadioState` is currently pinned by `isinstance` and by `.tx_down` / `.rx_down` in
  `test_cns_transceiver.py`. Whether it survives as a `CommState` subclass populated from the gate
  state, or is retired for a `radio_health(state)` accessor, is settled when §3 is implemented —
  the importer count across `scripts/` decides it.
- The three-substream tree (`ips.py`) and four-child tree (`estimator.py`) are unchanged. Gates draw
  from the existing comm stream; a gate needing its own generator would break ADR 0006 §6 and is
  not permitted.
- **Obligation:** if a second gate lands and the two need to observe each other (a range gate that
  should not fire while the radio is already down), revisit — the current contract evaluates gates
  independently and that will not express it.

## Relations

- Extends [[0006-communication-model-design]] — its state shape, delivery timing and RNG layout all
  stand; only §1's open question about extensibility is answered here.
- Rests on the directed-link design of [[0004-layered-directed-design-for-multiaircraft-and-ips]]:
  a gate is per **directed** link, which is why one aircraft's failure is not symmetric.
- The behaviour the gate seam has to preserve is measured in
  [[transceiver-outage-perception]] — including that at n = 2 a failed transmitter and a failed
  receiver are indistinguishable, so gate composition only becomes observable at n ≥ 3.
- Closes the veto half of [[time-varying-reception-probability]]; leaves the continuous-modulation
  half open per §4.
