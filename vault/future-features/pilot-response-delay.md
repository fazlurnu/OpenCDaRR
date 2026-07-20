# Future feature — pilot response delay

**Target version: post-v1.0** (`docs/roadmap.md`'s "community long game, beyond v1.0" bucket —
same placement as [[time-varying-reception-probability]]) — just a marker, not designed or
scheduled; flagged so it isn't lost.

## What

A resolved `Command` currently takes effect on the very next integration step
(`opencdarr/dynamics.py`, `step_dynamics`) — the instant MVP/VO computes a maneuver, the aircraft
starts flying it. Real crews (or an autopilot handing off to a human, or ATC-mediated resolution)
have a **response delay** between an alert/command and the maneuver actually starting — reaction
time, workload, procedure — before dynamics even begin to respond.

## Why deferred

Not part of the CNS-uncertainty layer (navigation / communication / surveillance,
`vault/phase-3-plan.md`) — it's a **CR execution-timing** effect, a different kind of delay from
broadcast latency (which delays *information*; this delays *action* on already-correct
information). Needs its own design pass: does it live in `step_dynamics`, in `loop.py`'s
`_decide`, or as a genuinely new layer — not decided.

## Marker: ground this in Stroeve (NLR), don't invent a distribution

**Follow Sybert Stroeve's (NLR) work on pilot/controller response-time modeling in conflict
resolution** — use an empirically-grounded response-delay distribution from that line of research,
not an arbitrary constant or a guessed shape. Read the paper before designing this, not after.
*(Exact citation not yet on file — add to `vault/papers/` when this is actually picked up.)*

## Relations

- `opencdarr/dynamics.py` — `step_dynamics`, where a command currently takes effect immediately.
- `opencdarr/loop.py` — `_decide`, the other plausible place a delay could live.
- `vault/papers/` — where the Stroeve citation belongs once picked up (not yet added).
