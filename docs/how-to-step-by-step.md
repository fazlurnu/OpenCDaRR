# How to drive a coding agent to build the new CDaRR

For **you** — still learning to manage an AI coding collaborator — to build the new CDaRR
one step at a time, staying in control and *understanding what's going on*. It is both a
**process** (Part A: how to run any step) and a **build order** (Part B: the milestones).
Living doc: it grows one step at a time. Do not expand it into a blueprint — see
`lesson-learnt.md` on the 36 KB trap.

**The four documents you carry into the clean session:**
- `design_brief.md` — **what** to build (goal + architecture + the BlueSky-as-library spine).
- `design-philosophy.md` — **how** to write it (the standards; the tiebreaker).
- `how-to-step-by-step.md` — the **order + process** (this file).
- `lesson-learnt.md` — **why** we work this way (what went wrong before).

---

## Part A — the loop you run for EVERY step

This is the whole skill. Run it for each milestone in Part B.

1. **You set the goal + the decisions only you can make.** One step, one goal. Name the
   choices that are yours (e.g. the coordination model, the level function) — don't let
   the agent decide them silently.
2. **Make the agent argue against it first.** Before any code: *"Restate the goal. What
   are the 3 biggest risks with this approach? What would you do differently? What are you
   unsure about?"* This is the guard against premise-lock (the main lesson). If it just
   agrees, push harder.
3. **Agent proposes a plan for THIS step only.** You approve or adjust. Reject scope creep.
4. **Small batch, then you read the diff.** Not the whole milestone at once. If you can't
   read the diff, the batch is too big — say so.
5. **Gate: the validation must pass, and you must see it run.** Every step has a check (a
   golden/anchor test, a match to a known number). Green is necessary, not sufficient —
   ask *"what would make this test pass while the code is still wrong?"*
6. **Understand-check — the point of all this.** Have the agent explain, in plain
   language, what changed and why. Then say it back in your own words. If you can't, you're
   not done: ask questions until you can.
7. **Commit small, with provenance.** One logical change per commit; a clear message.

Then repeat. **When something surprises you, STOP and investigate** — don't let the agent
smooth past it. That instinct is what found the real bugs last time.

### Red flags to watch for
- Silver-bullet / "just trust me" claims → make it show evidence.
- Tests pass but you're unsure they test the right thing → ask it to make the test fail on
  purpose, to prove the test bites.
- Scope creep ("while I was in here I also…") → revert the extra; one step.
- You've stopped reading diffs → slow down; throughput is not the goal, understanding is.
- You're building infrastructure while the paper deadline slips → ask *is this worth it vs.
  the paper?*

---

## Part B — the build order (thin first; validate the risky part first)

Each step: **goal · your decision · hand the agent · gate.** Keep them small.

**Step 0 — Scaffolding.**
Goal: empty repo + vault skeleton, these 4 docs in place, one RNG/seed convention chosen,
one config format. · *Your decision:* repo layout; RNG scheme (SeedSequence.spawn per
particle). · *Gate:* imports work; a trivial test runs; vault folders exist.

**Step 1 — Extract & validate `step_dynamics` (the make-or-break; do it FIRST).**
Goal: a pure `step_dynamics(state, command, dt) -> state` reproducing BlueSky's M600
turn/accel-limited motion, with no `bs.traf` driving it. · *Your decision:* what "close
enough" to BlueSky means (bit-exact, or a tolerance?). · *Hand the agent:* one recorded
BlueSky trajectory (HDG/SPD commands → lat/lon/trk over time) as the target. · *Gate:*
your pure step matches that trajectory to your chosen tolerance. **If it can't in a few
days, STOP and reconsider scope** — `design_brief.md` flags this as the one real risk.

**Step 2 — One clean pairwise encounter (the tracer bullet).**
Goal: own-state + `step_dynamics` + ONE CDR method (the simplest) + plain Monte Carlo →
reproduce one known IPR from the old code. · *Your decision:* which known result is the
anchor. · *Gate:* new IPR matches the anchor within Monte Carlo error, run end-to-end from
a config+seed.

**Step 3 — The rest of the CDR layer.**
Goal: bring in the pure detection/resolution/recovery functions (the `cd`/`cr`/`crr`
packages from the fp-refactor are reusable here), each validated. · *Gate:* each CDR
method reproduces its old-code anchor.

**Step 4 — Multi-aircraft environment.**
Goal: an N-aircraft world with a conflict graph + an explicit coordination model. · *Your
decision (write an ADR):* cooperative vs. priority vs. sequential — this is research, so
record the choice and why. · *Gate:* with N=2 it reduces to the Step 2 pairwise result (a
free regression check).

**Step 5 — Estimator interface.**
Goal: refactor the Monte Carlo into `advance / level / is_terminal`; plain MC still
matches. · *Gate:* identical results through the new interface.

**Step 6 — Blom–Bakker IPS.**
Goal: the interacting particle system over the interface; per-particle RNG; report
probability **+ confidence interval**, not IPR. · *Your decision (ADR):* the level
function. · *Gate:* in a **not-too-rare** regime, IPS agrees with brute-force MC (the
validation ladder). Only trust the rare regime once this holds.

**Step 7 — Production runs + analysis.**
Goal: the actual rare-event experiments, provenance cards, figures. · *Gate:* reproducible
from `config + seed + hash`; every figure traceable.

---

## When to stop
If the goal is the paper, the earliest step that answers a reviewer is a legitimate
stopping point. Building all seven steps is an *option*, not a requirement.
(See `lesson-learnt.md`.)
