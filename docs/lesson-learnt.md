# Lessons Learned — the FP refactor of CDaRR

Written 2026-07-18, at the end of the `fp-refactor` effort, as honest notes for
starting fresh. Not a post-mortem of failure — the work is green and reproducible —
but of *why the result can be fully "successful" and still feel bad*, and what to do
differently next time.

---

## The core lesson: refactor ≠ redesign

The whole exercise was gated on **bit-identical output** against golden baselines. That
was the right call for protecting reproducibility — but it has a consequence that's easy
to miss:

> A behaviour-preserving refactor can only ever *reorganise* bad code. It can never
> *fix* it. By construction, whatever was structurally wrong got faithfully carried
> into the new shape.

So the code feels bad because it *is* still the same design — now spread across `cd/`,
`cr/`, `crr/`, `cns/`, `cdarr/`, **plus** legacy shims, **plus** the old `sim_models/`
forwarding paths. That's *more* surface area, not less. We reorganised; we did not
redesign. If your real goal was "good code," a bit-identical refactor was the wrong tool
for it — that goal needs a redesign, which by definition changes behaviour and produces
new numbers.

**Decide this first, every time:** am I *reorganising* (keep behaviour, safe, gated on
equivalence) or *redesigning* (change behaviour, accept new results, re-validate the
science)? They are different projects with different risk. We did the first while partly
wanting the second.

---

## What the "bad" actually is (and you already knew)

The genuinely bad part of this codebase is **architectural, not stylistic**: it's built
on BlueSky's mutable global state — `bs.traf`, `bs.sim`, `bs.settings`, and the
`Entity`-metaclass singletons (`MVP`/`VO`). That coupling is the source of:

- the KI-1 cross-run recovery-state leak (a *singleton* silently shared results between
  runs),
- the ADSL shared-RNG bug found in Phase 3,
- the cwd-relative file reads, the 4-node message-passing dance, the "reset by calling
  `bs.traf.reset()` and hope" pattern.

No amount of functional wrapping removes that — the functions still take `ownship`/
`intruder` objects that are BlueSky traffic handles. You **already had the right
instinct** here: your `engine_rewrite_spec.md` (the Rust+PyO3, BlueSky-free engine) is
exactly the redesign that would fix the actual problem. This refactor was, in hindsight,
a detour *around* that instinct rather than toward it.

---

## The meta-lesson about working with an AI

This is the part most worth internalising for "managing an AI client":

1. **The AI optimises inside the frame you give it; it will not reliably challenge the
   frame.** I read the code, wrote a 36 KB plan, and executed 24 commits — competently.
   What I did *not* do forcefully enough was stop at the start and say *"a bit-identical
   functional refactor may not get you the good code you actually want; here's the
   cheaper path."* You have to own that question, because the model will happily build a
   cathedral inside whatever blueprint you hand it.
   - **Habit to adopt:** before any big plan, ask *"What are the 3 biggest risks with
     this approach, and does it even achieve my real goal? Argue against doing it."*
     Make the AI attack the premise before it executes it.

2. **AI lowers the cost of *starting* big undertakings — which makes it easy to start
   the wrong one.** A refactor of this size would normally be daunting enough that you'd
   question whether it's worth it. Because I could just *do* it, we did. The seductive
   part ("look how much got done") hid the opportunity-cost question: your actual
   deliverable pressure is the **reviewer action items on the paper**
   (`reviewer_action_items_todo.md`), not code aesthetics. A human pair-programmer who
   found the KI-1 bug might have said *"just fix this one line and get back to your
   paper."* I built infrastructure instead.
   - **Habit:** measure a task against your real deadline, not against "is it doable."
     Doable is now cheap; *worth it* is the scarce judgment.

3. **Autonomous loops trade *your understanding* for throughput.** The `/loop` streamed
   phases past you at gates. It's efficient, but you now likely understand the new
   structure *less* than if you'd done fewer, larger steps and read each diff yourself.
   For a codebase *you* have to maintain and defend in a paper, understanding > speed.
   - **Habit:** for anything you must own, run smaller batches and actually read the
     diffs. Use full autonomy for throwaway/exploratory work, not for your core artifact.

4. **The plan was mine, inferred from the code — not from your research intent.**
   `refactor_fp.md` is detailed but it encodes *what the code does*, not *what your
   research needs*. Plans should start from your goals and constraints, which the AI
   cannot infer from source alone.

---

## What genuinely went well — keep these

Don't over-correct. These were good and are worth repeating on the next project:

- **Freeze a reference + golden tests *before* touching anything.** This is excellent
  discipline and it's what caught the real bugs. Keep it — but point it at *new, clean*
  code as regression protection, not as a cage around old behaviour.
- **Worktree isolation** (`git worktree add`) — clean, no risk to `main`.
- **Small, well-described commits** — the git history is genuinely readable.
- **Stopping to investigate the anomaly instead of plowing through.** When the
  uncommitted KI-1 change surfaced, pausing to diff it found a real bug. That instinct is
  gold; never let a loop steamroll a surprise.
- **Pausing at phase gates for review** rather than fully unattended runs.

---

## The thing that actually mattered most here

Two real bugs, not the refactor:

- **KI-1** — the singleton leak affected `exp3/4/5`'s published numbers (only the first
  rep per joblib worker was clean; the rest inherited stale state). We measured the
  aggregate-IPR impact as *below the noise floor at a small batch* — **reassuring but not
  proven at full scale.** This is a genuine scientific loose end that outranks any code
  tidiness: **before you cite those figures, re-run the affected experiments with the fix
  and confirm the paper's conclusions hold.**
- **ADSL shared-RNG** — reception and noise shared one RNG, shifting the stream mid-run.

These are the real value that came out of the session. Don't let them get buried under
"we did a big refactor."

---

## If starting from scratch

1. **Start from the goal, not the codebase.** Write down what the research needs:
   reproducible experiments, legible CDR algorithms, and the ability to try new recovery
   criteria (your `my-observation.md` #14–16 — the signed-tCPA / balanced-criteria ideas
   you actually want to explore). Build the smallest thing that runs *one* experiment
   end-to-end cleanly, then grow it.
2. **Cut the BlueSky global-state dependency** — this is where the "bad" lives. Your
   `engine_rewrite_spec.md` already points the right way. A pure state → state step
   function (positions/velocities in, commands out — which `cdarr_step()` gestures at but
   is still tethered to BlueSky handles) is the target.
3. **Don't port. Rebuild the core you understand.** Porting drags the old design's bones
   into the new body (which is exactly what happened here). Re-derive the ~5 real
   equations (detection CPA, MVP, VO, the recovery criteria) into clean functions with
   no framework objects.
4. **Keep golden tests, but as guardrails for the new clean code**, seeded from a
   trusted run — not as a bit-identical straitjacket around the old behaviour.
5. **Timebox the redesign against the paper.** If it can't run one experiment cleanly in
   N days, the old code + the two bug fixes is a legitimate "good enough" stopping point.

---

## One-line version

*We successfully reorganised code whose real problem was its design, spent the effort of
a redesign to get the safety of a refactor, and the two bugs we found along the way
mattered more than the reorganisation. Next time: decide reorganise-vs-redesign up
front, make the AI argue against the plan before building it, and measure the work
against the paper, not against "can it be done."*
