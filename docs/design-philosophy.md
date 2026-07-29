# Design Philosophy — CDaRR

**Ratified.** It began as a menu to accept or reject; the build settled it, and the numbered
principles below are now cited as fixed references elsewhere in the repo — `pyproject.toml` cites
#10 and #12 for keeping the dependency list short, `vault/experiments/README.md` cites #4 for
provenance, `vault/derivations/README.md` cites #11 for naming things after the paper. Renumbering
them would break those citations, so treat the numbers as stable.

Adapted from *The Pragmatic Programmer* (Thomas & Hunt) and *Clean Code* (Martin), tuned for
research code that must be **reproducible and defensible to reviewers**, in Python.

*How to use it:* when you or an agent are unsure how to write something, this is the
tiebreaker. When two principles conflict — they will — the "Tensions" note at the bottom
says which wins here.

## The two books, in a paragraph each

*The Pragmatic Programmer* is about **the economics of change**. Its claim is that no
design survives contact with new requirements, so the real skill is keeping the cost of
changing your mind low: orthogonal pieces that don't drag each other along, decisions
made reversible, one authoritative source for every fact (DRY), a thin end-to-end tracer
bullet before any general machinery, and small rot fixed before it becomes the house
style. It is deliberately unfussy about beauty — "good enough" is an engineering target,
not an excuse.

*Clean Code* is about **the economics of reading**. Its claim is that you read code far
more than you write it, so every line should be optimised for the next person to
understand: names that say what the thing is, functions small enough to hold in your head
and doing exactly one thing, few arguments, no hidden side effects, third-party libraries
kept behind a boundary you own, and comments treated as an admission that the code failed
to explain itself. It is dogmatic where Pragmatic is pragmatic, and that's the useful
tension between them.

## The adjustment: written by AI, read and tested by human

Both books assume one person both writes and reads. Here the machine writes and **you
read, test, and defend** — which flips their cost model. Writing is nearly free;
*reading and verifying* is now the entire budget, and it is yours alone to spend. That
changes three things:

- **Clean Code gets stricter, and for a blunter reason.** Legibility is no longer a
  courtesy to your future self — it is the only channel through which you can check work
  you didn't do. Code you can't follow at reading speed is a defect even when it's
  correct.
- **Pragmatic's DRY loses its keystroke argument.** "Don't type it twice" is worthless
  when typing is free. What survives is "don't have two truths." An abstraction that only
  saves the machine effort but costs you a read is a net loss (this is principle 11,
  sharpened).
- **Broken windows are replaced by surplus.** The failure mode of an AI-written codebase
  isn't neglect, it's *volume* — more plausible code than a human can hold. Restraint,
  not tidiness, is the new discipline.

The principles below are unchanged where the books still apply; the **Authorship**
section is what this project adds.

## Core — purity & reproducibility (load-bearing; hardest to add later)

1. **Pure by default, effects at the edges.** Detection / dynamics / resolution /
   recovery are pure `state → value`. RNG, file I/O, and third-party calls live only in a thin
   shell. *(Clean Code: no side effects. Pragmatic: orthogonality.)*
2. **One owner of state; pass it, don't hide it.** No module-level or singleton state
   carrying results between calls. *(This is the exact bug class — KI-1, ADSL — that cost
   us last time.)*
3. **Every stochastic thing takes its own RNG.** No shared or global RNG; substreams
   spawn reproducibly. Non-negotiable for a rare-event estimator.
4. **Reproducibility is a feature, tested like one.** A run = `config + seed + code-hash
   → result`. If it can't be regenerated, it's broken — even if the number looks right.
5. **Wrap third parties at a boundary; never let their globals leak inward.** Applied to
   [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky), this principle worked so well that the
   dependency was removable: it sat behind our own interfaces until ADR 0003 replaced the last
   call with `opencdarr/geo.py`, and nothing above the boundary changed. Shipping code now depends
   only on `numpy` and `pyyaml`. *(Clean Code: boundaries. Pragmatic: reversibility.)*

## Legibility — write for the reviewer, not the machine

6. **Name it like the paper.** `dcpa`, `level(state)`, `resopairs` — a physicist should
   recognize the literature. *(Clean Code: meaningful names. Pragmatic: program close to
   the domain.)*
7. **Small functions, one job, the equation in the docstring.** If you can't say what it
   does in a few words, split it. *(Clean Code: small functions.)*
8. **Few arguments. Many arguments want a data type.** A function taking ten positional
   args is a missing dataclass. *(Clean Code: function arguments — a smell we had.)*
9. **Compute or change, not both.** A function either returns a value or causes an
   effect. *(Command–query separation.)*

## Judgment — where research breaks the books

10. **Tracer bullets before frameworks.** One thin end-to-end slice that runs and is
    validated, before any general machinery. *(Pragmatic: tracer bullets; see the first
    milestone in `design_brief.md`.)*
11. **Duplication that helps the reader beats DRY that hides the math.** Writing an
    equation in place so a reviewer can check it is worth a little repetition. *(A
    deliberate counter to dogmatic DRY.)*
12. **Match rigor to lifespan.** The core earns this whole list; a throwaway analysis
    script does not. Don't gold-plate. *(Pragmatic: good-enough software.)*
13. **A measured, logged, deferred bug is not a broken window.** Fix small rot early —
    but a known issue you've quantified and written down (KI-1 style) is a decision, not
    neglect. *(Pragmatic: broken windows, tuned by experience.)*

## Authorship — written by AI, read and tested by human

14. **You sign it.** "The model wrote it" is not a defence to a reviewer, and it isn't
    one to yourself. If you can't explain a line, either learn it or delete it — those
    are the only two options.
15. **The test is the human's half of the contract.** The AI writing both the code and
    the test that passes it proves self-consistency, nothing more. You own the expected
    values, and the best ones come from *outside* the code: a closed-form solution, a
    number from a paper, a hand calculation, a conservation law.
16. **Diff budget, not line budget.** A change too large to read in one sitting is too
    large to accept, no matter how fast it was produced. Split it until it fits your
    attention, not the model's.
17. **No unrequested generality.** Config flags, defensive branches, and "we might need
    this" abstractions are cheap to generate and expensive to verify. Every one is
    something you must now read forever. YAGNI, hardened.
18. **Boring Python beats fluent Python.** The model is fluent in every construct; that
    is not a reason to use them. Write in the subset *you* read fastest.
19. **Docstrings carry the provenance the author can't remember.** The AI has no memory
    of why it chose a branch or a form of an equation, and you can't interview it later.
    So record the source in place — the paper, the equation number, the ADR — at the
    moment it's written. This is the one place Clean Code's "comments are a failure"
    rule is overridden.
20. **Attack the frame before building inside it.** Ask for the three biggest risks and
    an argument *against* the plan first. A model will build a cathedral inside whatever
    blueprint it's handed. *(Straight from `lesson-learnt.md`.)*
21. **Understanding beats throughput on anything you must defend.** Full autonomy is for
    throwaway and exploratory work; the core gets smaller batches and diffs you actually
    read.

## Tensions we accept (when principles fight, this wins)

- Purity (1) vs. performance → **purity wins** until a *measured* bottleneck says otherwise.
- DRY vs. legibility (11) → in the **core math, legibility wins**; in plumbing, DRY wins.
- Clean Code rigor vs. speed (12) → **core = rigor, scripts = speed.**
- Generation speed vs. understanding (21) → **understanding wins** on anything that
  reaches a paper; speed wins on scripts you'd throw away without regret.
- Clean Code's "no comments" vs. provenance (19) → **provenance wins** in the core math;
  elsewhere let the code speak.

---
*Companion docs:* `design_brief.md` (what to build) · `how-to-step-by-step.md` (the
order & process) · `roadmap.md` (the milestone trajectory) · `lesson-learnt.md` (why we
work this way) · `vault/architecture-dataflow.md` (the architecture as built).
