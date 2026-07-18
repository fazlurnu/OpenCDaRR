# Design Philosophy — CDaRR (draft to ratify)

**This is a draft menu, not law.** Adapted from *The Pragmatic Programmer* (Thomas &
Hunt) and *Clean Code* (Martin), tuned for research code that must be **reproducible and
defensible to reviewers**, in Python, with BlueSky used as a *library*. Go through it and
**accept / reject / reword each principle until it's yours** — a principle you didn't
choose is one you won't follow. Keep it to a page; if it grows, cut.

*How to use it:* when you or an agent are unsure how to write something, this is the
tiebreaker. When two principles conflict — they will — the "Tensions" note at the bottom
says which wins here.

## Core — purity & reproducibility (load-bearing; hardest to add later)

1. **Pure by default, effects at the edges.** Detection / dynamics / resolution /
   recovery are pure `state → value`. RNG, file I/O, and BlueSky calls live only in a thin
   shell. *(Clean Code: no side effects. Pragmatic: orthogonality.)*
2. **One owner of state; pass it, don't hide it.** No module-level or singleton state
   carrying results between calls. *(This is the exact bug class — KI-1, ADSL — that cost
   us last time.)*
3. **Every stochastic thing takes its own RNG.** No shared or global RNG; substreams
   spawn reproducibly. Non-negotiable for a rare-event estimator.
4. **Reproducibility is a feature, tested like one.** A run = `config + seed + code-hash
   → result`. If it can't be regenerated, it's broken — even if the number looks right.
5. **Wrap third parties at a boundary; never let their globals leak inward.** BlueSky is
   called only behind your own interfaces, so it stays swappable and its global state
   never touches yours. *(Clean Code: boundaries. Pragmatic: reversibility.)*

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

## Tensions we accept (when principles fight, this wins)

- Purity (1) vs. performance → **purity wins** until a *measured* bottleneck says otherwise.
- DRY vs. legibility (11) → in the **core math, legibility wins**; in plumbing, DRY wins.
- Clean Code rigor vs. speed (12) → **core = rigor, scripts = speed.**

---
*Companion docs:* `design_brief.md` (what to build) · `how-to-step-by-step.md` (the
order & process) · `lesson-learnt.md` (why we work this way).
