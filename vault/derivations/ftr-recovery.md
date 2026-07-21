# Derivation — FTR (Free-To-Revert) recovery (2D)

How `opencdarr.crr.FTR` decides whether an aircraft may stop resolving and revert to its desired
(nominal) velocity. Re-derived from `CDaRR_git/sim_models/crr_resumenav_ftr.py`
(`resumenav_double_criteria`), double-criteria design.

- Implemented by: [`opencdarr/crr/ftr.py`](../../opencdarr/crr/ftr.py) (`FTR`)
- Validated by: [`tests/test_ftr.py`](../../tests/test_ftr.py)
- Probabilistic generalisation: [[probabilistic-ftr-recovery]]

## The check: would reverting clear the intruder?

Unlike [[pastcpa-recovery]] (resume once already diverging), FTR resumes **proactively**: as
soon as flying `own`'s desired velocity — instead of whatever it's currently doing to resolve —
would keep the closest-approach offset beyond `rpz`. `own`'s side of the comparison is always its
**desired** velocity (its own exact, declared intent), never its current (resolving) one.

For relative position $r$ (intr − own) and a candidate relative velocity $v$, the forward
closest-approach offset uses the same $t_{cpa}$ formula as [[cpa-detection]]:

$$ t_{cpa} = -\frac{r \cdot v}{v \cdot v}, \qquad d_{cpa} = \begin{cases} \lVert r \rVert & t_{cpa} \le 0 \text{ (already diverging / no relative motion)} \\ \lVert r + t_{cpa}\, v \rVert & t_{cpa} > 0 \end{cases} $$

`_clears` returns whether $d_{cpa} > \text{rpz}$. The $t_{cpa} \le 0$ branch matters: it falls
back to the **current** separation rather than extrapolating backward — an already-diverging pair
is judged safe by where it is now, not by a hypothetical past closest approach. (This is the one
place [[probabilistic-ftr-recovery]] deliberately does **not** follow suit — see that doc's
"known divergence" section.)

## Two criteria, gated by intent-sharing

1. **The intruder holds its current (observed) velocity.** Always evaluated — `velocity_enu(intr)`
   against `own.desired`.
2. **The intruder reverts to its own desired velocity too**, intent-based — only evaluated if
   `intr.desired is not None` (`run_encounter`'s `share_intent`, ADR 0006). Without it, FTR falls
   back to criterion 1 alone.

`should_resume` returns True only if every evaluated criterion clears.

## Notes

- Raises `ValueError` if `own.desired` is `None` — FTR needs its own intent to be meaningful;
  `run_encounter` always sets it (`AircraftState`'s docstring).
- The reference's second criterion compares against the intruder's velocity **logged at the start
  of the conflict** (per-pair memory, `_intr_init_vel`); ours compares against the intruder's
  **currently declared** desired velocity instead — no per-pair memory needed, since `desired` is
  already carried on the state and (if shared) perceived directly.
