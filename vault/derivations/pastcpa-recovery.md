# Derivation — Past-CPA conflict recovery (2D, directed)

When may `own` stop resolving and resume its nominal navigation? Past-CPA — the simplest
recovery criterion — resumes once the pair is **diverging** and separation is **no longer
lost**. Re-derived from `sim_models/crr_resumenav_cpa.py`, 2D, our convention (r, v = intr −
own).

- Implemented by: [`opencdarr/crr/pastcpa.py`](../../opencdarr/crr/pastcpa.py) (`PastCPA`)
- Validated by: [`tests/test_crr.py`](../../tests/test_crr.py)
- Shares the CPA algebra of [`cpa-detection.md`](cpa-detection.md).

## Predicate

With relative position $\mathbf r = $ intr − own and relative velocity $\mathbf v = $ intr −
own (East–North), $d = |\mathbf r|$:

- **Past CPA (diverging):** $t_{\text{cpa}} < 0$. Since $t_{\text{cpa}} = -(\mathbf r\cdot\mathbf v)/|\mathbf v|^2$, this is $\mathbf r\cdot\mathbf v > 0$.
- **Loss of separation:** $\texttt{is\_los} = d < R$.

$$ \boxed{\ \texttt{should\_resume} \;=\; (\mathbf r\cdot\mathbf v > 0)\ \wedge\ \neg\,\texttt{is\_los}\ } $$

Resume once past the closest approach and outside the protected zone. (Sign note: the old
code writes $\text{dot}(\mathbf r_{\text{intr}-\text{own}},\ \mathbf v_{\text{own}-\text{intr}}) < 0$,
the *same* as $\mathbf r\cdot\mathbf v > 0$ in our convention — see the `cpa-detection.md`
caveat.)

## Optional bouncing guard

Near-parallel conflicts can oscillate (resume → immediately re-detect → resolve → …). The
guard keeps resolving while tracks are near-parallel *and* still close to the zone:

$$ \texttt{is\_bouncing} = \big(|\Delta\psi| < 30°\big)\ \wedge\ \big(d < 1.05\,R\big) $$

where $\Delta\psi$ is the signed track difference. With the guard on, `should_resume` also
requires $\neg\,\texttt{is\_bouncing}$. **Off by default** — the pure criterion is the two
terms above; enable it to match the old code's robustness.

## Notes

- **Directed / per-ownship**; `intr` is `own`'s perceived intruder.
- **`nominal` not needed here** — Past-CPA is purely geometric. FTR-family criteria read the
  nominal from the state (a field added when the first such criterion lands) and hold their
  own detector + lookahead.
