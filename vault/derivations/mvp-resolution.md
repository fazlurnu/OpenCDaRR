# Derivation — Modified Voltage Potential resolution (2D, directed)

The horizontal MVP maneuver for one directed pair: what velocity should `own` adopt to just
clear the protected zone of `intr`? Re-derived from `sim_models/cr_mvp.py` (`MVP` + the
horizontal part of `resolve`), 2D, our convention (relative velocity = intr − own).

- Implemented by: [`opencdarr/cr/mvp.py`](../../opencdarr/cr/mvp.py) (`MVP`)
- Validated by: [`tests/test_cr.py`](../../tests/test_cr.py)
- Shares the CPA algebra of [`cpa-detection.md`](cpa-detection.md).

## Symbols

| symbol | meaning | unit |
|--------|---------|------|
| $\mathbf r$ | relative position, intr − own (E,N) | m |
| $\mathbf v$ | relative velocity, intr − own | m/s |
| $R$ | resolution zone $= \texttt{rpz}\times\texttt{margin}$ (margin ≥ 1) | m |
| $\mathbf c$ | relative position at CPA (own → intr) | m |
| $d_m$ | miss distance $=|\mathbf c|$ | m |

## 1. CPA geometry (MVP computes its own)

$$ t_{\text{cpa}} = -\frac{\mathbf r\cdot\mathbf v}{|\mathbf v|^2}, \qquad \mathbf c = \mathbf r + \mathbf v\,t_{\text{cpa}}, \qquad d_m = |\mathbf c| $$

$\mathbf c$ points from own toward intr at closest approach. **Head-on guard:** if
$d_m \le 10^{-3}$ m the direction is undefined, so set $\mathbf c$ perpendicular to $\mathbf r$,
$\mathbf c = (r_N, -r_E)\,d_m/|\mathbf r|$ — which picks a side to turn.

## 2. Resolution magnitude (the "modified" part)

To make the trajectory *tangent* to the zone of radius $R$ (not merely reach range $R$ at
CPA), the required outward gain along $\hat{\mathbf c}$ uses the erratum correction:

$$ \text{gain} = \begin{cases} \dfrac{R}{\varepsilon} - d_m, & R < d \ \text{and}\ d_m < d,\quad \varepsilon = \cos\!\big(\arcsin\tfrac{R}{d} - \arcsin\tfrac{d_m}{d}\big) \\[8pt] R - d_m, & \text{otherwise} \end{cases} $$

where $d = |\mathbf r|$ is the current range. The velocity increment that achieves this over
the time to CPA:

$$ \mathbf{dv} = \frac{\text{gain}}{|t_{\text{cpa}}|}\,\hat{\mathbf c}, \qquad \hat{\mathbf c} = \mathbf c / d_m $$

## 3. New velocity and command (steer away)

The ownship applies the **negative** of $\mathbf{dv}$ (the old code's `dv -= dv_mvp`) — it
steers *away* from the intruder's CPA position:

$$ \mathbf v_o' = \mathbf v_o - \mathbf{dv} $$
$$ \psi' = \operatorname{atan2}(v'_{o,E},\ v'_{o,N}), \qquad V' = |\mathbf v_o'| $$

Return `Command(hdg=ψ', spd=V')`. Speed is **not** capped here — `step_dynamics` clamps it to
the envelope, so `resolve` stays pure geometry and the dynamics own feasibility.

## Notes

- **Directed / cooperative:** each aircraft resolves against its own perceived intruder;
  pairwise, both call `resolve` and both maneuver (full horizontal resolution each, matching
  the old code, which halves only the vertical term).
- **`margin`** (MVP instance parameter, default 1.0) enlarges the resolution zone beyond
  `rpz` — the old `asas_marh` (1.05). A genuine per-algorithm parameter, hence a class.
- **2D:** the vertical resolution (`dv3`, `tsolV`) is dropped.
- **Degenerate** $|\mathbf v|\to 0$: no relative motion, nothing to resolve — return the
  nominal command unchanged.
