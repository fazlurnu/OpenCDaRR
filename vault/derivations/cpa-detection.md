# Derivation — state-based conflict detection (2D, directed)

Horizontal closest-point-of-approach (CPA) detection for one **directed** pair: does `own`,
using its (perceived) state of `intr`, predict a loss of separation within the lookahead?
Re-derived from first principles — **not** ported from `sim_models/cd_statebased.py`, whose
`n×n` matrix form uses transposed broadcasts (`ownu - intu.T`) with sign/index conventions
that are easy to get wrong; the scalar directed derivation below is unambiguous
(`lesson-learnt.md`: don't port).

- Implemented by: [`opencdarr/cd.py`](../../opencdarr/cd.py) (`detect`, `is_los`)
- Validated by: [`tests/test_cd.py`](../../tests/test_cd.py)
- Geometry helper: `opencdarr.geo.qdrdist` (bearing + range between two lat/lon points)

## Symbols

| symbol | code | meaning | unit |
|--------|------|---------|------|
| $\varphi,\lambda,\psi,v$ | `lat, lon, trk, gs` | state of own / intr | deg, deg, deg, m/s |
| $R$ | `rpz` | protected-zone radius (horizontal separation minimum) | m |
| $T$ | `t_lookahead` | detection lookahead time | s |
| $\mathbf r$ | — | relative position, **intr − own**, in East–North | m |
| $d,\ \chi$ | `dist, qdr` | range and bearing (own → intr) | m, deg |
| $\mathbf v$ | — | relative velocity, **intr − own** | m/s |

Subscripts $o$ = own, $i$ = intr. **Directed:** everything is computed from `own`'s frame and
its perceived `intr`; the reverse pair (intr → own) is a separate call and, under CNS
asymmetry, may differ.

## 1. Relative position and velocity (East–North local frame)

From the geodesy inverse $(\chi, d) = \texttt{geo.qdrdist}(\varphi_o,\lambda_o,\varphi_i,\lambda_i)$:

$$ \mathbf r = (r_E, r_N) = (d\sin\chi,\; d\cos\chi) $$

Velocities resolve into East (sin) and North (cos) with the aviation track convention:

$$ \mathbf v_o = (v_o\sin\psi_o,\; v_o\cos\psi_o), \quad \mathbf v_i = (v_i\sin\psi_i,\; v_i\cos\psi_i), \quad \mathbf v = \mathbf v_i - \mathbf v_o $$

> **Caveat — relative-velocity sign.** The prior paper used $\mathbf v = \mathbf v_o - \mathbf v_i$
> (own − intr). Here we deliberately use $\mathbf v = \mathbf v_i - \mathbf v_o$ (intr − own) so
> that relative *velocity* and relative *position* ($\mathbf r = $ intr − own) share the **same
> convention** — separation is then simply $\mathbf s(t) = \mathbf r + \mathbf v\,t$, with no
> stray minus sign. Flipping this convention flips the sign of $t_{\text{cpa}}$, so it matters
> when comparing formulas across the two codebases.

## 2. Closest point of approach

The separation vector at time $t$ is $\mathbf s(t) = \mathbf r + \mathbf v\,t$. Minimising $|\mathbf s|^2$ gives

$$ t_{\text{cpa}} = -\frac{\mathbf r \cdot \mathbf v}{|\mathbf v|^2}, \qquad d_{\text{cpa}} = \big|\,\mathbf r + \mathbf v\,t_{\text{cpa}}\,\big| $$

Sign check (head-on, `intr` $D$ north closing at $2V$): $\mathbf r=(0,D)$, $\mathbf v=(0,-2V)$
$\Rightarrow t_{\text{cpa}} = D/2V > 0$, $d_{\text{cpa}} = 0$ — CPA in the future, zero miss. ✓

**Degenerate** $|\mathbf v|^2 \to 0$ (parallel, equal speed): no approach; guard by treating it
as no predicted conflict ($t_{\text{cpa}}$ set large, $d_{\text{cpa}} = d$).

## 3. Conflict predicate (prediction only)

When $d_{\text{cpa}} < R$ the pair breaches the zone; the half-chord of time spent inside is

$$ \tau = \frac{\sqrt{R^2 - d_{\text{cpa}}^2}}{|\mathbf v|}, \qquad t_{\text{in}} = t_{\text{cpa}} - \tau, \qquad t_{\text{out}} = t_{\text{cpa}} + \tau $$

$$ \boxed{\;\text{conflict} \;=\; \big(d_{\text{cpa}} < R\big)\ \wedge\ \big(t_{\text{in}} < T\big)\ \wedge\ \big(t_{\text{out}} > 0\big)\;} $$

i.e. the breach window $[t_{\text{in}}, t_{\text{out}}]$ overlaps the lookahead window $[0, T]$.
`detect` computes $t_{\text{cpa}}, d_{\text{cpa}}$ to reach this boolean but **returns only the
boolean** — those quantities are CPA-specific, not a general detection output.

## 4. Loss of separation — separate, current-state

LoS is not a prediction; it is a fact about *now*, independent of any detection algorithm:

$$ \texttt{is\_los} \;=\; \big(d < R\big) $$

Kept as its own function so the loop counts predicted conflicts and actual LoS independently
(`IPR = 1 - n_{los}/n_{conflict}`).

## Notes

- **2D only:** the old code's vertical branch (`dalt`, `dvs`, `hpz`, `tinver/toutver`) is
  dropped — horizontal at fixed altitude, matching the rest of the platform.
- **Directed by construction:** feeding `own` its own perceived `intr` is what makes CNS
  surveillance asymmetry expressible later without changing this function.
- **Batched form deferred:** the `n×n` / SoA vectorisation is a perf-step concern, validated
  against this scalar reference.
