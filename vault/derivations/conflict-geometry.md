# Derivation — conflict-encounter geometry (`create_conflict`, 2D)

Place an intruder in conflict with a given ownship: crossing angle `dpsi`, miss distance
`dcpa`, and time-to-loss-of-separation `tlos`. This is the horizontal part of BlueSky's
`creconfs` (`traffic.py:316`), verified and re-derived in **our** convention (relative
velocity = intr − own; no wind; no vertical). It is the encounter generator the scenario
layer samples.

- Implemented by: [`opencdarr/scenario.py`](../../opencdarr/scenario.py) (`create_conflict`)
- Validated by: [`tests/test_scenario.py`](../../tests/test_scenario.py) — the generated pair,
  fed back through the CPA equations, reproduces the requested `dcpa` and `tlos`.
- Sibling: [`cpa-detection.md`](cpa-detection.md) (same relative-motion algebra, inverted).

## Symbols

| symbol | code | meaning | unit |
|--------|------|---------|------|
| $\psi_o, V_o$ | `own.trk, own.gs` | ownship track, ground speed | deg, m/s |
| $\Delta\psi$ | `dpsi` | crossing angle (intr track − own track) | deg |
| $V_i$ | `gs_intr` | intruder ground speed (default $V_o$) | m/s |
| $d_c$ | `dcpa` | desired miss distance at CPA | m |
| $t_L$ | `tlos` | desired time to loss of separation | s |
| $R$ | `rpz` | protected-zone radius | m |
| $s$ | `side` | which side the intruder passes, $\pm1$ | — |

## 1. Velocities and relative velocity (intr − own)

$$ \psi_i = \psi_o + \Delta\psi $$
$$ \mathbf v_o = (V_o\sin\psi_o,\ V_o\cos\psi_o), \quad \mathbf v_i = (V_i\sin\psi_i,\ V_i\cos\psi_i), \quad \mathbf w = \mathbf v_i - \mathbf v_o,\ \ v_{\text{rel}} = |\mathbf w| $$

(Our convention, matching `cpa-detection.md`; BlueSky's `creconfs` uses own − intr.)

## 2. Distances

Relative distance travelled from the start to CPA — time-to-LoS times closing speed, then the
half-chord from LoS entry to CPA:

$$ d_{\text{rel}} = t_L\,v_{\text{rel}} + \begin{cases}\sqrt{R^2 - d_c^2}, & d_c < R\ (\text{a real LoS})\\[2pt] 0, & d_c \ge R\ (t_L \text{ is then time-to-CPA})\end{cases} $$

Initial range, by Pythagoras (along-closing $d_{\text{rel}}$ ⟂ miss $d_c$):

$$ \text{dist} = \sqrt{d_{\text{rel}}^2 + d_c^2} $$

## 3. Initial relative position and intruder placement

At CPA the relative position is perpendicular to $\mathbf w$ with magnitude $d_c$; initially it
also has an along-$\mathbf w$ component of $-d_{\text{rel}}$ (negative = closing). With
$\hat{\mathbf w} = \mathbf w / v_{\text{rel}}$ and perpendicular $\hat{\mathbf n} = s\,(-w_N, w_E)/v_{\text{rel}}$:

$$ \mathbf r_0 = -d_{\text{rel}}\,\hat{\mathbf w} + d_c\,\hat{\mathbf n} \quad (\text{= intr} - \text{own}) $$

Bearing and placement (own → intr), using our own geodesy:

$$ \beta = \operatorname{atan2}(r_{0,E},\ r_{0,N}), \qquad (\varphi_i, \lambda_i) = \texttt{geo.forward}(\varphi_o, \lambda_o, \beta, \text{dist}) $$

The intruder is `AircraftState(lat=φ_i, lon=λ_i, trk=ψ_i, gs=V_i)`.

## 4. Correctness (why this reproduces `dcpa` and `tlos`)

Feed $\mathbf r_0, \mathbf w$ into the CPA equations of `cpa-detection.md`:

- $t_{\text{cpa}} = -(\mathbf r_0\cdot\mathbf w)/v_{\text{rel}}^2 = d_{\text{rel}}/v_{\text{rel}}$
  (the perpendicular part contributes nothing; the along part is $-d_{\text{rel}}$).
- $d_{\text{cpa}} = |\mathbf r_0 + \mathbf w\,t_{\text{cpa}}|$ = the perpendicular component = $d_c$. ✓
- LoS entry is a half-chord before CPA: $t_{\text{in}} = t_{\text{cpa}} - \sqrt{R^2-d_c^2}/v_{\text{rel}} = t_L$. ✓

**Head-on check** ($\Delta\psi=180°$, $d_c=0$, own heading N): intruder lands due north at range
$2V_o t_L + R$, closing at $2V_o$ → LoS at exactly $t_L$. ✓

## Notes

- **Degenerate** $v_{\text{rel}}\to 0$ ($\Delta\psi=0$, equal speed): no closing geometry exists;
  `create_conflict` rejects it (raise), since no conflict can be constructed.
- **Side** $s=\pm1$ mirrors the intruder left/right (same `dcpa`/`tlos`); the scenario sampler
  may randomise it from the seeded RNG for encounter diversity.
- **2D, no wind:** ground speed = true airspeed; the CAS/Mach/wind conversions in `creconfs`
  drop out.
