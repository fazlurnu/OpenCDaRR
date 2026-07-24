# Derivation — `FixedWing` coordinated-turn step (2D horizontal, wind-ready)

The governing equations for one time step of a fixed-wing point mass, bank- and stall-limited with a
finite roll rate, as a **pure** map `(state, command, perf, dt) -> state`. Re-derived from the
kinematic model of Reyner & Liem (*Energy-Efficient Trochoidal Path Planning...*, Drones 2026, 10, 426;
`../papers/drones-wind.pdf`, Eqs 1–17), **not ported** (`lesson-learnt.md`). The same model PX4's
`fw_lateral_longitudinal_control` implements.

- Implemented by: [`opencdarr/dynamics/fixedwing.py`](../../opencdarr/dynamics/fixedwing.py)
- Validated by: [`tests/test_fixedwing_dynamics.py`](../../tests/test_fixedwing_dynamics.py)
- How we validate: [ADR 0002](../decisions/0002-analytical-validation-of-dynamics.md) (analytical vs
  the paper's closed forms — the BlueSky anchor was retired with Dubins, [ADR 0013](../decisions/0013-fixedwing-coordinated-turn.md))
- Constants: [`opencdarr/performance.py`](../../opencdarr/performance.py) (`SMALL_FIXEDWING`)

## Symbols

| symbol | code | meaning | unit |
|--------|------|---------|------|
| $x,y$ | `lon, lat` (via `geo`) | position (east, north) | deg |
| $\psi$ | `yaw` | heading (nose / airspeed vector) | deg |
| $\chi$ | `trk` | ground-track course (output) | deg |
| $\phi$ | `bank` | bank (roll) angle, signed (+ = right) | deg |
| $V_\text{TAS}$ | `gs` (= airspeed at $w=0$) | true airspeed | m/s |
| $V_\text{GS}$ | `gs` (output) | ground speed $\sqrt{\dot x^2+\dot y^2}$ | m/s |
| $\phi_\max$ | `perf.phi_max` | max bank | deg |
| $p_\max$ | `perf.roll_rate_max` | max roll rate | deg/s |
| $V_s,\,V_\max$ | `perf.v_min, perf.v_max` | stall / max airspeed | m/s |
| $a_x$ | `perf.ax` | max airspeed acceleration | m/s² |
| $(w_x,w_y)$ | — | steady wind, **0 this pass** | m/s |
| $g$ | `9.80665` | gravity | m/s² |

Primes denote the next step. The wind term is written but held at zero; Phase 5 feeds a non-zero
vector with no change below.

## 1. Airspeed — clamp to the envelope, then ramp

The commanded airspeed is clamped into $[V_s, V_\max]$ and approached by at most $a_x\Delta t$
(unset ⇒ hold current). At $w=0$ the airspeed equals the ground speed carried in `gs`:

$$ V_t = \mathrm{clip}(V_\text{cmd},\, V_s,\, V_\max), \qquad V = V + \mathrm{clip}(V_t - V,\, \pm a_x\Delta t) $$

## 2. Bank authority — structural limit tightened by stall-in-turn

A coordinated turn raises the stall speed by the load factor $n=1/\cos\phi$, so
$V_\text{stall}(\phi)=V_s\sqrt{1/\cos\phi}$. Requiring $V \ge V_\text{stall}(\phi)$ gives
$\cos\phi \ge (V_s/V)^2$, i.e. a bank cap that tightens toward stall, combined with the structural cap:

$$ \phi_\text{max,eff} = \min\!\big(\phi_\max,\; \arccos\big[(V_s/V)^2\big]\big) $$

## 3. Heading target and the desired bank

The heading target is the commanded $\psi$ (`airspeed_direction`, which overrides `course`), else the
commanded course $\chi$ (at $w=0$, $\psi_\text{cmd}=\chi_\text{cmd}$), else hold. The heading error is
signed and shortest-way. A proportional controller (gain 1, deg → deg/s) caps the desired **turn rate**
at $\omega_\max = g\tan\phi_\text{max,eff}/V$, and the desired **bank** realises it via $\omega=g\tan\phi/V$:

$$ e = \big((\psi_\text{cmd}-\psi+180)\bmod 360\big)-180, \quad
   \omega_\text{des} = \mathrm{clip}(e,\,\pm\omega_\max), \quad
   \phi_\text{des} = \arctan\!\big(\omega_\text{des}\,V/g\big) $$

## 4. Finite roll — bank is state

Bank moves toward $\phi_\text{des}$ by at most $p_\max\Delta t$ (why `bank` must be state: the bound is
relative to the previous bank), then clamped:

$$ \phi' = \mathrm{clip}\big(\phi + \mathrm{clip}(\phi_\text{des}-\phi,\,\pm p_\max\Delta t),\; \pm\phi_\text{max,eff}\big) $$

## 5. Heading — integrate $\dot\psi = g\tan\phi/V$, or snap when within a step

$$
\psi' =
\begin{cases}
(\psi + \Delta t\,\omega')\bmod 360, & |e| > |\Delta t\,\omega'| \\[2pt]
\psi_\text{cmd}, & \text{otherwise}
\end{cases}
\qquad \omega' = \dfrac{g\tan\phi'}{V}\;[\text{deg/s}]
$$

## 6. Position — air-relative velocity + wind, then course/ground-speed as outputs

$$ \dot x = V\sin\psi' + w_x, \qquad \dot y = V\cos\psi' + w_y \qquad (w_x=w_y=0) $$
$$ V_\text{GS} = \sqrt{\dot x^2 + \dot y^2}, \qquad \chi' = \operatorname{atan2}(\dot x,\dot y) $$

then a great-circle step of $V_\text{GS}\Delta t$ metres along $\chi'$ via `geo.forward`. At $w=0$
this reduces to $\chi'=\psi'$, $V_\text{GS}=V$ — the wind-readiness invariant the tests assert.

## Result

$$ \text{state}' = \big(\text{id},\; x',\; y',\; \chi'=\text{trk},\; V_\text{GS}=\text{gs},\; \psi'=\text{yaw},\; \phi'=\text{bank}\big) $$

plus the odometry accumulators (`flight_time`, `distance_flown`) via the shared `odometry_update`
(ADR 0010).

## Checks (analytical, ADR 0002)

1. **Wind-readiness ($w=0$):** $\psi=\chi$ and $V_\text{GS}=V_\text{TAS}$ every step.
2. **Steady-turn radius:** settled at $\phi_\max$, $R = V^2/(g\tan\phi_\max)$.
3. **Finite-roll heading change:** during a constant-rate roll-in, $\Delta\psi = \frac{g}{V p_\max}\ln\frac{\cos\phi_a}{\cos\phi_b}$ (Eq 15).
4. **Non-holonomic:** cannot stop ($V\ge V_s>0$); cannot side-slip (ground velocity along $\chi$; a raw
   velocity command has no fixed-wing channel and fails fast).
5. **Stall-in-turn:** near $V_s$, the effective bank shrinks below $\phi_\max$.
