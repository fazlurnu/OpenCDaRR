# Derivation — `step_dynamics` for the M600 (2D horizontal)

The governing equations for one time step of a single M600, turn-rate- and speed-limited,
as a **pure** map `(state, command, perf, dt) -> state`. Re-derived from the BlueSky fork —
`bluesky/traffic/traffic.py:518-542` (integration) and `:288-289` (constants) — not
imported (`lesson-learnt.md`: don't port).

- Implemented by: [`opencdarr/dynamics.py`](../../opencdarr/dynamics.py)
- Validated by: [`tests/test_dynamics.py`](../../tests/test_dynamics.py)
- Decision on how we validate: [ADR 0002](../decisions/0002-analytical-validation-of-dynamics.md)
- Constants: [`opencdarr/performance.py`](../../opencdarr/performance.py)

## Symbols

| symbol | code | meaning | unit |
|--------|------|---------|------|
| $\varphi,\lambda$ | `lat, lon` | position | deg |
| $\psi$ | `trk` | track over ground (= heading, no wind) | deg |
| $v$ | `gs` | ground speed | m/s |
| $\omega$ | `turn_rate` | turn rate, signed (+ = clockwise) | deg/s |
| $\psi_c$ | `command.hdg` | commanded heading | deg |
| $v_c$ | `command.spd` | commanded speed | m/s |
| $\omega_{\max}$ | `perf.max_tr` = 15 | max turn rate | deg/s |
| $\alpha_{\max}$ | `perf.max_dtr2` = 10 | max turn-rate change | deg/s² |
| $v_{\min},v_{\max}$ | `perf.v_min,v_max` = −18, 18 | speed envelope | m/s |
| $\Delta t$ | `dt` | step | s |

Primes denote the next step. **No wind**, so track equals heading and a heading command
drives $\psi$ directly.

## 1. Speed — clamp to the envelope

$$ v' = \mathrm{clip}(v_c,\; v_{\min},\; v_{\max}) $$

Applied directly, with no acceleration ramp (a deliberate Phase-1 simplification, logged in
`vault/phase-1-plan.md` — add `ax` only when an experiment needs it). This is Check 3:
$v_c = 30 \Rightarrow v' = 18$.

## 2. Heading error — signed, shortest way round

$$ e = \big((\psi_c - \psi + 180)\bmod 360\big) - 180 \;\in\; (-180,\,180] $$

so a turn is always taken the short way, and the sign of $e$ is the turn direction.

## 3. Turn rate — the limiter (the heart of it)

Two limits act in series. First, the **desired** turn rate is the heading error itself,
capped at $\omega_{\max}$ — a proportional controller (gain 1, deg error → deg/s) that turns
at the maximum while far off and eases in near the target:

$$ \omega_{\text{des}} = \mathrm{clip}(e,\; -\omega_{\max},\; \omega_{\max}) $$

Second, the turn rate cannot *jump*: its change this step is bounded by $\alpha_{\max}\Delta t$
(this is why $\omega$ is state — the bound is relative to the previous $\omega$):

$$ \Delta\omega = \mathrm{clip}\big(\omega_{\text{des}} - \omega,\; -\alpha_{\max}\Delta t,\; \alpha_{\max}\Delta t\big) $$
$$ \omega' = \mathrm{clip}(\omega + \Delta\omega,\; -\omega_{\max},\; \omega_{\max}) $$

So from straight flight a hard 90° command ramps $\omega$ up at $\alpha_{\max}=10$ deg/s²,
reaching $\omega_{\max}=15$ deg/s after 1.5 s, holds, then eases out. This is Check 2:
at every step $|\omega'|\le\omega_{\max}$ and $|\omega'-\omega|\le\alpha_{\max}\Delta t$.

## 4. Heading — integrate, or snap when within a step

If the remaining error is larger than what this step would turn, integrate; otherwise the
target is reachable this step, so snap exactly onto it (avoids overshoot/chatter):

$$
\psi' =
\begin{cases}
(\psi + \Delta t\,\omega')\bmod 360, & |e| > |\Delta t\,\omega'| \\[2pt]
\psi_c, & \text{otherwise}
\end{cases}
$$

## 5. Position — great-circle forward step

Move distance $d = v'\Delta t$ along the (updated) track, using our own geodesy
(`opencdarr.geo.forward`, ADR 0003 — no BlueSky at runtime). With local WGS84 radius $R$ and
angular distance $\delta = d/R$:

$$ d = v'\,\Delta t \quad[\text{m}], \qquad \delta = d/R(\varphi) $$
$$ \varphi' = \arcsin\!\big(\sin\varphi\cos\delta + \cos\varphi\sin\delta\cos\psi'\big) $$
$$ \lambda' = \lambda + \operatorname{atan2}\!\big(\sin\psi'\sin\delta\cos\varphi,\; \cos\delta - \sin\varphi\sin\varphi'\big) $$

This mirrors BlueSky's `qdrpos` (same $R(\varphi)$), so the two agree to floating-point
precision — the guarded anchor test checks it. This is Check 1: $v=10$, straight, $10$ s
$\Rightarrow d \approx 100$ m, $\psi$ unchanged, $\omega \equiv 0$.

## Result

$$ \text{state}' = \big(\text{id},\; \varphi',\; \lambda',\; \psi',\; v',\; \omega'\big) $$

## Order of operations (matters)

speed clamp → heading error → turn-rate limiter → heading integrate/snap → **then** position
along the *updated* track. This matches BlueSky (heading updated before ground speed and
position). Everything is a pure function of the inputs; nothing reads or writes global state.
