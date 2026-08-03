# Derivation — DH-ORCA (Dual-Horizon ORCA) resolution (2D, directed)

Dual-Horizon ORCA: ORCA with a **second, conditional** velocity constraint carried on a longer time
horizon, so that shallow-angle pairs cross each other's paths instead of settling onto parallel
tracks. Re-derived from Alligier, Gianazza, Durand & Olive, *Dual-Horizon Reciprocal Collision
Avoidance for Aircraft and Unmanned Aerial Systems*, **Journal of Intelligent and Robotic Systems
107(1), 2022** ([`vault/papers/dh-orca.pdf`](../papers/dh-orca.pdf)) — §§4–6. Line numbers below are
the paper's own margin numbers.

The closed-form half-plane construction is **not** in that paper: it describes $\vec c_\tau$
geometrically (§4.1, ll. 326–334) and inherits the algebra from van den Berg, Guy, Lin & Manocha,
*Reciprocal n-Body Collision Avoidance* (ISRR 2011), §§5–6. Both layers are re-derived here in our
ENU (East, North) frame.

- Implemented by: [`opencdarr/cr/dh_orca.py`](../../opencdarr/cr/dh_orca.py) (`DHORCA`), on the
  shared machinery in [`opencdarr/cr/orca.py`](../../opencdarr/cr/orca.py) (`_half_plane`,
  `_solve`, `_solve_dense`)
- Recovery: [`opencdarr/crr/orca.py`](../../opencdarr/crr/orca.py) (`ORCARevert`) — see §5 below,
  it is the `Cross` predicate
- Validated by: [`tests/test_dh_orca.py`](../../tests/test_dh_orca.py),
  [`tests/test_orca.py`](../../tests/test_orca.py)
- Measured in: [`brouillon/orca_vs_mvp_vo.ipynb`](../../brouillon/orca_vs_mvp_vo.ipynb),
  [`brouillon/parallel_track_lock.ipynb`](../../brouillon/parallel_track_lock.ipynb)
- Contrast: [[mvp-resolution]] (potential field, sums), and the union-of-cones VO in
  `opencdarr/cr/vo.py`. A standalone `orca-resolution` note is still owed; §§1–3 here cover the
  ORCA machinery DH-ORCA reuses.

## Symbols

| symbol | paper | meaning | unit |
|--------|-------|---------|------|
| $\mathbf x$ | $\vec{AB}$ | relative position, intr − own (E,N) | m |
| $\mathbf v$ | $\vec{v_r}$ | relative velocity, **own − intr** | m/s |
| $\mathbf v_o$ | $\vec{v_A}$ | ownship velocity | m/s |
| $\mathbf v^\text{pref}$ | $\vec{v_A^\text{pref}}$ | ownship's goal velocity (`own.desired`) | m/s |
| $d$ | $d$ | separation standard (`rpz`) | m |
| $R$ | — | constraint radius $=\texttt{rpz}\times\texttt{margin}$ | m |
| $\tau_c$ | $\tau_\text{conflict}$ | short horizon, always enforced | s |
| $\tau_\times$ | $\tau_\text{cross}$ | long horizon, gates the optional constraint | s |
| $\mathbf u$ | $\vec{c_\tau}$ | minimal relative-velocity change | m/s |
| $\delta^-_\tau$ | $\delta^-_\tau$ | reciprocal velocity obstacle at horizon $\tau$ | — |

**Sign convention.** The paper takes $\vec{v_r} = \vec{v_A} - \vec{v_B}$ (§4.1, l. 371) while
`relative_enu` returns `intr − own` for both position and velocity. So the code negates the velocity
and keeps the position: `xe, xn = rel.rx, rel.ry` but `ve, vn = -rel.vx, -rel.vy`
([`orca.py:102–104`](../../opencdarr/cr/orca.py)). This is the one place a sign error would be
silent, hence the explicit comment there.

## 1. The reciprocal velocity obstacle $\delta^-_\tau$

A conflict occurs within $\tau$ iff $\exists t \in [0,\tau] : \lVert \mathbf x - \mathbf v t\rVert <
d$ (§4.1, ll. 300–313). The set of $\mathbf v$ satisfying this is the cone from the origin tangent
to the disc $\mathcal C(\mathbf x, R)$, **truncated** by the disc of radius $R/\tau$ centred at
$\mathbf x/\tau$ — the paper's "blunt cone" (Figure 1.b). The truncation is exactly the paper's
restatement $\lVert \vec{B'A} + \lambda \vec{v_r}\rVert < d/\tau$ (l. 309).

## 2. The minimal change $\mathbf u$ and the half-plane

$\mathbf u$ is the vector from $\mathbf v$ to the nearest point on $\partial\delta^-_\tau$ — the
paper's "connecting the tip of $\vec{v_r}$ with its orthogonal projection on the closest boundary"
(§4.1, ll. 326–330). Three cases, all in
[`_half_plane`](../../opencdarr/cr/orca.py) (`orca.py:82–146`):

**(a) nearest point on the truncation cap** — `orca.py:119–126`. With $\mathbf w = \mathbf v -
\mathbf x/\tau$, the cap is nearest iff $\mathbf w\cdot\mathbf x < 0$ and $(\mathbf w\cdot\mathbf
x)^2 > R^2\lVert\mathbf w\rVert^2$:

$$ \hat{\mathbf w} = \frac{\mathbf w}{\lVert\mathbf w\rVert}, \qquad
   \mathbf u = \left(\frac{R}{\tau} - \lVert\mathbf w\rVert\right)\hat{\mathbf w}, \qquad
   \mathbf n = (\hat w_N,\, -\hat w_E) $$

**(b) nearest point on a cone leg** — `orca.py:127–137`. With $\ell = \sqrt{\lVert\mathbf
x\rVert^2 - R^2}$, the left leg is taken when $\det(\mathbf x, \mathbf w) > 0$:

$$ \mathbf n_\text{left} = \frac{(x_E \ell - x_N R,\; x_E R + x_N \ell)}{\lVert\mathbf x\rVert^2},
\qquad
   \mathbf n_\text{right} = -\frac{(x_E \ell + x_N R,\; -x_E R + x_N \ell)}{\lVert\mathbf x\rVert^2}
$$
$$ \mathbf u = (\mathbf v\cdot\mathbf n)\,\mathbf n - \mathbf v $$

**(c) already inside the zone** ($\lVert\mathbf x\rVert \le R$) — `orca.py:138–145`. There is no
cone; the horizon collapses to `t_collision` and the constraint becomes "leave the disc within that
time". The paper uses the simulation step $\delta t$ here; ours is a named parameter defaulting to
the 1 s CDR cadence.

**The reciprocal split.** The paper's semi-planes (§4.1, ll. 345–351) are

$$ P^\tau_{B\to A} := \left\{\mathbf v'_A \;\middle|\; \left(\mathbf v'_A - \left(\mathbf v_A +
\tfrac{\vec{c_\tau}}{2}\right)\right)\cdot \vec{n_\tau} \ge 0\right\}, \qquad
P^\tau_{A\to B} := \left\{\mathbf v'_B \;\middle|\; \left(\mathbf v'_B - \left(\mathbf v_B -
\tfrac{\vec{c_\tau}}{2}\right)\right)\cdot \vec{n_\tau} \le 0\right\} $$

so each aircraft takes **half** of $\mathbf u$. That is the single line
`return _HalfPlane(vo_e + 0.5 * ue, vo_n + 0.5 * un, dir_e, dir_n)` (`orca.py:146`). The guarantee
it buys (ll. 358–360) — that both halves compose into a relative velocity outside $\delta^-_\tau$ —
is what `test_orca.py::test_both_halves_together_land_exactly_on_rpz` checks over 144 geometries:
the pair's closest approach lands on $d$ exactly, neither short nor over.

Stored as point + direction rather than normal + offset, because the linear program walks *along*
the line; feasibility is then $\det(\mathbf n, \mathbf p - \mathbf v) \le 0$
(`_HalfPlane.violated_by`, `orca.py:77–79`).

## 3. The untruncated constraint $P^{\tau=+\infty}$

CS-ORCA (§5.2, ll. 521–538) builds its semi-plane by projecting onto "the closest **side** of the
full cone of extremity A, instead of the closest boundary of the blunt cone" — i.e. the cone with no
truncation cap at all, which the paper denotes $P^{\tau=+\infty}_{B\to A}$ and $\delta^-_{\tau=+\infty}$
(l. 532). This is the constraint DH-ORCA adds.

In code it is `_half_plane(..., tau=None)` (`orca.py:109–113`): $\mathbf w = \mathbf v$ (the cone's
apex is the origin of relative-velocity space) and the cap branch is disabled outright, so the
projection always lands on a leg.

**Why `None` and not a large $\tau$.** As $\tau\to\infty$ the cap radius $R/\tau\to 0$ and its centre
$\mathbf x/\tau\to\mathbf 0$, so for $\mathbf v$ *inside* the cone the limit is the leg projection
and a big number would do. But for $\mathbf v$ **outside** it the cap test $\mathbf w\cdot\mathbf x <
0$ can still fire, and case (a) then returns $\mathbf u = -\lVert\mathbf v\rVert\hat{\mathbf v}$ —
"project onto the apex", which is not a side projection and not what §5.2 specifies. Since ORCA
enforces its constraints whether or not a conflict is currently detected (§4.1, ll. 364–369), that
branch is reached routinely. The untruncated cone is a genuinely different set, not a numerical
limit, so it gets its own branch.

## 4. `Cross(A,B)` — the gate

§6.1, ll. 636–644:

$$ \mathrm{Cross}(A,B) := \exists\, t\in[0,\tau_\times] \;:\;
\left\lVert \vec{BA} + \left(\vec{v_A^\text{pref}} - \vec{v_B}\right)t \right\rVert < d $$

Implemented as [`_crosses`](../../opencdarr/cr/dh_orca.py) (`dh_orca.py:63–84`). The existential is
evaluated in closed form rather than sampled: $\lVert \mathbf r + \boldsymbol\delta t\rVert$ is
convex in $t$, so its minimum over $[0,\tau_\times]$ sits at the **clamped** unconstrained CPA time

$$ t^\star = \min\!\left(\tau_\times,\; \max\!\left(0,\; -\frac{\mathbf r\cdot\boldsymbol\delta}
{\lVert\boldsymbol\delta\rVert^2}\right)\right), \qquad
\mathrm{Cross} \iff \lVert \mathbf r + \boldsymbol\delta\, t^\star\rVert < d $$

with $\mathbf r = $ `(rel.rx, rel.ry)` and $\boldsymbol\delta = \mathbf v_\text{intr} -
\mathbf v^\text{pref}$. Same clamped-CPA shape as [[ftr-recovery]]'s `_clears`, with a finite
horizon instead of an unbounded one.

Three interpretation points:

1. **The intruder's *observed* velocity, not its intent.** The paper is explicit (ll. 645–658):
   $\vec{v_B^\text{pref}}$ "would more accurately model a conflict between the intended paths of both
   aircraft, but would require aircraft B to share its intended velocity", so $\vec{v_B}$ is used as
   a proxy. Following it keeps DH-ORCA deployable with no intent broadcast — which is the property
   that distinguishes it from [[ftr-recovery]]'s second criterion.
2. **Measured against $d$, not $R$.** The paper's $d$ is the separation standard. `margin` is *our*
   addition (BlueSky's `asas_marh`, see [[mvp-resolution]]) and buffers what the resolver clears
   **to**; letting it also move the trigger would conflate two knobs — which encounters get the
   crossing constraint, and how hard it pushes. So `_crosses` takes `rpz`, while `_half_plane` takes
   `rpz_eff`.
3. **Asymmetry is a mid-manoeuvre property.** $\mathrm{Cross}(A,B) \ne \mathrm{Cross}(B,A)$ in
   general (§6.3), and only the aircraft whose own goal crosses pays for the extra constraint. But
   at $t=0$ both aircraft are still on their nominal, so $\vec{v_B} = \vec{v_B^\text{pref}}$, both
   questions reduce to the same relative motion, and they **always** agree. The asymmetry appears
   only once someone has deviated. Both halves are pinned:
   `test_cross_is_symmetric_only_while_both_are_on_their_nominal` and
   `test_cross_becomes_asymmetric_once_an_aircraft_has_deviated`.

## 5. The DH-ORCA constraint set

§6.1, ll. 664–674:

$$ P^{\text{DH-ORCA}\,\tau_\times}_{B\to A} := \begin{cases}
P^{\tau=\tau_c}_{B\to A} \cap P^{\tau=+\infty}_{B\to A} & \text{if } \mathrm{Cross}(A,B) \\[2pt]
P^{\tau=\tau_c}_{B\to A} & \text{otherwise}
\end{cases} $$

and "DH-ORCA is just Algorithm 1 with $P_{i\to j} = P^{\text{DH-ORCA}\,\tau_\text{cross}}_{i\to j}$"
(ll. 672–674). The loop in `DHORCA.resolve` (`dh_orca.py:149–160`) is that, verbatim: one
unconditional `_half_plane(..., self.t_conflict, ...)` per intruder, plus a second
`_half_plane(..., None, ...)` appended when `_crosses` fires. **Intersected, never substituted** —
which is why separation is not traded for the crossing, checked by
`test_both_halves_still_land_on_rpz_when_crossing`.

**The extra guard** `relative_enu(own, intr).dist > rpz_eff` (`dh_orca.py:157`) is ours, not the
paper's. Inside the zone `_half_plane` takes branch (c) regardless of `tau`, so the second call
would return a bit-identical duplicate of the first — harmless in the LP but pointless work, and
semantically "help them cross later" is not what an aircraft in loss of separation needs.

**Recovery.** Read the `Cross` predicate again: *would flying my nominal breach $d$ within
$\tau_\times$?* That is exactly what `ORCARevert` asks. So DH-ORCA's native revert rule **is**
`ORCARevert(t_horizon=t_cross)` — the ORCA revert at the long horizon — and no `DHORCARevert` class
exists because it would be a line-for-line duplicate. The identity
$\mathrm{Cross}(A,B) \equiv \neg\,$`ORCARevert(t_cross).should_resume(...)` is pinned over a grid by
`test_cross_is_exactly_the_revert_test`, so the resolver and the recovery cannot drift apart. It is
exact at `margin = 1.0`; above it, point 2 of §4 makes them differ slightly.

## 6. Solving for the new velocity

Algorithm 1 (ll. 419–445) takes $C_i = \bigcap_{k\ne i} P_{k\to i}$, intersects it with the
admissible arc $A_i$, and picks the velocity in $S_i = C_i\cap A_i$ closest to
$\vec{v_i^\text{pref}}$. Ours minimises $\lVert\mathbf v - \mathbf v^\text{pref}\rVert$ over
$\bigcap_k P_k \cap \{\lVert\mathbf v\rVert \le v_\text{max}\}$ by the incremental 2D linear program
(`_solve` / `_solve_1d`, `orca.py:150–212`): satisfied constraints are skipped, and the first
violated one is tight at the optimum, so the problem drops to one dimension along that line.

If $S_i$ is empty the paper relaxes every semi-plane equally until it is not (ll. 409–417); that is
`_solve_dense` (`orca.py:243–288`), which minimises the worst violation.

Two deviations worth stating plainly:

- **Constraint order is not randomised.** RVO2 shuffles for an expected-linear bound; we keep
  intruder order. When the problem is feasible this cannot change the answer — projection onto a
  closed convex set is unique — only the work. In the **infeasible** case `_solve_dense` *is*
  order-dependent, as RVO2's `linearProgram3` is. Determinism matters more here than the constant
  factor: an IPS clone must reproduce its parent exactly ([[architecture-dataflow]]), and the
  resolver interface carries no RNG to shuffle with.
- **A speed disc, not the paper's constant-speed arc.** See below.

## Notes — where this departs from the paper, and why

- **Constant speed is not imposed.** §4.2 (ll. 396–404) constrains $\lVert\vec{v'_A}\rVert =
  \lVert\vec{v_A}\rVert$ and limits the new velocity to a turn-rate-limited **arc** (3°/s over a 5 s
  step, ±15°). We use RVO2's speed **disc** $\lVert\mathbf v\rVert \le v_\text{max}$ instead. Three
  reasons: the M600 is a multirotor with a real speed channel, not a fixed-wing at cruise; the
  package's existing division of labour puts feasibility in the kinematics, not the resolver
  ([[mvp-resolution]]: "speed is **not** capped here — `step_dynamics` clamps it to the envelope, so
  `resolve` stays pure geometry"); and MVP/VO in the same experiment emit unconstrained velocity
  commands, so constraining only DH-ORCA would confound the comparison.
  **This makes the observed parallel lock a stronger result, not a weaker one**: the paper's
  pathology is driven by aircraft that *cannot* slow down, and we reproduce it with the speed
  channel free ([`parallel_track_lock.ipynb`](../../brouillon/parallel_track_lock.ipynb) — plain
  ORCA holds both tracks at exactly 1.00° for ~1000 s).
- **`v_max` is a constructor argument.** `ConflictResolver.resolve` carries no
  `Performance`, so the resolver cannot see the airframe envelope. `None` falls back to the
  preferred speed, which makes it turn-only — safe, but not the algorithm; the experiments pass
  `M600.v_max`.
- **The horizons are untuned, and do not transfer.** §7.5 grid-searches them and Table 4 reports
  $\tau_\text{conflict} = 52$ s, $\tau_\text{cross} = 372$ s for $d = 5$ NM at 230 kt.
  Non-dimensionalising by $d/V \approx 78$ s gives $0.67\,d/V$ and $4.8\,d/V$; our $\texttt{rpz}/V =
  50/10.29 \approx 4.9$ s would scale them to ~3 s and ~24 s, which is meaningless here because our
  encounter timescale is set by the experiment (`t_lookahead` 60 s, `tlos` 90 s), not by
  $\texttt{rpz}/V$. So the notebooks use $\tau_c = 60$ s — deliberately **equal to the `ORCA` being
  compared against**, so the two differ only by the crossing constraint — and $\tau_\times = 180$ s.
  Neither is tuned; this is the largest unquantified choice in every DH-ORCA number reported.
- **2D only, no wind, no vertical.** As with [[mvp-resolution]]; the paper is also horizontal-only
  (§3.2) and points at Snape & Manocha (2010) for a 3D lift.
- **`t_cross > t_conflict` is enforced** in `__init__`: below it the gate can only fire where the
  short constraint already binds, and DH-ORCA degenerates to ORCA with extra work. The paper only
  tries pairs with $\tau_\text{conflict} < \tau_\text{cross}$ (Table 3), so this makes its grid
  constraint a type error rather than a silent no-op.
- **Not implemented: CS-ORCA itself.** $P^{\text{CSORCA},\tau}$ (§5.2, ll. 536–538) *swaps*
  $P^{\tau=+\infty}$ in when $\vec{v_r}\in\delta^-_\tau$, where DH-ORCA *adds* it when
  $\mathrm{Cross}$. The machinery is all here — `_half_plane(..., tau=None)` plus a different gate —
  so it is a small addition if the three-way comparison the paper runs is ever wanted.
