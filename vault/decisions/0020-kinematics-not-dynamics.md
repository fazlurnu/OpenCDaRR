# ADR 0020 — The models are kinematic, not dynamic; the package is renamed to say so

- Status: accepted
- Date: 2026-07-30
- Deciders: Fazlur Rahman

## Context

`opencdarr/dynamics/` never modelled dynamics. Read what the two implementations actually compute:

| Model | What it integrates | What it never touches |
|---|---|---|
| `Multirotor` | velocity toward a setpoint under an isotropic acceleration cap; yaw stepped independently | mass, thrust, drag, energy |
| `FixedWing` | heading via the coordinated-turn relation ψ̇ = g·tan(φ)/V, bank slewed at a finite roll rate and clipped by the stall/load envelope | mass, thrust, drag, energy |

There is no force anywhere in the package. `Performance` is an *envelope* — speed bounds,
acceleration caps, bank and roll-rate limits — not a force model: nothing computes lift from an
angle of attack or acceleration from a thrust-minus-drag balance. Both models take a commanded
motion and produce the closest achievable motion subject to rate limits. That is the textbook
definition of a **kinematic** model.

The literature reserves *dynamics* for force-and-mass models — BADA's total-energy formulation, or
any 6-DoF model — and this repo already follows that convention elsewhere:
`docs/fixedwing-vs-bluesky.md` calls the comparison target "BlueSky's fixed-wing **kinematics**",
and `state.py` has always described `AircraftState` as "one aircraft's 2D horizontal **point-mass
kinematics**". The package name was the outlier.

`docs/design-philosophy.md` #6 is **"Name it like the paper — a physicist should recognize the
literature."** A physicist reading `Dynamics` expects forces and finds none. That principle is what
makes this a correctness fix rather than a preference.

**This is not the rename [[0019-channel-extension-by-link-gates]] rejected.** That one proposed
swapping `channel`/`Comm` for `broker`/`topic`/`subscriber` — trading correct vocabulary for
different-but-equally-correct vocabulary, which is exactly the churn `docs/lesson-learnt.md` warns
about. Here the existing term is *wrong*. The distinction is the whole justification, and it is the
test any future rename proposal should have to pass.

## Decision

### 1. `Dynamics` becomes `Kinematics`

| Old | New |
|---|---|
| `opencdarr/dynamics/` | `opencdarr/kinematics/` |
| `Dynamics` (ABC) | `Kinematics` |
| `dynamics=` parameter, `Agent.dynamics`, `Methods.dynamics` | `kinematics=` |
| `own_dynamics=` / `intr_dynamics=` | `own_kinematics=` / `intr_kinematics=` |
| `_DEFAULT_DYNAMICS`, `FleetEnv.dyns`, locals `dyn` / `dyns` / `DYN` | `_DEFAULT_KINEMATICS`, `FleetEnv.kinematics`, spelled out |
| `"dynamics"` in `experiment._COMPONENTS` | `"kinematics"` |

### 2. The old `kinematics.py` becomes `relative.py`

The name was already taken by the relative-geometry helpers, so they move first:
`opencdarr/kinematics.py` → `opencdarr/relative.py` (and `tests/test_kinematics_wind.py` →
`tests/test_wind_triangle.py`). All nine symbols stay together — `Relative`, `relative_enu`,
`segment_min_range`, `velocity_enu` and the five wind-triangle helpers. Lifting the wind triangle
into `wind.py` would be a refactor, not a rename, and is deliberately **not** done here.

### 3. Two commits, not one

`kinematics.py` and `kinematics/` must never coexist: Python's `FileFinder` checks directories
before file loaders, so the package would silently win and the module would become unreachable with
**no error**. Vacating the name is therefore its own commit, verified green before the package
takes it.

A related trap, worth recording because it is silent: `git mv opencdarr/dynamics
opencdarr/kinematics` moves only *tracked* files, leaving `opencdarr/dynamics/__pycache__/` behind.
A directory with no `__init__.py` is a valid PEP-420 namespace package, so `import
opencdarr.dynamics` keeps **succeeding**. The old directory must be removed outright.

### 4. History is not rewritten

Renamed only where the text describes the **present**. Left alone, because they name things that no
longer exist under any name — rewriting them would make the record false:

- `step_dynamics` — the v0.1 extracted pure function that `docs/` still narrates.
- `DubinsDynamics`, `HolonomicDynamics` — deleted in Phase 4c / superseded by `Multirotor`.
- The ADR filenames [[0002-analytical-validation-of-dynamics]],
  [[0007-dynamics-as-pluggable-interface]], [[0009-holonomic-dynamics]],
  [[0010-dynamics-subpackage-and-odometry-state]], `step-dynamics-m600.md`, and the phase-4 notes.
  ADRs are dated and append-only (`vault/decisions/README.md`); renaming them would also break 47+
  inbound wikilinks, several of them from code docstrings.
- The body of `docs/design_brief.md`, which states of itself that it is a historical record. Only
  its banner — added later, describing the present — is updated.
- `parallel.py`'s "joblib's dynamic dispatch", a different word.

`vault/architecture-dataflow.md` **is** updated: it is explicitly the current picture.

### 5. No compatibility shim

No `opencdarr/dynamics.py` re-exporting with a `DeprecationWarning`. It would be a permanent
residual that defeats the "is the rename complete?" check; the version is `0.0.0` with no external
consumers; and a `dynamics.py` beside a `kinematics/` package reintroduces exactly the
module-vs-package shadowing §3 exists to avoid.

## Alternatives rejected

- **`KinematicModel` / `MotionModel` for the ABC.** Both read better as English for a class, and
  `MotionModel` would pair with `MotionCommand`. Rejected because the one-word swap keeps the diff
  mechanical and reads the same way `Dynamics` did at every call site; `MotionModel` also drops the
  precise term the change exists to introduce.
- **Splitting `relative.py`'s wind triangle into `wind.py`.** Only five call sites, and each module
  would then do one thing. Deferred anyway: mixing a refactor into a rename makes both harder to
  review, and #17 does not ask for it yet.
- **Renaming the four ADR files and the ~520 vault mentions.** Maximally consistent, and wrong: it
  edits dated records into saying something that was not decided at the time, and breaks the
  wikilink graph.
- **Leaving it.** The cheapest option, and the one #6 forbids: a reader who knows the literature is
  actively misled about what the models contain.

## Consequences

- 91 files changed, 13 renamed. `pytest` stays at 426 green; `mypy opencdarr` at its 13 known
  errors; `ruff` slightly *better* than before (101 vs 103).
- **"kinematics" is four characters longer than "dynamics"**, which pushed 35 lines past the
  99-column limit — 22 of them in `opencdarr/`, which had only 8 lint findings in total. Those
  paragraphs are rewrapped and four call sites reformatted. A cosmetic rename is not free at a
  fixed column limit; worth remembering before the next one.
- `run_encounter`'s per-side list is now `per_side`. The mechanical rename would have named it
  `kinematics`, shadowing the parameter of that name and silently turning a `Kinematics` into a
  `list[Kinematics]`. `mypy` did not catch it — the repo-wide `mypy` invocation currently aborts on
  a duplicate-module error in `tests/`, so it checks nothing.
- A reader following a wikilink from a code docstring into ADRs 0002 / 0007 / 0009 / 0010 lands on
  a note that still says `Dynamics`. That is the intended cost of §4: the notes record what was
  decided then. This ADR is the bridge.
- **Obligation:** `opencdarr.github.io` still says "dynamics" throughout — 34 pages plus the
  structural paths `docs/build-your-own/dynamics.md` and `docs/modules/dynamics/*`. Renaming those
  changes **published URLs**, so it needs its own pass and probably redirects. Until it lands, the
  handbook and the code disagree.

## Relations

- Renames the interface introduced in [[0007-dynamics-as-pluggable-interface]] and the package
  split of [[0010-dynamics-subpackage-and-odometry-state]]; the models themselves are unchanged
  ([[0012-multirotor-and-yaw-carrying-state]], [[0013-fixedwing-coordinated-turn]]).
- Sets the bar [[0019-channel-extension-by-link-gates]] applied when it rejected a rename: a rename
  earns its churn only when the existing name is *wrong*, not merely improvable.
- `docs/design-philosophy.md` #6 (name it like the paper), #17 (no unrequested generality — hence
  no shim, no refactor bundled in).
