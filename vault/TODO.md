# TODO

Near-term intents first, then the release gates. Items 1–4 had no status marker and read as
pending long after they shipped; they carry checkboxes now.

1. - [x] Finishing the IPS on comm and nav uncertainty — done. The unified look-ahead conflict
     coordinate covers both families at once (ADR 0017, [[important-unified-coordinate]]), and
     `scripts/cns_sweep.sh` sweeps them; results in `scripts/cns_sweep_20260728_085447/` (14 cells)
     and the IPS-only replay in `scripts/ips_rerun_20260729_072950/`.
2. - [ ] Include more metrics on the log: total delta velocity, total extra flight time, total extra distance, total path deviation, total time resolving conflict.
     `AircraftState` already accumulates `flight_time` and `distance_flown`; the rest is unbuilt.
     This is what phase 9 is for.
3. - [ ] Make the user-facing interface very minimal. Things they only need is writing their own conf detect, conf reso, recovery (or combinations of them).
     Partly there — `examples/02_build_your_own_separation_manager.ipynb` walks exactly that path —
     but `opencdarr/__init__.py` still exports nothing, so every import is submodule-qualified.
4. - [x] Switch the a first run and how it works. People want to run things first and see how if they are curious — done. The handbook nav is Intro → Installation → How it works → A first run, and `examples/handbook/a_first_run.ipynb` is the notebook the README points at.
5. - [ ] **Post-freeze: scope `mypy` and `ruff` to match design-philosophy #12** ("core = rigor,
     scripts = speed"). `pyproject.toml` is currently stricter than the principle: it points mypy at
     `["opencdarr", "tests"]` and lets ruff walk the whole tree, so neither comes back green and the
     signal is buried. Measured 2026-07-29 against mypy 2.3.0 and ruff 0.16.0:
     - `ruff check` — 269 findings: 168 in `examples/` (ruff lints `.ipynb` by default), 86 in
       `scripts/`, **8 in `opencdarr/`** (6 × E501 at 100 vs the 99 limit, 2 × B027), 7 in `tests/`.
     - `mypy` — aborts on `Source file found twice under different module names` because `tests/`
       has no `__init__.py`. Past that: **11 errors in `opencdarr/`** (missing return annotations in
       `cns/noise_distributions.py`, unparameterised `ndarray` in `crr/probabilistic_ftr.py`,
       `replace()` kwargs in both dynamics modules, a re-export of `CnsStreams` from `fleet.py`)
       and 230 in `tests/`, which were never annotated.
     The shipping code is nearly clean; the config is what needs narrowing. Fix the 8 + 11 findings,
     add `tests/__init__.py` (or `explicit_package_bases`), and exclude `examples/` and `scripts/`
     from the strict gate — then the README can say all three commands are green.
6. V1.0 release:
	1. First level, someone can install the opencdarr package, import the library, run it on their jupyter notebook without error, and get the figures. This is only for the pairwise conflict
	2. Second level, someone can write their own conflict detection, resolution, and recovery code, run it, and get the results, this is all MC
	3. Third level, someone can write their own conflict detection, resolution, and recovery code, run it, and get the results, this is all IPS
7. V1.1 release:
	1. Someone can write their own dynamics, run it, and get the results
	2. People can do like v1.0, but on a multi-agent env, default has 3 env: the ring encounter, the waypoint in DH-ORCA, the circular airspace sector
8. V1.2 release:
	1. People can create their own environment using a GUI. Scaling will be an issue (think of aircraft vs drone, there should be a guard on flight time that drone can't fly too long)
9. V2 release:
	1. Split `Performance` into airframe-typed subtypes (`MultirotorPerformance` / `FixedWingPerformance`) so a mismatched envelope is unrepresentable, not just caught at runtime. Removes the "not-applicable field defaulted to 0.0" smell (M600 carrying phi_max=0, SMALL_FIXEDWING carrying yaw_rate_max=0). To keep `Dynamics.step(perf)` from an LSP-violating narrowed override, make `Dynamics` generic over its performance type (`Dynamics[P]`) and thread the type param through `Agent`. Supersedes the runtime `validate_performance` guard added in v1 (keep it as the non-typed-caller backstop).