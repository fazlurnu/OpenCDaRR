"""The top-level import surface — the first thing a new user touches.

``opencdarr/__init__.py`` re-exports the contribution surfaces, the reference implementations, the
runners and the estimators, so that writing your own detector / resolver / recovery and measuring
it does not require knowing the module layout ([[TODO]] #3, and the v1.0 release gates). These lock
the two properties that promise is made of: the names are actually there, and reaching them does
not drag in the optional extras.
"""

from __future__ import annotations

import subprocess
import sys

import opencdarr


def test_all_names_exist() -> None:
    """Every name in ``__all__`` resolves — no stale entries after a rename."""
    missing = [name for name in opencdarr.__all__ if not hasattr(opencdarr, name)]
    assert missing == []


def test_all_has_no_duplicates() -> None:
    """No name appears twice in ``__all__`` — a duplicate means a merge went wrong.

    Deliberately *not* asserting a sort order: ``RUF`` is not in this project's selected ruff rules
    (``pyproject.toml``), so ``__all__`` ordering is not a project convention, and re-implementing
    a linter's ordering rule in a test would be inventing one.
    """
    assert len(opencdarr.__all__) == len(set(opencdarr.__all__))


def test_the_contributor_path_imports() -> None:
    """The [[TODO]] #3 use case: write your own CD/CR/CRR/dynamics, run it, measure it.

    Spelled out as the imports a contributor actually writes, so a reshuffle that breaks the
    documented one-liners fails here rather than in someone's notebook.
    """
    from opencdarr import (  # noqa: F401 — the import *is* the assertion
        MVP,
        VO,
        Agent,
        AircraftState,
        ConflictDetector,
        ConflictResolver,
        Dynamics,
        MotionCommand,
        NavigationModel,
        NoiseDistribution,
        PastCPA,
        Performance,
        ProbabilisticFTR,
        RecoveryCriterion,
        StateBased,
        estimate_ipr,
        estimate_rare_prob,
        load_config,
        run_encounter,
        run_fleet,
    )


def test_importing_the_package_does_not_pull_the_optional_extras() -> None:
    """``import opencdarr`` stays numpy + pyyaml: no matplotlib, no joblib.

    Both are declared optional in ``pyproject.toml`` and imported lazily inside the functions that
    need them (``viz`` draws, ``parallel`` schedules), which is what keeps a plain install light. A
    top-level re-export that reached either eagerly would silently make them required — so this
    runs in a **fresh interpreter**, since this test session has almost certainly imported them
    already.
    """
    code = (
        "import sys, opencdarr; "
        "print('matplotlib' in sys.modules, 'joblib' in sys.modules)"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert proc.stdout.split() == ["False", "False"], proc.stdout


def test_plotting_is_reachable_without_matplotlib_installed_being_required_at_import() -> None:
    """``plot_pairwise`` is exported, but importing it must not import matplotlib."""
    assert callable(opencdarr.plot_pairwise)
    assert callable(opencdarr.extract_tracks)
