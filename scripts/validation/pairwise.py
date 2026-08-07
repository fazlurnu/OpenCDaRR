"""Part 1 — the crossing angle.

The pairwise encounter is where the two estimators can be compared most cleanly: two aircraft, one
geometry parameter, and a Monte-Carlo anchor that is still affordable. Every other part is checked
against the agreement established here.

    PYTHONPATH=. python scripts/validation/pairwise.py --jobs -1
"""
from __future__ import annotations

from campaign import Cell, over_rungs, parser, run_part  # noqa: E402  (sibling)

from opencdarr.scenario import PairwiseEncounter  # noqa: E402

ANGLES = [45.0, 90.0, 135.0, 180.0]


def main() -> None:
    args = parser("pairwise").parse_args()
    cells = over_rungs([Cell(label={"dpsi": a}, scenario=PairwiseEncounter(dpsi=a, dcpa=0.0))
             for a in ANGLES], args.rungs)
    print(f"pairwise — {len(cells)} crossing angles")
    print(f"wrote {run_part('pairwise', cells, args)}")


if __name__ == "__main__":
    main()
