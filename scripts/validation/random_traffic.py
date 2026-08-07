"""Part 3 — random traffic, swept over density.

The only drawn geometry of the three, and the only one with a measurement area: aircraft are
released on the simulation ring and counted only inside the smaller experimental disc, so the
release itself is never measured.

    PYTHONPATH=. python scripts/validation/random_traffic.py --jobs -1
"""
from __future__ import annotations

from campaign import Cell, over_rungs, parser, run_part  # noqa: E402  (sibling)

from opencdarr.scenario import RandomTraffic  # noqa: E402

DENSITIES = [2.0, 5.0, 10.0]
RADIUS = 900.0


def main() -> None:
    args = parser("random_traffic").parse_args()
    cells = over_rungs(
        [Cell(label={"density": d}, scenario=RandomTraffic(density=d, radius=RADIUS))
         for d in DENSITIES],
        args.rungs,
    )
    print(f"random_traffic — {len(cells)} densities over a {RADIUS:.0f} m disc")
    print(f"wrote {run_part('random_traffic', cells, args)}")


if __name__ == "__main__":
    main()
