"""Part 2 — the crossing ring, swept over fleet size.

``crossing_ring`` rather than ``swap_ring``: its routes are diameters at every ``n``, so stepping
the fleet size changes the size and nothing else. A ``swap_ring`` sweep would move the geometry at
every odd ``n`` as well, and the trend would have two causes.

    PYTHONPATH=. python scripts/validation/ring.py --jobs -1
"""
from __future__ import annotations

from campaign import Cell, over_rungs, parser, run_part  # noqa: E402  (sibling)

from opencdarr.scenario import CrossingRing  # noqa: E402

SIZES = [3, 4, 6, 8]
RADIUS = 900.0


def main() -> None:
    args = parser("ring").parse_args()
    cells = over_rungs(
        [Cell(label={"n": n}, scenario=CrossingRing(n=n, radius=RADIUS)) for n in SIZES],
        args.rungs,
    )
    print(f"ring — {len(cells)} fleet sizes on a {RADIUS:.0f} m ring")
    print(f"wrote {run_part('ring', cells, args)}")


if __name__ == "__main__":
    main()
