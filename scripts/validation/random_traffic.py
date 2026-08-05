"""Validation campaign, part: random_traffic. Run it on its own; `run_all.sh` runs the three in order."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from campaign import parser, run_part  # noqa: E402  - sibling module, path fixed up above

if __name__ == "__main__":
    run_part("random_traffic", parser("random_traffic").parse_args())
