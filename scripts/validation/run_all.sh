#!/usr/bin/env bash
# The MC-vs-IPS validation campaign, one part at a time.
#
#   ./scripts/validation/run_all.sh 100                  # 100 workers, default budgets
#   ./scripts/validation/run_all.sh 100 --mc-encounters 200000 --particles 4000
#
# The first argument is the worker count. Anything after it is passed through to every part
# (see `python scripts/validation/pairwise.py --help`).
#
# Run it from the repository root, so the parts import `opencdarr` from the working tree. Every
# part caches its conditions under `.opencdarr_cache/` at the repo root, so an interrupted
# campaign resumes where it stopped and the notebook reads the results without simulating again.
set -euo pipefail

JOBS="${1:?usage: run_all.sh <jobs> [extra args passed to each part]}"
shift

export PYTHONPATH="${PYTHONPATH:-.}"
STARTED=$(date +%s)

for PART in pairwise ring random_traffic; do
  echo "==============================================================================="
  echo "== ${PART}  —  started $(date '+%Y-%m-%d %H:%M:%S')"
  echo "==============================================================================="
  python "scripts/validation/${PART}.py" --jobs "${JOBS}" "$@"
  echo "== ${PART} finished $(date '+%Y-%m-%d %H:%M:%S')"
  echo
done

echo "==============================================================================="
echo "== campaign complete in $(( ($(date +%s) - STARTED) / 60 )) min"
echo "== results: results/validation/{pairwise,ring,random_traffic}.json"
echo "== cache:   .opencdarr_cache/  (the notebook reads this, it does not re-simulate)"
echo "==============================================================================="
