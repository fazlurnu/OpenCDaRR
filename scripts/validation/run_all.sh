#!/usr/bin/env bash
# The MC-vs-IPS validation campaign, one part at a time.
#
#   ./scripts/validation/run_all.sh 100                  # 100 workers, default budgets
#   ./scripts/validation/run_all.sh 100 --mc-encounters 200000 --particles 4000
#   ./scripts/validation/run_all.sh 100 --no-cache       # time a campaign that is already cached
#
# The first argument is the worker count. Anything after it is passed through to every part
# (see `python scripts/validation/pairwise.py --help`).
#
# Run it from the repository root, so the parts import `opencdarr` from the working tree. Every
# part caches its conditions under `.opencdarr_cache/` at the repo root, so an interrupted
# campaign resumes where it stopped and the notebook reads the results without simulating again.
#
# Each part times its own conditions and writes the times beside its rows; the times below are the
# wall clock of the whole part, which also holds the Python start-up and the write.
set -euo pipefail

JOBS="${1:?usage: run_all.sh <jobs> [extra args passed to each part]}"
shift

export PYTHONPATH="${PYTHONPATH:-.}"
STARTED=$(date +%s)

for PART in pairwise ring random_traffic; do
  PART_STARTED=$(date +%s)
  echo "==============================================================================="
  echo "== ${PART}  —  started $(date -u '+%Y-%m-%dT%H:%M:%SZ')"
  echo "==============================================================================="
  python "scripts/validation/${PART}.py" --jobs "${JOBS}" "$@"
  echo "== ${PART} finished $(date -u '+%Y-%m-%dT%H:%M:%SZ') after $(( ($(date +%s) - PART_STARTED) / 60 )) min"
  echo
done

echo "==============================================================================="
echo "== campaign complete in $(( ($(date +%s) - STARTED) / 60 )) min"
echo "== results: results/validation/{pairwise,ring,random_traffic}.json"
echo '==           each file carries a timing block: MC and IPS seconds per condition'
echo "== cache:   .opencdarr_cache/  (the notebook reads this, it does not re-simulate)"
echo "==============================================================================="
