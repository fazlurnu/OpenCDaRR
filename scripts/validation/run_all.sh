#!/usr/bin/env bash
# The whole validation campaign, in order. Each part caches per condition, so re-running after a
# crash pays only for the cells that were not finished.
#
#     scripts/validation/run_all.sh                 # production sizes
#     scripts/validation/run_all.sh --mc 2000 --particles 200 --reps 4    # a quick smoke run
set -euo pipefail
cd "$(dirname "$0")/../.."
export PYTHONPATH=.
export OMP_NUM_THREADS=1        # one thread per worker; the hot path makes no BLAS calls

for part in pairwise ring random_traffic; do
  echo "=== $part ==="
  python "scripts/validation/$part.py" "$@"
done
echo "=== done — rows in scripts/validation/out/ ==="
