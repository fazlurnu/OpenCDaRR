#!/usr/bin/env bash
# ips_rerun_sweep.sh — re-run only the IPS half of a previous cns_sweep, reusing its MC anchors.
#
# The Monte-Carlo anchor is ~90% of a sweep cell's wall time and is completely unaffected by IPS
# scheduling, so re-measuring it to check a scheduling change is hours of nothing. This replays
# each cell of an existing results directory with --skip-mc, feeding the old run's MC numbers back
# in via --mc-ref so the PASS/FAIL verdict still means what it always did.
#
# Scenario parameters come from the log FILENAME (family_pos{P}_vel{V}_rx{R}.log), which is how
# cns_sweep.sh names them; the MC anchor is scraped from inside the log.
#
#   bash scripts/ips_rerun_sweep.sh scripts/cns_sweep_20260728_085447            # all 96 cores
#   JOBS=50 bash scripts/ips_rerun_sweep.sh scripts/cns_sweep_20260728_085447    # share the box
#
# Writes ips_rerun_<timestamp>/ with one .log per cell plus a summary.tsv carrying BOTH the old and
# the new IPS probability, so a scheduling change that quietly moved a number is visible in one
# column comparison. They should be identical: the parallel driver is bit-identical to the serial
# path (ADR 0018), and this is the end-to-end check of that at production scale.
set -u

cd "$(dirname "$0")/.." || { echo "cannot cd to repo root"; exit 1; }
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1   # no BLAS oversubscription

SRC=${1:-}
[ -n "$SRC" ] && [ -d "$SRC" ] || { echo "usage: $0 <previous cns_sweep_* directory>"; exit 1; }

# must match the sweep being replayed, or the comparison is meaningless
DPSI=${DPSI:-90} ; TLOS=${TLOS:-70} ; LOOKAHEAD=${LOOKAHEAD:-60} ; DT=${DT:-0.2}
LEVELS=${LEVELS:-"150 135 122 112 104 97 90 82 74 68 63 59 56 54 52 51 50"}
PARTICLES=${PARTICLES:-10000} ; REPS=${REPS:-10} ; JOBS=${JOBS:-96}

OUT="ips_rerun_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
printf 'family\tpos\tvel\trx\tP_mc_old\tP_ips_old\tP_ips_new\tmatch\tcollapsed\tsecs_old\tsecs_new\tlog\n' \
    > "$SUMMARY"

echo "=== IPS-only replay of $SRC -> $OUT  (jobs=$JOBS, reps=$REPS, particles=$PARTICLES) ==="
N=0 ; MISMATCH=0

for log in "$SRC"/*.log; do
  base=$(basename "$log" .log)
  # family_pos{P}_vel{V}_rx{R}
  family=${base%%_*} ; rest=${base#*_}
  pos=${rest%%_*} ; pos=${pos#pos} ; rest=${rest#*_}
  vel=${rest%%_*} ; vel=${vel#vel} ; rest=${rest#*_}
  rx=${rest#rx}

  # the anchor to carry over, plus the previous IPS result to compare against
  read -r mc_p mc_lo mc_hi < <(sed -n 's/^MC .*P(LoS)=\([0-9.]*\) *95%CI\[\([0-9.]*\), *\([0-9.]*\)\].*/\1 \2 \3/p' "$log" | head -1)
  ips_old=$(sed -n 's/^IPS .* P=\([0-9.]*\) .*/\1/p' "$log" | head -1)
  secs_old=$(sed -n 's/^IPS .*(\([0-9]*\)s).*/\1/p' "$log" | head -1)   # the IPS half's old wall
  if [ -z "${mc_p:-}" ]; then
    echo "  skip $base — no MC line to carry over (interrupted cell?)"
    continue
  fi

  N=$((N + 1))
  echo "[$(date +%H:%M:%S)] cell $N: $base   (anchor P_mc=$mc_p)"
  t0=$(date +%s)
  python scripts/ips_validate.py --dpsi "$DPSI" --tlos "$TLOS" --lookahead "$LOOKAHEAD" --dt "$DT" \
      --pos "$pos" --vel "$vel" --reception "$rx" \
      --particles "$PARTICLES" --reps "$REPS" --levels $LEVELS --jobs "$JOBS" \
      --mc-ref "$mc_p" "$mc_lo" "$mc_hi" \
      > "$OUT/${base}.log" 2>&1
  t1=$(date +%s)

  ips_new=$(sed -n 's/^IPS .* P=\([0-9.]*\) .*/\1/p' "$OUT/${base}.log" | head -1)
  coll=$(sed -n 's/^IPS .*collapsed=\([0-9]*\/[0-9]*\).*/\1/p' "$OUT/${base}.log" | head -1)
  if [ "${ips_new:-x}" = "${ips_old:-y}" ]; then match=same; else match=DIFFERS; MISMATCH=$((MISMATCH + 1)); fi

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$family" "$pos" "$vel" "$rx" "$mc_p" "${ips_old:-NA}" "${ips_new:-NA}" \
      "$match" "${coll:-NA}" "${secs_old:-NA}" "$((t1 - t0))" "$OUT/${base}.log" >> "$SUMMARY"
done

echo "[$(date +%H:%M:%S)] done. $N cells -> $SUMMARY"
column -t -s $'\t' "$SUMMARY"
if [ "$MISMATCH" -gt 0 ]; then
  echo
  echo "WARNING: $MISMATCH cell(s) changed their IPS probability. The parallel driver is supposed"
  echo "to be bit-identical to the serial path -- investigate before trusting these numbers."
  exit 1
fi
echo
echo "OK: every cell reproduced its previous IPS probability exactly."
