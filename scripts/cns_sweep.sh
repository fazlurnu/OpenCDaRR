#!/usr/bin/env bash
# cns_sweep.sh — sweep P(LoS) over navigation and communication uncertainty on the fixed 90-deg
# crossing. For every cell it runs a parallel Monte-Carlo anchor + the IPS rare-event estimate
# (opencdarr/scripts/ips_validate.py) and appends one row to a summary table.
#
#   Family 1 — nav only : vary pos_ci95 and vel_ci95 explicitly, perfect comms (rx=1.0)
#   Family 2 — comm only: perfect nav (pos=vel=0), vary reception
#   Family 3 — both     : nav noise + reception together
#
# Run it in the environment where opencdarr is installed (e.g. `conda activate cdarr`).
# Results land in cns_sweep_<timestamp>/  (one .log per cell + summary.tsv).
set -u

cd "$(dirname "$0")/.." || { echo "cannot cd to repo root"; exit 1; }
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1   # no BLAS oversubscription

# ------------------------------------------------------------------ fixed settings (edit here)
DPSI=90 ; TLOS=70 ; LOOKAHEAD=60 ; DT=0.2
# One generous ladder reused across cells. It is a COMPROMISE: fine for P in ~1e-3..1e-5. Watch the
# `collapsed` column in summary.tsv — any nonzero means that cell needs more --particles or a ladder
# with more shells (re-run it alone). Easy (high-P) cells will over-survive the deep shells; harmless.
LEVELS="150 135 122 112 104 97 90 82 74 68 63 59 56 54 52 51 50"
# REPS == JOBS so all replications run in a single parallel wave (reps > jobs would spill into a
# second wave and roughly double the IPS wall-time per cell).
PARTICLES=10000 ; REPS=96 ; MCN=2000000 ; JOBS=96

# ------------------------------------------------------------------ sweep grids (edit here)
NAV_POS=(2 6 10) ; NAV_VEL=(0.5 1.0 2.0)          # family 1 grid
RX_LIST=(0.99 0.90 0.70 0.50 0.30 0.12)           # family 2
BOTH_POS=(6) ; BOTH_VEL=(1.0) ; BOTH_RX=(0.90 0.50)  # family 3

# ------------------------------------------------------------------ machinery
OUT="cns_sweep_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
printf 'family\tpos\tvel\trx\tP_mc\tP_ips\tcollapsed\tsecs\tlog\n' > "$SUMMARY"
N=0

run_cell () {                                       # family pos vel rx
  local family="$1" pos="$2" vel="$3" rx="$4"
  local tag="${family}_pos${pos}_vel${vel}_rx${rx}"
  local log="$OUT/${tag}.log"
  N=$((N + 1))
  echo "[$(date +%H:%M:%S)] cell $N: $tag"
  local t0 t1; t0=$(date +%s)
  python scripts/ips_validate.py --dpsi "$DPSI" --tlos "$TLOS" --lookahead "$LOOKAHEAD" --dt "$DT" \
      --pos "$pos" --vel "$vel" --reception "$rx" \
      --mc-n "$MCN" --particles "$PARTICLES" --reps "$REPS" --levels $LEVELS --jobs "$JOBS" \
      > "$log" 2>&1
  t1=$(date +%s)
  # scrape the numbers out of the log (empty if the run errored)
  local pmc pips coll
  pmc=$(grep -oP '^MC.*P\(LoS\)=\K[0-9.]+' "$log" | head -1)
  pips=$(grep '^IPS' "$log" | grep -oP 'P=\K[0-9.]+' | head -1)
  coll=$(grep '^IPS' "$log" | grep -oP 'collapsed=\K[0-9]+/[0-9]+' | head -1)
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$family" "$pos" "$vel" "$rx" "${pmc:-NA}" "${pips:-NA}" "${coll:-NA}" "$((t1 - t0))" "$log" \
      >> "$SUMMARY"
}

echo "=== CNS uncertainty sweep -> $OUT ==="

# Family 1 — navigation only (perfect comms)
for pos in "${NAV_POS[@]}"; do
  for vel in "${NAV_VEL[@]}"; do
    run_cell nav "$pos" "$vel" 1.0
  done
done

# Family 2 — communication only (perfect nav)
for rx in "${RX_LIST[@]}"; do
  run_cell comm 0 0 "$rx"
done

# Family 3 — both
for pos in "${BOTH_POS[@]}"; do
  for vel in "${BOTH_VEL[@]}"; do
    for rx in "${BOTH_RX[@]}"; do
      run_cell both "$pos" "$vel" "$rx"
    done
  done
done

echo "[$(date +%H:%M:%S)] done. $N cells -> $SUMMARY"
column -t -s $'\t' "$SUMMARY"
