#!/bin/bash
# run_finerange_sweep.sh — the 28-job disc_dia (RD) + backbone_length (BL) sweep,
# 4-tendon only, new 10-step 0-4.5N force ramp, mesh-validated value lists.
# Results kept fully separate from the old sweep in results_finerange/.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

# CQ_PYTHON must be exported before running this script (see README.md).
export CQ_PYTHON="${CQ_PYTHON:?set CQ_PYTHON to the cadquery env python first}"
export TDCR_TENSION_LEVELS="450,900,1350,1800,2250,2700,3150,3600,4050,4500"  # 0.45..4.5N, 10 steps

RD_VALS="26 27 29 31 32 35 37 38 40 42 45 46 48 50"
BL_VALS="100 108 115 123 131 138 146 154 162 169 177 185 192 200"

mkdir -p results_finerange

run_rd() {
  local v=$1 id="RD2_${v}"
  TDCR_SWEEP_CSV="$PWD/results_finerange/${id}.csv" \
    bash run_one.sh "$id" 14 "$v" 2 25 100 4 > "logs/${id}.finerange.log" 2>&1
  echo "  RD2_${v}  exit=$?  $([ -s "results_finerange/${id}.csv" ] && echo HAS_DATA || echo NO_DATA)"
}
run_bl() {
  local v=$1 id="BL2_${v}"
  TDCR_SWEEP_CSV="$PWD/results_finerange/${id}.csv" \
    bash run_one.sh "$id" 14 30 2 25 "$v" 4 > "logs/${id}.finerange.log" 2>&1
  echo "  BL2_${v}  exit=$?  $([ -s "results_finerange/${id}.csv" ] && echo HAS_DATA || echo NO_DATA)"
}
export -f run_rd run_bl
export CQ_PYTHON TDCR_TENSION_LEVELS

echo "=== launching 28 jobs in parallel ==="
for v in $RD_VALS; do run_rd "$v" & done
for v in $BL_VALS; do run_bl "$v" & done
wait
echo "=== all 28 jobs finished ==="
