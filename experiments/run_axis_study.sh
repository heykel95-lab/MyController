#!/usr/bin/env bash
# Guided runner for the calibrated-plane D-series campaign.
#
#   ./experiments/run_axis_study.sh status
#   ./experiments/run_axis_study.sh next
#
# `next` runs exactly one robot trial. The existing run.sh archives the raw
# logs, effective parameters, calibration, terminal transcript, and provenance.
# Metrics and figures are regenerated after every successful trial.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_IDS=(
  D0_flat_00deg
  D1_KRt1_05
  D1_KRt1_15
  D1_KRt1_50
  D2_KRt2_05
  D2_KRt2_15
  D2_KRt2_50
  D3_angle_t1_05deg
  D3_angle_t2_05deg
)

repeats_for() {
  case "$1" in
    D*) echo 5 ;;
    *) echo 0 ;;
  esac
}

show_status() {
  local run_id repeats complete i
  for run_id in "${RUN_IDS[@]}"; do
    repeats="$(repeats_for "$run_id")"
    complete=0
    for i in $(seq 1 "$repeats"); do
      if [ -d "$HERE/results/$run_id/$(printf 'r%02d' "$i")" ]; then
        complete=$((complete + 1))
      fi
    done
    printf "%-24s %d/%d complete\n" "$run_id" "$complete" "$repeats"
  done
}

find_next() {
  local run_id repeats i repeat_tag
  for run_id in "${RUN_IDS[@]}"; do
    repeats="$(repeats_for "$run_id")"
    for i in $(seq 1 "$repeats"); do
      repeat_tag="$(printf 'r%02d' "$i")"
      if [ ! -d "$HERE/results/$run_id/$repeat_tag" ]; then
        echo "$run_id $i"
        return 0
      fi
    done
  done
  return 1
}

case "${1:-status}" in
  status)
    show_status
    ;;
  next)
    if ! next_trial="$(find_next)"; then
      echo "All calibrated-plane axis-study trials are complete."
      exit 0
    fi
    read -r run_id repeat_index <<< "$next_trial"
    echo "Next trial: $run_id / $(printf 'r%02d' "$repeat_index")"
    "$HERE/run.sh" "$run_id" "$repeat_index" || exit $?
    python3 "$HERE/analysis/extract_metrics.py" || exit $?
    python3 "$HERE/analysis/make_figures.py" || exit $?
    echo ""
    echo "Trial archived and analysis refreshed."
    show_status
    ;;
  *)
    echo "usage: $(basename "$0") [status|next]" >&2
    exit 2
    ;;
esac
