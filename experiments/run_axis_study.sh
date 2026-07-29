#!/usr/bin/env bash
# Guided runner for the calibrated-plane MAIN A--C campaign.
#
#   ./experiments/run_axis_study.sh status
#   ./experiments/run_axis_study.sh next
#
# `next` runs exactly one robot trial. The existing run.sh archives the raw
# logs, effective parameters, calibration, terminal transcript, and provenance.
# Metrics and figures are regenerated after every successful trial.
#
# Case D is deliberately absent. Its gain values are selected only after the
# A--C screening results have been reviewed.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_IDS=(
  MAIN_A0_00deg
  MAIN_A1_t1_05deg
  MAIN_A2_t1_10deg
  MAIN_A3_t2_05deg
  MAIN_A4_t2_10deg
  MAIN_B1_KRt1_15
  MAIN_B1_KRt1_50
  MAIN_B2_KRt2_15
  MAIN_B2_KRt2_50
  MAIN_C1_KPt2_0300
  MAIN_C1_KPt2_0800
  MAIN_C1_interaction_KR50_KP300
  MAIN_C2_KPt1_0300
  MAIN_C2_KPt1_0800
  MAIN_C2_interaction_KR50_KP300
)

repeats_for() {
  case "$1" in
    MAIN_A*|MAIN_B*|MAIN_C*) echo 3 ;;
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
      echo "Cases A--C are complete. Analyse them before generating the final Case-D gains."
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
