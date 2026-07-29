#!/usr/bin/env bash
# Guided runner for the compact tilted-plane frame-transfer validation.
#
# Run this after the horizontal primary campaign. The final tuned validation
# condition is added only after horizontal Cases A--D have selected it.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_IDS=(
  VALID_T0_00deg
  VALID_T1_t1_10deg
  VALID_T2_t2_10deg
)

show_status() {
  local run_id complete i
  for run_id in "${RUN_IDS[@]}"; do
    complete=0
    for i in 1 2 3; do
      if [ -d "$HERE/results/$run_id/$(printf 'r%02d' "$i")" ]; then
        complete=$((complete + 1))
      fi
    done
    printf "%-24s %d/3 complete\n" "$run_id" "$complete"
  done
}

find_next() {
  local run_id i repeat_tag
  for run_id in "${RUN_IDS[@]}"; do
    for i in 1 2 3; do
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
      echo "The baseline tilted-plane validation is complete."
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
