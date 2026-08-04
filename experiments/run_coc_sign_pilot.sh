#!/usr/bin/env bash
# Guided one-repeat active centre-of-compliance sign pilot.
#
#   ./experiments/run_coc_sign_pilot.sh status
#   ./experiments/run_coc_sign_pilot.sh next

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_IDS=(
  PILOT_COC_t1_rc_t2_p060
  PILOT_COC_t1_rc_t2_m060
  PILOT_COC_t1_rc_t2_m040
)

repeats_for() {
  case "$1" in
    PILOT_COC_t1_rc_t2_m040) echo 3 ;;
    *) echo 1 ;;
  esac
}

repeat_complete() {
  local run_dir="$1"
  [ -d "$run_dir" ] &&
    [ -s "$run_dir/surface_grinding_controller_log.csv" ] &&
    grep -q '^controller_exit: 0$' "$run_dir/meta.txt" 2>/dev/null &&
    grep -q '^ *SET-UP RESULT$' "$run_dir/terminal.log" 2>/dev/null
}

show_status() {
  local run_id repeats complete i
  for run_id in "${RUN_IDS[@]}"; do
    repeats="$(repeats_for "$run_id")"
    complete=0
    for i in $(seq 1 "$repeats"); do
      if repeat_complete "$HERE/results/$run_id/$(printf 'r%02d' "$i")"; then
        complete=$((complete + 1))
      fi
    done
    printf "%-30s %d/%d complete\n" "$run_id" "$complete" "$repeats"
  done
}

find_next() {
  local run_id repeats i repeat_tag run_dir
  for run_id in "${RUN_IDS[@]}"; do
    repeats="$(repeats_for "$run_id")"
    for i in $(seq 1 "$repeats"); do
      repeat_tag="$(printf 'r%02d' "$i")"
      run_dir="$HERE/results/$run_id/$repeat_tag"
      if [ -d "$run_dir" ] && ! repeat_complete "$run_dir"; then
        echo "ERROR: incomplete trial blocks $run_id/$repeat_tag." >&2
        echo "Archive or remove that partial directory before retrying." >&2
        return 2
      fi
      if [ ! -d "$run_dir" ]; then
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
      echo "The centre-of-compliance sign and refinement pilots are complete."
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
