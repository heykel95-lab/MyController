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
)

repeat_complete() {
  local run_dir="$1"
  [ -d "$run_dir" ] &&
    [ -s "$run_dir/surface_grinding_controller_log.csv" ] &&
    grep -q '^controller_exit: 0$' "$run_dir/meta.txt" 2>/dev/null &&
    grep -q '^=== Set-up result ===$' "$run_dir/terminal.log" 2>/dev/null
}

show_status() {
  local run_id
  for run_id in "${RUN_IDS[@]}"; do
    if repeat_complete "$HERE/results/$run_id/r01"; then
      printf "%-30s 1/1 complete\n" "$run_id"
    else
      printf "%-30s 0/1 complete\n" "$run_id"
    fi
  done
}

find_next() {
  local run_id run_dir
  for run_id in "${RUN_IDS[@]}"; do
    run_dir="$HERE/results/$run_id/r01"
    if [ -d "$run_dir" ] && ! repeat_complete "$run_dir"; then
      echo "ERROR: incomplete trial blocks $run_id/r01." >&2
      echo "Archive or remove that partial directory before retrying." >&2
      return 2
    fi
    if [ ! -d "$run_dir" ]; then
      echo "$run_id"
      return 0
    fi
  done
  return 1
}

case "${1:-status}" in
  status)
    show_status
    ;;
  next)
    if ! run_id="$(find_next)"; then
      echo "The two centre-of-compliance sign pilots are complete."
      exit 0
    fi
    echo "Next trial: $run_id / r01"
    "$HERE/run.sh" "$run_id" 1 || exit $?
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
