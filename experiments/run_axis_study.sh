#!/usr/bin/env bash
# Guided runner for the calibrated-plane MAIN A--C campaign.
#
#   ./experiments/run_axis_study.sh status
#   ./experiments/run_axis_study.sh next
#   ./experiments/run_axis_study.sh case A
#
# `next` runs exactly one robot trial. `case` runs every remaining trial of one
# case letter, every setup and repeat, stopping between them so the setup can
# be reset and stopping outright on the first trial that does not archive.
# The existing run.sh archives the raw logs, effective parameters, calibration,
# terminal transcript, and provenance. `next` regenerates metrics and figures
# after its trial; `case` does it once at the end, since extraction re-parses
# every archived run and takes about a minute.
#
# Case D was enabled on 2026-07-31, once A--C were complete and its gains had
# been selected from them. See the note above the Case D block in
# lib/generate_setups.py for what the screening chose and why.

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
  MAIN_D1_t1_rc_t2_m060
  MAIN_D1_t1_rc_t2_p000
  MAIN_D1_t1_rc_t2_p060
  MAIN_D2_t2_rc_t1_m060
  MAIN_D2_t2_rc_t1_p000
  MAIN_D2_t2_rc_t1_p060
  MAIN_D3_t1_rc_face_centre
  MAIN_D3_t2_rc_face_centre
  MAIN_E1_tilt_about_y_long
  MAIN_E1_tilt_about_x_short
  MAIN_F1_nullspace_damping
  MAIN_F3_nullspace_damping_sigma
)

repeats_for() {
  case "$1" in
    MAIN_A*|MAIN_B*|MAIN_C*|MAIN_D*|MAIN_E*|MAIN_F*) echo 3 ;;
    *) echo 0 ;;
  esac
}

# What proves a trial ran to completion depends on what it was. A contact case
# must have pressed, which the set-up result block records. A hold case never
# presses; it must instead have reached the end of its scripted disturbance,
# which the release cue records.
repeat_complete() {
  local run_dir="$1" run_id marker
  run_id="$(basename "$(dirname "$run_dir")")"
  marker='^=== Set-up result ===$'
  if [ "$(cat "$HERE/setups/$run_id/startup_mode.txt" 2>/dev/null)" = "h" ]; then
    marker='RELEASE'
  fi
  [ -d "$run_dir" ] &&
    [ -s "$run_dir/surface_grinding_controller_log.csv" ] &&
    grep -q '^controller_exit: 0$' "$run_dir/meta.txt" 2>/dev/null &&
    grep -q "$marker" "$run_dir/terminal.log" 2>/dev/null
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
    printf "%-24s %d/%d complete\n" "$run_id" "$complete" "$repeats"
  done
}

find_next() {
  local run_id repeats i repeat_tag
  for run_id in "${RUN_IDS[@]}"; do
    repeats="$(repeats_for "$run_id")"
    for i in $(seq 1 "$repeats"); do
      repeat_tag="$(printf 'r%02d' "$i")"
      if [ -d "$HERE/results/$run_id/$repeat_tag" ] &&
         ! repeat_complete "$HERE/results/$run_id/$repeat_tag"; then
        echo "ERROR: incomplete trial blocks $run_id/$repeat_tag." >&2
        echo "Archive or remove that partial directory before retrying." >&2
        return 2
      fi
      if [ ! -d "$HERE/results/$run_id/$repeat_tag" ]; then
        echo "$run_id $i"
        return 0
      fi
    done
  done
  return 1
}

# Every setup of one case letter, in RUN_IDS order.
case_run_ids() {
  local letter="$1" run_id
  for run_id in "${RUN_IDS[@]}"; do
    case "$run_id" in
      MAIN_${letter}*) echo "$run_id" ;;
    esac
  done
}

# Extraction re-parses every archived CSV and takes about a minute, so a case
# loop pays it once at the end rather than after each of its trials.
refresh_derived() {
  echo ""
  echo "Refreshing metrics and figures..."
  python3 "$HERE/analysis/extract_metrics.py" || return $?
  python3 "$HERE/analysis/make_figures.py" || return $?
}

# One case, every setup and repeat, in order. Each trial is still driven by
# hand: the loop only sequences them and stops the moment one does not archive.
run_case() {
  local letter="$1"
  local auto="${2:-}"
  local run_ids repeats i repeat_tag run_dir
  local -a pending=()

  run_ids="$(case_run_ids "$letter")"
  if [ -z "$run_ids" ]; then
    echo "No case $letter in this runner. Cases present: A, B, C, D, E, F." >&2
    return 2
  fi

  for run_id in $run_ids; do
    repeats="$(repeats_for "$run_id")"
    for i in $(seq 1 "$repeats"); do
      repeat_tag="$(printf 'r%02d' "$i")"
      run_dir="$HERE/results/$run_id/$repeat_tag"
      if repeat_complete "$run_dir"; then
        continue
      fi
      if [ -d "$run_dir" ]; then
        echo "ERROR: incomplete trial blocks $run_id/$repeat_tag." >&2
        echo "Archive or remove that partial directory before retrying." >&2
        return 2
      fi
      pending+=("$run_id $i")
    done
  done

  if [ "${#pending[@]}" -eq 0 ]; then
    echo "Case $letter is already complete."
    return 0
  fi

  echo "Case $letter: ${#pending[@]} trial(s) to run, in this order:"
  for entry in "${pending[@]}"; do
    read -r run_id i <<< "$entry"
    echo "  $run_id / $(printf 'r%02d' "$i")"
  done
  echo ""
  if [ "$auto" = "auto" ]; then
    echo "UNATTENDED: each trial is driven by lib/auto_drive.py, which answers"
    echo "the prompts and moves straight to the next one. The robot runs the"
    echo "whole case without stopping. Stay at the e-stop."
  else
    echo "Each trial is driven by hand as usual. Between trials the loop stops"
    echo "so the tool and workpiece can be reset."
  fi
  printf "Press Enter to start, anything else to abort: "
  # A closed or piped stdin makes read fail with an empty answer, which must
  # not read as the bare Enter that starts a robot trial.
  if ! read -r answer || [ -n "$answer" ]; then
    echo "Aborted. Nothing was run."
    return 0
  fi

  local done_count=0
  for entry in "${pending[@]}"; do
    read -r run_id i <<< "$entry"
    done_count=$((done_count + 1))
    echo ""
    echo "=================================================================="
    echo "Case $letter trial $done_count of ${#pending[@]}: $run_id / $(printf 'r%02d' "$i")"
    echo "=================================================================="
    local trial_failed=0
    if [ "$auto" = "auto" ]; then
      python3 "$HERE/lib/auto_drive.py" "$run_id" "$i" || trial_failed=1
    else
      "$HERE/run.sh" "$run_id" "$i" || trial_failed=1
    fi
    if [ "$trial_failed" -ne 0 ]; then
      echo "" >&2
      echo "Trial $run_id/$(printf 'r%02d' "$i") did not archive. Stopping the case." >&2
      refresh_derived
      return 1
    fi
    # A trial that exits cleanly can still be inadmissible, so check the same
    # condition the status table uses before moving on.
    if ! repeat_complete "$HERE/results/$run_id/$(printf 'r%02d' "$i")"; then
      echo "" >&2
      echo "Trial $run_id/$(printf 'r%02d' "$i") has no Set-up result block." >&2
      echo "It does not count. Move it aside before rerunning this case." >&2
      refresh_derived
      return 1
    fi
    if [ "$done_count" -lt "${#pending[@]}" ] && [ "$auto" != "auto" ]; then
      echo ""
      printf "Trial archived. Reset the setup, then press Enter for the next one (q quits): "
      if ! read -r answer || [ "$answer" = "q" ]; then
        echo "Stopped after $done_count trial(s)."
        refresh_derived
        return 0
      fi
    fi
  done

  echo ""
  echo "Case $letter finished: $done_count trial(s) archived."
  refresh_derived
}

case "${1:-status}" in
  status)
    show_status
    ;;
  case)
    if [ $# -lt 2 ]; then
      echo "usage: $(basename "$0") case <A|B|C|D|E|F> [auto]" >&2
      exit 2
    fi
    run_case "$2" "${3:-}" || exit $?
    show_status
    ;;
  next)
    if ! next_trial="$(find_next)"; then
      echo "Cases A--F are complete."
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
    echo "usage: $(basename "$0") [status|next|case <A|B|C|D|E|F> [auto]]" >&2
    exit 2
    ;;
esac
