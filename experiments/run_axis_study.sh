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
#
# Case H continues from Case D's result: D found the lever that aligns each
# surface axis, H asks whether one lever can serve every tilt direction and
# whether the pole belongs above the plane, in it, or under it.
#
# Cases J and K finish the pole question that D, E and H opened. D, E and H
# only ever commanded a tilt leaning one way and a lever 60 mm long. J
# negates the tilt and asks whether negating the lever with it recovers the
# same correction; K holds the sign and sweeps the length against two
# initial tilts, so the lever can be reported as a function of the
# misalignment instead of as one number. Run J before K: K assumes the sign
# rule J tests.

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
  MAIN_F0_baseline
  MAIN_F1_damping_1p0
  MAIN_F1_damping_2p0
  MAIN_F1_damping_4p0
  MAIN_F2_ksigma_1p0
  MAIN_F2_ksigma_2p0
  MAIN_F2_ksigma_4p0
  MAIN_F7_baseline_20N_200mm
  MAIN_F7_damping_2p0_20N_200mm
  MAIN_F8_ksigma_1p5_20N_200mm
  MAIN_F8_ksigma_2p0_20N_200mm
  MAIN_H1_rot_yEE
  MAIN_H1_rot_diag_m45
  MAIN_H1_rot_xEE
  MAIN_H1_rot_diag_p45
  MAIN_H2_fix_diag_m45
  MAIN_H2_fix_xEE
  MAIN_H2_fix_diag_p45
  MAIN_H3_rcn_m060
  MAIN_H3_rcn_p020
  MAIN_H3_rcn_p060
  MAIN_H3_rcn_p120
  # Case J: the zero-lever runs first at each axis. They are the mirrored
  # no-lever reference and also the cheapest reachability check on a tilt sign
  # the arm has never been commanded to, so nothing adds a lever to a negative
  # tilt until one has been pressed without one.
  MAIN_J1_t1neg_rc_t2_p000
  MAIN_J1_t1neg_rc_t2_p060
  MAIN_J1_t1neg_rc_t2_m060
  MAIN_J2_t2neg_rc_t1_p000
  MAIN_J2_t2neg_rc_t1_m060
  MAIN_J2_t2neg_rc_t1_p060
  # Case K: shortest lever first at each tilt, so the run that grows the
  # commanded moment is always the one after a run that already pressed.
  MAIN_K1_t1_05deg_rho020
  MAIN_K1_t1_05deg_rho040
  MAIN_K1_t1_05deg_rho060
  MAIN_K1_t1_05deg_rho080
  MAIN_K1_t1_10deg_rho020
  MAIN_K1_t1_10deg_rho040
  MAIN_K1_t1_10deg_rho080
  MAIN_K2_t2_05deg_rho020
  MAIN_K2_t2_05deg_rho040
  MAIN_K2_t2_05deg_rho060
  MAIN_K2_t2_05deg_rho080
  MAIN_K2_t2_10deg_rho020
  MAIN_K2_t2_10deg_rho040
  MAIN_K2_t2_10deg_rho080
)

repeats_for() {
  case "$1" in
    MAIN_F2_ksigma_4p0) echo 1 ;;
    MAIN_A*|MAIN_B*|MAIN_C*|MAIN_D*|MAIN_E*|MAIN_F*|MAIN_H*|MAIN_J*|MAIN_K*)
      echo 3 ;;
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
  marker='^ *SET-UP RESULT$'
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

# Cases J and K enter regimes the campaign has not been in: J commands its
# first negative tilts, K sweeps the lever a third past anything archived. A
# trial that archives cleanly can still be wrong -- the tilt not mirroring at
# first contact, a press that never reached the plane, a load past the point
# the protocol says to stop at -- and unattended, the sweep would go on to
# repeat that seventeen more times. So each J and K trial is checked before the
# next one starts, in both driving modes.
#
# The check is deliberately not a hypothesis test. Its bounds come from the
# archive, and it passes all 108 archived MAIN contact trials, including the
# not-converged MAIN_D1_t1_rc_t2_m060/r01 that Case J exists to mirror. An
# unexpected result inside those bounds is a result and runs.
validate_trial() {
  local letter="$1" run_dir="$2"
  case "$letter" in
    J|K) python3 "$HERE/analysis/validate_contact_trial.py" "$run_dir" ;;
    *) return 0 ;;
  esac
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
    echo "No case $letter in this runner. Cases present: A, B, C, D, E, F, H, J, K." >&2
    return 2
  fi

  # The 20 N, +200 mm follow-on applies more moment than the archived
  # +100 mm campaign.  Do not let the unattended runner reach it until one
  # mode-0 pilot has both completed and passed the waveform/motion/task gate.
  if [ "$letter" = "F" ]; then
    local strong_pilot_dir="$HERE/results/PILOT_F_disturbance_20N_200mm/r01"
    if ! repeat_complete "$strong_pilot_dir"; then
      echo "Stronger Case-F runs are blocked pending:" >&2
      echo "  $HERE/run.sh PILOT_F_disturbance_20N_200mm 1" >&2
      return 2
    fi
    if ! python3 "$HERE/analysis/validate_nullspace_pilot.py" "$strong_pilot_dir"; then
      return 2
    fi
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
    echo ""
    if ! validate_trial "$letter" \
         "$HERE/results/$run_id/$(printf 'r%02d' "$i")"; then
      echo "" >&2
      echo "Trial $run_id/$(printf 'r%02d' "$i") archived but did not pass the" >&2
      echo "contact check above. Stopping case $letter with $((${#pending[@]} - done_count)) trial(s) unrun." >&2
      echo "The trial is kept: inspect it before deciding to continue." >&2
      refresh_derived
      return 1
    fi
    if [ "$done_count" -lt "${#pending[@]}" ] && [ "$auto" = "auto" ]; then
      # Each automatic trial starts a new libfranka session and moves back to
      # q_init.  Allow residual motion from the preceding controller shutdown
      # to settle before the next joint-motion generator is constructed.
      echo "Waiting 5 s for the arm to settle before the next trial..."
      sleep 5
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
      echo "usage: $(basename "$0") case <A|B|C|D|E|F|H|J|K> [auto]" >&2
      exit 2
    fi
    run_case "$2" "${3:-}" || exit $?
    show_status
    ;;
  next)
    if ! next_trial="$(find_next)"; then
      echo "Cases A--K are complete."
      exit 0
    fi
    read -r run_id repeat_index <<< "$next_trial"
    echo "Next trial: $run_id / $(printf 'r%02d' "$repeat_index")"
    "$HERE/run.sh" "$run_id" "$repeat_index" || exit $?
    # Same check the case loop applies, so a trial driven one at a time is
    # held to the condition an unattended one would have been.
    trial_letter="${run_id#MAIN_}"
    trial_letter="${trial_letter:0:1}"
    echo ""
    validate_trial "$trial_letter" \
      "$HERE/results/$run_id/$(printf 'r%02d' "$repeat_index")" || true
    python3 "$HERE/analysis/extract_metrics.py" || exit $?
    python3 "$HERE/analysis/make_figures.py" || exit $?
    echo ""
    echo "Trial archived and analysis refreshed."
    show_status
    ;;
  *)
    echo "usage: $(basename "$0") [status|next|case <A|B|C|D|E|F|H|J|K> [auto]]" >&2
    exit 2
    ;;
esac
