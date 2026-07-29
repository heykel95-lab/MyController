#!/usr/bin/env bash
# Run one experiment setup and archive its logs with full provenance.
#
#   experiments/run.sh <run_id> [repeat_index]
#   experiments/run.sh B2_pole_normal_p080 1
#
# What it does, in order:
#   1. Saves the current params/ directory so experimental overlays never
#      contaminate the next run.
#   2. Applies the setup's overlay onto params/.
#   3. Runs the controller. You drive it interactively as usual.
#   4. Copies both CSV logs plus the effective parameters, the git commit and
#      the terminal transcript into experiments/results/<run_id>/rNN/.
#   5. Restores the original params/ even if the run crashed or was aborted.
#
# The restore is in a trap, so Ctrl-C and libfranka reflex exits are safe.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
SGC="$REPO/surface_grinding_controller"
PARAMS="$SGC/params"

if [ $# -lt 1 ]; then
  echo "usage: $(basename "$0") <run_id> [repeat_index]" >&2
  echo "available setups:" >&2
  ls -1 "$HERE/setups" | grep -v INDEX.txt | sed 's/^/  /' >&2
  exit 1
fi

RUN_ID="$1"
REPEAT="${2:-1}"
SETUP_DIR="$HERE/setups/$RUN_ID"
OVERLAY="$SETUP_DIR/overlay.txt"
PLANE_OVERLAY="$HERE/calibration/active_plane_overlay.txt"
PLANE_REPORT="$HERE/calibration/plane_calibration_report.txt"
USES_PLANE_CALIBRATION=0

if [ ! -f "$OVERLAY" ]; then
  echo "ERROR: no setup '$RUN_ID' (missing $OVERLAY)" >&2
  exit 1
fi

REPEAT_TAG="$(printf 'r%02d' "$REPEAT")"
OUT="$HERE/results/$RUN_ID/$REPEAT_TAG"

if [ -d "$OUT" ]; then
  echo "ERROR: $OUT already exists. Use a different repeat index, or delete it." >&2
  exit 1
fi

BACKUP="$(mktemp -d)"
cp -a "$PARAMS/." "$BACKUP/"

# Sampled here, BEFORE the overlay is applied. The overlay deliberately edits
# tracked files in params/, so sampling after it would report every run with a
# non-empty overlay as dirty -- i.e. nearly the whole campaign. The overlay is
# already recorded verbatim in overlay.txt and params_effective/; what this
# field is for is whether anything ELSE was uncommitted when the run happened.
# Tracked changes only: untracked files are results/ piling up during a batch.
if [ -n "$(cd "$REPO" && git status --porcelain --untracked-files=no)" ]; then
  GIT_DIRTY=yes
else
  GIT_DIRTY=no
fi

restore_params() {
  # Ctrl-C fires INT and then EXIT, so without this guard the second call finds
  # the backup already deleted and prints a bogus "cp: cannot stat" error --
  # which is indistinguishable from a real restore failure at a glance.
  [ -d "$BACKUP" ] || return 0
  cp -a "$BACKUP/." "$PARAMS/"
  rm -rf "$BACKUP"
  echo ""
  echo "params/ restored from backup."
}
trap restore_params EXIT INT TERM

echo "=== $RUN_ID / $REPEAT_TAG ==="
sed -n '1,200p' "$SETUP_DIR/about.txt"
echo ""
echo "--- applying overlay ---"
case "$RUN_ID" in
  D*|MAIN_*)
    USES_PLANE_CALIBRATION=1
    if [ ! -f "$PLANE_OVERLAY" ] || [ ! -f "$PLANE_REPORT" ]; then
      echo "ERROR: calibrated-plane experiments require a validated plane calibration."
      echo "Create experiments/calibration/plane_points.csv and run:"
      echo "  python3 experiments/calibration/prepare_plane_calibration.py"
      exit 1
    fi
    if ! grep -q "^surface tangent t1 (P1->P2)" "$PLANE_REPORT" ||
       ! grep -q "^held-out signed distances" "$PLANE_REPORT"; then
      echo "ERROR: calibration is missing the P1--P2 tangent or held-out P4 check."
      echo "Regenerate it with:"
      echo "  python3 experiments/calibration/prepare_plane_calibration.py"
      exit 1
    fi
    if grep -q "^WARNING:" "$PLANE_REPORT"; then
      echo "ERROR: the held-out calibration point is more than 1 mm from the plane."
      cat "$PLANE_REPORT"
      exit 1
    fi
    python3 "$HERE/lib/apply_overlay.py" "$PLANE_OVERLAY" "$PARAMS" || exit 1
    ;;
esac
python3 "$HERE/lib/apply_overlay.py" "$OVERLAY" "$PARAMS" || exit 1
echo ""

PREFLIGHT="$SGC/tools/inspect_experiment_config"
if [ ! -x "$PREFLIGHT" ]; then
  echo "ERROR: missing experiment preflight tool. Build it with:" >&2
  echo "  make -C $SGC inspect_experiment_config" >&2
  exit 1
fi
"$PREFLIGHT" "$PARAMS" || exit 1
echo ""

mkdir -p "$OUT"

# Provenance: the exact code and the exact parameters that produced the data.
{
  echo "run_id:        $RUN_ID"
  echo "repeat:        $REPEAT"
  echo "timestamp:     $(date -Is)"
  echo "git_commit:    $(cd "$REPO" && git rev-parse HEAD)"
  echo "git_dirty:     $GIT_DIRTY"
  echo "host:          $(hostname)"
} > "$OUT/meta.txt"
cp -a "$PARAMS" "$OUT/params_effective"
cp "$OVERLAY" "$OUT/overlay.txt"
if [ "$USES_PLANE_CALIBRATION" -eq 1 ]; then
  cp "$PLANE_OVERLAY" "$OUT/plane_calibration_overlay.txt"
  if [ -f "$HERE/calibration/plane_calibration_report.txt" ]; then
    cp "$HERE/calibration/plane_calibration_report.txt" "$OUT/"
  fi
fi

echo "--- starting controller (drive it as usual; 'e'+Enter to stop) ---"
echo ""

# Marker used to tell this run's CSVs apart from ones left over in the
# working directory. Without it a stale log from an earlier session gets
# archived as though it belonged to this run.
STAMP="$(mktemp)"
touch "$STAMP"

cd "$SGC" || exit 1
set -o pipefail
./surface_grinding_controller 2>&1 | tee "$OUT/terminal.log"
STATUS=$?
set +o pipefail

echo "controller_exit: $STATUS" >> "$OUT/meta.txt"

shopt -s nullglob
MOVED=0
SKIPPED=0
for f in "$SGC"/*.csv; do
  if [ "$f" -nt "$STAMP" ]; then
    mv "$f" "$OUT/"
    MOVED=$((MOVED + 1))
  else
    echo "  skipping stale $(basename "$f") (predates this run, left in place)"
    SKIPPED=$((SKIPPED + 1))
  fi
done
shopt -u nullglob
rm -f "$STAMP"

echo ""
if [ "$MOVED" -eq 0 ]; then
  echo "WARNING: no CSV produced by this run. It probably stopped before writing."
  echo "         $OUT keeps the transcript and parameters anyway."
else
  echo "archived $MOVED CSV file(s) to:"
  echo "  $OUT"
fi
echo "exit status: $STATUS"

# Propagate it. Without this the script always returned 0 (the status of the
# echo above), so a batch written as
#   for i in 1 2 3; do ./experiments/run.sh <id> $i || break; done
# ran on after an aborted run and burned the remaining repeat indices in
# seconds. A run that produced no CSV is a failure and must stop the batch.
if [ "$MOVED" -eq 0 ] && [ "$STATUS" -eq 0 ]; then
  exit 1
fi
exit "$STATUS"
