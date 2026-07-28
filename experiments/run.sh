#!/usr/bin/env bash
# Run one experiment setup and archive its logs with full provenance.
#
#   experiments/run.sh <run_id> [repeat_index]
#   experiments/run.sh B2_pole_normal_p080 1
#
# What it does, in order:
#   1. Saves the current params/ directory (the controller REWRITES
#      sequence.txt after a set-up phase, so without this every run would
#      silently contaminate the next one).
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

restore_params() {
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
python3 "$HERE/lib/apply_overlay.py" "$OVERLAY" "$PARAMS" || exit 1
echo ""

mkdir -p "$OUT"

# Provenance: the exact code and the exact parameters that produced the data.
{
  echo "run_id:        $RUN_ID"
  echo "repeat:        $REPEAT"
  echo "timestamp:     $(date -Is)"
  echo "git_commit:    $(cd "$REPO" && git rev-parse HEAD)"
  echo "git_dirty:     $(cd "$REPO" && [ -n "$(git status --porcelain)" ] && echo yes || echo no)"
  echo "host:          $(hostname)"
} > "$OUT/meta.txt"
cp -a "$PARAMS" "$OUT/params_effective"
cp "$OVERLAY" "$OUT/overlay.txt"

echo "--- starting controller (drive it as usual; 'e'+Enter to stop) ---"
echo ""
cd "$SGC" || exit 1
set -o pipefail
./surface_grinding_controller 2>&1 | tee "$OUT/terminal.log"
STATUS=$?
set +o pipefail

echo "controller_exit: $STATUS" >> "$OUT/meta.txt"

shopt -s nullglob
MOVED=0
for f in "$SGC"/*.csv; do
  mv "$f" "$OUT/"
  MOVED=$((MOVED + 1))
done
shopt -u nullglob

echo ""
if [ "$MOVED" -eq 0 ]; then
  echo "WARNING: no CSV produced. The run probably stopped before writing."
  echo "         $OUT keeps the transcript and parameters anyway."
else
  echo "archived $MOVED CSV file(s) to:"
  echo "  $OUT"
fi
echo "exit status: $STATUS"
