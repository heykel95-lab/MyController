#!/usr/bin/env python3
"""Accept or reject the stronger automatic null-space disturbance pilot.

Usage:
  python3 experiments/analysis/validate_nullspace_pilot.py RUN_DIR

The gate prevents the unattended stronger Case-F series from starting merely
because a pilot process exited cleanly.  It requires a visibly larger redundant
excursion than the first campaign, a larger applied joint-torque moment, the
complete unclipped configured-force waveform, and Cartesian hold error below
2 mm.
"""

import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from disturbance_quality import report  # noqa: E402


MIN_EXCURSION_RAD = 0.040
MIN_TAU_PEAK_NM = 2.000
MIN_FORCE_PEAK_N = 19.9
MIN_TORQUE_SCALE = 0.999
MAX_TASK_ERROR_MM = 2.000


def peak_task_error_mm(run_dir):
    path = os.path.join(run_dir, "surface_grinding_controller_log.csv")
    peak = 0.0
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            try:
                norm = math.sqrt(sum(float(row[f"e_p_{axis}"]) ** 2
                                     for axis in "xyz"))
            except (KeyError, ValueError):
                continue
            peak = max(peak, 1000.0 * norm)
    return peak


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: validate_nullspace_pilot.py RUN_DIR")
    run_dir = sys.argv[1]
    result = report(run_dir)
    if not result or result.get("no_motion") or not result.get("automatic"):
        print("stronger-pilot gate: FAIL -- no complete automatic waveform")
        return 2

    task_peak = peak_task_error_mm(run_dir)
    checks = (
        (result["reached_full"], "full scheduled scale was not reached"),
        (result["force_peak"] >= MIN_FORCE_PEAK_N,
         f"force peak {result['force_peak']:.3f} N < {MIN_FORCE_PEAK_N:.1f} N"),
        (result["tau_peak"] > MIN_TAU_PEAK_NM,
         f"torque peak {result['tau_peak']:.3f} Nm <= {MIN_TAU_PEAK_NM:.3f} Nm"),
        (result["torque_scale_min"] >= MIN_TORQUE_SCALE,
         f"torque limit clipped the waveform (scale "
         f"{result['torque_scale_min']:.3f})"),
        (result["excursion"] >= MIN_EXCURSION_RAD,
         f"excursion {result['excursion']:.3f} rad < "
         f"{MIN_EXCURSION_RAD:.3f} rad"),
        (task_peak <= MAX_TASK_ERROR_MM,
         f"peak task error {task_peak:.3f} mm > {MAX_TASK_ERROR_MM:.3f} mm"),
    )

    print("stronger automatic null-space pilot")
    print(f"  force peak:       {result['force_peak']:.3f} N")
    print(f"  torque peak:      {result['tau_peak']:.3f} Nm")
    print(f"  torque scale min: {result['torque_scale_min']:.3f}")
    print(f"  excursion:        {result['excursion']:.3f} rad")
    print(f"  task error peak:  {task_peak:.3f} mm")
    failures = [message for passed, message in checks if not passed]
    if failures:
        for message in failures:
            print(f"  FAIL: {message}")
        print("stronger-pilot gate: FAIL -- unattended repeats remain blocked")
        return 2
    print("stronger-pilot gate: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
