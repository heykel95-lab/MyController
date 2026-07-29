#!/usr/bin/env python3
"""Alignment measured against the REAL plane instead of the configured one.

  python3 experiments/analysis/alignment_vs_real_plane.py

The controller scores alignment against the normal built from
alignment_target_tilt_angle_deg / _y_deg, which is deliberately left off the
physical plane. `align_gain` in metrics.csv is therefore a residual against an
assumption: it goes *more negative* the closer the tool gets to the real
surface, which reads backwards. This script rescores the B2 sweep against the
plane as measured by tools/measure_plane.

Two things are easy to get wrong here, so they are stated explicitly:

1. SIGN. orientationError(R_current, R_desired) in control_math.cpp returns the
   rotation taking current -> desired, and during set_up the desired is the
   orientation frozen at contact. So the logged e_R points BACK to the start
   and the tool's actual rotation is -e_R. Using +e_R makes the tool appear to
   rotate away from the surface and inverts the entire conclusion.

2. WHAT TO REPORT. The absolute residual depends on the measured plane, which
   is only known to about +/-2 deg from hand-seating. The IMPROVEMENT does not:
   across the three measured seatings it moves by less than 0.1 deg, because
   the tool rotates in essentially the same plane as the mismatch. Report the
   improvement; quote the residual only as a range.

The tool axis at the start of set_up is taken to be the configured normal. The
runs put align_before at 0.41-0.49 deg, so that assumption is good to ~0.5 deg.
"""

import glob
import math
import os
import re
import statistics as st
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sgc_log  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, ".."))

# The plane, the surface geometry helpers and the -e_R sign convention all live
# in sgc_log so metrics.csv and this script cannot disagree about them.
MEASURED_PLANE = sgc_log.MEASURED_PLANE
CONFIGURED_PLANE = sgc_log.CONFIGURED_PLANE
normal_from_tilt = sgc_log.normal_from_tilt
rotate = sgc_log.rotate_vector
angle_between = sgc_log.angle_between_deg

POLE_ORDER = ["m120", "m080", "m040", "p000", "p040", "p080", "p120", "p160"]


def final_tool_axes():
    """{pole_tag: [tool axis in base at the end of set_up, ...]}"""
    n_cfg = normal_from_tilt(*CONFIGURED_PLANE)
    out = {}
    pattern = os.path.join(EXP, "results", "B2_pole_normal_*", "r0*",
                           "*_log.csv")
    for path in sorted(glob.glob(pattern)):
        if "sigma_debug" in path:
            continue
        tag = re.search(r"B2_pole_normal_([mp]\d+)", path).group(1)
        d, _ = sgc_log.read_csv(path, ["phase", "e_R_x", "e_R_y", "e_R_z"])
        idx = np.where(d["phase"] == sgc_log.PHASE_SET_UP)[0]
        if idx.size == 0:
            continue
        i = idx[-1]
        e_R = np.array([d["e_R_x"][i], d["e_R_y"][i], d["e_R_z"][i]])
        # -e_R: see note 1 in the module docstring.
        out.setdefault(tag, []).append(
            rotate(n_cfg.copy(), -e_R, float(np.linalg.norm(e_R))))
    return out


def main():
    a_m = st.mean(p[0] for p in MEASURED_PLANE)
    b_m = st.mean(p[1] for p in MEASURED_PLANE)
    a_s = st.stdev([p[0] for p in MEASURED_PLANE])
    b_s = st.stdev([p[1] for p in MEASURED_PLANE])

    n_cfg = normal_from_tilt(*CONFIGURED_PLANE)
    n_real = normal_from_tilt(a_m, b_m)
    start = angle_between(n_cfg, n_real)

    print(f"measured plane : a = {a_m:+.2f} +/- {a_s:.2f} deg | "
          f"b = {b_m:+.2f} +/- {b_s:.2f} deg   (n={len(MEASURED_PLANE)})")
    print(f"configured     : a = {CONFIGURED_PLANE[0]:+.2f} deg | "
          f"b = {CONFIGURED_PLANE[1]:+.2f} deg")
    print(f"the tool is aimed at the configured normal, so it reaches contact "
          f"{start:.2f} deg off the real plane\n")

    axes = final_tool_axes()
    if not axes:
        sys.exit("no B2 runs found under experiments/results/")

    print(f"{'pole mm':>8} {'residual to real':>20} {'improvement':>14} "
          f"{'spread over planes':>20}")
    improvements = {}
    for tag in POLE_ORDER:
        if tag not in axes:
            continue
        vals = [angle_between(v, n_real) for v in axes[tag]]
        mean = float(np.mean(vals))
        sd = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
        improvements[tag] = start - mean

        alt = []
        for a_i, b_i in MEASURED_PLANE:
            n_i = normal_from_tilt(a_i, b_i)
            s_i = angle_between(n_cfg, n_i)
            alt.append(s_i - float(np.mean(
                [angle_between(v, n_i) for v in axes[tag]])))

        print(f"{tag:>8} {mean:13.2f} +/-{sd:4.2f} {start - mean:+13.2f} "
              f"{max(alt) - min(alt):19.2f}")

    if "p000" in improvements and "p160" in improvements:
        print(f"\n+160 mm versus pole-on-edge: "
              f"{improvements['p160'] - improvements['p000']:+.2f} deg")


if __name__ == "__main__":
    main()
