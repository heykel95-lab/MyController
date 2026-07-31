#!/usr/bin/env python3
"""Check what the operator actually did against what the run asked for.

  python3 experiments/analysis/disturbance_quality.py [run_dir ...]

With no arguments it walks every archived MAIN_F run.

The cue is an instruction, not a measurement. The hand can arrive before the
push cue and can stay on well past the release cue, and neither is visible in
the cue timestamps. This reads the null-space self-motion out of the general
log and reports when the arm was actually being moved, so a trial can be
segmented on what happened rather than on what was asked.

Why the null-space speed and not the external wrench: a push on the elbow is a
joint torque along the redundant direction, and that direction is orthogonal to
the range of J-transpose. No end-effector wrench produces it, so the
model-estimated external wrench is blind to it by construction. The self-motion
it causes is not.

Reported per run:

  push start    first sustained motion along the redundant axis
  motion end    last sustained motion; the hand left at or before this
  overshoot     motion end minus the release cue. Positive means the push
                continued after the cue, and the samples between the two are
                not free recovery.
  excursion     how far the arm travelled along the redundant axis, as the
                integral of its projected speed. This is the disturbance size,
                and it is the quantity that varied when the elbow was harder to
                move on some trials.
  recovery from the time from which an analysis may treat the arm as released.
"""

import csv
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, ".."))

# Below this the arm is drifting or held, not being driven by hand. Chosen
# well above the settled noise and well below the speeds a push produces.
MOTION_RAD_S = 0.02
# Motion must persist this long to count, so a single noisy sample does not
# start or extend the disturbance window.
MIN_RUN_S = 0.10


def read_log(path):
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        need = ["time", "nullspace_speed"]
        for name in need:
            if name not in idx:
                return None
        rows = []
        for row in reader:
            if len(row) <= idx["nullspace_speed"]:
                continue
            try:
                rows.append((float(row[idx["time"]]),
                             abs(float(row[idx["nullspace_speed"]]))))
            except ValueError:
                continue
    if not rows:
        return None
    a = np.array(rows)
    return a[:, 0], a[:, 1]


def cue_times(run_dir):
    """Cue times as configured for this run, from its archived parameters."""
    push = hold = release = None
    # Every parameter file, because the archive keeps them under their own
    # names rather than under a fixed set.
    for path in sorted(glob.glob(
            os.path.join(run_dir, "params_effective", "*.txt"))):
        with open(path) as f:
            for line in f:
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.split("#")[0].strip()
                try:
                    if key == "disturbance_push_time":
                        push = float(value)
                    elif key == "disturbance_hold_time":
                        hold = float(value)
                    elif key == "disturbance_release_time":
                        release = float(value)
                except ValueError:
                    pass
    return push, hold, release


def sustained_windows(t, speed):
    """Contiguous stretches of motion lasting at least MIN_RUN_S."""
    moving = speed > MOTION_RAD_S
    windows, start = [], None
    for i, m in enumerate(moving):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if t[i - 1] - t[start] >= MIN_RUN_S:
                windows.append((t[start], t[i - 1]))
            start = None
    if start is not None and t[-1] - t[start] >= MIN_RUN_S:
        windows.append((t[start], t[-1]))
    return windows


def report(run_dir):
    log = os.path.join(run_dir, "surface_grinding_controller_log.csv")
    if not os.path.isfile(log):
        return None
    data = read_log(log)
    if data is None:
        return None
    t, speed = data
    push_cue, hold_cue, release_cue = cue_times(run_dir)
    if push_cue is None or release_cue is None:
        return None

    windows = sustained_windows(t, speed)
    # Everything from the push cue onward, ignoring settling before it.
    after = [w for w in windows if w[1] > push_cue - 1.0]
    if not after:
        return dict(run_dir=run_dir, no_motion=True, push_cue=push_cue,
                    release_cue=release_cue)

    start = after[0][0]
    end = max(w[1] for w in after
              if w[0] <= release_cue + 5.0) if any(
                  w[0] <= release_cue + 5.0 for w in after) else after[0][1]

    # Distance travelled along the redundant axis while being driven.
    driven = (t >= start) & (t <= end)
    excursion = float(np.trapz(speed[driven], t[driven])) if driven.any() else 0.0

    return dict(
        run_dir=run_dir, no_motion=False,
        push_cue=push_cue, release_cue=release_cue,
        start=start, end=end,
        early=push_cue - start,
        # Driving should stop at the hold cue, not at the release cue. Motion
        # after it is the hand still moving the arm while it was meant to be
        # stationary, which is what blurs the transition.
        overshoot=end - (hold_cue if hold_cue is not None else release_cue),
        excursion=excursion,
        recovery_from=max(end, release_cue),
        run_end=float(t[-1]),
    )


def main():
    dirs = sys.argv[1:]
    if not dirs:
        dirs = sorted(glob.glob(os.path.join(EXP, "results", "MAIN_F*", "r*")))
    rows = []
    for d in dirs:
        r = report(d)
        if r:
            rows.append(r)
    if not rows:
        print("no MAIN_F runs with a general log found")
        return 0

    print(f"{'run':<34}{'start':>7}{'end':>7}{'early':>7}"
          f"{'over':>7}{'excur':>8}{'rec from':>9}  notes")
    for r in rows:
        label = "/".join(r["run_dir"].split(os.sep)[-2:])
        if r["no_motion"]:
            print(f"{label:<34}{'-':>7}{'-':>7}{'-':>7}{'-':>7}{'-':>8}"
                  f"{'-':>9}  NO MOTION - push missed?")
            continue
        notes = []
        if r["early"] > 0.2:
            notes.append(f"started {r['early']:.1f}s early")
        if r["overshoot"] > 0.3:
            notes.append(f"still moving {r['overshoot']:.1f}s past the hold cue")
        print(f"{label:<34}{r['start']:>7.1f}{r['end']:>7.1f}"
              f"{r['early']:>+7.1f}{r['overshoot']:>+7.1f}"
              f"{r['excursion']:>8.3f}{r['recovery_from']:>9.1f}"
              f"  {'; '.join(notes)}")

    good = [r for r in rows if not r["no_motion"]]
    if good:
        ex = np.array([r["excursion"] for r in good])
        sd = ex.std(ddof=1) if len(ex) > 1 else float("nan")
        print(f"\nexcursion rad: mean {ex.mean():.3f}, sd {sd:.3f}, "
              f"min {ex.min():.3f}, max {ex.max():.3f}  (n={len(ex)})")
        print("A trial whose excursion sits far from the others received a "
              "different disturbance, whatever its cue timing says.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
