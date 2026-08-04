#!/usr/bin/env python3
"""Check the hold disturbance actually applied in each Case F run.

  python3 experiments/analysis/disturbance_quality.py [run_dir ...]

With no arguments it walks every archived MAIN_F run.

New runs log the automatic link-point force and its equivalent joint torque,
so their waveform is checked directly. Archived hand-push runs fall back to
the null-space self-motion and cue timing used by the original protocol.

The automatic input is not an end-effector wrench. It is the joint-torque
equivalent of a force at a configured link point, which can excite the
redundant direction while the end-effector hold remains active.

Reported per run:

  push start    first applied automatic force, or first sustained hand motion
  motion end    end of the automatic release ramp, or last hand motion
  excursion     how far the arm travelled along the redundant axis, as the
                integral of its projected speed. This is the disturbance size,
                and it is the quantity that varied when the elbow was harder to
                move on some trials.
  recovery from the time from which an analysis may treat the arm as released.
  force / tau   peak applied values for an automatic disturbance.
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
        force_cols = [idx.get(f"disturbance_force_base_{axis}")
                      for axis in "xyz"]
        tau_cols = [idx.get(f"tau_disturbance_{joint}")
                    for joint in range(1, 8)]
        scale_col = idx.get("disturbance_scale")
        torque_scale_col = idx.get("disturbance_torque_scale")
        automatic = (scale_col is not None and
                     all(col is not None for col in force_cols + tau_cols))
        rows = []
        for row in reader:
            if len(row) <= idx["nullspace_speed"]:
                continue
            try:
                values = [float(row[idx["time"]]),
                          abs(float(row[idx["nullspace_speed"]]))]
                if automatic:
                    force = np.array([float(row[col]) for col in force_cols])
                    tau = np.array([float(row[col]) for col in tau_cols])
                    values += [float(row[scale_col]),
                               float(np.linalg.norm(force)),
                               float(np.linalg.norm(tau)),
                               float(row[torque_scale_col])]
                else:
                    values += [0.0, 0.0, 0.0, 1.0]
                rows.append(values)
            except ValueError:
                continue
    if not rows:
        return None
    a = np.array(rows)
    return dict(time=a[:, 0], speed=a[:, 1], scale=a[:, 2],
                force=a[:, 3], tau=a[:, 4], torque_scale=a[:, 5],
                automatic=bool(a[:, 2].max() > 1e-6))


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
    t, speed = data["time"], data["speed"]
    push_cue, hold_cue, release_cue = cue_times(run_dir)
    if push_cue is None or release_cue is None:
        return None

    if data["automatic"]:
        active = data["scale"] > 1e-3
        full = data["scale"] > 0.99
        start = float(t[np.flatnonzero(active)[0]])
        end = float(t[np.flatnonzero(active)[-1]])
        driven = (t >= start) & (t <= end)
        excursion = (float(np.trapz(speed[driven], t[driven]))
                     if driven.any() else 0.0)
        return dict(
            run_dir=run_dir, no_motion=False, automatic=True,
            push_cue=push_cue, release_cue=release_cue,
            start=start, end=end, early=push_cue - start,
            overshoot=end - release_cue,
            excursion=excursion, recovery_from=end,
            run_end=float(t[-1]),
            force_peak=float(data["force"].max()),
            tau_peak=float(data["tau"].max()),
            torque_scale_min=float(data["torque_scale"][active].min()),
            reached_full=bool(full.any()),
        )

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
        run_dir=run_dir, no_motion=False, automatic=False,
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

    print(f"{'run':<34}{'start':>7}{'end':>7}{'excur':>8}"
          f"{'force':>8}{'tau':>8}{'rec from':>9}  notes")
    for r in rows:
        label = "/".join(r["run_dir"].split(os.sep)[-2:])
        if r["no_motion"]:
            print(f"{label:<34}{'-':>7}{'-':>7}{'-':>8}{'-':>8}{'-':>8}"
                  f"{'-':>9}  NO MOTION - hand push missed?")
            continue
        notes = []
        if not r["automatic"] and r["early"] > 0.2:
            notes.append(f"started {r['early']:.1f}s early")
        if not r["automatic"] and r["overshoot"] > 0.3:
            notes.append(f"still moving {r['overshoot']:.1f}s past the hold cue")
        if r["automatic"] and not r["reached_full"]:
            notes.append("waveform never reached full scale")
        if r["automatic"] and r["torque_scale_min"] < 0.999:
            notes.append(f"torque limit active ({r['torque_scale_min']:.2f})")
        force = r.get("force_peak", float("nan"))
        tau = r.get("tau_peak", float("nan"))
        print(f"{label:<34}{r['start']:>7.1f}{r['end']:>7.1f}"
              f"{r['excursion']:>8.3f}{force:>8.2f}{tau:>8.2f}"
              f"{r['recovery_from']:>9.1f}"
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
