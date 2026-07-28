#!/usr/bin/env python3
"""Per-joint breakdown of what the null-space controller commands and what moves.

  python3 experiments/analysis/nullspace_joints.py [path/to/*sigma_debug.csv ...]

With no arguments it walks experiments/results/ and reports every archived
sigma-debug log.

The question this answers is not "does sigma improve" -- metrics.csv already
carries that -- but "which joints does the controller push, and which of them
actually move". Those are different columns and they come apart:

  n_best_i        the commanded null-space direction, a unit vector in joint
                  space. Where the controller WANTS to go.
  tau_sigma_i     the torque it puts on joint i to get there.
  dqN_i           the null-space component of joint i's actual velocity.
                  Where it ACTUALLY goes.
  tau_ext_delta_i external torque change on joint i since the baseline. With
                  the arm untouched this is essentially what resists the
                  motion, i.e. friction.

A joint with large tau_sigma and near-zero dqN is stuck: torque commanded,
stiction winning. That is the failure mode C4 exists to find, and it is why
the k_sigma sweep matters -- below the threshold the controller is pushing
joints that never move, and every null-space mode looks identical.

The `share` columns are fractions of the total across the seven joints, so
they say where the effort and the motion are concentrated rather than how big
they are in absolute terms.
"""

import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, ".."))

JOINTS = range(1, 8)


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
    keep = [i for i, c in enumerate(header) if c != "event"]
    names = [header[i] for i in keep]
    data = np.genfromtxt(path, delimiter=",", skip_header=1, usecols=keep)
    data = np.atleast_2d(data)
    return {n: data[:, i] for i, n in enumerate(names)}


def joint_block(d, template):
    """Stack the seven per-joint columns matching e.g. 'tau_sigma_{}_Nm'."""
    cols = []
    for j in JOINTS:
        name = template.format(j)
        if name not in d:
            return None
        cols.append(d[name])
    return np.column_stack(cols)


def report(path):
    d = load(path)
    if "sigma_min" not in d or d["sigma_min"].size < 5:
        print(f"  (too few samples) {path}")
        return

    # Only the part where the controller was actually commanding sigma torque.
    active = d.get("k_sigma_Nm", np.zeros(1)) > 0
    if active.sum() < 5:
        print(f"  (no active sigma samples) {os.path.relpath(path, EXP)}")
        return

    n_best = joint_block(d, "n_best_{}")
    tau = joint_block(d, "tau_sigma_{}_Nm")
    dqn = joint_block(d, "dqN_{}_rad_s")
    ext = joint_block(d, "tau_ext_delta_{}_Nm")
    if any(x is None for x in (n_best, tau, dqn)):
        print(f"  (missing per-joint columns) {os.path.relpath(path, EXP)}")
        return

    n_best, tau, dqn = n_best[active], tau[active], dqn[active]
    ext = ext[active] if ext is not None else np.zeros_like(tau)

    dur = d["run_time_s"][active][-1] - d["run_time_s"][active][0]
    sig = d["sigma_min"][active]
    k = float(np.median(d["k_sigma_Nm"][active]))

    print(f"\n{os.path.relpath(path, EXP)}")
    print(f"  k_sigma = {k:.2f} Nm | {dur:.1f} s active | "
          f"sigma_min {sig[0]:.5f} -> {sig[-1]:.5f} ({sig[-1] - sig[0]:+.5f})")

    a_dir = np.abs(n_best).mean(axis=0)
    a_tau = np.abs(tau).mean(axis=0)
    a_dqn = np.abs(dqn).mean(axis=0)
    a_ext = np.abs(ext).mean(axis=0)
    share = lambda v: v / v.sum() if v.sum() > 1e-12 else v  # noqa: E731

    print(f"  {'joint':>6} {'|n_best|':>9} {'dir share':>10} "
          f"{'tau_sigma':>10} {'|dq_null|':>10} {'move share':>11} "
          f"{'|tau_ext|':>10}  verdict")
    s_dir, s_dqn = share(a_dir), share(a_dqn)
    for i, j in enumerate(JOINTS):
        # "Stuck" = the controller puts a meaningful share of its effort into
        # this joint but the joint carries far less of the actual motion.
        stuck = s_dir[i] > 0.10 and s_dqn[i] < 0.5 * s_dir[i]
        verdict = "STUCK" if stuck else ("moves" if s_dqn[i] > 0.10 else "")
        print(f"  {'q' + str(j):>6} {a_dir[i]:9.3f} {100 * s_dir[i]:9.1f}% "
              f"{a_tau[i]:10.3f} {a_dqn[i]:10.5f} {100 * s_dqn[i]:10.1f}% "
              f"{a_ext[i]:10.3f}  {verdict}")

    speed = np.abs(dqn).sum(axis=1)
    print(f"  total null-space speed: mean {speed.mean():.5f} rad/s, "
          f"max {speed.max():.5f}")
    if speed.mean() < 1e-3 and a_tau.sum() > 1e-3:
        print("  >> torque commanded but the arm is not moving: below the "
              "friction threshold at this k_sigma")


def main():
    paths = sys.argv[1:]
    if not paths:
        paths = sorted(glob.glob(os.path.join(
            EXP, "results", "*", "r0*", "*sigma_debug.csv")))
    if not paths:
        sys.exit("no sigma-debug logs found. Run a setup with "
                 "print_sigma_debug = 1 first (C2/C4/C5).")
    for p in paths:
        report(p)


if __name__ == "__main__":
    main()
