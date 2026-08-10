#!/usr/bin/env python3
"""Accept or reject one archived contact trial before the next one runs.

Usage:
  python3 experiments/analysis/validate_contact_trial.py RUN_DIR

Cases J and K enter regimes the campaign has not been in. J commands the first
negative tilts, so the arm approaches contact leaning a way it never has and
the surface-frame sign conventions meet their mirror for the first time. K
sweeps the lever out to 80 mm, a third longer than anything archived, which is
a third more commanded moment at the same press.

The runner already refuses to continue a case when a trial fails to archive.
That only catches a trial that crashed. This catches one that completed and is
still wrong: the wrong tilt sign at first contact, a press that did not press,
a load outside the band the protocol says to stop at, or a set-up phase that
ended while the tool was still moving. Any of those means the remaining trials
of the sweep would repeat a mistake seventeen more times unattended.

Every threshold below is a protocol bound or a published campaign value, not a
number tuned to a result. This gate is not a hypothesis test: an unexpected
outcome inside these bounds -- an assisting lever that assists less than
predicted, a long lever that overshoots -- is a result and passes.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sgc_log  # noqa: E402
from extract_metrics import read_params  # noqa: E402

# The protocol in COC_SIGN_PILOT.md stops the operator above 70 N. The MAIN
# campaign commands 60 mm at 800 N/m and settles near 50 N, so a press that
# ends far below that did not reach the plane the way every archived run did.
MIN_STEADY_LOAD_N = 30.0
MAX_STEADY_LOAD_N = 70.0
MAX_PEAK_LOAD_N = 90.0

# First contact keeps most, not all, of the commanded tilt: the approach hands
# over at its settled residual, which the descend then works against. Measured
# over the 66 archived single-axis MAIN contact runs, |commanded| - |first
# contact| lies between +0.63 and +3.43 deg, and it is an offset rather than a
# fraction -- 5 deg commands lose 1.63 deg on average and 10 deg commands lose
# 2.34, not twice as much.
#
# So the bound is absolute. A fractional one would have to be loose enough to
# pass a 5 deg command keeping 47% of itself, which is what MAIN_A1_t1_05deg
# actually did, and would then be far too loose at 10 deg. Case K commands
# 5 deg conditions, so this distinction decides whether half of it can run.
MAX_APPROACH_LOSS_DEG = 4.5
MAX_FIRST_CONTACT_EXCESS_DEG = 1.5

# extract_metrics flags a run not-converged when the last 20% of the phase
# still contributes more than 10% of the rotation. That threshold is right for
# deciding whether one number enters a thesis mean, and too tight for deciding
# whether seventeen more trials may run: the campaign's largest correction,
# MAIN_D1_t1_rc_t2_m060/r01, drifts 0.63 deg over a 6.16 deg rotation and
# carries the flag, and Case J's job is to reproduce that condition mirrored.
# The gate is looser on purpose. A trial between the two bounds runs, archives,
# and is flagged for the analysis to exclude, which is what the flag is for.
MAX_DRIFT_FRACTION = 0.25
REPORTING_DRIFT_FRACTION = 0.10

# A lever that drives the excited axis further from flat than it started is
# the sweep going the wrong way. Case D's wrong-sign levers removed nothing;
# none made the error worse. Two degrees is well outside that.
MAX_WORSENING_DEG = 2.0


def commanded_tilt_deg(run_dir):
    """The tilt this trial actually ran, from its own archived parameters."""
    params = read_params(os.path.join(run_dir, "params_effective"))
    out = {}
    for axis, key in ((1, "tool_target_offset_tangent1_deg"),
                      (2, "tool_target_offset_tangent2_deg")):
        try:
            out[axis] = float(params[key])
        except (KeyError, TypeError, ValueError):
            return None
    return out


def excited_axis(tilt):
    """The axis the trial tilts about, or None if it commands both or neither."""
    active = [axis for axis, deg in tilt.items() if abs(deg) > 1e-6]
    return active[0] if len(active) == 1 else None


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: validate_contact_trial.py RUN_DIR")
    run_dir = sys.argv[1]

    general = None
    for name in sorted(os.listdir(run_dir)):
        if name.endswith(".csv") and "sigma_debug" not in name:
            general = os.path.join(run_dir, name)
            break
    if general is None:
        print("contact-trial gate: FAIL -- no general log in the archive")
        return 2

    m = sgc_log.setup_metrics(general)
    if not m.get("setup_present"):
        print("contact-trial gate: FAIL -- the trial never reached the press")
        return 2

    tilt = commanded_tilt_deg(run_dir)
    axis = excited_axis(tilt) if tilt else None

    checks = []
    print(f"contact trial {os.path.relpath(run_dir)}")

    steady = m.get("force_steady_N", float("nan"))
    peak = m.get("force_max_N", float("nan"))
    print(f"  steady load:        {steady:.2f} N")
    print(f"  peak load:          {peak:.2f} N")
    checks.append((steady >= MIN_STEADY_LOAD_N,
                   f"steady load {steady:.2f} N < {MIN_STEADY_LOAD_N:.0f} N -- "
                   f"the press did not reach the plane"))
    checks.append((steady <= MAX_STEADY_LOAD_N,
                   f"steady load {steady:.2f} N > {MAX_STEADY_LOAD_N:.0f} N -- "
                   f"the protocol stops here"))
    checks.append((peak <= MAX_PEAK_LOAD_N,
                   f"peak load {peak:.2f} N > {MAX_PEAK_LOAD_N:.0f} N"))

    tip = m.get("tip_final_deg", 0.0)
    drift = m.get("tip_drift_last20pct_deg", 0.0)
    print(f"  tip / last-20% drift: {tip:.2f} / {drift:.2f} deg")
    checks.append((not (tip > 0.0 and drift > MAX_DRIFT_FRACTION * tip),
                   f"still moving at the end of set-up: {drift:.2f} deg over "
                   f"the last 20% of a {tip:.2f} deg rotation"))
    if tip > 0.0 and drift > REPORTING_DRIFT_FRACTION * tip:
        print(f"  note: extract_metrics will flag this run not-converged "
              f"({drift / tip:.0%} of the rotation in the last 20%). The "
              f"sweep continues; the flag excludes it from the means.")

    if axis is None:
        print("  commanded tilt:     not a single-axis trial, sign check skipped")
    elif not m.get("has_alignment_components"):
        print("  commanded tilt:     no alignment components logged, "
              "sign check skipped")
    else:
        commanded = tilt[axis]
        before = m.get(f"align_t{axis}_before_deg", float("nan"))
        after = m.get(f"align_t{axis}_after_deg", float("nan"))
        print(f"  commanded t{axis}:        {commanded:+.2f} deg")
        print(f"  t{axis} at first contact: {before:+.2f} deg "
              f"(logged residual, opposite in sign to the command)")
        print(f"  t{axis} at end of set-up: {after:+.2f} deg")
        # The logged component is the residual to the target, so it opposes
        # the command by construction: across all 66 archived single-axis MAIN
        # contact runs, align_t*_before_deg has the opposite sign to
        # tool_target_offset_tangent*_deg, without exception.
        #
        # That relation is what Case J actually leans on. A mirrored command
        # that arrives at contact leaning the way the positive runs did means
        # the command, the plane frame or the tool frame is not mirroring with
        # it, and every later J trial would inherit the mistake.
        checks.append((before * commanded < 0.0,
                       f"first contact residual {before:+.2f} deg does not "
                       f"oppose the {commanded:+.2f} deg command -- the sign "
                       f"did not mirror"))
        loss = abs(commanded) - abs(before)
        print(f"  approach loss:      {loss:+.2f} deg "
              f"(archive: +0.63 to +3.43)")
        checks.append((loss <= MAX_APPROACH_LOSS_DEG,
                       f"first contact lost {loss:.2f} deg of the commanded "
                       f"tilt, more than the {MAX_APPROACH_LOSS_DEG:.1f} deg "
                       f"the approach has ever eaten"))
        checks.append((loss >= -MAX_FIRST_CONTACT_EXCESS_DEG,
                       f"first contact exceeds the command by "
                       f"{-loss:.2f} deg -- more than the command explains"))
        checks.append((abs(after) <= abs(before) + MAX_WORSENING_DEG,
                       f"set-up left the excited axis {abs(after) - abs(before):+.2f} "
                       f"deg worse than first contact"))

    failures = [message for passed, message in checks if not passed]
    if failures:
        for message in failures:
            print(f"  FAIL: {message}")
        print("contact-trial gate: FAIL -- the rest of the case stays blocked")
        return 2
    print("contact-trial gate: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
