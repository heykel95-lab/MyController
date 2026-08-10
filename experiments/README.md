# Thesis experiment workflow

Setup file → run → archived result → metrics → figure. Everything here is
self-contained: **numpy and matplotlib only**, because `pandas`, `scipy` and
`pip` are not installed on this machine and the analysis has to run where the
data lives.

```
experiments/
  setups/<run_id>/overlay.txt   parameter keys that differ from params/
  setups/<run_id>/plane_profile.txt  required physical plane, when applicable
  setups/<run_id>/about.txt     what the run tests, what counts as a pass
  setups/INDEX.txt              generated setup and repeat index
  run.sh                        run one setup, archive it with provenance
  lib/generate_setups.py        regenerates setups/ from one spec
  lib/apply_overlay.py          applies an overlay onto params/
  analysis/sgc_log.py           CSV loader + metric extraction
  analysis/extract_metrics.py   results/ -> derived/metrics.csv
  analysis/make_figures.py      derived/metrics.csv -> figures/*.pdf
  results/<run_id>/rNN/         raw logs (gitignored) + provenance (tracked)
  derived/metrics.csv           one row per run (tracked)
  figures/*.pdf                 thesis figures (tracked)
```

## Running one experiment

```bash
cd ~/Desktop/MyController
./experiments/run.sh B2_pole_normal_p080 1
```

The script backs up `params/`, applies the overlay, starts the controller,
archives both CSVs plus the effective parameters, git commit and terminal
transcript into `results/B2_pole_normal_p080/r01/`, then restores `params/`.
Drive the controller exactly as usual; stop with `e`+Enter.

## Main calibrated-plane study

The `MAIN_*` experiments keep the physical plane and commanded tool orientation
separate. The full case matrix and conventions are in
[`AXIS_STUDY.md`](AXIS_STUDY.md). Measure four physical probe points in base
coordinates: P1--P3 fit the plane and P4 validates it. Use fit points at least
50 mm apart. Mark the
\(+X_{\mathrm{EE}},+Y_{\mathrm{EE}}\) corner of the rectangular tool face and
touch that same corner to every point. With the robot stationary at each
point, run:

The primary campaign uses the named `horizontal` profile:

```bash
cd surface_grinding_controller
make capture_plane_point
./tools/capture_plane_point horizontal P1
./tools/capture_plane_point horizontal P2
./tools/capture_plane_point horizontal P3
./tools/capture_plane_point horizontal P4
cd ..
python3 experiments/calibration/prepare_plane_calibration.py horizontal
```

`capture_plane_point` is read-only and commands no robot motion. It converts
the EE pose to the configured tool-corner position and appends it to the
selected profile's `plane_points.csv`. If a different physical probe is used,
enter its calibrated base-frame coordinates in that profile's
`plane_points.example.csv` instead.

The script fits the plane from P1--P3, uses P1-to-P2 as \(+t_1\), checks P4,
and writes a profile-local overlay and report. Every calibrated setup declares
its profile in `plane_profile.txt`; `run.sh` refuses missing, mismatched or
failed calibration and archives the profile with the result.

The physical grinding-face normal must then be calibrated independently from
the point geometry. Place the complete face flat, change yaw about the plane
normal by roughly 20--45 degrees between poses, and capture T1--T4:

```bash
cd surface_grinding_controller
make capture_tool_axis
./tools/capture_tool_axis grinding_tool horizontal T1
./tools/capture_tool_axis grinding_tool horizontal T2
./tools/capture_tool_axis grinding_tool horizontal T3
./tools/capture_tool_axis grinding_tool horizontal T4
cd ..
python3 experiments/calibration/prepare_tool_axis_calibration.py grinding_tool
```

The estimator finds the EE-frame axis whose base-frame image is invariant
across the different yaw poses; the fitted plane normal is only a separate
sanity check. The runner requires the held-out tool-axis error to be at most
0.5 degrees and sufficient yaw observability. Recalibrate after any tool
regrasp that may change the mounting orientation.

Run the zero-offset control first:

```bash
./experiments/run_axis_study.sh status
./experiments/run_axis_study.sh next
```

Do not continue if the zero-offset first-contact alignment angle is not close
to zero or if a held-out plane point lies more than 1 mm from the fitted plane.
The guided runner executes one robot trial at a time, selects the next missing
repeat, verifies the exact gain matrices without connecting to the robot,
archives the trial through `run.sh`, and refreshes metrics and plots.

### Sign symmetry and lever magnitude (Cases J and K)

Cases D, E and H fix the direction of the compliance-centre lever but leave two
things open: they only ever commanded a tilt leaning one way, and 60 mm was the
only lever length ever run.

```bash
./experiments/run_axis_study.sh case J auto    # 18 trials, run this first
./experiments/run_axis_study.sh case K auto    # 42 trials
```

Both run unattended through the same `lib/auto_drive.py` path as Case F, with
no per-case support needed: the driver reads `startup_mode.txt` and answers `s`
for a contact case, and the stored `grinding_tool` mount profile answers the
only other prompt. Every J and K trial is checked by
`analysis/validate_contact_trial.py` before the next one starts, and the case
stops on the first failure — that check is what makes them safe to leave
running. Drop `auto` to stop between trials instead.

**Case J** mirrors every Case D coarse condition: the tilt is negated and all
three levers \(-60,0,+60\,\mathrm{mm}\) are kept, so the prediction that the
lever must reverse with the tilt can fail. `MAIN_J_sign_symmetry.pdf` draws the
Case D curve mirrored, as the prediction, under the measured negative-tilt one.

**Case K** holds the assisting sign and sweeps the length,
\(|r_{c,t}|\in\{20,40,60,80\}\,\mathrm{mm}\) against \(\theta\in\{5,10\}^\circ\)
on both axes, so the lever can be reported as \(\rho^\star(|\theta_0|)\)
instead of as one number. The \(10^\circ/60\,\mathrm{mm}\) corner is not re-run:
`MAIN_D1_t1_rc_t2_m060` and `MAIN_D2_t2_rc_t1_p060` already measured it at these
gains and the figure reads them in. Run `MAIN_K1_*` first if time is short.

Run J before K — K commands only the assisting sign, which is the rule J tests.
Within J the zero-lever runs come first, so the mirrored tilt is pressed without
a lever before one is added to it. Predictions for both cases are pre-registered
in
[`PREDICTIONS.md`](PREDICTIONS.md); the design is in
[`AXIS_STUDY.md`](AXIS_STUDY.md).

### Automatic nullspace study

Case F holds the end-effector while applying the same smooth point force at
link 3 for every nullspace mode. The controller derives the base-frame force
direction once at the initial pose, logs the applied force and joint torque,
and enforces the configured torque-norm limit.

```bash
make -C surface_grinding_controller inspect_nullspace_disturbance
./surface_grinding_controller/tools/inspect_nullspace_disturbance \
  surface_grinding_controller/params \
  experiments/setups/MAIN_F0_baseline/overlay.txt
./experiments/run_axis_study.sh case F auto
python3 experiments/analysis/disturbance_quality.py
python3 experiments/analysis/make_nullspace_figure.py
```

The inspector is optional and read-only. Run it only when the arm is already
at the Case F start pose; otherwise it describes the current pose instead.
The automatic push is an internally commanded equivalent of an external link
force, not a force generated by a second robot-control process.

The active follow-on keeps the original +100 mm Case-F records unchanged. It
applies 20 N at a +200 mm link-3 point, uses a 3.0 Nm disturbance-torque
ceiling, and tests sigma torque only at
1.5 and 2.0 Nm. Run its mode-0 hardware gate first:

```bash
./experiments/run.sh PILOT_F_disturbance_20N_200mm 1
python3 experiments/analysis/validate_nullspace_pilot.py \
  experiments/results/PILOT_F_disturbance_20N_200mm/r01
./experiments/run_axis_study.sh case F auto
```

The pilot delivered the full 20 N waveform without torque clipping, exceeded
2.0 Nm equivalent joint-torque norm and 0.040 rad redundant excursion, and
stayed within 2 mm peak Cartesian position error. The completed follow-on
contains three repetitions each of the mode-0 reference, projected damping at
2.0 N m s/rad, and sigma-only control at 1.5 and 2.0 N m. The 1 N m sigma
setting was omitted because the first campaign showed that it mostly loaded
joint friction. The earlier 4 N m record remains a single screening run and is
not part of the final matched campaign.

The rejected 40 N, +200 mm pilot is not scheduled for repetition. It reached
the 5 Nm torque ceiling, reduced the force waveform, and exceeded the Cartesian
error criterion.

The MAIN campaign uses a 60 mm virtual penetration with
`setup_Kp_surface_normal = 800 N/m`. The resulting quasi-static command is
48 N. The two loose-gate A0 diagnostics measured 24.33 N at 360 N/m; together
with the earlier 180 mm pilot, this supports an expected settled load close to
50 N at 800 N/m. The set-up window remains 5 s, allowing the short preload
ramp to settle before evaluation. Automatic damping is recomputed for the
higher stiffness at phase entry.

The MAIN overlays explicitly retain the reachable 2 degree
approach-orientation transition. A tested 0.5 degree threshold did not permit
descent because the mounted system settled near 1.5 degrees in free space.
With the 2 degree transition, the continued orientation control during
descent reduced the EE-inferred A0 first-contact residual to approximately
1 degree. The measured first-contact value, rather than the nominal command,
is used in analysis.

The calibrated `tool_axis_ee` assumes a rigid transform between the Franka EE
and the physical grinding face. Mark the tool and holder with a witness line
and reject a run if the marks move. A tool that can rotate by approximately
2 degrees relative to the EE cannot be treated as a fixed calibration
uncertainty: motion during contact is invisible to the robot pose and would be
mistaken for, or hide, alignment. Mechanically constrain that rotation before
the MAIN campaign and recalibrate T1--T4 after the final regrasp. The runner
asks for the mount state before every calibrated trial. Type `rigid` only for
a fixed mount. If work must continue temporarily, type `play2` to record the
known upper bound; the run is flagged `tool-play`, excluded from primary plot
means, and retained for exploratory EE-response analysis.

The separately calibrated `tilted` profile is used only for the compact
frame-transfer validation after the horizontal primary campaign:

```bash
./experiments/run_tilted_validation.sh status
./experiments/run_tilted_validation.sh next
```

**The backup is not optional.** It guarantees that no setup overlay contaminates
the next trial. Restoration runs from a shell trap, so Ctrl-C and libfranka
reflex exits are also covered.

## After a batch

```bash
python3 experiments/analysis/extract_metrics.py   # -> derived/metrics.csv
python3 experiments/analysis/make_figures.py      # -> figures/*.pdf
```

`extract_metrics.py` cross-checks every CSV-derived value against the
controller's own printed set-up report and flags disagreements. Flagged runs
are drawn hollow and excluded from means in the figures — a run that did not
converge must not quietly enter a thesis table.

Flags currently emitted:

| Flag | Meaning |
|---|---|
| `not-converged` | Tip still moving >10% of its final value over the last 20% of the phase. The number is a transient, not an equilibrium. |
| `task-disturbed` | Cartesian position drifted >2 mm during a hold — the null-space projector is not task-invariant. |
| `tool-play` | The operator declared up to 2° of unobserved tool motion relative to the EE. Exploratory only; excluded from primary physical-alignment means. |
| `tip-mismatch` | CSV disagrees with the controller's printed report. One of them is wrong; do not use the run. |
| `dirty-tree` | Recorded with uncommitted changes — provenance is incomplete. |

## Order to run things

Gates first. They are cheap and everything downstream depends on them.

1. `G1_decoupled_baseline` (×5) and `G1_coupled_block_diagonal` (×3) —
   the coupled path must reproduce the decoupled wrench. If it does not, every
   coupled result is an artifact.
2. `G2_equilibrium_t4` / `t8` / `t12` — **expected to fail as configured**, see
   the note below.
3. `G3_repeatability` (×5) — the noise floor. Without it no sweep difference
   can be claimed as an effect.

Then the irreducible set if time is short:
`A2_KRtan_*`, `B2_pole_normal_*`, `C2_hold_mode*`.

## Known issue found in the existing data

The archived run in `surface_grinding_controller_log.csv` shows the set-up
phase ending at **3.999 s against a `setup_timeout` of 4.0 s** — it stopped on
the timeout, not on convergence — with the tip still rising at **+0.34 deg/s**
and a final tip of only 1.41°. The last 20% of the phase contributed 19% of the
total rotation.

So the current configuration does not reach equilibrium, and any number taken
from it is a transient. That is what G2 is designed to expose. Raising
`setup_timeout` is the obvious first fix.

Related: the nominal `coupled_pole_from_edge = [-0.04, 0.08, 0]` resolves to
only **−3.5 mm along the surface normal** (the rest is tangential). The
historical campaign swept the pole along the *normal* and found its optimum at
+80 mm; a near-zero normal offset is close to the pole-on-edge case that
historically *suppressed* alignment to 0.6°. This is why `B2` sweeps
`s * n` computed from the tilt angles rather than hand-edited x/y/z.

## Manual steps you have to do yourself

These cannot be scripted from here:

1. **Physically reset the workpiece** between repeats, otherwise `G3` measures
   the setup's drift rather than the controller's repeatability.
2. **`B6_pole_on_measured_axis`** — edit its `overlay.txt` and paste the
   `axis_from_edge` values from the best `B2` run before running it. The setup
   ships with a zero placeholder.
3. **Hold tests (`C*`)** — drive the push-and-release protocol by hand: start
   the hold, release the arm, `p`+Enter to recapture, push to a displaced
   configuration, release, then let it recover ~20 s. Release the arm *before*
   pressing `p` or the torque baseline captures your hand force.
4. **Commit results.** Raw CSVs are gitignored (15 MB each); `metrics.csv`,
   the provenance files and the figures are tracked. After a batch:
   `git add experiments && git commit`.
5. **Copying figures into the thesis** is deliberately not automated — decide
   which ones earn a place first.

## Regenerating the setup matrix

`setups/` is generated, not hand-written:

```bash
python3 experiments/lib/generate_setups.py
```

Edit the `SPEC` list in that file to add or change runs. The pole offsets are
computed from the configured surface tilt, so they stay correct if the tilt
changes.
