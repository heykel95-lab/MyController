# Experiment workflow 2

This file is the execution companion for the alternative thesis chapter
`Results and Discussion 2`. It does not replace `../experiments/README.md`,
change the existing alignment metric, or overwrite recorded results. It
selects the remaining experiments, gives their order, and defines when a run
is admissible.

The existing workflow remains the source of truth:

```text
../experiments/setups/<run_id>/        generated setup overlay and acceptance note
../experiments/run.sh                  run and archive one repetition
../experiments/results/<run_id>/rNN/   raw logs and exact provenance
../experiments/derived/metrics.csv     one validated row per archived run
../experiments/figures/                generated thesis figures
../experiments/analysis/               extraction and plotting programs
```

The controller already logs `alignment_angle_deg`. Do not add a second
alignment column. The analysis also already rescales alignment against the
measured physical plane.

## 1. Safety and pre-run checks

Do not begin a contact or hold experiment while the tool can slip in the
gripper.

1. Support or remove the tool before homing the hand.
2. Home the empty hand with `./tools/home_gripper`.
3. Confirm that the reported `max_width` is close to the physical hand range,
   not the stale approximately 15 mm value observed before homing.
4. Set `gripper_grasp_width` to the measured tool width at the finger contact
   faces. It is an object-width target, not the fully open width.
5. Confirm `grasped flag : yes` and gently verify the grip while supporting
   the tool. Stop if the tool translates or rotates between the fingers.
6. Check the workpiece fixture, emergency stop, collision thresholds, tool
   geometry and free workspace.

The present experiment wrapper cannot prove that the mechanical grip remains
valid. A controller run completing normally does not make data from a loose
tool admissible.

Before the first run of a session:

```bash
cd /home/hm-panda/Desktop/MyController
git status --short --branch
make -C surface_grinding_controller
./surface_grinding_controller/tools/measure_plane
python3 experiments/analysis/extract_metrics.py
```

The existing `measure_plane` tool is a legacy orientation diagnostic based on
hand-seating the tool face. It does not perform the three-point calibration
defined below. Use it only to compare with old data. Do not mix a new
three-point calibration with old results without preserving the effective
plane points and command-angle offsets in each run folder.

## 2. Current archive boundary

At the creation of this file, `metrics.csv` contains 92 archived records:
89 are usable for their recorded quantities and three carry data-quality
faults. The completed core contact groups are:

```text
G1 decoupled/coupled command gate        8 / 8
G3 repeatability                         5 / 5
A2 rotational-stiffness sweep           15 / 15
B1--B4 compliance-centre placement      57 / 57
```

G2 has one run at each timeout but was planned for three repetitions at each
timeout. The null-space campaign is incomplete: C2 has only one usable
mode-0 run and C4 has no usable `k_sigma` run. Do not write a quantitative
null-space conclusion until the sequence below is complete.

## 3. Correctly separate the physical plane from the commanded tool angle

Do this before recording another contact campaign. The current controller uses
`alignment_target_normal` for two different things:

```text
physical plane normal used for height and projection
commanded tool-axis direction used during approach
```

That coupling prevents the physical plane from being calibrated correctly
while intentionally approaching it at a different angle. The replacement
should define the physical plane from three measured, non-collinear contact
points:

```text
u  = p2 - p1
v  = p3 - p1
nS = normalize(u x v)
pS = (p1 + p2 + p3) / 3
t1 = normalize(u)
t2 = nS x t1
```

Flip `nS` using an explicit normal-direction hint so it always points toward
free space. Reject the calibration when `|u x v|` is too small or the points
are too close together.

The three recorded positions must be physical probe/contact points, not raw TCP
positions. For a calibrated probe offset `c_probe_EE`, record

```text
p_i = p_EE_i + R_EE_i * c_probe_EE
```

Otherwise changing wrist orientation moves the TCP even when the same surface
point is touched.

Use `nS` and `pS` for:

```text
signed height above the surface
clearance transition
projection of the contact point onto the plane
surface task frame
physical alignment_angle_deg
```

Define the commanded tool normal separately as an offset relative to the
measured plane:

```text
nCommand = Rot(t2, delta_t2) * Rot(t1, delta_t1) * nS
```

The desired tool axis follows `nCommand`; the alignment metric continues to
measure the tool axis against `nS`. A zero offset therefore commands a flat
tool, while a nonzero offset creates the intentional mismatch without moving
the plane.

Before accepting a calibration, run these proposed checks:

| ID | Repetitions | Purpose | Acceptance |
|---|---:|---|---|
| `P0_plane_repeatability` | 5 calibrations | Repeat the same three points | normal spread below 0.25 deg and point-height spread below 0.5 mm |
| `P1_plane_heldout_points` | 5 points | Touch points not used in the fit | absolute signed residual below 1 mm |
| `P2_plane_workspace_locations` | 3 per location | Repeat approach at centre and two separated locations | clearance switch and final force agree inside the G3 floor |

These IDs are proposed experiment definitions, not yet generated setup
directories. Do not run them until the controller has separate physical-plane
and command-normal parameters.

## 4. Replace the 12 s default with a focused settling-time study

For the nominal push, the commanded ramp lasts approximately 4.0 s. Re-reading
the existing 8 s and 12 s traces at 5.0 s gives:

| Trace | Difference between 5 s and final tip | Alignment difference | Force difference |
|---|---:|---:|---:|
| existing 8 s run | 0.006 deg | 0.017 deg | 0.09 N |
| existing 12 s run | 0.003 deg | 0.002 deg | 0.12 N |

All are far below the G3 repeatability floor. A 5 s phase is therefore a
reasonable candidate for the current nominal push and is preferable to
waiting 12 s in every run.

Do not make 5 s a universal constant. The ramp duration changes when
`setup_push_end`, the captured start height or `setup_push_speed` changes. In
particular, a slow A5 trial can still be ramping at 5 s. Define timing as

```text
nominal ramp time = abs(setup_push_end - captured_start) / setup_push_speed
candidate stop    = ramp completion + 1.0 s settling dwell
hard timeout      = ramp completion + 2.0 s
```

The stronger implementation is a convergence exit after ramp completion:

```text
abs(alignment-rate) < 0.02 deg/s
abs(force-rate)     < 0.5 N/s
conditions held continuously for 0.5 s
```

Keep the derived hard timeout as a fallback. This normally ends near 5 s for
the current parameters but remains valid when push depth or speed changes.

Validate the shorter time after the three-point plane is active:

| ID | Timeout | Repetitions |
|---|---:|---:|
| `T1_settle_t4p5` | 4.5 s | 3 |
| `T1_settle_t5p0` | 5.0 s | 5 |
| `T1_settle_t6p0` | 6.0 s | 3 |
| `T1_settle_t8p0_reference` | 8.0 s | 3 |

Accept 5 s when its final alignment, force and tip agree with both 6 s and 8 s
inside the G3 noise floor, and every 5 s run meets the rate thresholds above.
There is no need to repeat 12 s if those checks pass.

## 5. Better contact-alignment experiments with the calibrated plane

After plane and timing validation, prioritise these over broad one-factor
sweeps:

1. **Signed command-angle sweep.** Hold the physical plane fixed and command
   offsets of `0`, `+/-5` and `+/-10` deg about each surface tangent. This
   tests whether contact correction depends on mismatch magnitude and sign.
2. **Local compliance-centre confirmation.** Record a 3x3 grid around the
   fitted pole optimum, approximately `t1 = [45, 60, 75] mm` and
   `t2 = [15, 30, 45] mm`, three repetitions per point. The existing B-series
   is a cross; this grid tests the two-dimensional optimum directly.
3. **Controller comparison at matched geometry.** At `0`, `5` and `10` deg
   initial mismatch, compare decoupled impedance, the pole on the contact edge,
   and the locally optimal pole. Keep push, stiffness, plane and timing
   identical.
4. **Location generalisation.** Repeat the best controller at three separated
   points on the calibrated plane. A correct plane should give the same signed
   height and similar alignment response across the workspace.
5. **Contact-edge sign check.** Deliberately reverse which tool edge touches
   first and test mirrored pole offsets. This is stronger evidence for the
   lever-sign interpretation than inferring the edge from the tool model.

The first three are the minimum useful additions. The last two establish
whether the result is geometric rather than specific to one contact location.

## 6. Record the core null-space comparison

### Manual hold protocol

Use the same physical sequence for every C-series run:

1. Start hold and release the robot completely.
2. Wait for the initial stationary segment.
3. Before each displaced trial, release the arm and press `p`+Enter to
   recapture the external-torque baseline.
4. Push to the intended displaced configuration, then release it.
5. Allow approximately 20 s of unassisted recovery.
6. Stop normally with `e`+Enter so both buffered logs are written.

Do not press `p` while applying hand force. That would capture the hand load
as the new baseline.

One of the two archived mode-0 runs lacks its general log. Preserve it as an
audit record and add two new valid repetitions:

```bash
./experiments/run.sh C2_hold_mode0_off 3
./experiments/run.sh C2_hold_mode0_off 4
```

Record three repetitions for the other modes:

```bash
for id in \
  C2_hold_mode1_damping \
  C2_hold_mode2_sigma \
  C2_hold_mode3_both
do
  for repeat in 1 2 3; do
    ./experiments/run.sh "$id" "$repeat" || break
  done
done
```

A sigma-enabled run is not successful merely because
`sigma_end - sigma_start` is positive. It must also satisfy:

```text
mean speed_toward_better > 0
Cartesian position-error drift < 1 mm
Jacobian/null-direction residual near numerical zero
no collision/reflex interruption
complete general and sigma-debug logs
```

Compare modes 0 and 1 to distinguish natural drift from damping. Compare modes
0 and 2 to isolate the active conditioning bias. Mode 3 shows the combined
behavior but cannot by itself identify which term caused a change.

## 7. Tune the conditioning torque before the probe

The two existing `C4_ksigma_0p5` folders are not usable for a three-run mean.
Preserve them and add three valid repetitions with new indices:

```bash
for repeat in 3 4 5; do
  ./experiments/run.sh C4_ksigma_0p5 "$repeat" || break
done
```

Then record the remaining torque settings:

```bash
for id in C4_ksigma_1p0 C4_ksigma_2p0 C4_ksigma_4p0; do
  for repeat in 1 2 3; do
    ./experiments/run.sh "$id" "$repeat" || break
  done
done
```

Select the smallest `k_sigma` that repeatedly produces motion toward the
selected direction. Reject a larger value if Cartesian drift exceeds 1 mm,
the robot reaches a collision/reflex threshold, or peak torque grows without
a corresponding improvement in `sigma_min`.

Inspect which joints were commanded and which actually moved:

```bash
python3 experiments/analysis/nullspace_joints.py
```

If sigma torque is present but projected joint velocity remains near zero,
the setting is below the effective friction/task threshold. Do not describe a
non-moving probe as optimization.

## 8. Probe, deadband, posture and inverse sensitivity

Run C5 only after selecting an admissible torque magnitude from C4:

```bash
for id in C5_alpha_0p02 C5_alpha_0p04 C5_alpha_0p08 C5_alpha_0p16; do
  for repeat in 1 2 3; do
    ./experiments/run.sh "$id" "$repeat" || break
  done
done
```

Select `alpha` from consistent sign selection and positive recovery, not from
the largest raw difference between the two probes. A large finite probe moves
farther along the instantaneous tangent and is not an exact step on the
constant-pose self-motion manifold.

Then run the deadband check:

```bash
for repeat in 1 2 3; do
  ./experiments/run.sh C6_deadband_at_optimum "$repeat" || break
done
```

Near a local optimum, the two probe values should fall inside the deadband and
the active sigma torque should switch off rather than chatter.

Test the selected setting across postures:

```bash
for id in \
  C8_qinit_horizontal \
  C8_qinit_tilted_tool \
  C8_qinit_tilted_close \
  C8_qinit_table
do
  for repeat in 1 2 3; do
    ./experiments/run.sh "$id" "$repeat" || break
  done
done
```

Finally, run the relative SVD cutoff sensitivity:

```bash
for id in C9_svdtol_1em03 C9_svdtol_1em04 C9_svdtol_1em05; do
  for repeat in 1 2 3; do
    ./experiments/run.sh "$id" "$repeat" || break
  done
done
```

The C9 comparison is informative only if the selected posture brings a
singular value close enough to a cutoff to change which values are inverted.
The implementation uses `1/sigma_i` for retained singular values; the
relative threshold decides retention, not the reciprocal formula.

## 9. Extract, validate and plot after every batch

```bash
cd /home/hm-panda/Desktop/MyController
python3 experiments/analysis/extract_metrics.py
python3 experiments/analysis/make_figures.py
python3 experiments/analysis/nullspace_joints.py
```

Read every emitted flag. Data flags such as `no-general-log`,
`not-converged`, `tip-mismatch` and `task-disturbed` exclude a run from means.
`dirty-tree` is a provenance warning and is admissible only because the exact
effective parameters are archived.

The null-space figure becomes thesis-ready only when all four C2 modes have
three admissible repetitions. Until then, the `Results and Discussion 2`
chapter must retain its explicit incomplete-evidence statement.

Copy regenerated figures into the thesis only after inspecting them:

```bash
cp experiments/figures/A2_stiffness_sweep.pdf \
  /home/hm-panda/Desktop/MyOwn-thesis/figures/
cp experiments/figures/B_pole_component.pdf \
  /home/hm-panda/Desktop/MyOwn-thesis/figures/
cp experiments/figures/B_pole_surface.pdf \
  /home/hm-panda/Desktop/MyOwn-thesis/figures/
cp experiments/figures/C2_nullspace_modes.pdf \
  /home/hm-panda/Desktop/MyOwn-thesis/figures/
cp experiments/figures/G2_equilibrium.pdf \
  /home/hm-panda/Desktop/MyOwn-thesis/figures/
```

Do not copy `C2_nullspace_modes.pdf` into the final merged chapter while it is
based on incomplete or excluded runs.

## 10. What must be done manually

- Reset and refixture the workpiece between repetitions.
- Support and inspect the gripped tool; software cannot certify friction at
  the fingers.
- Follow the hold push/release timing consistently.
- Record the three physical plane points and all held-out validation points,
  including the calibrated probe offset used to convert TCP pose to contact
  position.
- Observe which physical tool edge contacts first and record that observation
  in the run notes.
- Inspect every analysis flag and plot before using it in the thesis.
- Commit and push the controller results after each accepted batch.
- Commit the thesis separately after updating the alternative chapter.

Suggested controller commit sequence:

```bash
cd /home/hm-panda/Desktop/MyController
git status --short
git add experiments surface_grinding_controller/EXPERIMENT_WORKFLOW_2.md
git commit -m "Record next thesis experiment batch"
git push
```

Raw CSV files are intentionally ignored because of their size. The small
metrics table, figures and provenance files are tracked. Verify that every raw
run remains present locally or in the project data backup before relying on
the derived table alone.
