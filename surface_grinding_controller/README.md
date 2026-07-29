# Approach / Set-Up / Grind Controller

A Cartesian impedance controller for the Franka arm that brings a hand-held tool
down onto a virtual plane, lets contact seat it flat, and then grinds along the
surface.

## Phases

A sequence run walks three phases:

| # | Phase | What it does |
|---|-------|--------------|
| 1 | `approach_orient` → `approach_descend` | Rotate the tool onto the target plane normal, then descend until the active tool contact point is `descend_surface_clearance` above the plane. Both steps share one impedance (`approach_*`). |
| 2 | `set_up` | Press that contact edge into the plane. The rotational spring stays soft on the tipping axes, so real contact moment rotates the tool flat instead of the controller scripting the rotation. |
| 3 | `grind` | Hold the press constant and sweep along a surface tangent. Shares the phase-2 impedance (`setup_*`); only the target law differs. |

The switch from phase 1 to phase 2 is **geometric**, not force-based: it fires
at the clearance height above the configured plane. Estimated external force is
printed during the descent but never decides anything.

The two downward motions have independent trajectory parameters. Free-space
descent uses `descend_speed`, `descend_surface_clearance`, and the
`descend_max_distance` safety guard. At the handoff, set-up captures the actual
contact-point plane coordinate as its start, then ramps toward
`setup_push_end` at `setup_push_speed`. The ramp time is derived from start,
end, and speed; `setup_timeout` is an independent phase limit and may stop the
ramp before its endpoint.

Instead of the sequence, the startup menu can pick a plain **hold** at the start
pose. From a hold, `g+Enter` hands the tool over to manual guidance and
`p+Enter` re-captures the pose and resumes hold from there.

Optional Enter gates (`pause_before_set_up`, `pause_before_grind`) freeze the
run between phases with a stiff position lock until you press Enter.

## Surface plane and task frame

The constrained surface is a plane in the robot base frame:

```text
n^T * (p - p_surface) = 0
```

The plane orientation is set from two tilt angles:

```text
n = R_y(b) * R_x(a) * [0,0,1] = [sin(b)cos(a), -sin(a), cos(b)cos(a)]
```

with `a` the tilt about base x and `b` about base y. The task frame has column
order `[tangent1, tangent2, normal]`; only `alignment_target_tangent1` is
entered, and `tangent2 = normal x tangent1` is computed.

Every `*_tangent1 / *_tangent2 / *_normal` gain triple is written in that frame
and transformed to the base frame as:

```text
K_base = R_alignment_target * K_task * R_alignment_target^T
```

The rotational constraint flags mask angle-axis error components in the same
frame (not yaw/pitch/roll). The physical tool axis is configured in the EE
frame. With zero command offsets, alignment enforces:

```text
R_desired * tool_axis_ee = tool_axis_target_sign * alignment_target_normal
```

For controlled contact-misalignment experiments,
`tool_target_offset_tangent1_deg` and
`tool_target_offset_tangent2_deg` rotate this commanded tool axis relative to
the plane. They do not change the plane point, plane normal, clearance
geometry, surface-frame gains, or physical-plane alignment metric.

## Decoupled vs coupled stiffness

The default control law is **decoupled**: two independent 3×3 springs, one
producing force from position error and one producing moment from rotation
error.

The **coupled** law commands a single 6×6 spring instead, built by moving that
same diagonal spring from a chosen pole out to the TCP through the adjoint:

```text
K_TCP = Ad(r_c)^T * blockdiag(Kp, KR) * Ad(r_c),    r_c = p_TCP - pole
Ad(r_c) = [[I, skew(r_c)], [0, I]]
```

The off-diagonal quadrants of `K_TCP` are the lever coupling: rotation then
produces force and translation produces moment. The direct experiment knob is
`coupled_rc_[tangent1|tangent2|normal]`, with
`r_c = p_TCP - p_c`. The congruence preserves positive semi-definiteness, so
the result remains a valid symmetric spring; the offline preflight and set-up
report print its eigenvalues.

The calibrated campaign also sets `setup_translation_surface_frame = 1`.
Then `setup_Kp_surface_*` and `setup_Dp_surface_*` are interpreted in
`[tangent1,tangent2,normal]` and rotated to the base frame. Leaving the flag at
zero preserves the archived base-XYZ parameterization.

Two sources for the 6×6 gains are available:

1. `coupled_use_block_diagonal = 1` — plain block-diagonal set-up gains, no
   coupling. Through the 6×6 path this reproduces the decoupled wrench exactly,
   so it is the sanity check that the path itself is correct.
2. `coupled_pole_manual = 1` — rebuild from a deliberately commanded lever.
   New experiments set `coupled_use_direct_rc_surface = 1` and specify
   `coupled_rc_tangent1`, `coupled_rc_tangent2`, and `coupled_rc_normal` using
   the convention `r_c = p_TCP - p_c`. Archived setup files may still use the
   legacy base-frame `coupled_pole_from_edge` parameters.

## Nullspace

The projector is built from an SVD Moore-Penrose inverse:

```text
J+    = V * Sigma+ * U^T
N_tau = (I - J+ * J)^T
```

The available nullspace laws are:

```text
off:       tau = 0
damping:   tau = -d_null * N_tau * dq
sigma:     tau = k_sigma * sign * N_tau * n
both:      tau = -d_null * N_tau * dq + k_sigma * sign * N_tau * n
```

The interactive hold menu exposes all four modes: 0 off, 1 damping only,
2 sigma only, and 3 damping plus sigma. This allows the isolated terms and the
damped combined response to be tested from the same captured hold pose.

Here `n` is the one-dimensional 6x7 Jacobian nullspace direction. The sign is
chosen by comparing `sigma_min(q + alpha*n)` with
`sigma_min(q - alpha*n)`. `alpha` is only the sampling step; it does not scale
the commanded sigma torque.

With `print_sigma_debug = 1`, a sigma-enabled hold (mode 2 or 3) prints one
rate-limited tuning block. Its fields mean:

```text
min      current sigma_min
d/dt     measured change of sigma_min between debug lines
probe    abs(sigma_plus - sigma_minus)
C        probe/deadband (C <= 1 disables the sigma push)
|grad|   probe/(2*alpha), for comparing different alpha trials
tau      norm of the commanded sigma torque
vN       absolute nullspace joint speed
vBest    signed speed toward the selected better-sigma direction
nBest    sign-selected unit joint direction that improves sigma_min
dominant joint with the largest absolute nBest component and its squared share
dqN      projected nullspace velocity of q1..q7
moving   joint with the largest absolute dqN component and its squared share
tauS     commanded sigma-torque contribution for q1..q7
||J*nBest|| numerical check that nBest is in the Jacobian nullspace
```

The displayed direction share is `nBest_i^2`, because `nBest` is unit length.
The displayed motion share is `dqN_i^2 / ||dqN||^2`. In the CSV, joint index
zero and `sigma_direction_valid = 0` mean that no better direction was
available. Treat `||J*nBest||` only as a near-zero numerical consistency check;
its translational and rotational Jacobian rows have different physical units.

After releasing a manual push, a useful comeback has `vBest > 0` and normally
`d/dt > 0`. If `C` is frequently at or below 1 while the arm is visibly away
from its better configuration, the alpha probe is not clearly separating the
two directions; near the optimum, `C <= 1` is expected. If `tau` is active but
`vN` stays near zero, `k_sigma` is too weak to overcome the present robot/task
effects. The CSV
contains the raw sigma samples and response values needed to derive the printed
metrics for plotting and comparing short runs. The raw `sigma_direction` sign can flip with the
arbitrary SVD vector sign; use `vBest`, the absolute probe difference, and the
sigma trend to interpret physical behavior. `nBest` includes that sign choice,
so it is the physical better-sigma direction. Its `dominant` joint is only the
largest local component; it does not designate a permanent "nullspace joint."
The terminal block is intended for short diagnostic runs; set
`print_sigma_debug = 0` for timing-sensitive runs and use the buffered CSV
fields instead.

### Compact sigma debug file

Every sigma-enabled hold test (mode 2 or 3) is also buffered into:

```text
surface_grinding_controller_sigma_debug.csv
```

The controller writes this compact file at 20 Hz after a normal stop, before it
writes the much larger general log. There is no file I/O inside the 1 kHz
control callback. After a test, stop with `e+Enter`; the file is then available
in this project folder for direct analysis without copying terminal output. If
a collision/reflex or another control exception ends the test, the partial
compact trace is saved before the exception is reported.

The `event` column separates `hold_start`, `manual_guide_start`, `recapture`,
normal `sample` rows, `stop`, and `exception`. Each `p+Enter` recapture increments
`segment_id` and resets `phase_time_s`, so separate push-and-release trials can
be compared cleanly. The file includes raw `q1..q7` and `dq1..dq7`, the
sign-selected sigma direction and torque, sigma response, Cartesian
position/orientation errors, task torque, external wrench changes, Franka
contact flags, and the peak motion/load observed between 20 Hz samples.

The external joint-torque baseline is captured when hold starts and again at
`p+Enter`. Release the arm before pressing `p`; otherwise the new baseline can
include your hand force. External estimates and contact flags are
evidence of a manual push, not an infallible automatic “hands on” classifier,
so the event boundaries and the known test procedure still matter.

The file name, sample period, and buffer capacity are configured by
`sigma_debug_csv_file_name`, `sigma_debug_log_period`, and
`max_sigma_debug_rows` in `params/common.txt`.

## Parameter files

Read and merged in this order; keys are disjoint.

| File | Contents |
|------|----------|
| `params/common.txt` | Robot/logging, surface plane, tool geometry, q_init poses, nullspace, auto-damping toggles, gripper |
| `params/safety.txt` | Franka collision/reflex thresholds |
| `params/sequence.txt` | The three phases, gates, coupled stiffness. **Owns the auto-written matrices** |
| `params/hold.txt` | Hold-mode gains |
| `params/guidance.txt` | Manual hand-guidance start |

Each phase group can compute its damping online as `D = factor*2*sqrt(M*K)` from
the libfranka task-space inertia (`*_auto_damping`). The manual `Dp`/`DR` values
next to each `K` are the **fallback**, used when the toggle is off or when the
inertia estimate is unavailable that cycle (near a kinematic singularity, or a
non-finite / non-positive result).

## Source layout

| File | Contents |
|------|----------|
| `main.cpp` | Setup, the phase machine, the control law |
| `controller.h` | Shared types plus all module declarations |
| `config.cpp` | Parameters and parsing |
| `control_math.cpp` | Math, task frames, spatial gains, nullspace |
| `runtime_io.cpp` | Startup menus, gripper actions, debug printing, CSV output |
| `setup_report.cpp` | The one-shot set-up report and commanded-pole diagnostics |
| `tools/check_tool_offset.cpp` | Read-only TCP/stiffness-frame inspection tool |

## Build and run

```bash
make
./surface_grinding_controller
```

`make check_tool_offset` builds `tools/check_tool_offset`, a read-only helper
that prints the configured `F_T_EE` / `EE_T_K` transforms without commanding
any motion.

Stop the controller at any time with `e + Enter`. A bare `Enter` continues past
a phase gate.
