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

Instead of the sequence, the startup menu can pick a plain **hold** at the start
pose. From a hold, `g+Enter` hands the tool over to manual guidance and
`p+Enter` re-captures the pose and restarts the whole sequence from there.

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
frame, and alignment enforces:

```text
R_desired * tool_axis_ee = tool_axis_target_sign * alignment_target_normal
```

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
produces force and translation produces moment. The pole is the only knob in the
adjoint, so sweeping `coupled_pole_from_edge` moves the effective rotation
center. The congruence preserves positive semi-definiteness, so the result is
still a passive spring — the set-up report prints the eigenvalues to confirm it.

Three sources for the 6×6 gains, in priority order:

1. `coupled_use_block_diagonal = 1` — plain block-diagonal set-up gains, no
   coupling. Through the 6×6 path this reproduces the decoupled wrench exactly,
   so it is the sanity check that the path itself is correct.
2. `coupled_pole_manual = 1` — rebuild from `pole = contact edge +
   coupled_pole_from_edge`.
3. otherwise — the saved `coupled_K_tcp` / `coupled_D_tcp` matrices.

After a set-up phase with a valid finite screw axis, the controller measures the
axis the whole tipping motion actually followed (Chasles' theorem, computed from
the start and end pose, so it is not sensitive to per-cycle velocity noise) and
writes the implied `coupled_K_tcp` / `coupled_D_tcp` back into
`params/sequence.txt` for the next run.

## Nullspace

Off during the approach; active from phase 2 onward.

```text
tau_null  = N * (k_start * (q_start - q) - d_null * dq)
tau_sigma = N * (k_sigma * alpha * sign * n)
```

`nullspace_mode` selects posture only, sigma only, both, or off.

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
| `controller_types.h` | `ControlPhase`, `Parameters`, `LogData` |
| `controller_helpers.*` | Math, task frames, screw axis, spatial gains, nullspace |
| `controller_parameters.*` | Parameter-file parsing |
| `controller_startup.*` | Startup menus, gripper actions, manual guidance |
| `controller_report.*` | The one-shot set-up report and coupled-wrench comparison |
| `controller_printing.*` | Phase debug lines and all value formatting |
| `controller_logging.*` | CSV output |

## Build and run

```bash
make
./general_axis_constraint_nullspace_sigma_only_open_collision
```

`make check_tool_offset` builds a read-only helper that prints the configured
`F_T_EE` / `EE_T_K` transforms without commanding any motion.

Stop the controller at any time with `e + Enter`. A bare `Enter` continues past
a phase gate.
