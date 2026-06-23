# Surface Constraint Impedance Controller

This controller uses a surface/contact constraint plus a separate alignment
target frame for the tool orientation experiment.

The constrained surface is defined by:

```text
n^T * (p - p_surface) = 0
```

where `n` is the active position-surface normal in the robot base frame.

## Surface Mode

```text
constraint_enabled = 1
```

The alignment target can also be set like a surface normal. In the flat-table
self-alignment experiment, this can intentionally be a tilted tool-orientation
target, not the real table normal:

```text
Y-Z plane:       alignment_target_normal = [1, 0, 0]
X-Z plane:       alignment_target_normal = [0, 1, 0]
X-Y plane:       alignment_target_normal = [0, 0, 1]
45 degree plane: alignment_target_normal = [0.7071, 0, 0.7071]
```

The alignment-target/task frame is:

```text
R_alignment_target = [normal tangent1 tangent2]
```

Only `alignment_target_tangent1` is entered by the user. The code computes `tangent2`
automatically from `normal x tangent1`.

Task-frame gains are defined in that frame:

```text
Kp_normal / Dp_normal     = normal direction
Kp_tangent1 / Dp_tangent1 = first tangent direction
Kp_tangent2 / Dp_tangent2 = second tangent direction
```

The code transforms them to the robot base frame:

```text
K_base = R_alignment_target * K_task * R_alignment_target^T
D_base = R_alignment_target * D_task * R_alignment_target^T
```

## Rotation

The rotational constraint flags are interpreted in the alignment-target frame:

```text
constrain_rotation_about_alignment_normal   = rotation around target normal
constrain_rotation_about_alignment_tangent1 = rotation around first tangent
constrain_rotation_about_alignment_tangent2 = rotation around second tangent
```

These components are angle-axis orientation-error components, not yaw/pitch/roll.

The physical tool axis is
configured in the end-effector frame:

```text
R_desired * tool_axis_ee = tool_axis_target_sign * alignment_target_normal
```

## Nullspace

The nullspace term combines return-to-start posture regulation, damping, and a small singular-value improvement term:

```text
tau_null = N * (k_start * (q_start - q) - d_null * dq)
           + N * (k_sigma * alpha * sign * n)
```

## Contact Search

Optional contact search can set the virtual surface point automatically:

```text
use_contact_search = 1
```

The translational search phase uses estimated external force only as a contact
trigger, not as force control. On contact, the current TCP position becomes the
runtime surface point. In `post_contact_align`, the controller keeps pressing
the contacted edge into the surface with a stiff downward spring while holding
a very soft rotational spring (target = orientation at first contact); a real
contact moment at the pressed edge should passively tip the tool flat instead
of the controller scripting the rotation. A moment-threshold or time-based
exit then hands off to normal surface impedance.

## Build And Run

```bash
make
./general_axis_constraint_nullspace_sigma_only_open_collision
```

Stop during impedance mode with:

```text
e + Enter
```
