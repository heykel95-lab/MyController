# Surface Constraint Impedance Controller

This controller uses one unified surface constraint mode.

The constrained surface is defined by:

```text
n^T * (p - p_surface) = 0
```

where `surface_normal` is the plane normal in the robot base frame.

## Surface Mode

```text
constraint_enabled = 1
```

The same mode covers the old axis-aligned cases and inclined surfaces:

```text
Y-Z plane:       surface_normal = [1, 0, 0]
X-Z plane:       surface_normal = [0, 1, 0]
X-Y plane:       surface_normal = [0, 0, 1]
45 degree plane: surface_normal = [0.7071, 0, 0.7071]
```

The surface/task frame is:

```text
R_surface = [normal tangent1 tangent2]
```

Only `surface_tangent1` is entered by the user. The code computes `tangent2`
automatically from `normal x tangent1`.

The gains are defined in the surface frame:

```text
Kp_normal / Dp_normal     = normal direction
Kp_tangent1 / Dp_tangent1 = first tangent direction
Kp_tangent2 / Dp_tangent2 = second tangent direction
```

The code transforms them to the robot base frame:

```text
K_base = R_surface * K_surface * R_surface^T
D_base = R_surface * D_surface * R_surface^T
```

## Rotation

The rotational constraint flags are also interpreted in the surface frame:

```text
constrain_rotation_about_surface_normal   = rotation around surface normal
constrain_rotation_about_surface_tangent1 = rotation around first tangent
constrain_rotation_about_surface_tangent2 = rotation around second tangent
```

These components are angle-axis orientation-error components, not yaw/pitch/roll.

Tool alignment is separate from the surface frame. The physical tool axis is
configured in the end-effector frame:

```text
R_desired * tool_axis_ee = tool_axis_target_sign * surface_normal
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
runtime surface point. Moment comparison is used later in `post_contact_align`,
while the tool rotates after the first contact.

## Build And Run

```bash
make
./general_axis_constraint_nullspace_sigma_only_open_collision
```

Stop during impedance mode with:

```text
e + Enter
```
