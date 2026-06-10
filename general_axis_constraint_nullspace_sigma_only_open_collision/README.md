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

The gains are defined in the surface frame:

```text
Kp_x / Dp_x = normal direction
Kp_y / Dp_y = first tangent direction
Kp_z / Dp_z = second tangent direction
```

The code transforms them to the robot base frame:

```text
K_base = R_surface * K_surface * R_surface^T
D_base = R_surface * D_surface * R_surface^T
```

## Rotation

`fix_R_x/y/z` are also interpreted in the surface frame:

```text
x = rotation around surface normal
y = rotation around first tangent
z = rotation around second tangent
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

The estimated external force is used only as a contact trigger, not as force control.

## Build And Run

```bash
make
./general_axis_constraint_nullspace_sigma_only_open_collision
```

Stop during impedance mode with:

```text
e + Enter
```
