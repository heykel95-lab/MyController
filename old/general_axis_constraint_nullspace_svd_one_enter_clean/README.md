# General Axis Constraint Controller with Nullspace SVD Optimization

This version is rebuilt cleanly from the base:

```text
general_axis_constraint_enter_stop_clean_one_enter.zip
```

It keeps:

```text
general axis constraint mode
fix_p_x / fix_p_y / fix_p_z
fix_R_x / fix_R_y / fix_R_z
one startup Enter
e + Enter stop during impedance run
no friction compensation by default
Coriolis switch
q_goal_7 = 0.785398
```

## New feature: nullspace movement and SVD optimization

The Franka robot has 7 joints and the Cartesian end-effector task has 6 dimensions.
Therefore, there is approximately a one-dimensional nullspace.

The controller computes:

```text
J = U * Sigma * V^T
n = V.col(6)
```

Then it compares:

```text
q_plus  = q + alpha * n
q_minus = q - alpha * n
```

and computes:

```text
sigma_min(J(q_plus))
sigma_min(J(q_minus))
```

The direction with the larger smallest singular value is preferred.

At the same time, the controller projects the joint error:

```text
q_start - q
```

onto the nullspace direction, so the joints try to return toward the initial joint configuration without disturbing the Cartesian task too much.

## Torque command

```text
tau_cmd = J^T * F + coriolis + tau_nullspace
```

If `use_coriolis = 0`, then:

```text
tau_cmd = J^T * F + tau_nullspace
```

## Main parameters

```text
use_nullspace_optimization = 1
nullspace_k_start = 3.0
nullspace_damping = 1.0
nullspace_k_sigma = 0.5
nullspace_alpha = 0.01
nullspace_tau_max = 2.0
```

## Build

```bash
make clean
make
make check
make run
```
