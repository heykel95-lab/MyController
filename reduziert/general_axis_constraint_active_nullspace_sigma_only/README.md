# General Axis Constraint Controller — Active-Task Sigma-Only Nullspace

This version is based on the active/reduced Jacobian controller:

```text
general_axis_constraint_active_nullspace_return.zip
```

but `nullspace_k_start` was removed completely.

## What was removed

There is no return-to-start joint posture term anymore:

```text
nullspace_k_start
```

does not exist in this code or in `parameters.txt`.

## What remains

The controller still uses the reduced/active task Jacobian:

```text
J_active = rows of J that correspond to the fixed axes only
```

For example:

```text
fix_p = [1 0 0]
fix_R = [1 1 0]
```

means:

```text
J_active = rows x, Rx, Ry
```

The nullspace term now contains only:

```text
1. damping inside the active-task nullspace
2. optional sigma_min optimization inside the active-task nullspace
```

## Torque command

With Coriolis and nullspace enabled:

```text
tau_cmd = J^T * F + tau_active_nullspace + coriolis
```

## Parameters

```text
use_nullspace_optimization = 1
nullspace_damping = 1.0
nullspace_k_sigma = 0.5
nullspace_alpha = 0.01
nullspace_tau_max = 1.0
active_nullspace_lambda = 0.05
```

## Kept from the base

```text
general axis constraint mode
fix_p_x / fix_p_y / fix_p_z
fix_R_x / fix_R_y / fix_R_z
one startup Enter
e + Enter stop
no friction compensation by default
Coriolis switch
q_goal_7 = 0.785398
```

## Build

```bash
make clean
make
make check
make run
```
