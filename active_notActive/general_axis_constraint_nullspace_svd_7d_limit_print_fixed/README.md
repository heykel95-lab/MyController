# General Axis Constraint Controller with Nullspace SVD Optimization

Built from the base:

```text
general_axis_constraint_enter_stop_clean_one_enter.zip
```

## Kept from the base

```text
general axis constraint mode
fix_p_x / fix_p_y / fix_p_z
fix_R_x / fix_R_y / fix_R_z
one startup Enter
e + Enter stop during impedance control
no friction compensation by default
Coriolis switch
q_goal_7 = 0.785398
```

## New nullspace/SVD feature

The 6x7 Jacobian is decomposed with SVD:

```text
J = U * Sigma * V^T
```

The nullspace direction is:

```text
n = V.col(6)
```

The controller compares:

```text
J(q + alpha*n)
J(q - alpha*n)
```

and selects the direction with the larger smallest singular value:

```text
sigma_min(J)
```

It also projects the return-to-start joint error onto the nullspace direction:

```text
n^T * (q_start - q)
```

## Torque command

```text
tau_cmd = J^T * F + tau_nullspace + coriolis
```

If `use_coriolis = 0`:

```text
tau_cmd = J^T * F + tau_nullspace
```

## Important parameters

```text
use_nullspace_optimization = 1
nullspace_k_start = 3.0
nullspace_damping = 1.0
nullspace_k_sigma = 0.5
nullspace_alpha = 0.01
nullspace_tau_max = 2.0
```

## Build and run

```bash
make clean
make
make check
make run
```


## 7D nullspace torque limiter fix

The previous compile error came from calling the 3D Cartesian vector limiter

```cpp
limitVectorNorm(...)
```

with the 7D joint torque vector `tau_nullspace`.

This version uses a dedicated 7D limiter:

```cpp
limitJointTorqueVectorNorm(tau_nullspace, params.nullspace_tau_max)
```

so the nullspace torque can be limited correctly in joint space.


## Terminal output fix

The terminal now prints the correct torque command mode, including the nullspace term when enabled:

```text
tau_cmd = J^T * F + tau_nullspace + coriolis
```

Depending on the switches, it prints the corresponding variant:

```text
use_coriolis = 1, use_nullspace_optimization = 1:
tau_cmd = J^T * F + tau_nullspace + coriolis

use_coriolis = 0, use_nullspace_optimization = 1:
tau_cmd = J^T * F + tau_nullspace  (NO coriolis)

use_coriolis = 1, use_nullspace_optimization = 0:
tau_cmd = J^T * F + coriolis

use_coriolis = 0, use_nullspace_optimization = 0:
tau_cmd = J^T * F  (NO coriolis, NO nullspace)
```
