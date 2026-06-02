# General Axis Constraint Controller with Active-Task Nullspace Return

This version is intended to reduce the large configuration changes you observed.

## Main change

The previous nullspace used the full 6D Jacobian. This is not ideal when only some axes are fixed, for example:

```text
fix_p = [1 0 0]
fix_R = [1 1 0]
```

In that case the actual constrained task is only:

```text
x, Rx, Ry
```

So this version builds:

```text
J_active = rows of J that correspond to the fixed axes only
```

Then it computes the nullspace projector:

```text
N = I - J_active^T * (J_active * J_active^T + lambda^2 I)^(-1) * J_active
```

and applies joint return to start only through this projector:

```text
tau_active_nullspace =
    N * (K_start * (q_start - q) - D * dq)
```

This should keep the robot much closer to the initial joint configuration while still allowing the free plane motion.

## Default parameters

```text
use_nullspace_optimization = 1
nullspace_k_start = 8.0
nullspace_damping = 2.0
nullspace_k_sigma = 0.0
nullspace_alpha = 0.01
nullspace_tau_max = 4.0
active_nullspace_lambda = 0.05
```

`nullspace_k_sigma` is set to zero by default because your current main problem is not singularity avoidance, but too much configuration drift.

## Torque command

If Coriolis and nullspace are enabled:

```text
tau_cmd = J^T * F + tau_active_nullspace + coriolis
```

## Build

```bash
make clean
make
make check
make run
```
