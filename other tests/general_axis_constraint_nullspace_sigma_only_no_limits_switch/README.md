# General Axis Constraint Controller — Sigma-Only Nullspace

This version removes `nullspace_k_start` completely.

There is no return-to-start joint posture term anymore.

## Why

The return-to-start term conflicted with the desired free motion in the virtual plane. When the robot was pulled down in the Y-Z plane, the joint-return term made it feel like the robot wanted to go back to the start pose.

The controller now keeps the free-plane behavior and only uses the nullspace for the singular-value optimization.

## Kept features

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

## Nullspace parameters

```text
use_nullspace_optimization = 1
nullspace_damping = 1.0
nullspace_k_sigma = 0.5
nullspace_alpha = 0.01
nullspace_tau_max = 1.0
```

There is no `nullspace_k_start` parameter in this version.

## Torque command

With Coriolis and nullspace enabled:

```text
tau_cmd = J^T * F + tau_nullspace + coriolis
```

where `tau_nullspace` contains only damping and sigma-min optimization, not return-to-start.

## Build

```bash
make clean
make
make check
make run
```


## No-limits switch version

Based on `general_axis_constraint_nullspace_sigma_only`.

Default:
```text
disable_controller_side_limits = 0
```

Set to `1` only for careful testing to bypass the controller-side software
limiters `f_max`, `m_max`, and `delta_tau_max`.

This does not disable Franka internal safety/reflex/collision/joint/torque limits.
