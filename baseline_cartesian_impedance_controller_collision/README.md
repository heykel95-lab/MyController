# Baseline Cartesian Impedance Controller with Custom Collision Thresholds

This is the thesis baseline controller with the same robot-side collision-threshold option as the extended controller.

## Why this version exists

For a fair comparison, both controllers should use the same robot-side Franka collision/contact thresholds.

So this baseline includes:

```cpp
robot.setCollisionBehavior(...)
```

configured from `parameters.txt`.

But it still does **not** include the controller-side extensions:

```text
f_max
m_max
delta_tau_max
e_thresh
f_fric
m_fric
rho(e) or rho(E)
```

## Controller law

```text
f = Kp * (p_d - p_EE) - Dp * p_dot
m = KR * e_R - DR * omega
tau_cmd = J^T * [f; m] + coriolis
```

## Collision threshold parameters

```text
use_custom_collision_behavior = 1
collision_joint_torque_threshold = 30.0
collision_cartesian_force_threshold = 30.0
collision_cartesian_moment_threshold = 20.0
```

These should match the values used in the extended controller.

## Build

```bash
make clean
make
make check
```

## Run

```bash
make run
```

## Thesis comparison

Use:

```text
Baseline Cartesian impedance controller
Extended Cartesian impedance controller
```

Both can now be compared using the same initial pose and the same robot-side collision thresholds. The difference is that the extended controller adds force/moment saturation, torque-rate limiting, and threshold-based friction compensation.
