# Safe 6-Axis Horizontal Pose Controller with Optional Collision Threshold Tuning

This version is based on your latest horizontal-pose controller and adds configurable:

```cpp
robot.setCollisionBehavior(...)
```

from `parameters.txt`.

## Important

This only changes Franka contact/collision thresholds. It does **not** change the internal `joint_velocity_violation` limit.

Use it only for testing `cartesian_reflex` behavior, and keep the emergency stop reachable.

## Build

```bash
make clean
make
make check
```

`make check` should show:

```text
/home/hm-panda/libfranka/build/libfranka.so.0.7
```

## Run

```bash
make run
```

## Collision threshold parameters

By default this is disabled:

```text
use_custom_collision_behavior = 0
```

To test it, set:

```text
use_custom_collision_behavior = 1
```

Initial moderate values:

```text
collision_joint_torque_threshold = 30.0
collision_cartesian_force_threshold = 30.0
collision_cartesian_moment_threshold = 20.0
```

If the controller is stable but still stops by `cartesian_reflex`, you can carefully test:

```text
collision_joint_torque_threshold = 35.0
collision_cartesian_force_threshold = 35.0
collision_cartesian_moment_threshold = 25.0
```

Avoid jumping to very high values.

## Your current best controller parameters

This ZIP starts from your latest good balanced setup:

```text
f_fric_x = 1.0
f_fric_y = 2.0
f_fric_z = 0.5

f_max = 8.0
m_max = 1.5
delta_tau_max = 1.0
```

## Suggested test order

1. Run first with:

```text
use_custom_collision_behavior = 0
```

2. Then set:

```text
use_custom_collision_behavior = 1
```

with moderate thresholds.

3. Compare:
   - reflex/no reflex
   - final position error norm
   - final rotation error norm
   - whether the robot feels too aggressive

## Thesis interpretation

If custom thresholds reduce `cartesian_reflex`, write clearly that the robot's contact/collision thresholds were changed for testing, but the internal safety system was not disabled. If `joint_velocity_violation` occurs, these thresholds are not the correct solution.
