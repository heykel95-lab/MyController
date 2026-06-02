# Extended Impedance Controller with Gravity Compensation

This version is based on the latest horizontal-pose collision controller.

## What changed

The torque command now includes model-based gravity compensation:

```text
tau_cmd = J^T * wrench + coriolis + gravity
```

instead of the previous Coriolis-only command:

```text
tau_cmd = J^T * wrench + coriolis
```

In code, this is done with:

```cpp
std::array<double, 7> gravity_array = model.gravity(state);
Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

tau_raw = tau_task + coriolis + gravity;
```

## What stays the same

This version still includes:

```text
custom collision thresholds
horizontal initial pose
f_max
m_max
delta_tau_max
e_thresh
f_fric
m_fric
```

## Why test this

Your experiments showed asymmetric return behavior, especially for negative-direction disturbances and for disturbances with z-motion or rotation. Missing explicit gravity compensation may contribute to this behavior.

This version lets you compare:

```text
Extended controller, Coriolis only
vs.
Extended controller, Coriolis + gravity
```

## Build

```bash
make clean
make
make check
```

`make check` should show the local libfranka 0.7 library.

## Run

```bash
make run
```

## Test procedure

First test without touching:

```text
0--12 s: no touch
```

Then:

```text
0--3 s: no touch
3--5 s: gentle manual disturbance
5--12 s: release fully
```

## Important

Adding gravity compensation can change the robot behavior strongly. Keep the emergency stop reachable.

If the robot feels too strong, reduce:

```text
Kp_x, Kp_y, Kp_z
KR_x, KR_y, KR_z
f_fric_x, f_fric_y, f_fric_z
```

or return to the Coriolis-only version.
