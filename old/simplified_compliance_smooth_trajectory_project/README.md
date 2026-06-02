# Simplified Compliance Smooth Trajectory Test

This version fixes the sudden desired-position step.

Instead of jumping directly to:

```text
p_d = p_start + delta_p
```

the desired position is generated smoothly:

```text
p_d(t) = p_start + s(t) * delta_p
```

with:

```text
s(r) = 10 r^3 - 15 r^4 + 6 r^5
r = t / trajectory_duration
```

This gives zero velocity and zero acceleration at the start and end of the desired trajectory.

## Main settings

In `simplified_compliance_smooth_trajectory.cpp`:

```cpp
Eigen::Vector3d delta_p(0.02, 0.00, 0.00);
double K1_p = 100.0;
double K2_p = 100.0;
double K3_p = 100.0;
const double trajectory_duration = 4.0;
const double experiment_duration = 6.0;
```

CSV logging is still inside the callback.

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

or:

```bash
./simplified_compliance_smooth_trajectory
```

## Output

The terminal prints:

- initial position `p_start`,
- final target `p_end`,
- final desired position,
- final reached position,
- final position error,
- CSV filename.

## Safety

For first tests, keep:

```cpp
Eigen::Vector3d delta_p(0.02, 0.00, 0.00);
```

This is a 2 cm motion in base-frame x-direction.

If the robot still triggers a reflex, reduce either:

```cpp
delta_p
```

or the stiffness values, and increase:

```cpp
trajectory_duration
```
