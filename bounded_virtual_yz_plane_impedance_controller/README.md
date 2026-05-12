# Bounded Virtual Y-Z Plane Cartesian Impedance Controller

This version is the safer next step after the unbounded virtual Y-Z plane.

## Goal

The end-effector should move mainly inside a virtual Y-Z plane:

```text
x direction: constrained
y direction: allowed, but bounded
z direction: allowed, but bounded
```

## Why this version is needed

The previous virtual Y-Z plane allowed:

```cpp
p_d(1) = p_EE(1);
p_d(2) = p_EE(2);
```

so y and z were completely free. This can let the robot move too far in the plane and approach joint limits.

This bounded version adds workspace limits.

## Control logic

The controller uses:

```cpp
p_d(0) = p_start(0);
p_d(1) = clamp(p_EE(1), y_min, y_max);
p_d(2) = clamp(p_EE(2), z_min, z_max);
```

where:

```text
y_min = y_start - y_limit_neg
y_max = y_start + y_limit_pos

z_min = z_start - z_limit_down
z_max = z_start + z_limit_up
```

## Meaning

Inside the allowed Y-Z box:

```text
e_y = 0
e_z = 0
```

so the robot can move freely in the plane.

Outside the allowed Y-Z box:

```text
e_y != 0 or e_z != 0
```

so the controller pushes back to the nearest boundary.

## Default workspace

```text
y_limit_neg = 0.060 m
y_limit_pos = 0.060 m
z_limit_down = 0.060 m
z_limit_up = 0.040 m
```

So the robot can move about ±6 cm in y, 6 cm downward, and 4 cm upward from the start pose.

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

## What to check

For the virtual plane itself, look mainly at:

```text
e_x
```

It should stay small.

For workspace safety, check that final desired y/z do not go far beyond the start pose bounds.

## Important

This is still a soft virtual constraint. Push gently first. The controller does not replace robot safety limits.
