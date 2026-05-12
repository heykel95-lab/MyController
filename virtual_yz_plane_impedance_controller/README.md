# Virtual Y-Z Plane Cartesian Impedance Controller

This version lets you test a **virtual plane** behavior.

The goal is:

```text
x direction: constrained / blocked softly
y direction: allowed
z direction: allowed
```

So the end-effector can be moved mainly inside the **Y-Z plane**.

## Control idea

In normal holding mode, the desired position is fixed:

```text
p_d = p_start
```

In this virtual Y-Z plane mode, the desired position is modified online:

```text
x_d = x_start
y_d = y_current
z_d = z_current
```

This means:

```text
e_x = x_start - x_current
e_y = 0
e_z = 0
```

So the controller creates a restoring force only in x, while y and z are allowed.

## Damping in y and z

Even though y and z are allowed, damping is still active:

```text
Dp_y = 8.0
Dp_z = 8.0
```

This makes the motion feel controlled. If you want freer y-z motion, reduce:

```text
Dp_y
Dp_z
```

## Important

This is a **soft virtual constraint**, not a hard mechanical lock.

If you push very hard in x, the robot will still move in x, but it should resist and return to the virtual plane.

Do not use very high x stiffness at the beginning. The included value is:

```text
Kp_x = 250.0
```

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

## Test procedure

1. First run without touching.
2. Then move the end-effector gently in y and z.
3. Try a small x disturbance and check whether it returns to the plane.

## What to look at

The important final error is mainly:

```text
e_x
```

For a good virtual Y-Z plane, `e_x` should stay small, while y and z are allowed to change.
