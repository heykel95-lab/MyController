# Friend-like Simple Impedance Comparison Controller

This project is meant to compare your safer controller with a simpler friend-like compliance controller.

## What this code does

It uses the basic Cartesian impedance law:

```text
f = Kp * (p_d - p_EE) - Dp * p_dot
m = KR * e_R - DR * omega
tau_cmd = J^T * [f; m] + coriolis
```

The desired pose is the current pose after moving to the initial joint configuration.

## What this code does NOT do

Compared to your safer controller, this friend-like version does **not** include:

```text
f_max
m_max
delta_tau_max
e_thresh
f_fric
m_fric
rho(e)
```

So if it stops by reflex more easily, that shows the value of your added safety/compensation terms.

## Terminal output added

At the end it prints:

```text
Final desired position
Final reached position
Final position error
Max position error
Final rotation error
Max rotation error
Final force command
Final moment command
Final torque command
```

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

## Suggested comparison

Use the same push timing as your safer controller:

```text
0--3 s: do not touch
3--5 s: push gently
5--12 s: release fully
```

Then compare:

```text
final position error norm
max position error norm
final rotation error norm
reflex/no reflex
```
