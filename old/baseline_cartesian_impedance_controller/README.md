# Baseline Cartesian Impedance Controller

This is the thesis baseline controller. It is not described as someone else's code.

## Controller law

The baseline controller computes:

```text
f = Kp * (p_d - p_EE) - Dp * p_dot
m = KR * e_R - DR * omega
tau_cmd = J^T * [f; m] + coriolis
```

The desired pose is the current pose after the robot moves to the initial joint configuration.

## What is intentionally not included

This baseline does not include:

```text
f_max
m_max
delta_tau_max
e_thresh
f_fric
m_fric
rho(e) or rho(E)
custom collision-threshold tuning
```

This makes it a clean reference for comparing against the extended controller.

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

## Suggested test

Use the same timing as the extended controller:

```text
0--3 s: do not touch
3--5 s: manual disturbance
5--12 s: release fully
```

Compare:

```text
final position error norm
max position error norm
final rotation error norm
max rotation error norm
reflex/no reflex
```

## Thesis naming

Use:

```text
Baseline Cartesian impedance controller
Extended Cartesian impedance controller
```
