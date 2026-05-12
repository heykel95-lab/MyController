# Simplified Impedance Safe 6-Axis Controller — Horizontal Pose Version

This version makes the initial joint configuration tunable from `parameters.txt`.

## What changed

The start pose is read from:

```text
q_goal_1 ... q_goal_7
```

The program also prints the local tool axes expressed in the base frame:

```text
Tool x-axis in base frame
Tool y-axis in base frame
Tool z-axis in base frame
```

This helps you judge whether the tool is aligned well enough for manual axis tests.

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

## Default initial pose

This project starts from:

```text
q_goal_1 = 0.000000
q_goal_2 = -0.785398
q_goal_3 = 0.000000
q_goal_4 = -2.356194
q_goal_5 = 0.000000
q_goal_6 = 1.570796
q_goal_7 = 0.000000
```

The old pose used `q_goal_7 = 0.785398`.

## Test procedure

First run without touching:

```text
0--12 s: no touch
```

Then:

```text
0--3 s: no touch
3--5 s: gentle push
5--12 s: release fully
```

This does not disable Franka safety reflexes.
