# Simplified Impedance Fixed with Semi-Automatic Recovery

This project adds a semi-automatic recovery step before `setDefaultBehavior(robot)`.

## What changed

Before applying default behavior, the program now does:

```cpp
robot.automaticErrorRecovery();
setDefaultBehavior(robot);
```

The program asks you to press Enter first, so you can check that:

- the workspace is clear,
- the emergency stop is reachable,
- the robot is safe to recover.

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
./simplified_impedance_fixed_recovery
```

## Recommended first test

In `parameters.txt`, keep:

```text
use_current_pose = 1
```

This makes the robot hold its current pose.

## After a reflex stop

Run the program again. It will ask:

```text
Press Enter to attempt recovery and continue...
```

After you confirm, it calls:

```cpp
robot.automaticErrorRecovery();
```

If automatic recovery fails, recover/unlock manually in Franka Desk.

## Safety

Do not use automatic recovery blindly. Always check the workspace before pressing Enter.
