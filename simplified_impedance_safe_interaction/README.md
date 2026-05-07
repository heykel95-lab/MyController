# Simplified Impedance Safe Interaction

This project adds safer interaction limits to the working recovery version.

## New safety additions

1. Cartesian force saturation:

```text
f_max = 8.0
```

2. Cartesian moment saturation:

```text
m_max = 2.0
```

3. Torque-rate limiting:

```text
delta_tau_max = 1.0
```

4. Semi-automatic recovery:

```cpp
robot.automaticErrorRecovery();
```

5. Buffered CSV logging.

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

## Recommended compliance test

Keep:

```text
use_current_pose = 1
experiment_duration = 10.0
Kp_x = 50.0
Kp_y = 50.0
Kp_z = 50.0
Dp_x = 14.0
Dp_y = 14.0
Dp_z = 14.0
f_max = 8.0
m_max = 2.0
delta_tau_max = 1.0
```

Then:
- 0--2 s: do not touch
- 2--5 s: push gently
- 5--10 s: release and observe return

These limits do not disable Franka safety reflexes. If you push too fast or too hard, the robot can still stop by reflex.
