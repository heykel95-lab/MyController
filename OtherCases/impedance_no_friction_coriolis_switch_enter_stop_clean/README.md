# Clean e + Enter Stop Version

This version fixes the duplicate stop-block compile error.

## Stop

During the run, type:

```text
e
```

and press Enter.

The controller then exits through:

```cpp
franka::MotionFinished(...)
```

## Time mode

```text
experiment_duration = 0.0
```

runs indefinitely until `e + Enter`.

```text
experiment_duration = 8.0
```

stops after 8 s, or earlier with `e + Enter`.

## Defaults

```text
q_goal_7 = 0.785398
all f_fric_* = 0.0
all m_fric_* = 0.0
```

## Build

```bash
make clean
make
make check
make run
```
