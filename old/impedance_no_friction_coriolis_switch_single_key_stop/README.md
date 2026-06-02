# Impedance Controller without Friction Compensation — Single-Key Stop

This version is prepared for professor testing.

## Main properties

- Friction/residual compensation is disabled.
- All `f_fric_*` and `m_fric_*` values are `0.0`.
- Default `q_goal_7 = 0.785398`.
- The final terminal output prints:
  - position error in m and mm
  - rotation error in rad and deg
- Coriolis compensation can be switched on/off.
- The run can be finite or indefinite.
- You can stop manually by pressing `e` once.

## Time mode

In `parameters.txt`:

```text
experiment_duration = 0.0
```

means:

```text
run indefinitely until you press e
```

For a short automatic test:

```text
experiment_duration = 8.0
```

means:

```text
run for 8 seconds, or stop earlier by pressing e
```

## Coriolis switch

```text
use_coriolis = 1
```

uses:

```text
tau_cmd = J^T * F + coriolis
```

```text
use_coriolis = 0
```

uses:

```text
tau_cmd = J^T * F
```

## Build and run

```bash
make clean
make
make check
make run
```

During the run, press:

```text
e
```

to stop safely through `franka::MotionFinished(...)`.

This version uses Linux raw terminal mode with `termios`, so it does not require pressing Enter.
