# Impedance Controller without Friction Compensation — Optional Infinite Run

This version is prepared for professor testing.

## Main properties

- Friction/residual compensation is disabled.
- All `f_fric_*` and `m_fric_*` values are `0.0`.
- The final terminal output prints:
  - position error in m and mm
  - rotation error in rad and deg
- Coriolis compensation can be switched on/off.
- The run can be finite or indefinite.
- You can stop manually by typing `q` and pressing Enter.

## Time mode

In `parameters.txt`:

```text
experiment_duration = 0.0
```

means:

```text
run indefinitely until q + Enter
```

For a short automatic test:

```text
experiment_duration = 8.0
```

means:

```text
run for 8 seconds, or stop earlier with q + Enter
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

During the run, type:

```text
q
```

and press Enter to stop safely through `franka::MotionFinished(...)`.


## Robust keyboard stop version

This version uses Linux `select()` + `read()` instead of `std::cin` for the stop command.

During the run:

```text
q + Enter
```

should print:

```text
Keyboard stop requested. Finishing control loop...
```

and then finish through `franka::MotionFinished(...)`.

If `experiment_duration = 0.0`, the controller runs indefinitely until `q + Enter`.
If `experiment_duration > 0.0`, it stops after that time or earlier with `q + Enter`.
