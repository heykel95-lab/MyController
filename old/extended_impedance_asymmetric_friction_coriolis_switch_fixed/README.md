# Extended Impedance Controller with Coriolis Switch

This version is for testing whether Coriolis compensation should be included explicitly.

## Test switch

In `parameters.txt`:

```text
use_coriolis = 1
```

uses:

```text
tau_cmd = J^T * F + coriolis
```

and:

```text
use_coriolis = 0
```

uses:

```text
tau_cmd = J^T * F
```

## Purpose

You want to check whether adding Coriolis explicitly improves behavior, or whether it behaves like it is already compensated somewhere else.

## Important expectation

For slow manual quasi-static tests, the Coriolis term is usually small because joint velocities are small. Therefore, the difference may be small.

If the robot moves faster, Coriolis compensation can become more relevant.

## Test procedure

Use the same exact protocol for both tests.

Recommended:

1. Set `use_coriolis = 1`.
2. Run 3 single-disturbance tests.
3. Save or rename the CSV files.
4. Set `use_coriolis = 0`.
5. Run 3 single-disturbance tests.
6. Compare:
   - reflex / no reflex
   - final position error norm
   - final rotation error norm
   - smoothness / feeling

## Build

```bash
make clean
make
make check
make run
```

## Notes

This test does not include explicit gravity compensation. The previous gravity test caused a joint velocity violation and should not be used as the final implementation.
