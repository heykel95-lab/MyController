# Fixed Compliance Makefile Projects v2

This version adds:

- automatic stop after `experiment_duration` seconds,
- terminal output after the experiment,
- final reached end-effector position,
- final desired position,
- final position error,
- CSV logging inside the callback.

Both projects still link to the local working libfranka 0.7 build:

```text
/home/hm-panda/libfranka/build/libfranka.so.0.7
```

## Simplified positional compliance test

```bash
cd simplified_compliance_test
make clean
make
make check
make run
```

## Full Cartesian compliance controller

```bash
cd full_cartesian_compliance
make clean
make
make check
make run
```

`make check` should show:

```text
/home/hm-panda/libfranka/build/libfranka.so.0.7
```

## Output

After the experiment, the terminal prints for example:

```text
Experiment finished.
Desired position p_d [m]:        ...
Final reached position p_EE [m]: ...
Final position error e_p [m]:    ...
Final position error norm [m]:   ...
CSV log written to: ...
```

## Important

The desired position is still an absolute position in the robot base frame:

```cpp
Eigen::Vector3d p_d(0.45, 0.00, 0.35);
```

Make sure this position is reachable and safe.
