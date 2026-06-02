# Simplified Compliance Delta Test

This project changes the desired position definition.

Instead of using an absolute desired position, the code reads the initial end-effector position and computes:

```text
p_d = p_start + delta_p
```

This is closer to the idea used in your friend's code.

## Main settings

In `simplified_compliance_delta_test.cpp`:

```cpp
Eigen::Vector3d delta_p(0.02, 0.00, 0.00);
const double experiment_duration = 3.0;
double K1_p = 100.0;
double K2_p = 100.0;
double K3_p = 100.0;
```

CSV logging is still inside the callback, as requested.

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
./simplified_compliance_delta_test
```

## Output

The terminal prints:

- initial position `p_start`,
- desired position `p_d = p_start + delta_p`,
- final reached position,
- final position error,
- CSV filename.

## Safety

For the first test, keep `delta_p` small, for example:

```cpp
Eigen::Vector3d delta_p(0.02, 0.00, 0.00);
```

This means a 2 cm displacement in the base-frame x-direction.
