# Simplified Positional Compliance Test

This project builds a simplified Cartesian positional compliance controller for the Franka Emika Panda.

## Files

- `simplified_compliance_test.cpp`
- `CMakeLists.txt`

The code uses:

```cpp
#include "/home/hm-panda/libfranka/examples/examples_common.h"
```

and the CMake file also compiles:

```text
/home/hm-panda/libfranka/examples/examples_common.cpp
```

This is required for:

- `setDefaultBehavior(robot)`
- `MotionGenerator`

## Build

Open a terminal in this project folder and run:

```bash
mkdir -p build
cd build
cmake ..
make
```

## Run

Make sure the robot is reachable at:

```text
172.16.0.2
```

Then run:

```bash
./simplified_compliance_test
```

The program will first print a warning and wait for Enter.

After pressing Enter:

1. The robot moves to the initial joint configuration.
2. The positional compliance controller starts.
3. The controller writes the log file:

```text
simplified_compliance_test_log.csv
```

## Important safety notes

Before running:

- Make sure the robot workspace is free.
- Keep the emergency stop available.
- Start with low stiffness values.
- Make sure the desired position `p_d` is reachable and safe.

## What the controller does

The controller computes:

```text
f = Kp * e_p - Dp * pdot
```

then builds:

```text
F = [f, 0]^T
```

because rotational control is disabled.

Then it computes:

```text
tau_task = J^T * F
```

and sends:

```text
tau = tau_task + tau_g + tau_c
```

to the robot after torque limiting.

## CSV columns

The CSV file contains:

```text
time,
p_EE_x,p_EE_y,p_EE_z,
p_d_x,p_d_y,p_d_z,
e_p_x,e_p_y,e_p_z,
pdot_x,pdot_y,pdot_z,
f_x,f_y,f_z,
tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7
```
