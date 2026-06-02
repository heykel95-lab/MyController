# Fixed Franka Compliance Projects Using Makefile

This ZIP contains two separate projects. Both use the same Makefile style as the working project and link directly to:

```text
/home/hm-panda/libfranka/build/libfranka.so.0.7
```

This avoids the incompatible CMake-found libfranka version.

## Projects

### 1. simplified_compliance_test

Position-only compliance test.

Build and run:

```bash
cd simplified_compliance_test
make clean
make
make check
make run
```

Expected `make check` result should include:

```text
/home/hm-panda/libfranka/build/libfranka.so.0.7
```

### 2. full_cartesian_compliance

Full Cartesian compliance controller with position and orientation control.

Build and run:

```bash
cd full_cartesian_compliance
make clean
make
make check
make run
```

Expected `make check` result should include:

```text
/home/hm-panda/libfranka/build/libfranka.so.0.7
```

## Important

Both codes use:

```cpp
#include "examples_common.h"
```

The Makefile finds this header using:

```makefile
-I/home/hm-panda/libfranka/examples
```

The Makefile also compiles:

```text
/home/hm-panda/libfranka/examples/examples_common.cpp
```

because it is needed for:

- `setDefaultBehavior(robot)`
- `MotionGenerator`

## Robot IP

The robot IP is fixed in both C++ files:

```cpp
const std::string robot_ip = "172.16.0.2";
```

## Safety

Before running:

- Make sure the robot workspace is clear.
- Keep the emergency stop reachable.
- Start with low stiffness values.
- Make sure the desired position is safe and reachable.
