# Simplified Impedance Fixed Like Friend

This project applies the fixes from the working friend project.

## Fixes

1. No gravity compensation is added.

The command is:

```text
tau_cmd = tau_task + coriolis
```

not:

```text
tau_cmd = tau_task + gravity + coriolis
```

2. CSV logging is buffered.

Inside the callback, data are stored in a vector.  
After the control loop finishes, the CSV file is written.

3. The controller stops automatically using `franka::MotionFinished`.

4. Safe first test mode is available.

In `parameters.txt`:

```text
use_current_pose = 1
```

This means the robot uses its current end-effector pose as the desired pose and should only hold its pose.

5. Smooth relative motion is available.

Set:

```text
use_current_pose = 0
```

Then the controller moves smoothly from:

```text
p_start
```

to:

```text
p_start + delta_p
```

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
./simplified_impedance_fixed
```

## Recommended first test

Keep this in `parameters.txt`:

```text
use_current_pose = 1
```

After this works, test a small motion:

```text
use_current_pose = 0
delta_p_x = 0.005
trajectory_duration = 8.0
```

## CSV

The CSV file is written after the experiment.

Default file:

```text
simplified_impedance_fixed_log.csv
```
