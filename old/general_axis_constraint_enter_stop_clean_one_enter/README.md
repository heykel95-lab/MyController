# General Axis-Constraint Controller Based on Clean e+Enter Base

This version is based on `impedance_no_friction_coriolis_switch_enter_stop_clean`.

It keeps:

```text
no friction compensation
Coriolis switch
e + Enter stop
q_goal_7 = 0.785398
```

and adds selectable fixed/free axes for translation and rotation.

## Stop

During the run, type:

```text
e
```

and press Enter.

## Time mode

```text
experiment_duration = 0.0
```

runs indefinitely until `e + Enter`.

```text
experiment_duration = 8.0
```

runs for 8 seconds, or stops earlier with `e + Enter`.

## Translation constraints

In `parameters.txt`:

```text
fix_p_x = 1
fix_p_y = 0
fix_p_z = 0
```

means:

```text
x fixed
y free
z free
```

This creates a virtual Y-Z plane.

Examples:

```text
fix_p_x = 0
fix_p_y = 1
fix_p_z = 0
```

creates an X-Z plane.

```text
fix_p_x = 0
fix_p_y = 0
fix_p_z = 1
```

creates an X-Y plane.

Fixing two axes creates a virtual line. Fixing all three axes gives normal return-to-point impedance.

## Rotation constraints

```text
fix_R_x = 1
fix_R_y = 1
fix_R_z = 1
```

keeps all rotational stiffness components.

If you set:

```text
fix_R_y = 0
```

then the y component of the orientation error is set to zero, so there is no rotational spring in that component.

Damping still acts if `DR_y > 0`. For a fully free rotational component use:

```text
fix_R_y = 0
DR_y = 0.0
```

## Build

```bash
make clean
make
make check
make run
```


## One startup Enter version

This version asks for only one confirmation at startup.

After the single Enter it performs:

```text
1. automatic error recovery if needed
2. collision threshold setup
3. move to initial joint configuration
```

There is no second Enter before moving to the initial joint configuration.
