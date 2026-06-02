# General Axis-Constraint Cartesian Impedance Controller

This controller generalizes the previous virtual Y-Z plane.

You can choose which translational axes are fixed and which are free.

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

So this creates a virtual Y-Z plane.

Other examples:

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

You can also choose which rotational error components are fixed:

```text
fix_R_x = 1
fix_R_y = 1
fix_R_z = 1
```

A fixed rotational component keeps the rotational spring. A free rotational component sets the corresponding orientation error component to zero.

Important: damping still acts if `DR_i > 0`. For a fully free rotational component, set both:

```text
fix_R_i = 0
DR_i = 0.0
```

## Default

Default is the Y-Z plane:

```text
fix_p_x = 1
fix_p_y = 0
fix_p_z = 0
```

with all rotational components fixed:

```text
fix_R_x = 1
fix_R_y = 1
fix_R_z = 1
```

and:

```text
q_goal_7 = 0.785398
```

## Build and run

```bash
make clean
make
make check
make run
```
