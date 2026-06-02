# Simplified Impedance Safe 6-Axis Friction

This controller is already positional and rotational.

It computes:

```text
f = Kp * e_p + Dp * (pdot_d - pdot)
m = KR * e_R - DR * omega
```

Then it builds:

```text
wrench = [f_x, f_y, f_z, m_x, m_y, m_z]^T
```

and sends:

```text
tau_cmd = J^T * wrench + coriolis
```

So yes: it controls position and orientation.

## New feature

This version adds friction-compensation terms in all 6 axes:

### Position

```text
e_thresh_p_x, e_thresh_p_y, e_thresh_p_z
f_fric_x, f_fric_y, f_fric_z
```

### Rotation

```text
e_thresh_R_x, e_thresh_R_y, e_thresh_R_z
m_fric_x, m_fric_y, m_fric_z
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

## Recommended first values

```text
use_current_pose = 1
Kp_x = 150.0
Kp_y = 100.0
Kp_z = 100.0

KR_x = 3.0
KR_y = 3.0
KR_z = 3.0

f_fric_x = 0.5
f_fric_y = 0.5
f_fric_z = 0.5

m_fric_x = 0.05
m_fric_y = 0.05
m_fric_z = 0.05
```

If the robot becomes too aggressive, reduce `f_fric_*` and `m_fric_*`.

If you only want position compensation, set:

```text
m_fric_x = 0.0
m_fric_y = 0.0
m_fric_z = 0.0
```

If you only want x compensation, set:

```text
f_fric_y = 0.0
f_fric_z = 0.0
m_fric_x = 0.0
m_fric_y = 0.0
m_fric_z = 0.0
```


## Naming

The parameters are now named:

```text
f_fric_x, f_fric_y, f_fric_z
m_fric_x, m_fric_y, m_fric_z
```

instead of:

```text
f_min_x, f_min_y, f_min_z
m_min_x, m_min_y, m_min_z
```

This is clearer for the thesis because these terms are intended to help overcome friction/stiction when the error exceeds a small threshold.


## Threshold naming

The activation thresholds are now named:

```text
e_thresh_p_x, e_thresh_p_y, e_thresh_p_z
e_thresh_R_x, e_thresh_R_y, e_thresh_R_z
```

instead of `e_dead_*`.

The meaning is:

```text
if |error| > e_thresh:
    add the friction-compensation term
```
