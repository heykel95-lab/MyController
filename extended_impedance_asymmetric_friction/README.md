# Extended Cartesian Impedance Controller with Asymmetric Friction Compensation

This controller implements the next idea from your experiments:

> The return behavior is direction-dependent, so the compensation should depend on the **axis** and also on the **sign of the error**.

## Controller idea

The Cartesian impedance part is:

```text
f = Kp * e_p + Dp * (pdot_d - pdot)
m = KR * e_R - DR * omega
```

Then a threshold-based asymmetric compensation is added.

## Sign convention

The code uses:

```text
e = desired - measured
```

For each axis:

```text
if e_i > +e_thresh_i:
    command_i += f_fric_i_pos

if e_i < -e_thresh_i:
    command_i -= f_fric_i_neg
```

So the compensation is always in the restoring direction, but the magnitude can be different for positive and negative error.

## Why this is useful

Your experiments showed that the robot may return well in one direction but not in the opposite direction. A symmetric term such as

```text
f_fric_i * sign(e_i)
```

uses the same compensation in both directions. This new version uses:

```text
f_fric_i_pos for e_i > 0
f_fric_i_neg for e_i < 0
```

## Important example

If you push the robot in negative y, usually:

```text
p_EE_y < p_d_y
```

therefore:

```text
e_y = p_d_y - p_EE_y > 0
```

If it does not return, increase:

```text
f_fric_y_pos
```

If you push the robot in positive y and it does not return, the final error may be:

```text
e_y < 0
```

Then increase:

```text
f_fric_y_neg
```

Always tune based on the **logged final error sign**, not only your hand-push direction.

## Build

```bash
make clean
make
make check
```

## Run

```bash
make run
```

## Current starting values

```text
f_fric_x_pos = 1.0
f_fric_y_pos = 2.0
f_fric_z_pos = 0.5

f_fric_x_neg = 0.5
f_fric_y_neg = 0.5
f_fric_z_neg = 0.5
```

Rotational asymmetric compensation is included but disabled by default:

```text
m_fric_x_pos = 0.0
m_fric_y_pos = 0.0
m_fric_z_pos = 0.0

m_fric_x_neg = 0.0
m_fric_y_neg = 0.0
m_fric_z_neg = 0.0
```

## Safe tuning procedure

1. Run without touching.
2. Push in one direction and release fully.
3. Look at the final error sign.
4. Increase only the corresponding compensation value.
5. Repeat.

Example:

```text
final e_y = +0.020 m
```

Increase:

```text
f_fric_y_pos
```

Example:

```text
final e_y = -0.020 m
```

Increase:

```text
f_fric_y_neg
```

Do not tune many parameters at once.
