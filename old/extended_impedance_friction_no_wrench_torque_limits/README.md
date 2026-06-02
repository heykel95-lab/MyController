# Extended Impedance with Friction Compensation — No Wrench/Torque Limits Variant

This is a comparison variant.

It keeps:
- horizontal initial pose
- custom `robot.setCollisionBehavior(...)` thresholds
- threshold-based translational friction compensation

It removes:
- Cartesian force saturation `f_max`
- Cartesian moment saturation `m_max`
- joint torque-rate limiting `delta_tau_max`

## Purpose

This lets you compare three controller variants:

1. **Baseline Cartesian impedance controller**
   - no friction compensation
   - no controller-side limits

2. **Extended controller**
   - friction compensation
   - `f_max`, `m_max`
   - `delta_tau_max`

3. **This no-limits variant**
   - friction compensation
   - no `f_max`, no `m_max`, no `delta_tau_max`

## Warning

This variant can feel more aggressive. Push gently first.

The robot-side collision thresholds are still active, but this code does not smooth or limit the controller torque command before sending it.

## Controller structure

The controller computes:

```text
f = Kp * e_p + Dp * (pdot_d - pdot) + f_fric
m = KR * e_R - DR * omega + m_fric
tau_cmd = J^T * [f; m] + coriolis
```

but it does not apply:

```text
f = sat(f, f_max)
m = sat(m, m_max)
tau = rate_limit(tau, delta_tau_max)
```

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

## Test procedure

Use the same manual test as the other variants:

```text
0--3 s: no touch
3--5 s: gentle manual disturbance
5--12 s: release fully
```

Compare:
- reflex/no reflex
- final position error norm
- max position error norm from CSV
- final rotation error norm
- max rotation error norm from CSV
