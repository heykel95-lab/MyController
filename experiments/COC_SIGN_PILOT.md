# Active centre-of-compliance sign pilot

## Purpose

The completed `MAIN_A1_t1_05deg` runs are decoupled references. They press the
initially selected leading edge along the surface direction `-n`, but normal
translation does not produce a commanded alignment moment.

The two pilot cases enable the point-shifted impedance and vary only the signed
surface-frame lever:

| Run | `r_c,t2` | Other conditions |
|---|---:|---|
| `PILOT_COC_t1_rc_t2_p060` | +60 mm | Same as `MAIN_A1_t1_05deg` |
| `PILOT_COC_t1_rc_t2_m060` | -60 mm | Same as `MAIN_A1_t1_05deg` |

The controller convention is

```text
r_c = p_TCP - p_c
```

Therefore, a positive `r_c,t2` places the compliance centre on the negative
`t2` side of the TCP. Do not describe the sign as simply “left” or “right”
without resolving the surface frame in the observed robot pose.

For a predominantly normal controller force, the point shift adds

```text
m = -r_c x f
```

so opposite lever signs must produce opposite `t1` moment biases.

## Procedure

Run one trial at a time:

```bash
./experiments/run_coc_sign_pilot.sh next
```

Select `play2` for the current mount. Do not manually straighten the tool
between the two signs unless required for safety. After each trial, record:

- initially loaded physical edge;
- final physical contact: same edge, full face, or opposite edge;
- any visible tool motion inside the gripper.

Stop above 70 N, if the workpiece moves, or if tool motion becomes unsafe.
The expected steady normal load remains approximately 50 N.

## Selection rule

Use the sign that:

1. moves the measured `t1` component toward zero;
2. moves physical contact toward full-face contact;
3. does not create a larger cross-axis error or unsafe peak load.

The two trials are a sign check, not the final centre-position optimization.
After selecting the helpful sign, refine its magnitude in the main Case-D
campaign.
