# Orthogonal centre-of-compliance sign pilot

## Purpose

The completed t1 pilot selected `r_c,t2 = -40 mm`. A mismatch about t2
requires the perpendicular lever along t1. These cases keep every other
setting fixed and test:

| Run | Tool command | Active lever |
|---|---:|---:|
| `PILOT_COC_t2_rc_t1_m040` | +5 deg about t2 | `r_c,t1 = -40 mm` |
| `PILOT_COC_t2_rc_t1_p040` | +5 deg about t2 | `r_c,t1 = +40 mm` |

Both use the coupled point-shifted law, the calibrated horizontal plane,
`Kp = [2000, 2000, 800] N/m`, `KR = [5, 5, 50] Nm/rad`, a 60 mm virtual
press and a 5 s set-up interval.

For the right-handed surface frame, a predominantly negative-normal force
and the convention `r_c = p_TCP - p_c` predict that the negative t1 lever
will generate the corrective t2 moment for the commanded positive offset.
The opposite sign remains necessary as an experimental convention check.

## Procedure

Run one case at a time:

```bash
./experiments/run_coc_t2_pilot.sh next
```

The stored grinding-tool mount profile automatically selects `play2`. Do not
manually straighten the tool between signs unless required for safety. Record:

- the initially loaded physical edge;
- final contact: same edge, full face, or opposite edge;
- visible tool motion inside the gripper; and
- any clear cross-axis motion.

Stop above 70 N, if the workpiece moves, or if tool motion becomes unsafe.

## Selection

Prefer the sign that reaches full-face contact, reduces the logged t2
component toward zero, and does not introduce a larger t1 component. A single
run selects only the sign. Repeat the successful setting before adopting it in
the main campaign.
