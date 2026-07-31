# Cases A and B at a 90 deg commanded spin

27 complete trials, 2026-07-31: Case A 15/15 and Case B 12/12, all admissible.
Superseded by the re-run at `tool_target_offset_normal_deg = 115.05`, kept
because the data itself is sound and the two sets are directly comparable.

## Why they were superseded

`tool_target_offset_normal_deg` is measured **from tangent1**, so the same
number means a different physical orientation in a different frame. The
calibrated `horizontal` profile takes tangent1 from the projected P1->P2 probe
direction, `[-0.906, -0.423, 0.027]`, which is **25.05 deg** round from the
`[1, 0, 0]` tangent1 in `params/`.

So `= 90` in the calibrated frame put the face long axis at heading 115.05 deg
in base coordinates, while the same `= 90` in a direct `s` run put it at
90.0 deg. The campaign was presenting the tool 25 deg away from the
orientation the operator works with at the rig. `= 115.05` in the calibrated
frame reproduces heading 90.0 deg, verified across A0, A2, A4 and B1 setups.

Nothing here is wrong: every trial converged to `spin_err < 0.5 deg` against
the frame it was given, and the alignment metric uses that same frame. The
re-run changes which physical orientation the face presents, not the accuracy
with which it is reached.

## What these runs contain

```text
MAIN_A0_00deg      3/3    MAIN_B1_KRt1_15   3/3
MAIN_A1_t1_05deg   3/3    MAIN_B1_KRt1_50   3/3
MAIN_A2_t1_10deg   3/3    MAIN_B2_KRt2_15   3/3
MAIN_A3_t2_05deg   3/3    MAIN_B2_KRt2_50   3/3
MAIN_A4_t2_10deg   3/3
```

Group means, alignment in degrees against the calibrated plane:

| setup | K_R | before | after | gain |
|---|---|---:|---:|---:|
| A0 (0 deg) | 5 | 1.04 | 1.73 | -0.69 |
| A1 (t1, 5 deg) | 5 | 2.60 | 1.93 | +0.67 |
| A2 (t1, 10 deg) | 5 | 7.17 | 6.67 | +0.49 |
| A3 (t2, 5 deg) | 5 | 4.35 | 2.84 | +1.51 |
| A4 (t2, 10 deg) | 5 | 8.91 | 6.67 | +2.24 |
| B1_KRt1_15 | 15 | 6.97 | 6.47 | +0.49 |
| B1_KRt1_50 | 50 | 7.05 | 6.63 | +0.41 |
| B2_KRt2_15 | 15 | 8.34 | 7.05 | +1.29 |
| B2_KRt2_50 | 50 | 8.93 | 7.64 | +1.29 |

Steady force sat at 47-49 N throughout, so the preload is not confounding the
comparison. The t2 conditions correct several times more than the t1 ones at
matched command, and that asymmetry survives the whole stiffness sweep. The
re-run is the check on whether it survives a 25 deg change of face heading
as well.
