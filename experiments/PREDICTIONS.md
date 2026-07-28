# Predictions recorded before the runs

A prediction written after seeing the data is a story. This file is committed
before the corresponding runs exist, so the commit date is the evidence.

## B3 — pole swept along surface tangent t1

**Recorded 2026-07-28, before any `B3_pole_tangent_*` run was recorded.**

### The rule being tested

Stated by the experimenter from hands-on observation, independently of the
measured data: *the centre of compliance should sit on the opposite side from
whichever part of the tool touches the surface first.* Left touches first, pole
goes right; front touches first, pole goes back.

### The observations it rests on

- The tool contacts the surface at the **+x_EE, +y_EE corner** of the 40x120 mm
  face. Measured, not assumed: 48 of 52 recorded runs locked
  `offset_ee = [+20, +60, +20] mm`, the remaining 4 at the edge centre
  `[0, +60, +20]`.
- That contact side is the one facing the operator, in the direction of
  growing x_EE.
- `coupled_pole_from_edge` is a **base-frame** offset (main.cpp:1080, added
  directly to `edge_ref`), and the EE x axis in base is `[0.995, -0.044,
  -0.088]`, i.e. 5.7 deg off base +x. So +x_EE and +x_base are interchangeable
  for the purpose of this prediction.

### The prediction

Contact is at +x, so the pole belongs at **-x**. Alignment improvement toward
the measured plane should therefore be **monotonically ordered**:

    B3_pole_tangent_m080  >  m040  >  p000  >  p040
    (x = most negative)                      (x = +46.8 mm)

### Why it is a real test

The B2 sweep pushes the other way on this axis. Every B2 pole sat at positive
base x -- the normal has a +x component from the surface tilt -- so every B2
pole was on the *same* side as the contact, and +160 mm still produced the best
alignment (+3.47 deg, against +1.98 deg at pole-on-edge). B2 and the rule above
disagree about the tangential direction, and B3 is what separates them.

A result either way is informative. If the ordering holds, the rule is a
confirmed mechanism. If it inverts, the normal-offset effect dominates the
tangential one and that is worth knowing before any of this reaches Chapter 5.

### How it will be scored

`python3 experiments/analysis/alignment_vs_real_plane.py`, extended to the B3
tags -- improvement toward the plane measured by `tools/measure_plane`
(a = -0.72 +/- 0.31 deg, b = 14.99 +/- 1.91 deg), not the configured normal.
Noise floor from G3: sigma(tip) 0.194 deg, so differences below ~0.4 deg
between adjacent settings are not claimable.
