# Predictions and their provenance

A prediction written after seeing the data is a story. This file records what
was predicted and, honestly, when — so a reader can judge it themselves.

## B3 — pole swept along surface tangent t1

**Status: NOT pre-registered. See the timeline below.**

An earlier version of this file claimed the prediction was recorded before any
B3 run existed. That was wrong, and the correction is the point of this
section.

### Timeline

| time (2026-07-28) | event |
|---|---|
| 21:31:46 – 21:37:31 | `B3_pole_tangent_*` runs recorded (m080 x3, m040 x3, p000 x3, p040 r01) |
| 21:37:36 | this file first committed, claiming pre-registration |
| shortly after | runs discovered on disk, claim retracted, this section rewritten |

The prediction below was derived only from the experimenter's stated rule and
from the contact geometry measured in the A2/B2/G-series runs. Its author had
not examined any B3 result when writing it. But the runs *did* already exist,
so this is **not** a pre-registered prediction and must not be presented as
one. Treat it as a mechanism stated independently of the B3 outcome, which is
weaker evidence than pre-registration and stronger than a post-hoc fit.

### The rule being tested

Stated by the experimenter from hands-on observation: *the centre of compliance
should sit on the opposite side from whichever part of the tool touches the
surface first.* Left touches first, pole goes right; front touches first, pole
goes back.

### The observations it rests on

- The tool contacts the surface at the **+x_EE, +y_EE corner** of the 40x120 mm
  face. Measured, not assumed: 48 of 52 runs recorded before B3 locked
  `offset_ee = [+20, +60, +20] mm`, the remaining 4 at the edge centre
  `[0, +60, +20]`.
- That contact side is the one facing the operator, in the direction of
  growing x_EE.
- `coupled_pole_from_edge` is a **base-frame** offset (main.cpp:1080, added
  directly to `edge_ref`), and the EE x axis in base is `[0.995, -0.044,
  -0.088]`, i.e. 5.7 deg off base +x. So +x_EE and +x_base are interchangeable
  here.

### The prediction

Contact is at +x, so the pole belongs at **-x**. Alignment improvement toward
the measured plane should be **monotonically ordered**:

    B3_pole_tangent_m080  >  m040  >  p000  >  p040
    (x = most negative)                      (x = +46.8 mm)

### Why it is still worth testing

The B2 sweep pushes the other way on this axis. Every B2 pole sat at positive
base x -- the surface tilt gives the normal a +x component -- so every B2 pole
was on the *same* side as the contact, and +160 mm still produced the best
alignment (+3.47 deg, against +1.98 deg at pole-on-edge). B2 and the rule above
disagree about the tangential direction, and B3 is what separates them.

### How it is scored

`python3 experiments/analysis/alignment_vs_real_plane.py`, extended to the B3
tags -- improvement toward the plane measured by `tools/measure_plane`
(a = -0.72 +/- 0.31 deg, b = 14.99 +/- 1.91 deg), not the configured normal.
Noise floor from G3: sigma(tip) 0.194 deg, so differences below ~0.4 deg
between adjacent settings are not claimable.

### For future predictions

Check `experiments/results/` for the run id before committing a prediction
about it. Pre-registration is only worth something if the claim is true.
