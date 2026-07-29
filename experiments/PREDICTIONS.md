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

### Outcome — the ordering came out exactly inverted

| setup | pole_x [mm] | improvement | predicted rank | actual rank |
|---|---|---|---|---|
| `m080` | -72.7 | **-10.31** | BEST | worst |
| `m040` | -32.9 | -1.78 | 2nd | 3rd |
| `p000` | +7.0 | +2.73 | 3rd | 2nd |
| `p040` | +46.8 | **+6.89** | WORST | **best** |

At `m080` the tool rotates 10 deg *away* from the surface, ending 20.3 deg off
having arrived 10.0 deg off. At `p040` it improves by 6.89 deg -- the best
alignment anywhere in the campaign, better than B2's best of +3.47 deg. The
span is 17.2 deg against a noise floor of ~0.4 deg.

**Whether this refutes the rule is still open.** The rule says "pole opposite
the side that touches first". The controller's contact selection reports the
+x_EE corner, but the experimenter notes the approach angle is small enough
that which side physically touches first is not yet established by observation.
So the measured relationship is certain; its mapping onto the rule is not. If
contact is in fact on the -x side, the rule is confirmed rather than refuted.
Resolve by direct observation before writing this up either way.

### The larger finding: the normal offset was a confound

Fitting improvement against the commanded pole across all 35 B2 and B3 runs:

    improvement ~ pole_x only     R^2 = 0.907    +0.1284 deg/mm
    improvement ~ pole_z only     R^2 = 0.008    -0.0045 deg/mm
    improvement ~ pole_x and z    R^2 = 0.958    x +0.1326, z -0.0113 deg/mm

**The tangential component controls alignment; the normal component does
essentially nothing.** B2 swept the pole along the surface normal over 277 mm
of pole_z and moved improvement by only ~1.5 deg -- but because the configured
normal carries a +x component from the surface tilt, pole_x drifted from -5.0
to +27.6 mm across that sweep. The B2 trend is that x drift, not the normal
offset. B3 holds pole_z near 100 mm and varies pole_x, which breaks the
confound.

---

## B4 — pole swept along surface tangent t2

**Recorded 2026-07-29. Verified before writing: `experiments/results/` contains
no `B4_*` directory, and none of `B3_pole_tangent_p080`, `p120` either. This
one is genuinely pre-registered; the B3 entry above is not, and the difference
is deliberate.**

### Why this axis at all

Across all 44 pole runs recorded so far the t2 component of the commanded pole
sat at approximately zero, while t1 and n were both swept widely:

    t1: -71.4 .. +63.4 mm   swept
    t2:  -1.0 ..  +0.0 mm   never varied within B2/B3
    n:  -97.7 .. +179.7 mm  swept

So the placement rule — *the compliance centre belongs opposite the side that
contacts first* — has not been tested. B3 sweeps t1 and produces a large
effect, but t1 is not necessarily the axis the rule speaks to.

### What the contact geometry says, and how confidently

The contact point was recovered from the archived wrench, without any new runs,
by solving `m = r x f` for the line of action and intersecting it with the
tool-face plane. Two results, with different confidence:

- **Reliable.** In t1 the contact sits near the middle of the 40 mm face width
  (mean -1.6 mm, range -11.6 to +20.4 across ten runs), *not* at the +20 mm
  corner the controller's tie-breaking selection reports in 48 of 52 runs. The
  EE x axis used for this projection was recovered empirically from 63 runs, so
  this component is trustworthy.
- **Not reliable.** In t2 the contact came out consistently toward the +y end
  (+20 to +59 mm), which would make t2 the rule's axis. But decomposing t2
  requires the tool-axis direction, which is not logged; the assumed direction
  was found to be about 5% inconsistent with the known geometry, and the orient
  phase leaves ~1.9 deg of residual. Treat this as a hint.

### The prediction

**Conditional on the contact being toward +t2**, the rule places the pole at
-t2, so alignment improvement toward the measured plane should be ordered:

    B4_pole_tangent2_m080  >  m040  >  p040  >  p080

If instead the contact is confirmed at -t2, the ordering reverses and the same
runs still test the rule — the prediction is the *sign relationship*, not the
labels. If the response is flat within the 0.4 deg noise floor, t2 does not
matter and the rule is not about the contact side at all.

Settling which by direct observation before running B4 would make this a
sharper test. The check: with the tool loaded at the gate, find where a paper
strip pinches along each edge of the face.

### How it will be scored

`python3 experiments/analysis/alignment_vs_real_plane.py` extended to the B4
tags, and the two-panel figure regenerated with t2 as a third panel. Noise
floor from G3: differences below ~0.4 deg are not claimable.

### Outcome — refuted, and not monotonic

| setup | pole_y [mm] | improvement | predicted rank | actual rank |
|---|---|---|---|---|
| `m080` | -91.5 | **-6.54 +/- 0.36** | BEST | worst |
| `m040` | -41.0 | +1.78 +/- 0.03 | 2nd | 3rd |
| `p040` | +39.4 | **+2.93 +/- 0.21** | 3rd | **best** |
| `p080` | +79.5 | +2.66 +/- 0.06 | WORST | 2nd |

Range 9.48 deg against a 0.59 deg noise floor, so t2 unambiguously matters --
the flat-response outcome is excluded. The predicted ordering is inverted at
the extremes, which makes this the second refutation of the placement rule, on
a second axis, in the same direction: positive offsets win on both t1 and t2.

Unlike t1, t2 is **not monotonic**. p040 beats p080, so there is an interior
optimum. Fitting all 48 pole runs with a quadratic in t2 puts it at
**+34 mm** and lifts the model to R^2 = 0.917.

### What the three axes now say together

    pole_x (t1) only          R^2 = 0.586   +0.115 deg/mm
    pole_y (t2) only          R^2 = 0.157   +0.048 deg/mm
    pole_z (n)  only          R^2 = 0.008   -0.005 deg/mm
    x and y                   R^2 = 0.774
    x, y and z                R^2 = 0.833   (z contributes -0.014 deg/mm)
    x, y and y^2              R^2 = 0.917   t2 optimum at +34 mm

Both in-plane components govern the alignment and the normal component does
not. t1 is roughly 2.4x more influential per millimetre than t2 and was still
rising at the end of its tested range; t2 has a located optimum. The earlier
claim that t1 alone explains the outcome at R^2 = 0.907 held only while t2 was
pinned at zero -- with t2 varied, t1 alone drops to 0.586.

### Verdict on the placement rule

The experimenter reports the contacting edge at **-y_EE**, and the EE y axis
maps to base -y (offset_ee = [0, 60, 20] mm resolves to [-6.2, -59.8, -39.6] mm
in base), so the contact is at **base +y**. The best pole is also at base +y.

**The compliance centre belongs on the SAME side as the contacting edge, not
opposite it.** Both sweeps agree.

One caveat is unresolved and should be settled before this is written as a
general rule: the reported contact was described as a long 120 mm side, but the
long sides lie at x = +/-20 mm, not at +/-y. So either the contacting edge is a
short 40 mm end at -y_EE, or it is a long side at -x_EE and the axis in the
report is a slip. The measured relationship is unaffected either way -- positive
offsets win on both axes -- but which edge contacts decides how the rule should
be phrased.

---

## Postscript — the t1 response turns over

Extending B3 to `p080` and `p120` (six further runs) located the optimum that
the earlier entries record as unbounded:

| pole_x [mm] | improvement |
|---|---|
| -70.9 | -10.31 +/- 1.04 |
| -26.6 |  -1.78 +/- 0.09 |
| +15.2 |  +2.73 +/- 0.29 |
| **+62.9** | **+6.96 +/- 0.38** |
| +115.4 |  +2.37 +/- 0.77 |
| +160.8 |  -3.75 +/- 0.09 |

Past the optimum the response reverses rather than flattening: at +161 mm the
pole is worse than commanding no coupling at all.

**This supersedes the R^2 = 0.907 linear figure recorded earlier in this file.**
That fit sampled only the rising flank. Over all 54 pole runs a linear model in
t1 now accounts for R^2 = 0.047, and the correct model is quadratic in both
in-plane components:

    t1 linear                   R^2 = 0.047
    t1 + t1^2                   R^2 = 0.644
    normal only                 R^2 = 0.010
    t1 + t1^2 + t2 + t2^2       R^2 = 0.962   optima +62 and +30 mm
      and the normal            R^2 = 0.966

The alignment response is a quadratic surface in the two in-plane pole
components with an interior maximum. The normal component remains irrelevant.

The measured turnover is consistent with the point-shifted stiffness term:
beyond the beneficial region, increasing the lever can re-stiffen the
rotational response. The experiment therefore treats the deliberately
commanded surface-frame lever as the independent variable.
