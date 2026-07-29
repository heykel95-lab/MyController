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
