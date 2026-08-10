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

---

## J and K — pre-registered, 2026-08-10

**Status: pre-registered.** Written before any `MAIN_J*` or `MAIN_K*` run
existed. `experiments/results/` contains 84 run directories at this commit and
none of them is a J or K run, which is checkable from the repository rather
than from this claim. The B3 section above exists because that check was not
made once; it is made here.

### What is already settled, and what is not

Cases D, E and H fix the lever's **direction**: perpendicular to the tilt axis,
rotating with it, no fixed pole serving every direction. What they do not fix
is its **sign against a tilt that leans the other way**, because every tilt the
campaign has commanded leans the same way, and its **length**, because
\(60\,\mathrm{mm}\) is the only magnitude anything has been run at.

### J — the sign rule

From \(f=-Fn_s\) and \(m=f\times r_c=F\,(r_{c,t_2},-r_{c,t_1},0)\), negating the
lever negates the moment.

**Prediction J1.** At \(-10^\circ\) about \(t_1\), the lever
\(r_{c,t_2}=+60\,\mathrm{mm}\) removes an amount comparable to what
\(-60\,\mathrm{mm}\) removed at \(+10^\circ\) in `MAIN_D1` (6.1 deg, 7.0 to
0.9). "Comparable" is fixed in advance as **within 25% of the Case D twin**,
which is wider than the 0.4 deg largest within-group spread on \(t_1\) and
narrower than the difference between assisting and non-assisting levers,
which in Case D was the whole effect.

**Prediction J2.** At \(-10^\circ\) about \(t_2\), \(r_{c,t_1}=-60\,\mathrm{mm}\)
does the same against `MAIN_D2` (6.4 deg, 8.5 to 2.1), on the same 25% band.
The \(t_2\) band is expected to be met less cleanly than \(t_1\): the declared
mount play is a rotation about \(Y_{EE}\), 25 deg from \(t_2\), and the tool
slip that follows from it can only make a \(t_2\) condition read worse.

**Prediction J3.** The lever Case D selected, applied to the mirrored tilt,
removes **nothing distinguishable from the zero-lever run** at the same tilt.
This is the falsifier. If it instead assists, the pole is a fixed property of
the fixture and the whole selection law is wrong.

**Prediction J4.** The zero-lever mirrored runs reproduce their Case A
counterparts in magnitude: \(|\theta|\) before contact near the commanded
\(10^\circ\) less the settled approach residual, and the removed angle near
`MAIN_A2` (0.64 deg on \(t_1\)) and `MAIN_A4` (1.06 deg on \(t_2\)).

**Scoring.** J is confirmed if J1, J2 and J3 all hold. J1 or J2 failing while
J3 holds means the rule has the sign right and the magnitude is not symmetric,
which is a fixture result and must be reported as one, not folded into the
rule. J3 failing refutes the rule outright and no wording rescues it.

### K — the length

The moment is linear in the lever, \(\lVert m\rVert=\lVert r_c\rVert\lVert
f\rVert\), against a restoring \(K_R\) linear in the angle. But the point shift
also adds \(K_{p,t}\lVert r_c\rVert^2\) of rotational stiffness, which resists
the correction and grows faster. The historical B3/B4 sweeps in this file
measured exactly that competition and found an interior maximum with the
response *reversing* past it, not flattening.

**Prediction K1.** The removed angle rises with \(|r_{c,t}|\) and then turns
over. It is **not** monotonic across \(20\ldots80\,\mathrm{mm}\) at both tilts.

**Prediction K2.** The turnover is at a **shorter lever for the \(5^\circ\)
tilt than for the \(10^\circ\) one**. This is the substantive claim and the
reason the case exists: it is what makes \(\rho^\star\) a function of
\(|\theta_0|\) rather than a constant. If the two tilts turn over at the same
length, \(\rho^\star\) is a property of the contact and the gains alone, the
selection law collapses to a constant, and 60 mm can be reported as an
optimum after all.

**Prediction K3.** At \(5^\circ\) with \(|r_{c,t}|=80\,\mathrm{mm}\), the signed
residual crosses zero — the tool is carried past flat. This is the concrete
form of "a lever useful at \(10^\circ\) is too much at a smaller tilt". If no
condition overshoots, the useful range is wider than the mechanism suggests
and that is the result.

**Prediction K4.** The steady load is flat across the sweep, within the G3
band. The lever changes the moment, not the normal press. A load that trends
with the lever means the lever is moving the contact patch, which is a contact
finding and not a spring one.

**Scoring.** K is reported as a curve \(\rho^\star(|\theta_0|)\) with its
saturation or turnover point per axis and per tilt, never as a single optimum.
K2 is scored on whether the two turnover points differ by more than the
lever spacing, \(20\,\mathrm{mm}\). If the sampled grid puts the turnover at an
endpoint, the honest statement is that the optimum is not bracketed, and the
grid is extended rather than the endpoint reported as the optimum — which is
the mistake the B3 postscript above records.

### J — outcome: the sign rule holds, the magnitude is not symmetric

Recorded 2026-08-10, 18 trials, all passing the contact gate, all `rigid`
mount, loads 38.4 to 45.3 N. Excited-axis error removed, deg:

| lever | \(t_1\), \(+10^\circ\) (D) | \(t_1\), \(-10^\circ\) (J) | \(t_2\), \(+10^\circ\) (D) | \(t_2\), \(-10^\circ\) (J) |
|---:|---:|---:|---:|---:|
| \(-60\) | **+6.41** ± 0.30 | −0.63 ± 0.01 | +0.03 ± 0.06 | **+5.70** ± 0.20 |
| \(0\) | +0.73 ± 0.00 | −0.36 ± 0.01 | +1.31 ± 0.01 | −0.90 ± 0.07 |
| \(+60\) | +0.58 ± 0.01 | **+3.13** ± 0.13 | **+6.44** ± 0.43 | −1.58 ± 0.03 |

**J3 holds on both axes, and it was the falsifier.** The lever Case D selected,
applied to the mirrored tilt, removes nothing: −0.63 against a −0.36 no-lever
baseline on \(t_1\), −1.58 against −0.90 on \(t_2\). Had the same pole kept
working, the pole would have been a property of this fixture and the selection
law would be dead. It is not.

**J2 holds.** \(t_2\) mirrored at +5.70 against its twin's +6.44, 89%, inside
the 25% band.

**J1 fails.** \(t_1\) mirrored at +3.13 against +6.41, 49%, outside the band.

**J4 fails on both axes.** The mirrored zero-lever runs drift *away* from flat,
−0.36 and −0.90, where their positive counterparts moved toward it, +0.73 and
+1.31.

The axis expectation was also backwards. J2 predicted \(t_2\) would meet its
band less cleanly than \(t_1\), because the declared mount play is a rotation
about \(Y_{EE}\), 25 deg from \(t_2\). \(t_2\) mirrored better. Whatever
produced the \(t_1\) gap, it is not the slip axis that was named in advance.

By the scoring rule written before any run: J1 or J2 failing while J3 holds
means the rule has the sign right and the magnitude is not symmetric, and that
is a fixture result rather than part of the rule. That is what is reported.

**A candidate for the asymmetry, offered as a candidate.** First contact is
about 1 deg larger in the mirrored direction on both axes: \(+8.00\) against
\(-7.03\) on \(t_1\), \(+9.41\) against \(-8.33\) on \(t_2\). The calibrated
plane carries \(b=+0.988^\circ\) of its own tilt, so a \(-10^\circ\) command
starts further from the physical surface than a \(+10^\circ\) one does, and the
same lever is given the same 5 s to remove a larger error. This accounts for
the direction of J1 and J4 on both axes. It does not obviously account for the
*size* of the \(t_1\) gap, where half the correction is missing against a 1 deg
larger starting error, so it is not presented as the explanation.

The consequence for the thesis is a strengthening rather than a retreat.
Direction and sign follow a law that survived its falsifier on both axes;
magnitude depends on conditions and is not recoverable from one number. That is
the question Case K measures.

### K — outcome: no single optimum, and the turnover is not what was predicted

Recorded 2026-08-10, 42 trials, all passing the contact gate, loads 31.5 to
46.4 N. Error removed and signed residual left, deg:

| \|r_c\| | \(t_1\) 5°: rm / left | \(t_1\) 10°: rm / left | \(t_2\) 5°: rm / left | \(t_2\) 10°: rm / left |
|---:|---:|---:|---:|---:|
| 20 | +1.14 / −0.93 | +1.69 / −4.86 | +1.40 / −2.47 | +1.43 / −6.81 |
| 40 | +1.13 / +0.92 | +3.22 / −3.52 | +1.65 / −2.30 | +2.03 / −6.23 |
| 60 | +0.70 / +1.33 | **+6.41** / −0.56 | **+3.74** / −0.23 | +6.44 / −1.89 |
| 80 | +0.61 / +1.44 | +5.75 / +0.95 | +1.57 / +2.44 | **+7.89** / −0.31 |

**K2 holds, and it is the headline.** The turnover moves with the tilt: \(t_1\)
peaks at 20--40 mm at 5° and 60 mm at 10°; \(t_2\) peaks at 60 mm at 5° and has
not turned by 80 mm at 10°. Both differ by more than the 20 mm lever spacing.
\(\rho^\star\) is a function of \(|\theta|\), and no single lever is optimal.

**K3 holds on both axes.** At 5° with 80 mm the signed residual crosses zero:
\(t_1\) ends at +1.44 from −2.03, \(t_2\) at +2.44 from −3.97. On \(t_1\) the
crossing begins as early as 40 mm.

**K1 holds on three of four.** \(t_2\) at 10° has not turned over by 80 mm, so
its optimum is **not bracketed**. Per the rule written above, the honest
statement is that the grid must be extended, not that 80 mm is the optimum.

**K4 fails.** The steady load was predicted flat and is not: it falls with the
lever, \(t_1\) at 10° going 44.1 → 31.5 N from 20 to 80 mm. It falls furthest
where the correction is largest, so this is the tool rolling onto the face and
relieving the commanded penetration rather than the lever loading the contact.
The contact is not invariant across the sweep, which the design assumed.

#### The mechanism is not the one predicted

K1's reasoning was that the point shift adds \(K_{p,t}\lVert r_c\rVert^2\) of
rotational stiffness which eventually resists the correction. That is not what
the data shows. The applied moment \(F\lVert r_c\rVert\) **rises monotonically
in all sixteen conditions**, 0.91 → 3.34 N·m and equivalents; it never
saturates and never turns over.

The turnover is overshoot. The peak of "error removed" lands exactly at the
last lever before the signed residual crosses zero, in four conditions of four:

| condition | peak removed | residual crosses zero |
|---|---|---|
| \(t_1\) 5° | 20 mm | between 20 and 40 |
| \(t_1\) 10° | 60 mm | between 60 and 80 |
| \(t_2\) 5° | 60 mm | between 60 and 80 |
| \(t_2\) 10° | 80 mm | never (still rising) |

So \(\rho^\star\) is not a tuning optimum but a geometric one: **the lever that
just brings the residual to zero**, growing with \(|\theta|\) because more
initial error needs more rotation. That is a stronger and more predictive
result than the saturation the prediction assumed, and it was not anticipated.

#### K refutes the plane-tilt candidate offered for J

The J outcome above offered the plane's own \(+0.988^\circ\) tilt as a
candidate: the mirrored command starts about 1 deg further out. K measures that
sensitivity directly over a 5 deg span instead of 1 deg, and it has the wrong
sign. Within the positive direction more initial error means **more**
correction, +1.15 deg/deg on \(t_1\) and +0.62 on \(t_2\). Extrapolating K's
residual-vs-initial slope at the same 60 mm to the mirrored starting angles:

| | predicted \|residual\| | measured | miss |
|---|---:|---:|---:|
| \(t_2\) | 2.30 | 3.71 | +1.41 |
| \(t_1\) | 0.41 | 4.82 | **+4.40** |

\(t_2\)'s mirror is modestly off and is largely covered by its larger starting
angle needing a longer lever, which is \(\rho^\star(|\theta|)\) working as
stated. \(t_1\)'s is not covered by anything measured. The candidate is
withdrawn as an explanation for \(t_1\), and that deficit stands open.

Case L tests it: the same two mirrored tilts at the lever \(\rho^\star\) asks
for at their starting angle, 80 mm on \(t_1\) and 90 mm on \(t_2\). If 80 mm
brings the mirrored \(t_1\) residual to zero, the J deficit was an
under-levered condition and the law holds in both directions. If it does not,
the deficit is direction-dependence the moment rule does not contain.
