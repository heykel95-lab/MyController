# Main Calibrated-Plane Alignment Study

This file is the operational specification for the new thesis campaign. The
previous measurements and setup folders remain archived; they are not mixed
with the `MAIN_*` results.

## Fixed conventions

- The physical plane is fitted from three non-collinear points. A fourth,
  held-out point must lie within 1 mm of the fit before any MAIN trial runs.
- The full MAIN A--D campaign is bound to the named `horizontal` calibration
  profile.
- A separately calibrated `tilted` profile is reserved for the compact
  frame-transfer validation and cannot overwrite the primary calibration.
- The projected direction from \(P_1\) to \(P_2\) defines \(+t_1\);
  \(t_2=n_s\times t_1\). These directions should be marked on the workpiece.
- The physical tool-face normal is calibrated independently from the plane
  points using T1--T3 plus a held-out T4 sample. Every MAIN and validation run
  requires and archives the `grinding_tool` calibration profile.
- The EE-to-tool transform must remain rigid. A witness mark across the tool
  and holder is checked before and after every repeat; any relative rotation
  rejects the run. A possible 2 degree slip is not folded into the alignment
  uncertainty because it can change during contact and is not observable from
  the robot EE pose. Temporary runs declared with the runner's `play2` option
  are retained as exploratory EE-response data and excluded from primary
  physical-alignment means.
- The plane defines its point, normal and surface frame
  \(R_s=[t_1,t_2,n_s]\).
- The commanded tool orientation is an independent signed offset about
  \(t_1\) and/or \(t_2\).
- Alignment is always evaluated against the calibrated physical plane.
- The approach-to-descend transition is \(2^\circ\) for every MAIN and
  validation condition. A tested \(0.5^\circ\) threshold was unreachable for
  the mounted system, which settled near \(1.5^\circ\) and therefore never
  descended. Continued orientation control during descent reduces the A0
  first-contact residual further; the measured value remains the reported
  independent variable.
- Translational experimental gains are diagonal in the surface frame:

  \[
  K_{p,0}=R_s\operatorname{diag}(K_{p,t_1},K_{p,t_2},K_{p,n})R_s^\top .
  \]

- Centre-of-compliance experiments command the direct lever

  \[
  r_c=p_{\mathrm{TCP}}-p_c
  =R_s[r_{c,t_1},r_{c,t_2},r_{c,n}]^\top .
  \]

- Centre-of-compliance results use only the directly commanded \(r_c\).
- Cases A--E use null-space damping only. The sigma bias is reintroduced only
  in the matched Case-F control.
- The set-up observation is 5 seconds. Automatic damping is recomputed from
  each changed stiffness and the phase-entry task inertia.

## Case A: calibration and initial angle

Three repeats of each configuration:

| Setup | Tool command |
|---|---:|
| `MAIN_A0_00deg` | \(0^\circ\) |
| `MAIN_A1_t1_05deg` | \(+5^\circ\) about \(t_1\) |
| `MAIN_A2_t1_10deg` | \(+10^\circ\) about \(t_1\) |
| `MAIN_A3_t2_05deg` | \(+5^\circ\) about \(t_2\) |
| `MAIN_A4_t2_10deg` | \(+10^\circ\) about \(t_2\) |

Case A contributes 15 trials. A0 is the calibration gate. The nonzero cases
use the measured first-contact angle, not the nominal command, in the plots.

## Case B: axis-specific rotational stiffness

The \(5\,\mathrm{Nm/rad}\) references are the two 10-degree Case-A setups.
Additional settings are:

| Excitation | Changed gain |
|---|---|
| \(+10^\circ\) about \(t_1\) | \(K_{R,t_1}=15,50\,\mathrm{Nm/rad}\) |
| \(+10^\circ\) about \(t_2\) | \(K_{R,t_2}=15,50\,\mathrm{Nm/rad}\) |

The orthogonal rotational gain remains \(5\,\mathrm{Nm/rad}\). Three repeats
of the four additional settings contribute 12 trials.

## Case C: translational stiffness and interaction

The geometrically cross-matched gain is varied:

| Excitation | Changed gain |
|---|---|
| \(+10^\circ\) about \(t_1\) | \(K_{p,t_2}=300,800,2000\,\mathrm{N/m}\) |
| \(+10^\circ\) about \(t_2\) | \(K_{p,t_1}=300,800,2000\,\mathrm{N/m}\) |

The \(2000\,\mathrm{N/m}\) references are already in Case A. One additional
interaction corner per axis,
\(K_R=50\,\mathrm{Nm/rad}, K_p=300\,\mathrm{N/m}\), completes the endpoint
\(2\times2\) comparison. Case C contributes 18 additional trials.

Cases A--C therefore contain 45 unique robot trials.

## Case D: centre of compliance

Case D is mandatory but is not run until Cases A--C have selected suitable
gains. The generated coarse configurations currently use the baseline gains as
placeholders.

For a \(t_1\) angular error, sweep the perpendicular lever \(r_{c,t_2}\).
For a \(t_2\) angular error, sweep \(r_{c,t_1}\). Initial levels are
\(-60,0,+60\,\mathrm{mm}\), with three repeats per level and axis.

The zero-lever coupled result must agree with its matched decoupled reference.
The opposite signs test the predicted moment direction. Intermediate or
farther points are generated only after the coarse response is reviewed.

## Case E: virtual push and stored preload

After selecting gains and a pole:

1. compare virtual post-plane references \(0.10,0.14,0.18\,\mathrm{m}\);
2. compare a controlled 5-second retained-preload observation with a
   controlled 5-second released-preload observation.

The reference is a virtual spring coordinate, not physical penetration.
Report alignment at first contact, at the end of set-up and at the end of the
post-set-up observation.

## Case F: null-space attribution

At one representative contact condition compare:

- mode 1: projected null-space damping only;
- mode 3: projected damping plus the smallest-singular-value bias.

Use three repeats initially. Compare alignment, task drift, joint motion and
smallest singular value. This case determines whether the active sigma torque
changes the contact response.

## Case H: one pole for every tilt direction, and which side of the plane

Case D found the lever that corrects each surface axis. Case H asks whether one
lever can serve every tilt direction, and settles the normal coordinate.

The press force is normal, \(f=-Fn_s\), so the moment it makes about the TCP is

\[
m = f\times r_c = F\,(r_{c,t_2},\,-r_{c,t_1},\,0).
\]

Only the tangential lever turns the tool, and it turns it perpendicular to
itself. A tilt \(\theta\) about \(u=(\cos a,\sin a)\) in the tangent plane needs
a corrective moment along \(-u\), so it needs the lever

\[
r_{c,t} = \rho\,(\sin a,\,-\cos a),
\]

perpendicular to the tilt axis and rotating with it. Case D measured both ends
of that rule and both agree: \(a=0^\circ\) needs \(r_{c,t_2}=-60\,\mathrm{mm}\)
(7.0 to 0.9 deg), \(a=90^\circ\) needs \(r_{c,t_1}=+60\,\mathrm{mm}\) (8.5 to
2.1 deg), and the same levers on the wrong axis or with the wrong sign removed
nothing.

The rule therefore predicts that **no fixed pole serves every direction**. Case
H tests that prediction rather than assuming it, at \(\rho=60\,\mathrm{mm}\) and
a \(10^\circ\) tilt, with the directions named in the tool frame as Case E named
them.

| Setup | Tilt | Lever \(r_{c}\) [mm] |
|---|---|---|
| `MAIN_H1_rot_yEE` | 10 deg about \(Y_{EE}\) | \((+54.4,+25.4,0)\) |
| `MAIN_H1_rot_diag_m45` | 45 deg between the tool axes | rule lever |
| `MAIN_H1_rot_xEE` | 10 deg about \(X_{EE}\) | \((+25.4,-54.4,0)\) |
| `MAIN_H1_rot_diag_p45` | the other diagonal | rule lever |
| `MAIN_H2_fix_diag_m45` | as H1 | fixed at the \(Y_{EE}\) lever |
| `MAIN_H2_fix_xEE` | as H1 | fixed at the \(Y_{EE}\) lever |
| `MAIN_H2_fix_diag_p45` | as H1 | fixed at the \(Y_{EE}\) lever |
| `MAIN_H3_rcn_m060` | 10 deg about \(Y_{EE}\) | H1 lever, \(r_{c,n}=-60\) |
| `MAIN_H3_rcn_p020` | same | \(r_{c,n}=+20\) |
| `MAIN_H3_rcn_p060` | same | \(r_{c,n}=+60\) |
| `MAIN_H3_rcn_p120` | same | \(r_{c,n}=+120\) |

**H1** — the rule holds off the surface axes if the fraction removed is the same
at every direction. Each H1 run has a matched decoupled reference in Case E at
the identical commanded tilt, which removed about 1.1 deg of 6.8 with no pole.

**H2** — the general-pole claim itself. The rule predicts the loss follows the
cosine of the direction change and reaches nothing at 90 deg, where Case D
already measured a wrong-axis lever removing 0.0 deg.

**H3** — above, in, or under the plane. The normal lever makes no moment against
a normal press: it drops out of \(f\times r_c\) entirely, and enters only through
the tangential stiffness as \(K_{p,t}r_n^2\) of added rotational stiffness, which
resists the correction. The prediction is a loss symmetric in the sign of
\(r_{c,n}\), growing as \(r_n^2\), and measurable only past
\(|r_n|\approx\sqrt{K_R/K_{p,t}}=\sqrt{5/2000}\approx 50\,\mathrm{mm}\).
`MAIN_D3` tested \(+20\,\mathrm{mm}\) and changed nothing, which is consistent
but far too small to separate the prediction from no effect at all. Since
\(r_c=p_{\mathrm{TCP}}-p_c\), a positive normal lever puts the pole **below** the
TCP; the TCP stands about 20 mm off the plane at contact, so \(+20\) is the pole
in the plane, \(+60\) and \(+120\) are under it, and \(-60\) is 80 mm above it.
An asymmetry between above and under is not in the rule and would be the
contact, not the spring.

Case H contributes 33 trials.

## Case J: the mirror, or does the lever reverse with the tilt?

Cases D, E and H between them fix the lever's **direction**: perpendicular to
the tilt axis, rotating with it, one pole per direction. Every one of those
runs commanded a tilt leaning the same way. So the rule has never met the
opposite sign of the error it claims to correct, and a sign rule that has only
been shown one sign has not been tested as one.

The rule itself is unambiguous. From
\(m=f\times r_c=F\,(r_{c,t_2},-r_{c,t_1},0)\), negating the lever negates the
moment, so a tilt leaning the other way needs the lever pointing the other way.
Written against what Case D measured:

| initial tilt | assisting lever | status |
|---|---:|---|
| \(+10^\circ\) about \(t_1\) | \(r_{c,t_2}=-60\,\mathrm{mm}\) | archived, `MAIN_D1` |
| \(-10^\circ\) about \(t_1\) | \(r_{c,t_2}=+60\,\mathrm{mm}\) | Case J |
| \(+10^\circ\) about \(t_2\) | \(r_{c,t_1}=+60\,\mathrm{mm}\) | archived, `MAIN_D2` |
| \(-10^\circ\) about \(t_2\) | \(r_{c,t_1}=-60\,\mathrm{mm}\) | Case J |

The question worth asking is **not** "does the same pole still work for a
negative tilt?" — the rule already says it cannot, and a run confirming that
tests nothing. It is whether *reversing the pole together with the tilt*
recovers a comparable correction.

All three levels \(-60,0,+60\,\mathrm{mm}\) are kept at each mirrored
condition, not only the predicted one. The wrong-sign run is what makes the
claim falsifiable: if \(-10^\circ\) about \(t_1\) were still corrected by
\(-60\,\mathrm{mm}\), the pole would be a fixed property of the fixture rather
than a function of the measured tilt. The zero-lever runs carry the mirrored
no-lever reference, which no archived run provides, because Case A never
commanded a negative tilt either.

No change to the metric is needed: `align_t*_improve_deg` is
\(|\theta_{\text{before}}|-|\theta_{\text{after}}|\), so each mirrored run is
directly comparable to its Case D twin as recorded.

| run | tilt | lever |
|---|---|---:|
| `MAIN_J1_t1neg_rc_t2_p000` | \(-10^\circ\) about \(t_1\) | \(0\) |
| `MAIN_J1_t1neg_rc_t2_p060` | same | \(+60\) — predicted assisting |
| `MAIN_J1_t1neg_rc_t2_m060` | same | \(-60\) — Case D's lever, predicted useless |
| `MAIN_J2_t2neg_rc_t1_p000` | \(-10^\circ\) about \(t_2\) | \(0\) |
| `MAIN_J2_t2neg_rc_t1_m060` | same | \(-60\) — predicted assisting |
| `MAIN_J2_t2neg_rc_t1_p060` | same | \(+60\) — Case D's lever, predicted useless |

Case J contributes 18 trials and produces `MAIN_J_sign_symmetry.pdf`.

**Safety.** These are the campaign's first negative commanded tilts, so the arm
approaches contact leaning a way it never has. Case J runs unattended like
every other case; what protects it is that each trial is checked against
`analysis/validate_contact_trial.py` before the next one starts, and the sweep
stops on the first trial that does not pass. The zero-lever `J1` runs are still
ordered first, so the mirrored tilt is pressed without a lever before one is
added to it.

## Case K: how long a lever, and does that depend on the tilt?

Cases D, H and J fix the lever's direction and sign. None of them says anything
about its **length**. \(60\,\mathrm{mm}\) was the only magnitude ever tested,
and it was chosen, not measured. So the campaign cannot claim an optimum pole,
and the thesis correctly does not.

There is a reason to expect that no single length can be optimal. The moment
the lever makes is \(\lVert m\rVert=\lVert r_c\rVert\lVert f\rVert\), linear in
the lever, against a restoring \(K_R\) linear in the angle. A lever suited to a
\(10^\circ\) error therefore commands the same moment against a \(2^\circ\) one,
where it can carry the tool past flat. The right object is not a number but a
**selection law**,

\[
r_c^\star=\rho^\star(|\theta|)\;d(u_\theta,\operatorname{sgn}\theta),
\]

in which the direction \(d\) is already settled by Cases D, H and J, and
\(\rho^\star\) is the only unmeasured part.

Case K measures it: two initial tilts against four lengths, on both surface
axes, always at the assisting sign.

\[
\theta\in\{5^\circ,10^\circ\},\qquad
|r_{c,t}|\in\{20,40,60,80\}\,\mathrm{mm}.
\]

The \(10^\circ/60\,\mathrm{mm}\) corner of each axis is exactly
`MAIN_D1_t1_rc_t2_m060` and `MAIN_D2_t2_rc_t1_p060`, already archived at these
gains with three repeats, so it is not run again — the analysis reads those
runs into the K curves. That leaves 14 new setups and 42 trials.

**The most correction is not on its own the best setting.** Read the residual,
the 90% time and the steady load beside it. `MAIN_K_lever_magnitude.pdf` keeps
the *sign* of the residual for that reason: a lever long enough to carry the
tool past flat appears there as a crossing below zero, which a
removed-angle panel cannot distinguish from a good correction. A length that
removes more angle while leaving the final error, the settling time or the
load worse is not the better setting and must not be reported as one.

The reportable outcome is \(\rho^\star=\rho^\star(|\theta_0|)\) with its
saturation point, not \(\rho^\star=60\,\mathrm{mm}\).

If time is short, run axis \(t_1\) first: `MAIN_K1_*` is 7 setups and 21 trials
and gives the shape of the curve at both tilts on its own.

## Evaluation outputs

- scalar physical-plane alignment before and after set-up;
- signed \(t_1\) and \(t_2\) error before and after set-up;
- absolute and percentage error removed;
- 90% alignment time;
- peak and steady model-estimated load;
- TCP and selected-feature travel;
- peak commanded torque;
- post-set-up alignment change; and
- provenance, convergence and safety flags.

## Guided execution

The runner exposes Cases A--F, H, J and K:

```bash
./experiments/run_axis_study.sh status
./experiments/run_axis_study.sh case J auto
./experiments/run_axis_study.sh case K auto
```

`case <letter> auto` runs every remaining trial of one case without an operator
at the keyboard, driven by `lib/auto_drive.py`, which answers the four
controller prompts with exactly what a hand would type. Dropping `auto` runs
the same sequence but stops between trials so the setup can be reset. Either
way `run.sh` archives the raw logs, effective parameters, plane calibration,
terminal transcript and Git provenance, and the metrics and figures are
refreshed once at the end of the case.

`auto` needs no per-case support: `auto_drive.py` reads `startup_mode.txt` from
the setup, so a contact case answers `s` and a hold case `h`, and the stored
`grinding_tool` mount profile answers `run.sh`'s only other prompt. Cases J and
K are contact cases like D, E and H and run through the same path unchanged.

Run Case J before Case K. K commands only the assisting sign, which is the rule
J tests; running K first would assume the answer.

**Per-trial checking.** Every J and K trial is passed to
`analysis/validate_contact_trial.py` before the next one starts, in both
driving modes, and the case stops on the first trial that does not pass. This
is what makes the two new cases safe to leave unattended: it catches a trial
that archived cleanly and is still wrong — the tilt not mirroring at first
contact, a press that never reached the plane, a load past the 70 N the
protocol stops at, or a set-up phase that ended while the tool was still
moving. Its bounds are taken from the archive rather than chosen, and it passes
all 108 archived MAIN contact trials, including the `not-converged`
`MAIN_D1_t1_rc_t2_m060/r01` that Case J exists to mirror. An unexpected
*result* inside those bounds — an assisting lever that assists less than
predicted, a long lever that overshoots — passes and runs.

**`next` does not work on this archive**, and did not before Cases J and K were
added. 25 archived setups, `MAIN_A0_00deg` among them, have a `terminal.log`
without the `SET-UP RESULT` block that `repeat_complete` looks for, so
`find_next` stops on the first of them. Use `case` and name the letter.

All primary conditions use the same normal preload command:

\[
\delta_n=0.060\ \mathrm{m},\qquad
K_{p,n}=800\ \mathrm{N/m},\qquad
F_{n,\mathrm{qs}}\approx K_{p,n}\delta_n=48.0\ \mathrm{N}.
\]

The five-second set-up window is retained. Thus the preload reaches its final
value early enough for the alignment and load to settle before evaluation.
The expected measured equilibrium is close to 50 N based on the 24.33 N
loose-gate A0 mean at 360 N/m and the earlier 180 mm preload pilot.

After horizontal Cases A--D, the compact tilted-plane baseline validation is:

```bash
./experiments/run_tilted_validation.sh status
./experiments/run_tilted_validation.sh next
```

It repeats the \(0^\circ\), \(10^\circ\) about \(t_1\), and \(10^\circ\) about
\(t_2\) baseline-gain conditions. A final tuned condition is generated only
after the horizontal campaign selects its gains and compliance centre.

## Mount status decision, 2026-07-31

`grinding_tool` is declared `rigid` although the holder retains a small
rotational play about \(Y_{EE}\), on the order of the 2 degrees the runner's
`play2` option describes. The experimenter's judgement is that the mount is as
fixed as this fixture can be made and that no mount is perfectly rigid.

The consequence is recorded here rather than left implicit in the profile file.
`tool-play` is a data flag: it removes a run from every mean and every figure,
so declaring `play2` would have archived the whole A--F campaign without
producing a single plotted point. Declaring `rigid` keeps the campaign
analysable, and the residual play is an unquantified contribution to the
alignment uncertainty that the reported figures do not carry.

The play cannot be calibrated out. It is not observable from the robot EE pose
and it can change during contact, which is why the conventions above refuse to
fold it into the stated uncertainty. It also cannot be steered away from the
measurement: play about \(Y_{EE}\) tilts the tool axis along \(t_1\) when the
commanded spin puts the face long axis on \(t_2\), and along \(t_2\) when the
spin is zero, so it lands on the measured component of one half of Case A
either way.

## Tool slip is an axis-specific limitation, 2026-07-31

The declared mount play is a rotation about \(Y_{EE}\), and the commanded
twist places \(Y_{EE}\) only 25 deg from \(t_2\). For a \(t_2\) excitation the
slip axis therefore nearly coincides with the axis being corrected: the
contact moment can be taken up by the tool turning inside the gripper instead
of the arm turning, and because the alignment metric infers the tool from the
EE pose, that reads as no correction rather than as a slip.

The archived campaign is consistent with this. Median within-group standard
deviation is 0.017 deg across the ten \(t_1\)-excited groups and 0.033 deg
across the ten \(t_2\)-excited ones, and the two largest spreads in the whole
campaign are both \(t_2\): `MAIN_A4_t2_10deg` at 1.62 deg, where one repeat
removed exactly 0.00 deg while its siblings removed 1.56 and 1.62, and
`MAIN_D2_t2_rc_t1_p060` at 0.55 deg. The effect is not purely axis-specific --
spread grows with correction magnitude on both axes, and the largest \(t_1\)
spread is the 6.05 deg `MAIN_D1_t1_rc_t2_m060` condition -- but \(t_2\) being
twice as noisy overall, with both extremes, points at the slip axis.

The consequence for the results is one-directional. A slip absorbs correction
that the metric cannot see, so it can only make a \(t_2\) condition look worse
than it is. The headline asymmetry, \(t_2\) correcting more than \(t_1\), is
therefore a lower bound rather than an artefact.

Runs where the witness mark moved are rejected by the conventions above, but a
slip is not observable from the robot, so nothing automatic detects it. Write
a line `reject: <reason>` into that repeat's `operator_observation.txt` and the
extractor flags it `operator-reject(...)`, which removes it from every mean and
draws it hollow.

`MAIN_A4_t2_10deg/r03` is deliberately **not** rejected. The slip is a property
of the fixture rather than a recording fault, and it is the clearest evidence
the campaign holds that the limitation is real; excluding it would remove the
finding along with the outlier. It carries an `operator_observation.txt`
describing what happened, with no `reject:` line, so it stays in every mean and
figure and is reported in the thesis instead. Its group therefore reads
+1.06 +/- 0.75 deg rather than the +1.59 +/- 0.03 of its two siblings, and any
figure showing Case A at a \(t_2\) excitation carries that error bar.
