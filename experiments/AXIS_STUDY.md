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
- The plane defines its point, normal and surface frame
  \(R_s=[t_1,t_2,n_s]\).
- The commanded tool orientation is an independent signed offset about
  \(t_1\) and/or \(t_2\).
- Alignment is always evaluated against the calibrated physical plane.
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

The runner currently exposes only Cases A--C:

```bash
./experiments/run_axis_study.sh status
./experiments/run_axis_study.sh next
```

`next` performs exactly one robot trial, archives the raw logs, effective
parameters, plane calibration, terminal transcript and Git provenance, then
refreshes the derived metrics and figures. It stops after Case C so that the
selected gains can be reviewed before Case D is enabled.

All primary conditions use the same normal preload command:

\[
\delta_n=0.060\ \mathrm{m},\qquad
K_{p,n}=360\ \mathrm{N/m},\qquad
F_{n,\mathrm{qs}}\approx K_{p,n}\delta_n=21.6\ \mathrm{N}.
\]

The five-second set-up window is retained. Thus the preload reaches its final
value early enough for the alignment and load to settle before evaluation.

After horizontal Cases A--D, the compact tilted-plane baseline validation is:

```bash
./experiments/run_tilted_validation.sh status
./experiments/run_tilted_validation.sh next
```

It repeats the \(0^\circ\), \(10^\circ\) about \(t_1\), and \(10^\circ\) about
\(t_2\) baseline-gain conditions. A final tuned condition is generated only
after the horizontal campaign selects its gains and compliance centre.
