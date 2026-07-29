# Calibrated-Plane Axis-Specific Alignment Study

## Experimental decision

The main campaign uses the existing tilted physical plane. Absolute plane
inclination is held fixed; the controlled excitation is the relative
tool-to-plane angle about surface tangent \(t_1\) or \(t_2\).

A horizontal plane is not mixed into the primary matrix because changing the
absolute plane orientation also changes robot posture, Jacobian, gravity
loading, reachable workspace, and contact geometry. A matched horizontal-plane
repeat can be added later as a transfer test after the primary effects are
established.

## Separation implemented in the controller

The calibrated surface plane determines:

- plane point and normal;
- geometric clearance and contact projection;
- surface frame \([t_1,t_2,n_s]\);
- translational and rotational gain directions; and
- the physical-plane alignment metric.

The independent parameters

```text
tool_target_offset_tangent1_deg
tool_target_offset_tangent2_deg
```

rotate only the commanded tool axis. A nonzero value therefore creates a known
contact mismatch without falsifying the plane used by the geometry.

## Calibration gate

1. Mark the \(+X_{\mathrm{EE}},+Y_{\mathrm{EE}}\) corner of the configured
   rectangular tool face.
2. Touch that same corner to three separated plane locations and capture P1,
   P2, and P3 with `tools/capture_plane_point`.
3. Capture P4 at a different location as a held-out check.
4. Run `experiments/calibration/prepare_plane_calibration.py`.
5. Require all triangle edges to exceed 50 mm and the absolute P4 distance to
   the fitted plane to be at most 1 mm.
6. Run the D0 zero-offset control. Its first-contact physical-plane alignment
   angle should be close to zero. Stop and correct the calibration if it is
   not.

The first three points define the plane; P4 is never used in the fit.

## Primary test matrix

All runs use the decoupled stiffness command and a 5 s set-up. The normal
rotational stiffness remains \(50\,\mathrm{Nm/rad}\).

| IDs | Commanded mismatch | Swept gain | Fixed orthogonal gain | Repeats |
|---|---:|---:|---:|---:|
| D0 | \(0^\circ\) | none | \(K_{R,t_1}=K_{R,t_2}=5\) | 5 |
| D1 | \(+10^\circ\) about \(t_1\) | \(K_{R,t_1}=5,15,50\) | \(K_{R,t_2}=5\) | 5 per setting |
| D2 | \(+10^\circ\) about \(t_2\) | \(K_{R,t_2}=5,15,50\) | \(K_{R,t_1}=5\) | 5 per setting |
| D3 | \(+5^\circ\) about \(t_1\) or \(t_2\) | none | both tangent gains \(=5\) | 5 per axis |

D0, D3, and the \(5\,\mathrm{Nm/rad}\) arms of D1/D2 provide the
\(0^\circ\), \(5^\circ\), and \(10^\circ\) initial-angle comparison without
duplicating runs.

## Evaluation quantities

The controller logs both the scalar physical-plane residual and its signed
surface-frame components:

```text
alignment_angle_deg
alignment_error_t1_deg
alignment_error_t2_deg
alignment_error_normal_deg
```

For each run, the analysis extracts:

- alignment angle before and after set-up;
- physical-plane alignment improvement;
- \(t_1\)- and \(t_2\)-component improvement;
- time to reach 90% of the run's final alignment change;
- steady and peak model-estimated contact load;
- edge travel; and
- peak commanded joint-torque norm.

The primary plot compares alignment improvement against the independently
swept stiffness for \(t_1\) and \(t_2\). Supporting panels show 90% alignment
time and steady estimated normal load. A second plot compares the
\(0^\circ\), \(5^\circ\), and \(10^\circ\) initial-angle conditions.

## Guided execution

From the MyController root:

```bash
./experiments/run_axis_study.sh status
./experiments/run_axis_study.sh next
```

`next` runs exactly one robot trial. It selects the first missing repeat,
archives the logs and effective parameters, then regenerates the metrics and
plots. Review the terminal output and archived trial before calling `next`
again.

## Optional transfer study

After the tilted-plane campaign is complete, recalibrate a horizontal physical
plane and repeat only the \(5\) and \(50\,\mathrm{Nm/rad}\) endpoints for both
axes. This is a robustness test of frame consistency and robot-configuration
dependence, not part of the primary stiffness estimate.
