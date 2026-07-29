# Rejected A0 diagnostics: loose approach-orientation gate

These two zero-offset runs are preserved as diagnostics and are not primary
campaign repeats. They used the calibrated horizontal plane and physical tool
axis, the corrected 60 mm virtual press, and the controller-wide
`approach_orient_error_threshold = 0.035 rad` (approximately 2 degrees).

| Quantity | r01 | r02 |
|---|---:|---:|
| First-contact alignment error [deg] | 0.992 | 0.990 |
| End-of-set-up alignment error [deg] | 1.874 | 1.814 |
| Alignment change [deg] | -0.882 | -0.824 |
| Steady contact load [N] | 24.70 | 23.95 |
| Peak contact load [N] | 29.06 | 27.79 |
| Selected-feature travel [mm] | 2.92 | 1.95 |

The two repeats demonstrate that reducing the virtual press from 180 mm to
60 mm corrected the contact load, but the nominal zero-angle condition still
entered set-up with about 1 degree of residual error. The response was
repeatable: mean alignment change was -0.853 degrees with a two-sample
standard deviation of 0.041 degrees, and mean steady load was 24.33 N.

An attempted 0.5 degree approach-orientation gate was subsequently rejected:
the mounted system settled near 1.5 degrees and could not enter descent. The
replacement MAIN campaign therefore retains the reachable 2 degree gate and
uses the measured first-contact error rather than treating the nominal zero
command as an exact physical angle. These archived runs remain separate
because they used the earlier 360 N/m normal stiffness.

They also showed that the 360 N/m normal stiffness produced only 24.33 N at
the corrected 60 mm virtual press. The replacement MAIN campaign uses
800 N/m, corresponding to a 48 N quasi-static command and an expected measured
equilibrium near the established 50 N operating range.

The r02 `dirty-tree` marker was caused only by metrics and figures regenerated
after r01; no controller, calibration, or setup input changed. The runner was
updated to exclude these derived outputs from the input-provenance check.
