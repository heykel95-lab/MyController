# Rejected A0 pilot: pre-calibration, 180 mm preload

This folder preserves the first horizontal zero-offset trial as a diagnostic
pilot, not as a primary-campaign repeat.

- controller commit: `0ea188d8bc6b8e09db37139a657b6c5346f3d879`
- plane P4 residual: -0.817 mm (passed)
- alignment before set-up: 1.020 deg
- alignment after 5 s: 1.862 deg
- alignment change: -0.842 deg (worsened)
- final/peak load: 66.2/70.9 N
- virtual penetration: 180 mm
- normal stiffness: 360 N/m

The run motivated two workflow corrections:

1. calibrate the physical grinding-face normal independently instead of
   assuming it is exactly `+Z_EE`;
2. use a 60 mm primary-campaign penetration, corresponding to a
   quasi-static load target of 21.6 N and an empirically scaled target of about
   22 N.

The original archived provenance and terminal transcript are retained beside
this note in the working experiment repository. The large raw CSV remains
ignored by Git.
