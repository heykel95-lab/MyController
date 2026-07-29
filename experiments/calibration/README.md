# Named physical-plane calibration

The `tilted` and `horizontal` directories are independent calibration
profiles. Each profile contains its own four measured points, generated
controller overlay, and validation report. The measured files are ignored by
Git because they contain robot/workpiece coordinates.

For each physical plane:

1. Mark one tool-face corner and use that same corner for P1--P4.
2. Choose P1 and P2 at least 50 mm apart. P1 to P2 defines positive \(t_1\).
3. Place P3 away from that line so P1--P3 span a large triangle.
4. Place P4 elsewhere on the plane; it is not used in the fit.
5. Generate the profile and require the P4 residual to be at most 1 mm.

Example for the horizontal primary plane:

```bash
cd surface_grinding_controller
./tools/capture_plane_point horizontal P1
./tools/capture_plane_point horizontal P2
./tools/capture_plane_point horizontal P3
./tools/capture_plane_point horizontal P4
cd ..
python3 experiments/calibration/prepare_plane_calibration.py horizontal
```

Use `tilted` in both commands for the later validation plane. A setup chooses
its plane through `experiments/setups/<run_id>/plane_profile.txt`; the runner
does not use a global active-plane file.
