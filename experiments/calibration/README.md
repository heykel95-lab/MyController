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

## Physical tool-axis calibration

Plane-point calibration does not identify a mounting angle between the
physical grinding face and the nominal `+Z_EE` axis. Calibrate that angle
separately after every tool regrasp that can change its orientation.

With the horizontal plane already validated, place the complete tool face flat
at four positions. Keep it flat and deliberately change its yaw about the
plane normal by roughly 20--45 degrees between samples. The capture is
read-only:

```bash
cd surface_grinding_controller
make capture_tool_axis
./tools/capture_tool_axis grinding_tool horizontal T1
./tools/capture_tool_axis grinding_tool horizontal T2
./tools/capture_tool_axis grinding_tool horizontal T3
./tools/capture_tool_axis grinding_tool horizontal T4
cd ..
python3 experiments/calibration/prepare_tool_axis_calibration.py grinding_tool
```

T1--T3 estimate the EE-frame axis whose mapped base-frame direction remains
invariant across yaw; T4 validates it. This estimate is independent of the
plane-normal fit. The plane normal is used only as a separate sanity check.
Both the fit-sample spread and the held-out error must be at most 0.5 degrees,
and the yaw samples must provide sufficient observability. MAIN and validation
setups declare `grinding_tool` in `tool_profile.txt`; the runner refuses to
start without a passing report and archives the exact tool-axis overlay with
every result.
