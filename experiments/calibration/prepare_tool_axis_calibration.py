#!/usr/bin/env python3
"""Estimate the physical grinding-face normal independently of the plane fit.

Capture T1--T4 while the complete tool face is flat on a validated plane:

  cd surface_grinding_controller
  ./tools/capture_tool_axis grinding_tool horizontal T1
  ./tools/capture_tool_axis grinding_tool horizontal T2
  ./tools/capture_tool_axis grinding_tool horizontal T3
  ./tools/capture_tool_axis grinding_tool horizontal T4
  cd ..
  python3 experiments/calibration/prepare_tool_axis_calibration.py grinding_tool

T1--T3 record EE rotations at different yaw angles while the face is flat.
Their invariant right-singular direction is the physical tool normal in EE
coordinates. T4 is held out. The fitted plane normal is used only for a
separate base-frame sanity check, not to estimate the tool axis.
"""

import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_PROFILES = ("grinding_tool",)
MAX_ANGULAR_ERROR_DEG = 0.5
MAX_PLANE_DISAGREEMENT_DEG = 2.0
MIN_SINGULAR_GAP = 0.005


def angle_deg(a, b):
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def parameter_values(path):
    values = {}
    with open(path) as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if "=" in line:
                key, value = [part.strip() for part in line.split("=", 1)]
                values[key] = value
    return values


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in TOOL_PROFILES:
        sys.exit("usage: prepare_tool_axis_calibration.py grinding_tool")
    profile = sys.argv[1]
    profile_dir = os.path.join(HERE, "tools", profile)
    samples_path = os.path.join(profile_dir, "tool_axis_samples.csv")
    overlay_path = os.path.join(profile_dir, "tool_axis_overlay.txt")
    report_path = os.path.join(
        profile_dir, "tool_axis_calibration_report.txt"
    )

    if not os.path.exists(samples_path):
        sys.exit(
            f"Missing {samples_path}\n"
            "Capture T1--T4 with the complete tool face flat first."
        )

    samples = []
    with open(samples_path, newline="") as handle:
        rows = csv.DictReader(handle)
        required = {
            "label", "plane_profile",
            "r00", "r01", "r02",
            "r10", "r11", "r12",
            "r20", "r21", "r22",
        }
        if rows.fieldnames is None or not required.issubset(rows.fieldnames):
            sys.exit(
                f"{samples_path} must contain "
                "label,plane_profile and the nine R_EE entries r00--r22"
            )
        for row in rows:
            rotation = np.array(
                [
                    [float(row["r00"]), float(row["r01"]), float(row["r02"])],
                    [float(row["r10"]), float(row["r11"]), float(row["r12"])],
                    [float(row["r20"]), float(row["r21"]), float(row["r22"])],
                ],
                dtype=float,
            )
            if not np.all(np.isfinite(rotation)):
                sys.exit(f"Invalid rotation sample {row['label']}")
            if (
                np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 1e-6
                or abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6
            ):
                sys.exit(f"R_EE for {row['label']} is not a valid rotation.")
            samples.append(
                (row["label"].strip(), row["plane_profile"].strip(), rotation)
            )

    labels = [label for label, _, _ in samples]
    if labels[:4] != ["T1", "T2", "T3", "T4"] or len(samples) < 4:
        sys.exit("The first four samples must be ordered T1, T2, T3, T4.")
    plane_profiles = {plane for _, plane, _ in samples[:4]}
    if len(plane_profiles) != 1:
        sys.exit("T1--T4 must use the same validated plane profile.")
    plane_profile = next(iter(plane_profiles))

    fit = np.array([rotation for _, _, rotation in samples[:3]])
    mean_rotation = fit.mean(axis=0)
    left, singular_values, right_t = np.linalg.svd(mean_rotation)
    tool_axis_ee = right_t[0].copy()
    base_axis = left[:, 0].copy()
    if float(np.dot(tool_axis_ee, np.array([0.0, 0.0, 1.0]))) < 0.0:
        tool_axis_ee = -tool_axis_ee
        base_axis = -base_axis
    tool_axis_ee /= float(np.linalg.norm(tool_axis_ee))
    base_axis /= float(np.linalg.norm(base_axis))

    fit_errors = [
        angle_deg(rotation @ tool_axis_ee, base_axis)
        for rotation in fit
    ]
    held_label, _, held_rotation = samples[3]
    held_error = angle_deg(held_rotation @ tool_axis_ee, base_axis)
    nominal_offset = angle_deg(
        tool_axis_ee, np.array([0.0, 0.0, 1.0])
    )
    singular_gap = float(singular_values[0] - singular_values[1])

    repo = os.path.normpath(os.path.join(HERE, "..", ".."))
    plane_values = parameter_values(
        os.path.join(
            HERE, "planes", plane_profile, "plane_overlay.txt"
        )
    )
    common_values = parameter_values(
        os.path.join(
            repo, "surface_grinding_controller", "params", "common.txt"
        )
    )
    a = math.radians(
        float(plane_values["alignment_target_tilt_angle_deg"])
    )
    b = math.radians(
        float(plane_values["alignment_target_tilt_angle_y_deg"])
    )
    surface_normal = np.array(
        [math.sin(b) * math.cos(a), -math.sin(a),
         math.cos(b) * math.cos(a)],
        dtype=float,
    )
    sign = 1.0 if float(common_values["tool_axis_target_sign"]) >= 0.0 else -1.0
    target_base_axis = sign * surface_normal
    plane_disagreement = angle_deg(base_axis, target_base_axis)

    passed = (
        max(fit_errors) <= MAX_ANGULAR_ERROR_DEG
        and held_error <= MAX_ANGULAR_ERROR_DEG
        and singular_gap >= MIN_SINGULAR_GAP
        and plane_disagreement <= MAX_PLANE_DISAGREEMENT_DEG
    )

    report_lines = [
        "Physical tool-axis calibration",
        f"tool profile: {profile}",
        f"reference plane profile: {plane_profile}",
        "fit samples: T1, T2, T3",
        f"held-out sample: {held_label}",
        "tool axis EE [-]: "
        f"[{tool_axis_ee[0]:+.12f}, {tool_axis_ee[1]:+.12f}, "
        f"{tool_axis_ee[2]:+.12f}]",
        "invariant base axis [-]: "
        f"[{base_axis[0]:+.12f}, {base_axis[1]:+.12f}, "
        f"{base_axis[2]:+.12f}]",
        f"offset from nominal +Z_EE [deg]: {nominal_offset:.6f}",
        "fit angular residuals [deg]: "
        + ", ".join(f"{value:.6f}" for value in fit_errors),
        f"held-out angular residual [deg]: {held_error:.6f}",
        f"acceptance limit [deg]: {MAX_ANGULAR_ERROR_DEG:.3f}",
        "mean-rotation singular values [-]: "
        + ", ".join(f"{value:.9f}" for value in singular_values),
        f"axis observability gap [-]: {singular_gap:.9f}",
        f"minimum observability gap [-]: {MIN_SINGULAR_GAP:.3f}",
        f"axis-to-plane disagreement [deg]: {plane_disagreement:.6f}",
        "axis-to-plane limit [deg]: "
        f"{MAX_PLANE_DISAGREEMENT_DEG:.3f}",
        f"status: {'PASS' if passed else 'FAIL'}",
    ]
    if not passed:
        report_lines.append(
            "WARNING: repeatability, yaw diversity, or plane agreement "
            "failed its acceptance limit."
        )

    os.makedirs(profile_dir, exist_ok=True)
    with open(overlay_path, "w") as handle:
        handle.write("# Generated by prepare_tool_axis_calibration.py.\n")
        handle.write(f"# Physical tool profile: {profile}.\n")
        handle.write(f"tool_axis_ee_x = {tool_axis_ee[0]:.12f}\n")
        handle.write(f"tool_axis_ee_y = {tool_axis_ee[1]:.12f}\n")
        handle.write(f"tool_axis_ee_z = {tool_axis_ee[2]:.12f}\n")
    with open(report_path, "w") as handle:
        handle.write("\n".join(report_lines) + "\n")

    print("\n".join(report_lines))
    print(f"\nwrote {overlay_path}")
    print(f"wrote {report_path}")


if __name__ == "__main__":
    main()
