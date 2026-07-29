#!/usr/bin/env python3
"""Create one named controller surface frame from measured probe points.

P1, P2 and P3 define the plane. The projected P1-to-P2 direction defines +t1,
and t2 = n x t1. P4 and any later rows are held out and reported as validation
distances. Coordinates are base-frame metres at the calibrated physical probe
point, not uncorrected end-effector positions.

Usage:
  cp experiments/calibration/planes/tilted/plane_points.example.csv \
     experiments/calibration/planes/tilted/plane_points.csv
  # Replace the example coordinates with measured points, then:
  python3 experiments/calibration/prepare_plane_calibration.py tilted
"""

import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES = ("tilted", "horizontal")


def load_points(path):
    points = []
    with open(path, newline="") as handle:
        rows = csv.DictReader(
            line for line in handle if line.strip() and not line.lstrip().startswith("#")
        )
        required = {"label", "x_m", "y_m", "z_m"}
        if rows.fieldnames is None or not required.issubset(rows.fieldnames):
            sys.exit(f"{path} must contain columns: label,x_m,y_m,z_m")
        for row in rows:
            points.append(
                (
                    row["label"].strip(),
                    np.array(
                        [float(row["x_m"]), float(row["y_m"]), float(row["z_m"])],
                        dtype=float,
                    ),
                )
            )
    if len(points) < 4:
        sys.exit(
            "Four points are required: P1--P3 fit the plane and P4 validates it."
        )
    expected = ["P1", "P2", "P3", "P4"]
    actual = [label for label, _ in points[:4]]
    if actual != expected:
        sys.exit(
            "The first four rows must be labelled and ordered P1, P2, P3, P4."
        )
    return points


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in PROFILES:
        sys.exit(
            "usage: prepare_plane_calibration.py tilted|horizontal"
        )
    profile = sys.argv[1]
    profile_dir = os.path.join(HERE, "planes", profile)
    points_path = os.path.join(profile_dir, "plane_points.csv")
    overlay_path = os.path.join(profile_dir, "plane_overlay.txt")
    report_path = os.path.join(profile_dir, "plane_calibration_report.txt")

    if not os.path.exists(points_path):
        sys.exit(
            f"Missing {points_path}\n"
            f"Capture P1--P4 for profile '{profile}', or copy that profile's "
            "plane_points.example.csv to plane_points.csv and enter measured "
            "base-frame probe coordinates."
        )

    labelled = load_points(points_path)
    fit = np.array([point for _, point in labelled[:3]])
    v12 = fit[1] - fit[0]
    v13 = fit[2] - fit[0]
    cross = np.cross(v12, v13)
    twice_area = float(np.linalg.norm(cross))
    edge_lengths = [
        float(np.linalg.norm(fit[1] - fit[0])),
        float(np.linalg.norm(fit[2] - fit[0])),
        float(np.linalg.norm(fit[2] - fit[1])),
    ]
    if twice_area < 1e-6:
        sys.exit("The first three points are collinear or too close together.")
    if min(edge_lengths) < 0.05:
        sys.exit("Use probe points at least 50 mm apart to limit angular error.")

    normal = cross / twice_area
    if normal[2] < 0.0:
        normal = -normal
    point = fit.mean(axis=0)
    tangent1 = v12 - normal * float(normal @ v12)
    tangent1_norm = float(np.linalg.norm(tangent1))
    if tangent1_norm < 0.05:
        sys.exit("P1 and P2 do not define a usable tangent direction.")
    tangent1 /= tangent1_norm
    tangent2 = np.cross(normal, tangent1)
    tangent2 /= float(np.linalg.norm(tangent2))

    # Inverse of n = R_y(b) R_x(a) e_z.
    a_rad = math.asin(float(np.clip(-normal[1], -1.0, 1.0)))
    b_rad = math.atan2(float(normal[0]), float(normal[2]))
    a_deg = math.degrees(a_rad)
    b_deg = math.degrees(b_rad)

    validation = []
    for label, candidate in labelled[3:]:
        distance = float(normal @ (candidate - point))
        validation.append((label, distance))

    report_lines = [
        "Three-point physical-plane calibration",
        f"profile: {profile}",
        f"fit points: {', '.join(label for label, _ in labelled[:3])}",
        "surface point [m]: "
        f"[{point[0]:+.9f}, {point[1]:+.9f}, {point[2]:+.9f}]",
        "surface normal [-]: "
        f"[{normal[0]:+.9f}, {normal[1]:+.9f}, {normal[2]:+.9f}]",
        "surface tangent t1 (P1->P2) [-]: "
        f"[{tangent1[0]:+.9f}, {tangent1[1]:+.9f}, {tangent1[2]:+.9f}]",
        "surface tangent t2 = n x t1 [-]: "
        f"[{tangent2[0]:+.9f}, {tangent2[1]:+.9f}, {tangent2[2]:+.9f}]",
        f"tilt a about base x [deg]: {a_deg:+.6f}",
        f"tilt b about base y [deg]: {b_deg:+.6f}",
        "fit triangle edge lengths [mm]: "
        + ", ".join(f"{1000.0 * value:.1f}" for value in edge_lengths),
    ]
    report_lines.append("held-out signed distances [mm]:")
    report_lines.extend(
        f"  {label}: {1000.0 * distance:+.3f}"
        for label, distance in validation
    )
    max_distance = max(abs(distance) for _, distance in validation)
    if max_distance > 0.001:
        report_lines.append(
            "WARNING: at least one validation point is more than 1 mm "
            "from the fitted plane."
        )

    with open(overlay_path, "w") as handle:
        handle.write("# Generated by prepare_plane_calibration.py.\n")
        handle.write(f"# Physical plane profile: {profile}.\n")
        handle.write("use_start_as_surface_point = 0\n")
        handle.write(f"surface_point_x = {point[0]:.9f}\n")
        handle.write(f"surface_point_y = {point[1]:.9f}\n")
        handle.write(f"surface_point_z = {point[2]:.9f}\n")
        handle.write(f"alignment_target_tilt_angle_deg = {a_deg:.9f}\n")
        handle.write(f"alignment_target_tilt_angle_y_deg = {b_deg:.9f}\n")
        handle.write(f"alignment_target_tangent1_x = {tangent1[0]:.9f}\n")
        handle.write(f"alignment_target_tangent1_y = {tangent1[1]:.9f}\n")
        handle.write(f"alignment_target_tangent1_z = {tangent1[2]:.9f}\n")

    with open(report_path, "w") as handle:
        handle.write("\n".join(report_lines) + "\n")

    print("\n".join(report_lines))
    print(f"\nwrote {overlay_path}")
    print(f"wrote {report_path}")


if __name__ == "__main__":
    main()
