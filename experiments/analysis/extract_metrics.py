#!/usr/bin/env python3
"""Scan experiments/results/ and write one metrics row per run.

  python3 experiments/analysis/extract_metrics.py

Produces experiments/derived/metrics.csv -- small, text, and meant to be
committed. The raw 15 MB logs stay out of git; this file is what the plots and
the thesis tables are actually built from.

Every run is also cross-checked against the controller's own set-up report from
the saved terminal transcript. A disagreement between the CSV-derived value and
the printed value means one of them is wrong, and the run is flagged rather
than quietly averaged in.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sgc_log  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, ".."))
RESULTS = os.path.join(EXP, "results")
DERIVED = os.path.join(EXP, "derived")

FIELDS = [
    "run_id", "repeat", "series",
    "git_commit", "timestamp",
    "has_alignment_metric",
    "setup_present", "setup_duration_s",
    "tip_final_deg", "tip_max_deg", "tip_drift_last20pct_deg",
    "align_before_deg", "align_after_deg", "align_gain_deg",
    "force_final_N", "force_max_N", "force_steady_N",
    "edge_travel_mm", "tau_max_Nm", "tau_norm_max_Nm",
    "report_tip_deg", "report_force_N", "report_phase_time_s",
    "report_align_gain_deg",
    "axis_valid", "axis_trustworthy", "axis_angle_deg",
    "axis_from_edge_mm", "axis_pitch_mm_per_rad",
    "hold_present", "nullspace_mode",
    "sigma_start", "sigma_end", "sigma_gain",
    "tau_sigma_norm_mean", "nullspace_speed_mean", "speed_toward_better_mean",
    "direction_valid_fraction",
    "task_pos_error_drift_mm", "task_rot_error_drift_deg",
    "jacobian_null_residual_max",
    "pole_x", "pole_y", "pole_z", "pole_normal_mm",
    "flags",
]


def read_meta(path):
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                out[k.strip()] = v.strip()
    return out


def surface_normal_from_params(params_dir):
    """Recover the surface normal so the pole offset can be resolved into its
    normal component -- the physically meaningful coordinate for series B."""
    import math
    a_deg = b_deg = 0.0
    p = os.path.join(params_dir, "common.txt")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if "=" not in line:
                continue
            k, v = [x.strip() for x in line.split("=", 1)]
            if k == "alignment_target_tilt_angle_deg":
                a_deg = float(v)
            elif k == "alignment_target_tilt_angle_y_deg":
                b_deg = float(v)
    a, b = math.radians(a_deg), math.radians(b_deg)
    return (math.sin(b) * math.cos(a), -math.sin(a), math.cos(b) * math.cos(a))


def pole_from_params(params_dir):
    p = os.path.join(params_dir, "sequence.txt")
    vals = {}
    if not os.path.exists(p):
        return None
    with open(p) as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if "=" not in line:
                continue
            k, v = [x.strip() for x in line.split("=", 1)]
            if k.startswith("coupled_pole_from_edge_"):
                vals[k[-1]] = float(v)
    if len(vals) != 3:
        return None
    return (vals["x"], vals["y"], vals["z"])


def process_run(run_id, repeat_tag, run_dir):
    row = {k: "" for k in FIELDS}
    row["run_id"] = run_id
    row["repeat"] = repeat_tag
    row["series"] = run_id.split("_")[0]
    flags = []

    meta = read_meta(os.path.join(run_dir, "meta.txt"))
    row["git_commit"] = meta.get("git_commit", "")[:10]
    row["timestamp"] = meta.get("timestamp", "")
    if meta.get("git_dirty") == "yes":
        flags.append("dirty-tree")

    params_dir = os.path.join(run_dir, "params_effective")
    pole = pole_from_params(params_dir)
    normal = surface_normal_from_params(params_dir)
    if pole:
        row["pole_x"], row["pole_y"], row["pole_z"] = (f"{v:.6f}" for v in pole)
        if normal:
            s = sum(pole[i] * normal[i] for i in range(3))
            row["pole_normal_mm"] = f"{1000.0 * s:.2f}"

    general = None
    for name in sorted(os.listdir(run_dir)):
        if name.endswith(".csv") and "sigma_debug" not in name:
            general = os.path.join(run_dir, name)
            break

    if general is None:
        flags.append("no-general-log")
    else:
        try:
            m = sgc_log.setup_metrics(general)
            for k, v in m.items():
                if k in row:
                    row[k] = v
            if m.get("setup_present"):
                drift = m.get("tip_drift_last20pct_deg", 0.0)
                tip = m.get("tip_final_deg", 0.0)
                if tip > 0 and drift > 0.1 * tip:
                    flags.append("not-converged")
        except Exception as exc:  # noqa: BLE001
            flags.append(f"setup-parse-error({type(exc).__name__})")

        try:
            h = sgc_log.hold_metrics(general)
            for k, v in h.items():
                if k in row:
                    row[k] = v
            if h.get("hold_present") and h.get("task_pos_error_drift_mm", 0) > 1.0:
                flags.append("task-disturbed")
        except Exception as exc:  # noqa: BLE001
            flags.append(f"hold-parse-error({type(exc).__name__})")

    rep = sgc_log.parse_setup_report(os.path.join(run_dir, "terminal.log"))
    for k, v in rep.items():
        if k in row:
            row[k] = v

    # Cross-check CSV against the controller's own printed report.
    try:
        if row["tip_final_deg"] != "" and row["report_tip_deg"] != "":
            a, b = float(row["tip_final_deg"]), float(row["report_tip_deg"])
            if abs(a - b) > max(0.15, 0.05 * max(abs(a), abs(b))):
                flags.append(f"tip-mismatch(csv={a:.2f},report={b:.2f})")
    except (TypeError, ValueError):
        pass

    if row.get("axis_valid") == 1 and row.get("axis_trustworthy") == 0:
        flags.append("axis-untrustworthy")

    row["flags"] = ";".join(flags)
    return row


def main():
    if not os.path.isdir(RESULTS):
        sys.exit(f"no results directory: {RESULTS}")

    rows = []
    for run_id in sorted(os.listdir(RESULTS)):
        run_root = os.path.join(RESULTS, run_id)
        if not os.path.isdir(run_root):
            continue
        for repeat_tag in sorted(os.listdir(run_root)):
            run_dir = os.path.join(run_root, repeat_tag)
            if not os.path.isdir(run_dir):
                continue
            print(f"  {run_id}/{repeat_tag}")
            rows.append(process_run(run_id, repeat_tag, run_dir))

    if not rows:
        print("No runs found yet. Record some with experiments/run.sh first.")
        return

    os.makedirs(DERIVED, exist_ok=True)
    out = os.path.join(DERIVED, "metrics.csv")
    with open(out, "w") as f:
        f.write(",".join(FIELDS) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(k, "")).replace(",", ";")
                             for k in FIELDS) + "\n")

    flagged = [r for r in rows if r["flags"]]
    print(f"\nwrote {len(rows)} runs to {out}")
    if flagged:
        print(f"\n{len(flagged)} run(s) flagged:")
        for r in flagged:
            print(f"  {r['run_id']}/{r['repeat']}: {r['flags']}")


if __name__ == "__main__":
    main()
