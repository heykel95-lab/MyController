#!/usr/bin/env python3
"""Loader and metric extraction for surface_grinding_controller logs.

Deliberately depends only on numpy: pandas and scipy are not installed on the
robot PC and pip is unavailable, so anything heavier could not be run where the
data actually lives.

Handles both log schemas:
  - current (113 columns, includes alignment_angle_deg)
  - pre-alignment-metric (112 columns); alignment metrics are then reported as
    NaN rather than silently faked.
"""

import os

import numpy as np

# ControlPhase enum order in controller.h.
PHASE_APPROACH_ORIENT = 0
PHASE_APPROACH_DESCEND = 1
PHASE_SET_UP = 2
PHASE_GRIND = 3
PHASE_HOLD = 4
PHASE_MANUAL_GUIDE = 5

PHASE_NAMES = {
    0: "approach_orient",
    1: "approach_descend",
    2: "set_up",
    3: "grind",
    4: "hold",
    5: "manual_guide",
}

# Below this rotation the finite screw axis is numerically meaningless: the
# axis position divides a displacement by a near-zero angle. The controller's
# own guard is 1e-3 rad (0.057 deg), which is far too permissive -- a 0.6 deg
# suppressed run passes it and yields an axis hundreds of mm off. We use a
# defensible reporting threshold instead.
MIN_TRUSTWORTHY_ANGLE_DEG = 2.0


def read_csv(path, columns=None):
    """Read a controller CSV into {name: 1-D array}.

    columns=None loads everything. Passing the subset you need is much faster
    on the 120k-row general log.
    """
    with open(path) as f:
        header = f.readline().strip().split(",")
    header = [h.strip() for h in header]

    if columns is None:
        use = list(range(len(header)))
        names = header
    else:
        missing = [c for c in columns if c not in header]
        if missing:
            raise KeyError(f"{os.path.basename(path)} lacks columns {missing}")
        use = [header.index(c) for c in columns]
        names = list(columns)

    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=use, ndmin=2)
    return {name: data[:, i] for i, name in enumerate(names)}, header


def has_alignment_metric(header):
    return "alignment_angle_deg" in header


def phase_mask(phase, wanted):
    return phase == wanted


def _norm3(d, prefix):
    return np.sqrt(d[f"{prefix}_x"] ** 2
                   + d[f"{prefix}_y"] ** 2
                   + d[f"{prefix}_z"] ** 2)


def setup_metrics(path):
    """Extract the per-run set-up metrics used by the thesis tables.

    Returns a dict of scalars. Values that cannot be computed from the
    available schema are NaN, never invented.
    """
    wanted = [
        "time", "phase",
        "e_R_x", "e_R_y", "e_R_z",
        "p_EE_x", "p_EE_y", "p_EE_z",
        "tool_contact_x", "tool_contact_y", "tool_contact_z",
        "first_contact_x", "first_contact_y", "first_contact_z",
        "external_force_x", "external_force_y", "external_force_z",
        "contact_force_bias_x", "contact_force_bias_y", "contact_force_bias_z",
        "tau_cmd_1", "tau_cmd_2", "tau_cmd_3", "tau_cmd_4",
        "tau_cmd_5", "tau_cmd_6", "tau_cmd_7",
    ]
    with open(path) as f:
        header = [h.strip() for h in f.readline().strip().split(",")]
    if has_alignment_metric(header):
        wanted.append("alignment_angle_deg")

    d, header = read_csv(path, wanted)

    out = {
        "n_rows": len(d["time"]),
        "duration_s": float(d["time"][-1] - d["time"][0]) if len(d["time"]) else np.nan,
        "has_alignment_metric": has_alignment_metric(header),
    }

    setup = phase_mask(d["phase"], PHASE_SET_UP)
    out["setup_samples"] = int(setup.sum())
    if out["setup_samples"] < 2:
        out["setup_present"] = False
        return out
    out["setup_present"] = True

    t = d["time"][setup]
    out["setup_duration_s"] = float(t[-1] - t[0])

    # Tip angle: rotation away from the orientation frozen at the clearance
    # transition. This is how far the tool TURNED.
    e_r = np.sqrt(d["e_R_x"][setup] ** 2
                  + d["e_R_y"][setup] ** 2
                  + d["e_R_z"][setup] ** 2)
    out["tip_final_deg"] = float(np.degrees(e_r[-1]))
    out["tip_max_deg"] = float(np.degrees(e_r.max()))

    # Alignment: residual angle to the configured surface. This is how FLAT it
    # ended up, which is the quantity the thesis calls e_R before/after.
    if out["has_alignment_metric"]:
        a = d["alignment_angle_deg"][setup]
        out["align_before_deg"] = float(a[0])
        out["align_after_deg"] = float(a[-1])
        out["align_gain_deg"] = float(a[0] - a[-1])
    else:
        out["align_before_deg"] = np.nan
        out["align_after_deg"] = np.nan
        out["align_gain_deg"] = np.nan

    # Contact force relative to the bias captured at the clearance transition.
    fx = d["external_force_x"][setup] - d["contact_force_bias_x"][setup]
    fy = d["external_force_y"][setup] - d["contact_force_bias_y"][setup]
    fz = d["external_force_z"][setup] - d["contact_force_bias_z"][setup]
    f = np.sqrt(fx ** 2 + fy ** 2 + fz ** 2)
    out["force_final_N"] = float(f[-1])
    out["force_max_N"] = float(f.max())
    # Steady value: median of the last quarter, robust to the contact spike.
    tail = max(1, len(f) // 4)
    out["force_steady_N"] = float(np.median(f[-tail:]))

    # Edge travel: how far the pressed contact feature slid.
    ex = d["tool_contact_x"][setup] - d["first_contact_x"][setup]
    ey = d["tool_contact_y"][setup] - d["first_contact_y"][setup]
    ez = d["tool_contact_z"][setup] - d["first_contact_z"][setup]
    out["edge_travel_mm"] = float(1000.0 * np.sqrt(ex[-1] ** 2 + ey[-1] ** 2 + ez[-1] ** 2))

    tau = np.vstack([d[f"tau_cmd_{i}"][setup] for i in range(1, 8)])
    out["tau_max_Nm"] = float(np.abs(tau).max())
    out["tau_norm_max_Nm"] = float(np.linalg.norm(tau, axis=0).max())

    # Equilibrium check: how much the tip still moved over the last 20% of the
    # phase. Large values mean the reported number is a transient, not an
    # equilibrium, which invalidates the quasi-static reading.
    last = max(2, len(e_r) // 5)
    out["tip_drift_last20pct_deg"] = float(
        np.degrees(abs(e_r[-1] - e_r[-last])))

    return out


def hold_metrics(path):
    """Extract null-space metrics from a general log covering a hold test."""
    wanted = [
        "time", "phase", "nullspace_mode",
        "sigma_current", "sigma_difference", "sigma_direction_valid",
        "tau_sigma_norm", "nullspace_speed", "sigma_speed_toward_better",
        "sigma_Jn_norm", "tau_nullspace_norm",
        "e_p_x", "e_p_y", "e_p_z",
        "e_R_x", "e_R_y", "e_R_z",
    ]
    d, _ = read_csv(path, wanted)

    hold = phase_mask(d["phase"], PHASE_HOLD)
    out = {"hold_samples": int(hold.sum())}
    if out["hold_samples"] < 2:
        out["hold_present"] = False
        return out
    out["hold_present"] = True

    out["nullspace_mode"] = int(np.median(d["nullspace_mode"][hold]))
    s = d["sigma_current"][hold]
    out["sigma_start"] = float(s[0])
    out["sigma_end"] = float(s[-1])
    out["sigma_min"] = float(s.min())
    out["sigma_max"] = float(s.max())
    out["sigma_gain"] = float(s[-1] - s[0])

    out["tau_sigma_norm_mean"] = float(d["tau_sigma_norm"][hold].mean())
    out["tau_sigma_norm_max"] = float(d["tau_sigma_norm"][hold].max())
    out["nullspace_speed_mean"] = float(d["nullspace_speed"][hold].mean())
    out["speed_toward_better_mean"] = float(
        d["sigma_speed_toward_better"][hold].mean())
    out["direction_valid_fraction"] = float(
        d["sigma_direction_valid"][hold].mean())

    # Task invariance: null-space torque must not disturb the Cartesian task.
    # This is the empirical proof that the projector is correct.
    ep = _norm3({k: d[k][hold] for k in ("e_p_x", "e_p_y", "e_p_z")}, "e_p")
    er = _norm3({k: d[k][hold] for k in ("e_R_x", "e_R_y", "e_R_z")}, "e_R")
    out["task_pos_error_max_mm"] = float(1000.0 * ep.max())
    out["task_pos_error_drift_mm"] = float(1000.0 * abs(ep[-1] - ep[0]))
    out["task_rot_error_max_deg"] = float(np.degrees(er.max()))
    out["task_rot_error_drift_deg"] = float(np.degrees(abs(er[-1] - er[0])))
    out["jacobian_null_residual_max"] = float(d["sigma_Jn_norm"][hold].max())

    return out


def parse_setup_report(terminal_log_path):
    """Pull the controller's own set-up report out of the saved transcript.

    Gives an independent cross-check of the CSV-derived metrics: if the two
    disagree, one of them is wrong and the run should not be used.
    """
    out = {}
    if not os.path.exists(terminal_log_path):
        return out
    with open(terminal_log_path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("stop:"):
                for part in line.split("|"):
                    part = part.strip()
                    if part.startswith("tip="):
                        out["report_tip_deg"] = float(part[4:].split()[0])
                    elif part.startswith("F="):
                        out["report_force_N"] = float(part[2:].split()[0])
                    elif part.startswith("M="):
                        out["report_moment_Nm"] = float(part[2:].split()[0])
                    elif part.startswith("t="):
                        out["report_phase_time_s"] = float(part[2:].split()[0])
            elif line.startswith("alignment:"):
                for part in line.replace("alignment:", "").split("|"):
                    part = part.strip()
                    if part.startswith("before="):
                        out["report_align_before_deg"] = float(part[7:].split()[0])
                    elif part.startswith("after="):
                        out["report_align_after_deg"] = float(part[6:].split()[0])
                    elif part.startswith("gain="):
                        out["report_align_gain_deg"] = float(part[5:].split()[0])
            elif line.startswith("finite_axis:"):
                if "too small" in line:
                    out["axis_valid"] = 0
                    body = line.split("angle=")[1]
                    out["axis_angle_deg"] = float(body.split()[0])
                else:
                    body = line.replace("finite_axis:", "")
                    for part in body.split("|"):
                        part = part.strip()
                        if part.startswith("angle="):
                            out["axis_angle_deg"] = float(part[6:].split()[0])
                        elif part.startswith("axis_from_edge="):
                            vec = part.split("[")[1].split("]")[0]
                            v = [float(x) for x in vec.split(",")]
                            out["axis_from_edge_mm"] = float(
                                np.linalg.norm(v))
                            out["axis_from_edge_x_mm"] = v[0]
                            out["axis_from_edge_y_mm"] = v[1]
                            out["axis_from_edge_z_mm"] = v[2]
                        elif part.startswith("pitch="):
                            out["axis_pitch_mm_per_rad"] = float(part[6:].split()[0])
                    out["axis_valid"] = 1

    # Apply the honest trustworthiness threshold on top of the controller's
    # far more permissive internal guard.
    if "axis_angle_deg" in out:
        out["axis_trustworthy"] = int(
            out.get("axis_valid", 0) == 1
            and out["axis_angle_deg"] >= MIN_TRUSTWORTHY_ANGLE_DEG)
    return out


def read_overlay(path):
    """Read a setup overlay into {key: value} for labelling plots."""
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    return out
