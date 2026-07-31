#!/usr/bin/env python3
"""Build the thesis figures from experiments/derived/metrics.csv.

  python3 experiments/analysis/make_figures.py

Writes PDFs into experiments/figures/. Each figure corresponds to a specific
table or claim in the thesis, named in FIGURES below, so it is obvious which
result a plot is meant to support.

Flags come in two kinds and are treated differently.

DATA flags (not-converged, no-setup-phase, no-general-log, tip-mismatch,
task-disturbed, tool-play) mean the numbers themselves are untrustworthy for
the primary physical-tool claim. Those runs are excluded from every mean and
drawn hollow.

PROVENANCE flags -- currently just dirty-tree -- mean the repository had an
uncommitted change when the run was recorded. That says nothing about the
measurement: every run archives the exact configuration it used in
params_effective/, so it remains reproducible. Excluding these would have
dropped all twelve B3 runs, the largest effect in the campaign, on a
bookkeeping technicality. They are included, and marked in the caption.
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, ".."))
METRICS = os.path.join(EXP, "derived", "metrics.csv")
FIGURES = os.path.join(EXP, "figures")

# Text is matched to the thesis rather than to matplotlib's defaults. usetex
# is deliberately not used: it needs dvipng for the Agg backend and dvipng is
# not installed, and it would make every figure depend on a preamble kept
# somewhere else. Naming the same faces gets the same look without that.
#
#   "latex" -- Latin Modern Roman with Computer Modern maths, the face plain
#              LaTeX sets when the preamble loads no font package.
#   "times" -- Liberation Serif, the installed metric-compatible Times face,
#              with STIX maths. This is what newtxtext or mathptmx give.
#
# Switch here if the thesis loads a font package; nothing else needs changing.
FONT_STYLE = "latex"

_FONT_STYLES = {
    "latex": {
        "font.serif": ["Latin Modern Roman", "CMU Serif", "cmr10",
                       "DejaVu Serif"],
        "mathtext.fontset": "cm",
    },
    "times": {
        "font.serif": ["Liberation Serif", "Times New Roman", "Times",
                       "Nimbus Roman"],
        "mathtext.fontset": "stix",
    },
}

plt.rcParams.update({
    "font.family": "serif",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.3,
    # One legend everywhere: no box, no shadow, the same size relative to the
    # body text, and tight enough not to crowd the data.
    "legend.frameon": False,
    "legend.fontsize": 8,
    "legend.handlelength": 1.6,
    "legend.handletextpad": 0.5,
    "legend.labelspacing": 0.3,
    "legend.columnspacing": 1.2,
    "legend.borderaxespad": 0.4,
})
plt.rcParams.update(_FONT_STYLES[FONT_STYLE])


def load_metrics(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
        rows = []
        for line in f:
            if line.strip():
                rows.append(dict(zip(header, line.rstrip("\n").split(","))))
    return rows


def fnum(row, key):
    v = row.get(key, "")
    if v in ("", "nan", "None"):
        return np.nan
    try:
        return float(v)
    except ValueError:
        return np.nan


EXCLUDED_LABEL = "excluded (data flag)"


def errorbar_from_buckets(ax, buckets, label, color, marker="o"):
    xs = sorted(k for k in buckets if not np.isnan(k))
    if not xs:
        return False
    means, errs, plotted = [], [], []
    for x in xs:
        vals = buckets[x]["good"]
        if not vals:
            continue
        plotted.append(x)
        means.append(np.mean(vals))
        errs.append(np.std(vals, ddof=1) if len(vals) > 1 else 0.0)
    if plotted:
        ax.errorbar(plotted, means, yerr=errs, marker=marker, capsize=3,
                    label=label, color=color, linewidth=1.5)
    # Excluded runs drawn hollow so nothing is hidden. Labelled once per axes,
    # so the legend says what a hollow marker is instead of leaving it to a
    # caption the figure may get separated from.
    labelled = any(h.get_label() == EXCLUDED_LABEL for h in ax.get_lines())
    for x in xs:
        for v in buckets[x]["bad"]:
            ax.plot(x, v, marker="o", mfc="none", mec="0.6", ms=5,
                    linestyle="none",
                    label=None if labelled else EXCLUDED_LABEL)
            labelled = True
    return bool(plotted)


def figure_legend(fig, ax, ncol=3):
    """One legend under a multi-panel figure.

    An in-axes legend on a narrow subplot lands on the y label or the data.
    Below the figure it belongs to every panel at once, which is what a shared
    series list means anyway, and bbox_inches="tight" keeps it in the crop.
    """
    handles, labels = ax.get_legend_handles_labels()
    if not handles:
        return
    fig.legend(handles, labels, loc="lower center", ncol=ncol,
               bbox_to_anchor=(0.5, -0.04), fontsize=8)


def save(fig, name):
    os.makedirs(FIGURES, exist_ok=True)
    path = os.path.join(FIGURES, name)
    if fig._suptitle is None:
        fig.tight_layout()
    else:
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {os.path.relpath(path, EXP)}")
    return path


# ---------------------------------------------------------------------------


def fig_a2_stiffness(rows):
    """Replaces the invented Table 5.1 (candidate stiffness settings)."""
    sub = [r for r in rows if r["run_id"].startswith("A2_KRtan")]
    if not sub:
        return None
    for r in sub:
        r["_kr"] = float(r["run_id"].split("_")[-1])

    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.0))
    for ax, key, ylabel in (
        (axes[0], "align_gain_deg", "alignment gain [deg]"),
        (axes[1], "force_steady_N", "steady contact force [N]"),
        (axes[2], "tau_norm_max_Nm", "peak commanded torque [Nm]"),
    ):
        buckets = {}
        for r in sub:
            y = fnum(r, key)
            if np.isnan(y):
                continue
            buckets.setdefault(r["_kr"], {"good": [], "bad": []})
            buckets[r["_kr"]]["good" if not data_suspect(r) else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.set_xlabel(r"$K_{R,t_1}=K_{R,t_2}$ [Nm/rad]")
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes[0])
    fig.suptitle("Rotational stiffness sweep", fontsize=10)
    return save(fig, "A2_stiffness_sweep.pdf")


def _axis_study_buckets(rows, prefix, xkey, ykey):
    buckets = {}
    for row in rows:
        if not row["run_id"].startswith(prefix):
            continue
        x, y = fnum(row, xkey), fnum(row, ykey)
        if np.isnan(x) or np.isnan(y):
            continue
        buckets.setdefault(x, {"good": [], "bad": []})
        bucket = "bad" if data_suspect(row) else "good"
        buckets[x][bucket].append(y)
    return buckets


def fig_d_axis_stiffness(rows):
    """Independent t1/t2 stiffness effects at a fixed 10-degree mismatch."""
    if not any(r["run_id"].startswith(("D1_", "D2_")) for r in rows):
        return None

    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    panels = (
        ("align_gain_deg", "physical-plane improvement [deg]"),
        ("alignment_time90_s", "90% alignment time [s]"),
        ("force_steady_N", "steady estimated normal load [N]"),
    )
    for ax, (key, ylabel) in zip(axes, panels):
        t1 = _axis_study_buckets(rows, "D1_KRt1_", "setup_KR_t1", key)
        t2 = _axis_study_buckets(rows, "D2_KRt2_", "setup_KR_t2", key)
        errorbar_from_buckets(ax, t1, r"$t_1$ excitation", "C0", marker="o")
        errorbar_from_buckets(ax, t2, r"$t_2$ excitation", "C1", marker="s")
        ax.set_xlabel(r"excited-axis $K_R$ [Nm/rad]")
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes[0])
    fig.suptitle("Axis-specific rotational stiffness", fontsize=10)
    return save(fig, "D_axis_stiffness.pdf")


def fig_d_initial_angle(rows):
    """Alignment response at 0, 5 and 10 degrees for each tangent axis."""
    selected = [
        row
        for row in rows
        if row["run_id"] == "D0_flat_00deg"
        or row["run_id"].startswith("D3_angle_")
        or row["run_id"] in ("D1_KRt1_05", "D2_KRt2_05")
    ]
    if not selected:
        return None

    fig, ax = plt.subplots(figsize=(5.4, 3.5))
    for axis, prefixes, xkey, color, marker in (
        (r"$t_1$", ("D0_", "D3_angle_t1_", "D1_KRt1_05"),
         "tool_offset_t1_deg", "C0", "o"),
        (r"$t_2$", ("D0_", "D3_angle_t2_", "D2_KRt2_05"),
         "tool_offset_t2_deg", "C1", "s"),
    ):
        buckets = {}
        for row in selected:
            if not row["run_id"].startswith(prefixes):
                continue
            x = abs(fnum(row, xkey))
            y = fnum(row, "align_gain_deg")
            if np.isnan(x) or np.isnan(y):
                continue
            buckets.setdefault(x, {"good": [], "bad": []})
            bucket = "bad" if data_suspect(row) else "good"
            buckets[x][bucket].append(y)
        errorbar_from_buckets(
            ax, buckets, f"offset about {axis}", color, marker=marker
        )
    ax.set_xlabel("initial tool-plane angle [deg]")
    ax.set_ylabel("physical-plane improvement [deg]")
    ax.legend()
    ax.set_title("D0/D3: initial-angle response", fontsize=10)
    return save(fig, "D_initial_angle.pdf")


def _main_rows(rows, prefixes):
    return [
        row for row in rows
        if row["run_id"].startswith(prefixes)
    ]


def fig_main_initial_angle(rows):
    selected = _main_rows(rows, ("MAIN_A",))
    if not selected:
        return None
    fig, ax = plt.subplots(figsize=(5.6, 3.5))
    for axis, ids, color, marker in (
        (1, ("MAIN_A0_", "MAIN_A1_", "MAIN_A2_"),
         "C0", "o"),
        (2, ("MAIN_A0_", "MAIN_A3_", "MAIN_A4_"),
         "C1", "s"),
    ):
        buckets = {}
        for row in selected:
            if not row["run_id"].startswith(ids):
                continue
            x = abs(fnum(row, f"align_t{axis}_before_deg"))
            y = fnum(row, f"align_t{axis}_improve_deg")
            if np.isnan(x) or np.isnan(y):
                continue
            buckets.setdefault(x, {"good": [], "bad": []})
            buckets[x]["bad" if data_suspect(row) else "good"].append(y)
        errorbar_from_buckets(
            ax, buckets, rf"$t_{axis}$ excitation", color, marker=marker
        )
    ax.axhline(0.0, color="0.45", linewidth=1)
    ax.set_xlabel("measured initial tool-plane angle [deg]")
    ax.set_ylabel("excited-axis error removed [deg]")
    ax.legend()
    ax.set_title("Case A: initial-angle response", fontsize=10)
    return save(fig, "MAIN_A_angle.pdf")


def fig_main_rotational_stiffness(rows):
    selected = _main_rows(rows, ("MAIN_A2_", "MAIN_A4_", "MAIN_B"))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, key, ylabel in (
        (axes[0], "axis_improvement", "excited-axis error removed [deg]"),
        (axes[1], "alignment_time90_s", "90% alignment time [s]"),
        (axes[2], "force_steady_N", "steady estimated load [N]"),
    ):
        for axis, ids, xkey, color, marker in (
            (1, ("MAIN_A2_", "MAIN_B1_"), "setup_KR_t1", "C0", "o"),
            (2, ("MAIN_A4_", "MAIN_B2_"), "setup_KR_t2", "C1", "s"),
        ):
            buckets = {}
            for row in selected:
                if not row["run_id"].startswith(ids):
                    continue
                x = fnum(row, xkey)
                y = fnum(
                    row,
                    f"align_t{axis}_improve_deg"
                    if key == "axis_improvement" else key,
                )
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax, buckets, rf"$t_{axis}$ excitation", color, marker=marker
            )
        ax.set_xlabel(r"excited-axis $K_R$ [Nm/rad]")
        ax.set_ylabel(ylabel)
    axes[0].legend()
    fig.suptitle("Case B: rotational stiffness", fontsize=10)
    return save(fig, "MAIN_B_KR.pdf")


def fig_main_translational_stiffness(rows):
    selected = _main_rows(rows, ("MAIN_A2_", "MAIN_A4_", "MAIN_C"))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, key, ylabel in (
        (axes[0], "axis_improvement", "excited-axis error removed [deg]"),
        (axes[1], "edge_travel_mm", "selected-feature travel [mm]"),
        (axes[2], "force_steady_N", "steady estimated load [N]"),
    ):
        for axis, ids, xkey, color, marker in (
            (1, ("MAIN_A2_", "MAIN_C1_KPt2_"), "setup_Kp_t2", "C0", "o"),
            (2, ("MAIN_A4_", "MAIN_C2_KPt1_"), "setup_Kp_t1", "C1", "s"),
        ):
            buckets = {}
            for row in selected:
                if not row["run_id"].startswith(ids):
                    continue
                x = fnum(row, xkey)
                y = fnum(
                    row,
                    f"align_t{axis}_improve_deg"
                    if key == "axis_improvement" else key,
                )
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax, buckets, rf"$t_{axis}$ excitation", color, marker=marker
            )
        ax.set_xscale("log")
        ax.set_xlabel(r"cross-direction $K_p$ [N/m]")
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes[0])
    fig.suptitle("Case C: translational stiffness", fontsize=10)
    return save(fig, "MAIN_C_KP.pdf")


def fig_main_interaction(rows):
    selected = _main_rows(
        rows,
        ("MAIN_A2_", "MAIN_A4_", "MAIN_B1_KRt1_50",
         "MAIN_B2_KRt2_50", "MAIN_C1_KPt2_0300",
         "MAIN_C2_KPt1_0300", "MAIN_C1_interaction",
         "MAIN_C2_interaction"),
    )
    if not selected:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(7.5, 3.2), sharey=True)
    for axis, ax in enumerate(axes, start=1):
        kr_key = f"setup_KR_t{axis}"
        kp_key = "setup_Kp_t2" if axis == 1 else "setup_Kp_t1"
        for kp, color, marker in ((300.0, "C2", "s"), (2000.0, "C0", "o")):
            buckets = {}
            for row in selected:
                is_axis = (
                    (axis == 1 and row["run_id"].startswith(
                        ("MAIN_A2_", "MAIN_B1_", "MAIN_C1_")))
                    or
                    (axis == 2 and row["run_id"].startswith(
                        ("MAIN_A4_", "MAIN_B2_", "MAIN_C2_")))
                )
                if not is_axis or abs(fnum(row, kp_key) - kp) > 1e-6:
                    continue
                x = fnum(row, kr_key)
                y = fnum(row, f"align_t{axis}_improve_deg")
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax, buckets, rf"$K_p={kp:.0f}$ N/m", color, marker=marker
            )
        ax.set_xlabel(rf"$K_{{R,t_{axis}}}$ [Nm/rad]")
        ax.set_title(rf"$t_{axis}$ excitation", fontsize=9)
    axes[0].set_ylabel("excited-axis error removed [deg]")
    figure_legend(fig, axes[0])
    fig.suptitle(r"Case C: $K_R$--$K_p$ interaction", fontsize=10)
    return save(fig, "MAIN_C_interaction.pdf")


def fig_main_compliance_centre(rows):
    selected = _main_rows(rows, ("MAIN_D",))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, key, ylabel in (
        (axes[0], "axis_improvement", "excited-axis error removed [deg]"),
        (axes[1], "alignment_time90_s", "90% alignment time [s]"),
        (axes[2], "force_steady_N", "steady estimated load [N]"),
    ):
        for axis, prefix, xkey, color, marker in (
            (1, "MAIN_D1_", "rc_t2_mm", "C0", "o"),
            (2, "MAIN_D2_", "rc_t1_mm", "C1", "s"),
        ):
            buckets = {}
            for row in selected:
                if not row["run_id"].startswith(prefix):
                    continue
                x = fnum(row, xkey)
                y = fnum(
                    row,
                    f"align_t{axis}_improve_deg"
                    if key == "axis_improvement" else key,
                )
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax, buckets,
                rf"$t_{axis}$ error; perpendicular lever",
                color, marker=marker,
            )
        # D3 has no perpendicular lever to sit on the x axis: its pole is
        # 20 mm along the tool axis, off the tangent plane the sweep lives in.
        # Drawn as a level rather than a point, so the comparison against the
        # r_c = 0 runs is readable without inventing an x position for it.
        for axis, prefix, color in ((1, "MAIN_D3_t1_", "C0"),
                                    (2, "MAIN_D3_t2_", "C1")):
            vals = [
                fnum(row, f"align_t{axis}_improve_deg"
                     if key == "axis_improvement" else key)
                for row in selected
                if row["run_id"].startswith(prefix) and not data_suspect(row)
            ]
            vals = [v for v in vals if not np.isnan(v)]
            if vals:
                ax.axhline(
                    float(np.mean(vals)), color=color, linewidth=1.1,
                    linestyle=":",
                    label=rf"$t_{axis}$ error; pole at face centre",
                )
        ax.axhline(0.0, color="0.45", linewidth=1)
        ax.set_xlabel("commanded perpendicular lever component [mm]")
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes[0], ncol=4)
    fig.suptitle(
        r"Case D: commanded centre of compliance, "
        r"$r_c=p_{\mathrm{TCP}}-p_c$",
        fontsize=10,
    )
    return save(fig, "MAIN_D_CoC.pdf")


# Categorical slots 1 and 2 of the validated default palette. Two series only:
# the all-pairs floors hold for the first three slots, and marker shape carries
# identity as well as hue so the figure survives greyscale printing.
SERIES_B2 = "#2a78d6"
SERIES_B3 = "#eb6834"
SERIES_B4 = "#1baf7a"   # slot 3; first three slots validate all-pairs
INK = "#0b0b0b"
INK_MUTED = "#52514e"


PROVENANCE_FLAGS = {"dirty-tree"}


def data_suspect(row):
    """True if the run's numbers are untrustworthy, not merely its provenance."""
    flags = {f.split("(")[0] for f in row.get("flags", "").split(";") if f}
    return bool(flags - PROVENANCE_FLAGS)


def fig_main_tool_axis_tilt(rows):
    """Does the t1/t2 asymmetry belong to the plane or to the tool face?

    A2 and A4 tilt about the surface tangents; E1 tilts the same 10 deg about
    the tool's own axes, which the commanded twist puts 25 deg away from them.
    If the difference tracks the tool axes the asymmetry is the 40 x 120 mm
    face; if it tracks the surface axes it is the plane.
    """
    groups = [
        (r"about $t_1$", ("MAIN_A2_",), 1, "C0"),
        (r"about $t_2$", ("MAIN_A4_",), 2, "C1"),
        (r"about $Y_{EE}$ (120 mm edge)", ("MAIN_E1_tilt_about_y_long",), None, "C2"),
        (r"about $X_{EE}$ (40 mm edge)", ("MAIN_E1_tilt_about_x_short",), None, "C3"),
    ]
    labels, means, errs, colors = [], [], [], []
    for label, prefixes, axis, color in groups:
        vals = []
        for row in rows:
            if not any(row["run_id"].startswith(p) for p in prefixes):
                continue
            if data_suspect(row):
                continue
            # A tool-axis tilt lands on both surface axes at once, so the
            # scalar gain is the only measure common to all four groups.
            v = fnum(row, "align_gain_deg")
            if not np.isnan(v):
                vals.append(v)
        if not vals:
            continue
        labels.append(label)
        means.append(float(np.mean(vals)))
        errs.append(float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)
        colors.append(color)
    if not means:
        return None

    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    xs = np.arange(len(means))
    for i, (x, m, e, c, lb) in enumerate(zip(xs, means, errs, colors, labels)):
        ax.bar(x, m, yerr=e, capsize=3, color=c, width=0.6,
               edgecolor="0.25", linewidth=0.6, label=lb)
    ax.set_xticks(xs)
    ax.set_xticklabels(["surface\n$t_1$", "surface\n$t_2$",
                        "tool\n$Y_{EE}$", "tool\n$X_{EE}$"][:len(means)])
    ax.set_ylabel("alignment error removed [deg]")
    ax.axhline(0.0, color="0.45", linewidth=1)
    ax.legend(ncol=2, fontsize=7)
    ax.set_title(r"Case E: 10 deg tilt about surface vs tool axes", fontsize=10)
    return save(fig, "MAIN_E_tool_axis.pdf")


def fig_plane_validation(rows):
    """Matched horizontal-primary and tilted-validation baseline conditions."""
    definitions = (
        ("0 deg", "MAIN_A0_", "VALID_T0_", None),
        (r"$10^\circ$ about $t_1$", "MAIN_A2_", "VALID_T1_", 1),
        (r"$10^\circ$ about $t_2$", "MAIN_A4_", "VALID_T2_", 2),
    )
    selected = [
        row for row in rows
        if any(
            row["run_id"].startswith((horizontal, tilted))
            for _, horizontal, tilted, _ in definitions
        )
    ]
    if not any(row["run_id"].startswith("VALID_T") for row in selected):
        return None

    fig, axes = plt.subplots(1, 2, figsize=(7.8, 3.3), sharex=True)
    x_base = np.arange(len(definitions), dtype=float)
    for profile, prefix_index, offset, color, marker in (
        ("horizontal", 1, -0.08, "C0", "o"),
        ("tilted", 2, +0.08, "C1", "s"),
    ):
        initial_means, initial_errs = [], []
        residual_means, residual_errs = [], []
        for _, horizontal_prefix, tilted_prefix, axis in definitions:
            prefix = (horizontal_prefix, tilted_prefix)[prefix_index - 1]
            matching = [
                row for row in selected
                if row["run_id"].startswith(prefix) and not data_suspect(row)
            ]
            if axis is None:
                initial = [fnum(row, "align_before_deg") for row in matching]
                residual = [fnum(row, "align_after_deg") for row in matching]
            else:
                initial = [
                    abs(fnum(row, f"align_t{axis}_before_deg"))
                    for row in matching
                ]
                residual = [
                    abs(fnum(row, f"align_t{axis}_after_deg"))
                    for row in matching
                ]
            initial = [value for value in initial if not np.isnan(value)]
            residual = [value for value in residual if not np.isnan(value)]
            initial_means.append(np.mean(initial) if initial else np.nan)
            residual_means.append(np.mean(residual) if residual else np.nan)
            initial_errs.append(
                np.std(initial, ddof=1) if len(initial) > 1 else 0.0
            )
            residual_errs.append(
                np.std(residual, ddof=1) if len(residual) > 1 else 0.0
            )
        for ax, means, errors in (
            (axes[0], initial_means, initial_errs),
            (axes[1], residual_means, residual_errs),
        ):
            ax.errorbar(
                x_base + offset, means, yerr=errors, label=profile,
                color=color, marker=marker, linewidth=1.5, capsize=3,
            )

    axes[0].set_ylabel("measured initial error [deg]")
    axes[1].set_ylabel("residual after set-up [deg]")
    for ax in axes:
        ax.set_xticks(x_base)
        ax.set_xticklabels([definition[0] for definition in definitions])
    axes[0].legend()
    fig.suptitle("Horizontal primary and tilted validation", fontsize=10)
    return save(fig, "PLANE_validation.pdf")


def _pole_points(rows, prefix, xkey):
    """(x, improvement) for every run of a series that commanded a pole."""
    xs, ys = [], []
    for r in rows:
        if not r["run_id"].startswith(prefix) or data_suspect(r):
            continue
        x, y = fnum(r, xkey), fnum(r, "align_improve_real_deg")
        if np.isnan(x) or np.isnan(y):
            continue
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)


def fig_b_pole_axis(rows):
    """Which component of the pole actually governs alignment.

    Plotted against the improvement toward the MEASURED plane, not the
    configured one -- see sgc_log.alignment_improvement_deg. The two panels are
    the same runs against the two pole components, which is why this is small
    multiples and not a second y-axis.
    """
    series = (("B2_pole_normal", "B2: swept along normal n", SERIES_B2, "o"),
              ("B3_pole_tangent_", "B3: swept along tangent $t_1$", SERIES_B3, "^"),
              ("B4_pole_tangent2", "B4: swept along tangent $t_2$", SERIES_B4, "s"))
    if not any(_pole_points(rows, p, "pole_cmd_x_mm")[0].size for p, *_ in series):
        return None

    fig, axes = plt.subplots(1, 3, figsize=(11.0, 3.4), sharey=True)
    for ax, xkey, xlabel in (
            (axes[0], "pole_cmd_x_mm", "pole offset along $t_1$ [mm]"),
            (axes[1], "pole_cmd_y_mm", "pole offset along $t_2$ [mm]"),
            (axes[2], "pole_cmd_z_mm", "pole offset along normal [mm]")):
        allx, ally = [], []
        for prefix, label, color, marker in series:
            x, y = _pole_points(rows, prefix, xkey)
            if not x.size:
                continue
            ax.plot(x, y, marker, color=color, ms=7, mew=0.8, mec="white",
                    linestyle="none", label=label)
            allx.append(x)
            ally.append(y)
        if allx:
            X = np.concatenate(allx)
            Y = np.concatenate(ally)
            # The two in-plane axes turn over inside the tested range, so a
            # straight line through them is not just imprecise, it points the
            # wrong way past the optimum. The normal axis stays linear.
            deg = 1 if xkey == "pole_cmd_z_mm" else 2
            coef = np.polyfit(X, Y, deg)
            pred = np.polyval(coef, X)
            ss = 1.0 - ((Y - pred) ** 2).sum() / ((Y - Y.mean()) ** 2).sum()
            grid = np.linspace(X.min(), X.max(), 200)
            ax.plot(grid, np.polyval(coef, grid), "-", color=INK_MUTED,
                    linewidth=1.5, zorder=0)
            if deg == 2:
                peak = -coef[1] / (2.0 * coef[0])
                note = f"$R^2$ = {ss:.3f}\noptimum {peak:+.0f} mm"
            else:
                note = f"$R^2$ = {ss:.3f}\n{coef[0]:+.3f} deg/mm"
            ax.annotate(note, xy=(0.04, 0.94), xycoords="axes fraction",
                        va="top", fontsize=8, color=INK)
        ax.axhline(0.0, color="0.55", linewidth=1, zorder=0)
        ax.set_xlabel(xlabel, color=INK_MUTED)
    axes[0].set_ylabel("alignment gained toward the real plane [deg]",
                       color=INK_MUTED)
    axes[0].legend(fontsize=8, loc="lower right", frameon=False)
    fig.suptitle("B: both in-plane pole components govern alignment; "
                 "the normal component does not", fontsize=10, color=INK)
    return save(fig, "B_pole_component.pdf")


def fig_b_pole_surface(rows):
    """The contribution in one plot: alignment as a surface over the pole plane.

    Diverging scale, because the quantity crosses zero and the sign is the
    point -- above zero the coupling drives the tool onto the surface, below it
    drives the tool away. Neutral gray at zero, blue/red poles.
    """
    pts = []
    for r in rows:
        if not r["run_id"].startswith(("B1_", "B2_", "B3_", "B4_")):
            continue
        if data_suspect(r):
            continue
        x, y = fnum(r, "pole_cmd_x_mm"), fnum(r, "pole_cmd_y_mm")
        v = fnum(r, "align_improve_real_deg")
        if np.isnan(x) or np.isnan(y) or np.isnan(v):
            continue
        pts.append((x, y, v))
    if len(pts) < 12:
        return None
    P = np.array(pts)
    X, Y, V = P[:, 0], P[:, 1], P[:, 2]

    A = np.column_stack([X, X ** 2, Y, Y ** 2, np.ones(len(P))])
    coef, res, *_ = np.linalg.lstsq(A, V, rcond=None)
    r2 = 1.0 - res[0] / ((V - V.mean()) ** 2).sum()
    x_opt = -coef[0] / (2.0 * coef[1])
    y_opt = -coef[2] / (2.0 * coef[3])

    gx = np.linspace(X.min() - 10, X.max() + 10, 240)
    gy = np.linspace(Y.min() - 10, Y.max() + 10, 240)
    GX, GY = np.meshgrid(gx, gy)
    GZ = (coef[0] * GX + coef[1] * GX ** 2
          + coef[2] * GY + coef[3] * GY ** 2 + coef[4])

    # The runs form a cross, not a grid: t1 was swept at t2 ~ 0 and t2 at
    # t1 ~ 15 mm. Drawing the fitted surface across the unsampled corners would
    # show confident contours over regions no run visited, and would let the
    # extrapolated minimum set the colour scale. Mask anything far from a
    # measured point, and scale the colours by the measured range.
    reach = 45.0
    d2 = np.min((GX[..., None] - X) ** 2 + (GY[..., None] - Y) ** 2, axis=-1)
    GZ = np.ma.masked_where(d2 > reach ** 2, GZ)

    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    lim = np.abs(V).max()
    cf = ax.contourf(GX, GY, GZ, levels=np.linspace(-lim, lim, 15),
                     cmap="RdBu_r", extend="both")
    ax.contour(GX, GY, GZ, levels=[0.0], colors="0.25", linewidths=1.2)
    ax.plot(X, Y, "o", ms=5, mfc="none", mec=INK, mew=0.9,
            linestyle="none", label="measured runs")
    ax.plot([x_opt], [y_opt], "*", ms=15, color=INK, linestyle="none",
            label=f"optimum ({x_opt:+.0f}, {y_opt:+.0f}) mm")
    cb = fig.colorbar(cf, ax=ax)
    cb.set_label("alignment gained toward the real plane [deg]",
                 color=INK_MUTED)
    ax.set_xlabel("pole offset along $t_1$ [mm]", color=INK_MUTED)
    ax.set_ylabel("pole offset along $t_2$ [mm]", color=INK_MUTED)
    ax.set_title(f"Fitted pole surface, $R^2$ = {r2:.3f} over {len(P)} runs; "
                 "shaded only where runs support it", fontsize=9, color=INK)
    ax.legend(fontsize=7, loc="lower right", framealpha=0.9)
    ax.grid(False)
    return save(fig, "B_pole_surface.pdf")


def fig_c2_nullspace(rows):
    """Null-space modes: sigma recovery and the task-invariance proof."""
    sub = [r for r in rows if r["run_id"].startswith("C2_hold_mode")]
    if not sub:
        return None

    order = {"0": 0, "1": 1, "2": 2, "3": 3}
    names = {0: "off", 1: "damping", 2: "sigma", 3: "both"}

    fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.2))
    for ax, key, ylabel in (
        (axes[0], "sigma_gain", r"recovered $\Delta\sigma_{\min}$"),
        (axes[1], "task_pos_error_drift_mm", "task position drift [mm]"),
    ):
        buckets = {}
        for r in sub:
            mode = r["run_id"].split("_")[2].replace("mode", "")
            if mode not in order:
                continue
            y = fnum(r, key)
            if np.isnan(y):
                continue
            x = order[mode]
            buckets.setdefault(x, {"good": [], "bad": []})
            buckets[x]["good" if not data_suspect(r) else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.set_xticks(sorted(buckets))
        ax.set_xticklabels([names[int(k)] for k in sorted(buckets)])
        ax.set_xlabel("null-space mode")
        ax.set_ylabel(ylabel)
    axes[1].axhline(0.0, color="0.3", linewidth=1)
    axes[1].set_title("must stay near zero", fontsize=8)
    figure_legend(fig, axes[0])
    fig.suptitle("C2/C3: null-space modes and task invariance", fontsize=10)
    return save(fig, "C2_nullspace_modes.pdf")


def fig_g2_convergence(rows):
    """Did the set-up phase actually reach equilibrium?"""
    sub = [r for r in rows if r["run_id"].startswith("G2_equilibrium")]
    if not sub:
        return None

    fig, ax = plt.subplots(figsize=(5.2, 3.4))
    buckets = {}
    for r in sub:
        x = fnum(r, "setup_duration_s")
        y = fnum(r, "tip_final_deg")
        if np.isnan(x) or np.isnan(y):
            continue
        buckets.setdefault(round(x), {"good": [], "bad": []})
        buckets[round(x)]["good" if not data_suspect(r) else "bad"].append(y)
    errorbar_from_buckets(ax, buckets, "final tip", "C0")
    ax.set_xlabel("set-up phase duration [s]")
    ax.set_ylabel("final tip angle [deg]")
    ax.legend()
    ax.set_title("G2: is 4 s long enough to reach equilibrium?", fontsize=10)
    return save(fig, "G2_equilibrium.pdf")


def main():
    if not os.path.exists(METRICS):
        sys.exit(f"no metrics file: {METRICS}\n"
                 f"Run extract_metrics.py first.")
    rows = load_metrics(METRICS)
    print(f"{len(rows)} runs in metrics.csv")

    made = []
    for fn in (
        fig_g2_convergence,
        fig_a2_stiffness,
        fig_d_axis_stiffness,
        fig_d_initial_angle,
        fig_main_initial_angle,
        fig_main_rotational_stiffness,
        fig_main_translational_stiffness,
        fig_main_interaction,
        fig_main_compliance_centre,
        fig_main_tool_axis_tilt,
        fig_plane_validation,
        fig_b_pole_axis,
        fig_b_pole_surface,
        fig_c2_nullspace,
    ):
        try:
            p = fn(rows)
            if p:
                made.append(p)
            else:
                print(f"  (skipped {fn.__name__}: no data yet)")
        except Exception as exc:  # noqa: BLE001
            print(f"  ERROR in {fn.__name__}: {type(exc).__name__}: {exc}")

    print(f"\n{len(made)} figure(s) written to {FIGURES}")


if __name__ == "__main__":
    main()
