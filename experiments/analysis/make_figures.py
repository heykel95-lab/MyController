#!/usr/bin/env python3
"""Build the thesis figures from experiments/derived/metrics.csv.

  python3 experiments/analysis/make_figures.py

Writes PDFs into experiments/figures/. Each figure corresponds to a specific
table or claim in the thesis, named in FIGURES below, so it is obvious which
result a plot is meant to support.

Flagged runs are drawn hollow and excluded from the mean/error bars: a run that
did not converge must not silently enter a thesis figure.

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

plt.rcParams.update({
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.3,
})


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
    # Excluded runs drawn hollow so nothing is hidden.
    for x in xs:
        for v in buckets[x]["bad"]:
            ax.plot(x, v, marker="o", mfc="none", mec="0.6", ms=5,
                    linestyle="none")
    return bool(plotted)


def save(fig, name):
    os.makedirs(FIGURES, exist_ok=True)
    path = os.path.join(FIGURES, name)
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
            buckets[r["_kr"]]["good" if not r["flags"] else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.set_xlabel(r"$K_{R,\mathrm{tangent}}$ [Nm/rad]")
        ax.set_ylabel(ylabel)
    fig.suptitle("A2: rotational stiffness sweep (replaces thesis Table 5.1)",
                 fontsize=10)
    return save(fig, "A2_stiffness_sweep.pdf")


def fig_b2_pole(rows):
    """The central pole experiment: effort vs pole offset along the normal."""
    sub = [r for r in rows if r["run_id"].startswith("B2_pole_normal")]
    if not sub:
        return None

    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.0))
    for ax, key, ylabel in (
        (axes[0], "tip_final_deg", "tip angle [deg]"),
        (axes[1], "force_steady_N", "steady contact force [N]"),
        (axes[2], "edge_travel_mm", "edge travel [mm]"),
    ):
        buckets = {}
        for r in sub:
            x = fnum(r, "pole_normal_mm")
            y = fnum(r, key)
            if np.isnan(x) or np.isnan(y):
                continue
            buckets.setdefault(x, {"good": [], "bad": []})
            buckets[x]["good" if not r["flags"] else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.axvline(0.0, color="0.4", linestyle="--", linewidth=1)
        ax.set_xlabel("pole offset along surface normal [mm]")
        ax.set_ylabel(ylabel)
    axes[0].annotate("pole on edge", xy=(0, 0), xytext=(4, 4),
                     textcoords="offset points", fontsize=7, color="0.4")
    fig.suptitle("B2: pole swept along the surface normal", fontsize=10)
    return save(fig, "B2_pole_normal_sweep.pdf")


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
            buckets[x]["good" if not r["flags"] else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.set_xticks(sorted(buckets))
        ax.set_xticklabels([names[int(k)] for k in sorted(buckets)])
        ax.set_xlabel("null-space mode")
        ax.set_ylabel(ylabel)
    axes[1].axhline(0.0, color="0.3", linewidth=1)
    axes[1].set_title("must stay near zero", fontsize=8)
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
        buckets[round(x)]["good" if not r["flags"] else "bad"].append(y)
    errorbar_from_buckets(ax, buckets, "final tip", "C0")
    ax.set_xlabel("set-up phase duration [s]")
    ax.set_ylabel("final tip angle [deg]")
    ax.set_title("G2: is 4 s long enough to reach equilibrium?", fontsize=10)
    return save(fig, "G2_equilibrium.pdf")


def main():
    if not os.path.exists(METRICS):
        sys.exit(f"no metrics file: {METRICS}\n"
                 f"Run extract_metrics.py first.")
    rows = load_metrics(METRICS)
    print(f"{len(rows)} runs in metrics.csv")

    made = []
    for fn in (fig_g2_convergence, fig_a2_stiffness, fig_b2_pole,
               fig_c2_nullspace):
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
