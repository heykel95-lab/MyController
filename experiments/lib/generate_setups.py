#!/usr/bin/env python3
"""Generate the experiment setup overlays from one declarative spec.

Each setup is a directory under experiments/setups/<run_id>/ holding:
  overlay.txt   the parameter keys that differ from params/ (the nominal set)
  about.txt     what the run tests and what result would count as a pass

Generating them instead of hand-writing 30 directories keeps the matrix
reproducible: the spec below is the single source of truth, and the pole
offsets are computed from the configured surface tilt rather than typed in.

Run:  python3 experiments/lib/generate_setups.py
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SETUPS = os.path.normpath(os.path.join(HERE, "..", "setups"))

# Surface tilt of the nominal configuration (params/common.txt).
TILT_A_DEG = 0.0
TILT_B_DEG = 5.0


def surface_normal(a_deg, b_deg):
    """n = R_y(b) R_x(a) e_z, matching makeAlignmentTargetFrame()."""
    a = math.radians(a_deg)
    b = math.radians(b_deg)
    return (
        math.sin(b) * math.cos(a),
        -math.sin(a),
        math.cos(b) * math.cos(a),
    )


def surface_tangents(a_deg, b_deg, t1_entered=(1.0, 0.0, 0.0)):
    """Gram-Schmidt, matching makeSurfaceFrameFromNormalTangent()."""
    n = surface_normal(a_deg, b_deg)
    dot = sum(n[i] * t1_entered[i] for i in range(3))
    t1 = [t1_entered[i] - n[i] * dot for i in range(3)]
    norm = math.sqrt(sum(v * v for v in t1))
    t1 = [v / norm for v in t1]
    t2 = [
        n[1] * t1[2] - n[2] * t1[1],
        n[2] * t1[0] - n[0] * t1[2],
        n[0] * t1[1] - n[1] * t1[0],
    ]
    return t1, t2, list(n)


def pole_offset_along(direction, distance):
    return [direction[i] * distance for i in range(3)]


def pole_keys(offset):
    return [
        ("coupled_pole_from_edge_x", f"{offset[0]:.6f}"),
        ("coupled_pole_from_edge_y", f"{offset[1]:.6f}"),
        ("coupled_pole_from_edge_z", f"{offset[2]:.6f}"),
    ]


T1, T2, N = surface_tangents(TILT_A_DEG, TILT_B_DEG)

# --------------------------------------------------------------------------
# The run matrix. Each entry: (run_id, purpose, pass_criterion, [(key, value)])
# An empty override list means "run the nominal configuration unchanged".
# --------------------------------------------------------------------------
SPEC = []


def add(run_id, purpose, criterion, overrides, repeats=3):
    SPEC.append((run_id, purpose, criterion, overrides, repeats))


# ---- Gates -----------------------------------------------------------------
add(
    "G1_coupled_block_diagonal",
    "Coupled 6x6 path with block-diagonal gains must reproduce the decoupled "
    "wrench exactly. This is the correctness gate for every coupled result.",
    "Tip angle and steady force within the G3 repeatability band of "
    "G1_decoupled_baseline. Historically 8.6 deg vs 9.0 deg.",
    [("use_coupled_stiffness", "1"), ("coupled_use_block_diagonal", "1")],
)
add(
    "G1_decoupled_baseline",
    "Decoupled wrench, no coupling. The reference every other run is "
    "compared against.",
    "Establishes the baseline. No pass/fail on its own.",
    [("use_coupled_stiffness", "0")],
    repeats=5,
)
add(
    "G2_equilibrium_t4",
    "Is the 4 s set-up phase long enough to reach equilibrium?",
    "Compare with G2_equilibrium_t8 and t12. If the tip angle still grows "
    "after 4 s, every reported value is a transient and the quasi-static "
    "gain-selection argument does not hold.",
    [("setup_timeout", "4.0")],
)
add("G2_equilibrium_t8", "Same, 8 s.", "See G2_equilibrium_t4.",
    [("setup_timeout", "8.0")])
add("G2_equilibrium_t12", "Same, 12 s.", "See G2_equilibrium_t4.",
    [("setup_timeout", "12.0")])
add(
    "G3_repeatability",
    "Noise floor. Nominal configuration, full teardown and reset between "
    "repeats.",
    "Report the standard deviation of tip, alignment gain, force, edge travel "
    "and axis distance. No later difference smaller than ~2 sigma may be "
    "claimed as an effect.",
    [],
    repeats=5,
)

# ---- Series A: the alignment sequence --------------------------------------
for tilt in (0.0, 5.0, 10.0, 15.0):
    t1, t2, n = surface_tangents(TILT_A_DEG, tilt)
    add(
        f"A1_tilt_{int(tilt):02d}deg",
        f"Initial misalignment sweep, surface tilt b = {tilt} deg.",
        "Alignment gain should grow with initial misalignment, then saturate. "
        "Gives the x-axis for the alignment claim.",
        [("alignment_target_tilt_angle_y_deg", f"{tilt}")]
        + pole_keys(pole_offset_along(n, 0.08)),
    )

# Three points, roughly x3 apart, rather than four evenly spaced ones: 30 was
# the least informative of the original set, sitting between 15 and 50 on a
# response that is expected to be monotonic. 5.0 is the nominal value, so that
# arm doubles as a replication check against G1_decoupled_baseline.
for kr in (5.0, 15.0, 50.0):
    add(
        f"A2_KRtan_{int(kr):02d}",
        f"Rotational stiffness sweep on the tipping axes, KR_tangent = {kr} "
        "N m/rad, decoupled command.",
        "Expect tip down and peak force up as KR rises. This run series "
        "replaces the invented Table 5.1 in the thesis.",
        [
            ("use_coupled_stiffness", "0"),
            ("setup_KR_tangent1", f"{kr}"),
            ("setup_KR_tangent2", f"{kr}"),
        ],
        repeats=5,
    )

for kp in (2000.0, 800.0, 300.0):
    add(
        f"A3_lateral_Kp_{int(kp):04d}",
        f"Lateral translational compliance, setup_Kp_x = setup_Kp_y = {kp} N/m.",
        "Tests the 'added lateral compliance' case (thesis Test 4). Expect the "
        "measured axis to move toward the contact edge if sliding helps.",
        [
            ("use_coupled_stiffness", "0"),
            ("setup_Kp_x", f"{kp}"),
            ("setup_Kp_y", f"{kp}"),
        ],
    )

for push in (0.10, 0.14, 0.18):
    add(
        f"A4_push_end_{int(push * 100):03d}mm",
        f"Press depth, setup_push_end = {push} m.",
        "Historically force rose while tip stayed flat. Reproducing that "
        "confirms the equilibrium is geometry-limited, not press-limited.",
        [("setup_push_end", f"{push}")],
    )

for speed in (0.02, 0.05, 0.10):
    add(
        f"A5_push_speed_{int(speed * 1000):03d}",
        f"Press rate, setup_push_speed = {speed} m/s.",
        "Chapter 4 gain selection assumes quasi-static behaviour. If the tip "
        "depends on speed, that assumption needs qualifying in the text.",
        [("setup_push_speed", f"{speed}")],
    )

for clear in (0.010, 0.020, 0.030):
    add(
        f"A6_clearance_{int(clear * 1000):02d}mm",
        f"Handoff height, descend_surface_clearance = {clear} m.",
        "The final equilibrium should not depend on where set-up took over. "
        "If it does, the frozen first_contact references are contaminated.",
        [("descend_surface_clearance", f"{clear}")],
    )

for case in ("tilted_tool", "tilted_close", "table"):
    add(
        f"A7_qinit_{case}",
        f"Arm posture, q_init_case = {case}.",
        "Separates a property of the impedance from a property of the "
        "configuration.",
        [("q_init_case", case)],
    )

# ---- Series B: centre of compliance / pole ---------------------------------
add(
    "B1_pole_at_tcp",
    "Coupled law with the pole placed at the TCP (zero lever).",
    "Must reproduce the decoupled baseline. Second correctness gate: the "
    "adjoint reduces to the identity when r_c = 0.",
    [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_pole_freeze_at_contact", "1"),
    ]
    + pole_keys([0.0, 0.0, 0.0]),
)

for s in (-0.12, -0.08, -0.04, 0.0, 0.04, 0.08, 0.12, 0.16):
    tag = f"{'m' if s < 0 else 'p'}{abs(int(round(s * 1000))):03d}"
    add(
        f"B2_pole_normal_{tag}",
        f"Pole swept ALONG THE SURFACE NORMAL, s = {s:+.2f} m from the contact "
        "edge. Offset is written in base coordinates as s * n.",
        "MEASURED (24 runs): this sweep does NOT isolate the normal offset. "
        "The configured normal carries a tangential component from the surface "
        "tilt, so sweeping along n also drags the pole tangentially from -5 to "
        "+28 mm, and that drift accounts for the whole trend. Against the "
        "normal component alone the response is flat (R^2 = 0.005). Kept for "
        "the record; B3/B4 are the sweeps that isolate a single axis.",
        [
            ("use_coupled_stiffness", "1"),
            ("coupled_use_block_diagonal", "0"),
            ("coupled_pole_manual", "1"),
            ("coupled_pole_freeze_at_contact", "1"),
        ]
        + pole_keys(pole_offset_along(N, s)),
    )

for s in (-0.08, -0.04, 0.0, 0.04, 0.08, 0.12):
    tag = f"{'m' if s < 0 else 'p'}{abs(int(round(s * 1000))):03d}"
    offset = [
        pole_offset_along(N, 0.08)[i] + pole_offset_along(T1, s)[i]
        for i in range(3)
    ]
    add(
        f"B3_pole_tangent_{tag}",
        f"Pole swept along surface tangent t1, {s:+.2f} m, at fixed normal "
        "offset +0.08 m.",
        "Isolates the tangential pole component -- the variable that actually "
        "governs alignment (R^2 = 0.907, +0.125 deg/mm over 36 runs). Measured "
        "-10.3 deg at s = -0.08 to +6.9 deg at s = +0.04, still rising at the "
        "positive end: +0.08 and +0.12 exist to locate the optimum.",
        [
            ("use_coupled_stiffness", "1"),
            ("coupled_use_block_diagonal", "0"),
            ("coupled_pole_manual", "1"),
            ("coupled_pole_freeze_at_contact", "1"),
        ]
        + pole_keys(offset),
    )

add(
    "B5_pole_live_tracking",
    "Pole recomputed from the moving edge each cycle instead of frozen at "
    "first contact.",
    "Historically the equilibrium was essentially unchanged and freezing was "
    "preferred for predictability. Confirms the design recommendation.",
    [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_pole_freeze_at_contact", "0"),
    ]
    + pole_keys(pole_offset_along(N, 0.08)),
)

# B6_pole_on_measured_axis is deliberately absent: it commanded the pole AT the
# measured Chasles axis, and the finite-screw-axis line of work was dropped.

for s in (-0.08, -0.04, 0.04, 0.08):
    tag = f"{'m' if s < 0 else 'p'}{abs(int(round(s * 1000))):03d}"
    offset = [
        pole_offset_along(N, 0.08)[i] + pole_offset_along(T2, s)[i]
        for i in range(3)
    ]
    add(
        f"B4_pole_tangent2_{tag}",
        f"Pole swept along surface tangent t2, {s:+.2f} m, at fixed normal "
        "offset +0.08 m and zero t1 offset.",
        "The untested axis. Across all 44 pole runs recorded so far the t2 "
        "component was held at approximately zero, so the placement rule "
        "'the compliance centre belongs opposite the side that contacts "
        "first' has never actually been tested -- B3 sweeps t1, which the "
        "recovered contact geometry suggests is not the relevant axis. See "
        "PREDICTIONS.md for the ordering predicted before these runs exist.",
        [
            ("use_coupled_stiffness", "1"),
            ("coupled_use_block_diagonal", "0"),
            ("coupled_pole_manual", "1"),
            ("coupled_pole_freeze_at_contact", "1"),
        ]
        + pole_keys(offset),
        repeats=3,
    )


# ---- Series C: hold mode, null-space --------------------------------------
add(
    "C1_hold_drift_mode0",
    "Hold, null-space off, 60 s, arm untouched.",
    "Control group: establishes the sigma_min drift caused by noise alone.",
    [("nullspace_mode", "0"), ("print_sigma_debug", "1")],
    repeats=2,
)
for mode, name in ((0, "off"), (1, "damping"), (2, "sigma"), (3, "both")):
    add(
        f"C2_hold_mode{mode}_{name}",
        f"Hold, push-and-release protocol, nullspace_mode = {mode} ({name}).",
        "Core comparison. Metrics: recovered sigma_min, recovery time, vBest "
        "sign, tau_sigma norm. C3 (task invariance) is extracted from the same "
        "logs at no extra cost. Three repeats rather than five: the disturbance "
        "is a hand push and so is not a controlled input, which puts a floor on "
        "the precision two extra repeats could buy. Differences between modes "
        "have to clear the G3 noise floor either way.",
        [("nullspace_mode", f"{mode}"), ("print_sigma_debug", "1")],
        repeats=3,
    )

for k in (0.5, 1.0, 2.0, 4.0):
    add(
        f"C4_ksigma_{str(k).replace('.', 'p')}",
        f"Sigma torque magnitude sweep, nullspace_k_sigma = {k}, mode 2.",
        "Finds the threshold where the sigma torque overcomes friction. The "
        "README failure mode is 'tau active but nullspace speed near zero'.",
        [
            ("nullspace_mode", "2"),
            ("nullspace_k_sigma", f"{k}"),
            ("print_sigma_debug", "1"),
        ],
    )

for alpha in (0.02, 0.04, 0.08, 0.16):
    add(
        f"C5_alpha_{str(alpha).replace('.', 'p')}",
        f"Probe distance sweep, nullspace_alpha = {alpha}, mode 2.",
        "Tests the claim that alpha is only a sampling step. The selected sign "
        "must not flip with alpha, and probe/(2*alpha) should be roughly "
        "alpha-invariant.",
        [
            ("nullspace_mode", "2"),
            ("nullspace_alpha", f"{alpha}"),
            ("print_sigma_debug", "1"),
        ],
    )

add(
    "C6_deadband_at_optimum",
    "Hold started at a good-sigma configuration, mode 2.",
    "Expect C <= 1 and no sustained push: confirms the deadband settles "
    "instead of hunting at the optimum.",
    [
        ("nullspace_mode", "2"),
        ("q_init_case", "horizontal"),
        ("print_sigma_debug", "1"),
    ],
)

for case in ("horizontal", "tilted_tool", "tilted_close", "table"):
    add(
        f"C8_qinit_{case}",
        f"Null-space generality, mode 3, q_init_case = {case}.",
        "Improvement should scale with distance from the optimum, not depend "
        "on one lucky pose.",
        [
            ("nullspace_mode", "3"),
            ("q_init_case", case),
            ("print_sigma_debug", "1"),
        ],
    )

for tol in (0.001, 0.0001, 0.00001):
    add(
        f"C9_svdtol_{tol:.0e}".replace("-", "m").replace("+", ""),
        f"SVD truncation sensitivity, nullspace_svd_relative_tolerance = {tol}.",
        "Likely only matters near a singularity. If flat, report as a one-line "
        "robustness note rather than a thesis section.",
        [
            ("nullspace_mode", "3"),
            ("nullspace_svd_relative_tolerance", f"{tol}"),
            ("print_sigma_debug", "1"),
        ],
    )


def write_setups():
    os.makedirs(SETUPS, exist_ok=True)
    index = []
    for run_id, purpose, criterion, overrides, repeats in SPEC:
        d = os.path.join(SETUPS, run_id)
        os.makedirs(d, exist_ok=True)

        with open(os.path.join(d, "overlay.txt"), "w") as f:
            f.write(f"# {run_id}\n")
            f.write("# Applied on top of surface_grinding_controller/params/.\n")
            f.write("# Only keys listed here differ from the nominal set.\n")
            if not overrides:
                f.write("# (nominal configuration, no overrides)\n")
            for key, value in overrides:
                f.write(f"{key} = {value}\n")

        with open(os.path.join(d, "about.txt"), "w") as f:
            f.write(f"run_id:   {run_id}\n")
            f.write(f"repeats:  {repeats}\n\n")
            f.write("purpose:\n  " + purpose.replace("\n", "\n  ") + "\n\n")
            f.write("pass criterion:\n  " + criterion.replace("\n", "\n  ") + "\n")

        index.append((run_id, repeats, purpose))

    with open(os.path.join(SETUPS, "INDEX.txt"), "w") as f:
        f.write(f"{len(index)} setups, "
                f"{sum(r for _, r, _ in index)} total runs\n\n")
        for run_id, repeats, purpose in index:
            # Split on sentence ends only, so decimals like 0.10 survive.
            summary = purpose.replace("\n", " ")
            for end in (". ", "? "):
                if end in summary:
                    summary = summary.split(end)[0] + end.strip()
                    break
            f.write(f"{run_id:34s} x{repeats}  {summary}\n")

    print(f"wrote {len(index)} setups "
          f"({sum(r for _, r, _ in index)} runs) to {SETUPS}")


if __name__ == "__main__":
    write_setups()
