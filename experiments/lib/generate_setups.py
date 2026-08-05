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

# Surface tilt of the nominal configuration (params/Plane_Definition.txt).
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


def merged_overrides(base, replacements):
    """Return one ordered value per key, with replacements taking priority."""
    merged = dict(base)
    merged.update(replacements)
    return list(merged.items())


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

# ---- MAIN campaign: calibrated plane, independent command and gains --------
# These runs use surface-frame translational gains and null-space damping only.
# The sigma bias is reintroduced later as its own matched control rather than
# being allowed to influence every contact-alignment trial.
# Angle from tangent1 to the face long axis. 115.05 and not 90 because the
# calibrated tangent1 is the projected P1->P2 probe direction, 25.05 deg round
# from the [1,0,0] tangent1 in params/.
MAIN_TWIST_DEG = 115.05

MAIN_COMMON = [
    ("use_coupled_stiffness", "0"),
    # The approach controller settles near 1.5 deg under the mounted-tool load;
    # a tested 0.5 deg switch condition therefore deadlocked in orient. The
    # explicit 2 deg gate is reachable and the subsequent descend reduced the
    # EE-inferred first-contact residual to about 1 deg in repeated A0 runs.
    ("approach_orient_error_threshold", "0.035"),
    # The spin about the tool axis has its own gate: sharing the one above let
    # it through at 2 deg while still converging, and the clearance capture
    # freezes whatever error is standing at handover into the contact frame.
    ("approach_orient_spin_error_threshold", "0.009"),
    # The axis error settles at a value the commanded tilt decides, 1.3 deg at
    # 0 and 2.1 deg at 10, so the 10 deg conditions never reach the gate. Hand
    # over at the settled value rather than waiting on one that cannot arrive.
    ("approach_orient_timeout", "5.0"),
    ("setup_timeout", "5.0"),
    # The two loose-gate A0 diagnostics reached 24.33 N at 0.060 m with
    # Kp,n=360 N/m. The primary campaign instead targets the established
    # 50 N operating range: 800 N/m gives a 48 N quasi-static command and
    # about 50 N after the measured low-load offset is accounted for. The
    # 5 s phase retains ample settling time after the short preload ramp.
    ("setup_push_end", "0.060"),
    ("setup_push_speed", "0.050"),
    ("grind_sweep_enabled", "0"),
    ("nullspace_mode", "1"),
    ("setup_translation_surface_frame", "1"),
    ("setup_Kp_surface_normal", "800.0"),
    ("setup_KR_normal", "50.0"),
    # The two tilts set the tool-axis direction only. Without the spin about
    # that axis commanded, q_init decides which edge of the 40 x 120 mm face
    # leads, and a 120 mm edge is a different contact condition from a 40 mm
    # one at the same angle. 90 deg puts the long axis along t2, so the short
    # edge leads. Pinned here so every MAIN run records it.
    ("command_tool_twist", "1"),
    # 115.05 and not 90: the angle is measured from tangent1, and the
    # calibrated plane's tangent1 is the projected P1->P2 probe direction,
    # 25.05 deg round from the +X tangent1 in params/. 115.05 in the
    # calibrated frame puts the face where 90 puts it in the nominal one, so
    # the campaign presents the same physical orientation as a direct run.
    ("tool_target_offset_normal_deg", f"{MAIN_TWIST_DEG}"),
]


def main_gain_overrides(angle_t1=0.0, angle_t2=0.0, kr_t1=5.0,
                        kr_t2=5.0, kp_t1=2000.0, kp_t2=2000.0,
                        q_init_case="horizontal_table_search"):
    return MAIN_COMMON + [
        ("q_init_case", q_init_case),
        # Two decimals: a tilt about a tool axis resolves to values like
        # -4.23 / 9.06, and rounding those to one decimal swings the tilt axis
        # by a quarter of a degree, against a repeatability floor of 0.05.
        ("tool_target_offset_tangent1_deg", f"{angle_t1:.2f}"),
        ("tool_target_offset_tangent2_deg", f"{angle_t2:.2f}"),
        ("setup_KR_tangent1", f"{kr_t1:.1f}"),
        ("setup_KR_tangent2", f"{kr_t2:.1f}"),
        ("setup_Kp_surface_tangent1", f"{kp_t1:.1f}"),
        ("setup_Kp_surface_tangent2", f"{kp_t2:.1f}"),
    ]


# Case A: calibration and initial relative angle.
for run_id, a1, a2 in (
    ("MAIN_A0_00deg", 0.0, 0.0),
    ("MAIN_A1_t1_05deg", 5.0, 0.0),
    ("MAIN_A2_t1_10deg", 10.0, 0.0),
    ("MAIN_A3_t2_05deg", 0.0, 5.0),
    ("MAIN_A4_t2_10deg", 0.0, 10.0),
):
    add(
        run_id,
        f"Case A: calibrated plane with independent tool command offsets "
        f"(t1={a1:+.0f} deg, t2={a2:+.0f} deg).",
        "A0 must start close to zero physical-plane alignment error. The "
        "nonzero cases establish the measured first-contact angle and the "
        "fraction removed within the 5 s set-up.",
        main_gain_overrides(a1, a2),
        repeats=3,
    )

# Case B: axis-specific rotational stiffness. The 5 N m/rad references are
# MAIN_A2 and MAIN_A4, so only the additional levels are generated here.
for axis in (1, 2):
    for kr in (15.0, 50.0):
        add(
            f"MAIN_B{axis}_KRt{axis}_{int(kr):02d}",
            f"Case B: +10 deg about t{axis}; vary only K_R,t{axis} to "
            f"{kr:.0f} N m/rad.",
            "Compare with the corresponding Case-A 10 deg reference at "
            "5 N m/rad. The changed signed axis component must exceed the "
            "repeatability interval before it is attributed to K_R.",
            main_gain_overrides(
                10.0 if axis == 1 else 0.0,
                10.0 if axis == 2 else 0.0,
                kr_t1=kr if axis == 1 else 5.0,
                kr_t2=kr if axis == 2 else 5.0,
            ),
            repeats=3,
        )

# Case C: cross-direction translational stiffness. A t1 angular mismatch is
# paired with Kp,t2; a t2 mismatch is paired with Kp,t1.
for axis in (1, 2):
    kp_axis = 2 if axis == 1 else 1
    for kp in (300.0, 800.0):
        add(
            f"MAIN_C{axis}_KPt{kp_axis}_{int(kp):04d}",
            f"Case C: +10 deg about t{axis}; vary only K_p,t{kp_axis} to "
            f"{kp:.0f} N/m.",
            "Compare with the corresponding 2000 N/m Case-A reference. "
            "Evaluate alignment, edge travel, normal load and 90% time.",
            main_gain_overrides(
                10.0 if axis == 1 else 0.0,
                10.0 if axis == 2 else 0.0,
                kp_t1=kp if kp_axis == 1 else 2000.0,
                kp_t2=kp if kp_axis == 2 else 2000.0,
            ),
            repeats=3,
        )
    add(
        f"MAIN_C{axis}_interaction_KR50_KP300",
        f"Case C interaction corner for t{axis}: K_R,t{axis}=50 N m/rad "
        f"and K_p,t{kp_axis}=300 N/m.",
        "Completes the low/high 2x2 interaction using the three already "
        "shared endpoint settings.",
        main_gain_overrides(
            10.0 if axis == 1 else 0.0,
            10.0 if axis == 2 else 0.0,
            kr_t1=50.0 if axis == 1 else 5.0,
            kr_t2=50.0 if axis == 2 else 5.0,
            kp_t1=300.0 if kp_axis == 1 else 2000.0,
            kp_t2=300.0 if kp_axis == 2 else 2000.0,
        ),
        repeats=3,
    )

# Short sign pilot before the full centre-of-compliance campaign. The completed
# MAIN_A1 repeats provide the decoupled reference. These two cases change only
# the signed t2 lever required to generate a t1 moment under normal preload:
#   m_t1 = (-r_c x f)_t1.
# Direct surface-frame r_c avoids the old base-frame pole convention.
for rc_t2_mm in (-60, -40, 60):
    sign_tag = f"{'m' if rc_t2_mm < 0 else 'p'}{abs(rc_t2_mm):03d}"
    overrides = [
        pair for pair in main_gain_overrides(angle_t1=5.0)
        if pair[0] != "use_coupled_stiffness"
    ] + [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_use_direct_rc_surface", "1"),
        ("coupled_rc_tangent1", "0.0"),
        ("coupled_rc_tangent2", f"{rc_t2_mm / 1000.0:.3f}"),
        ("coupled_rc_normal", "0.0"),
    ]
    add(
        f"PILOT_COC_t1_rc_t2_{sign_tag}",
        f"Active compliance-centre sign pilot: +5 deg about t1 and "
        f"r_c,t2={rc_t2_mm:+d} mm.",
        "Compare with the completed decoupled MAIN_A1 reference. Select the "
        "sign that reduces the measured t1 error and moves the physical "
        "contact from the initially loaded edge toward full-face contact.",
        overrides,
        repeats=3 if rc_t2_mm == -40 else 1,
    )

# Orthogonal sign pilot. A t2 angular mismatch requires a t1 lever. The
# magnitude starts at the repeatable 40 mm selected above; both signs are
# tested because the physical EE/surface-axis correspondence is pose dependent.
for rc_t1_mm in (-40, 40):
    sign_tag = f"{'m' if rc_t1_mm < 0 else 'p'}{abs(rc_t1_mm):03d}"
    overrides = [
        pair for pair in main_gain_overrides(angle_t2=5.0)
        if pair[0] != "use_coupled_stiffness"
    ] + [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_use_direct_rc_surface", "1"),
        ("coupled_rc_tangent1", f"{rc_t1_mm / 1000.0:.3f}"),
        ("coupled_rc_tangent2", "0.0"),
        ("coupled_rc_normal", "0.0"),
    ]
    add(
        f"PILOT_COC_t2_rc_t1_{sign_tag}",
        f"Orthogonal compliance-centre sign pilot: +5 deg about t2 and "
        f"r_c,t1={rc_t1_mm:+d} mm.",
        "Select the sign that reduces the measured t2 component and moves "
        "the physical contact from the initially loaded edge toward "
        "full-face contact without increasing the t1 cross-axis error.",
        overrides,
        repeats=1,
    )

# Case D coarse centre-of-compliance sweep. Gains fixed 2026-07-31 from the
# completed A--C screening, as the conventions require before D runs, and both
# land on the Case A baseline: K_R = 5 N m/rad was best or tied at every
# condition and fell monotonically as it rose (t1 0.64/0.57/0.49, t2
# 1.59/1.58/1.14 for 5/15/50), and K_P stays at 2000 N/m.
#
# K_P = 300 corrected more in the screening -- 1.79 deg and 21 % removed on t2
# against 1.59 at baseline -- but adopting it would have changed the pole and
# the stiffness together, leaving any difference from Case A ambiguous between
# the two. At baseline the pole is the only variable, and the zero-lever runs
# have an exact matched reference: D1_p000 against A2, D2_p000 against A4.
for axis in (1, 2):
    rc_axis = 2 if axis == 1 else 1
    for rc_mm in (-60, 0, 60):
        tag = f"{'m' if rc_mm < 0 else 'p'}{abs(rc_mm):03d}"
        overrides = [
            pair for pair in main_gain_overrides(
                10.0 if axis == 1 else 0.0,
                10.0 if axis == 2 else 0.0,
            )
            if pair[0] != "use_coupled_stiffness"
        ] + [
            ("use_coupled_stiffness", "1"),
            ("coupled_use_block_diagonal", "0"),
            ("coupled_pole_manual", "1"),
            ("coupled_use_direct_rc_surface", "1"),
            ("coupled_rc_tangent1", f"{rc_mm / 1000.0:.3f}" if rc_axis == 1 else "0.0"),
            ("coupled_rc_tangent2", f"{rc_mm / 1000.0:.3f}" if rc_axis == 2 else "0.0"),
            ("coupled_rc_normal", "0.0"),
        ]
        add(
            f"MAIN_D{axis}_t{axis}_rc_t{rc_axis}_{tag}",
            f"Case D coarse centre-of-compliance sweep: +10 deg about t{axis}, "
            f"direct r_c,t{rc_axis}={rc_mm:+d} mm.",
            "The zero lever must agree with the decoupled reference. Opposite "
            "lever signs test the predicted moment direction. Refine only "
            "after this coarse sweep is analysed.",
            overrides,
            repeats=3,
        )

# Case D3: the compliance centre at the grinding-face centre. The lever is
# r_c = p_TCP - p_c and contact.tcp is p_EE, so r_c = 0 already places the pole
# at p_EE -- that is what the p000 runs above are. The face the tool actually
# grinds with sits FACE_CENTER_M along the tool axis from p_EE, so putting the
# pole there is r_c = -FACE_CENTER_M * (tool axis), resolved in the surface
# frame at the commanded tilt. Mostly normal, with a small tangential part the
# tilt introduces; the t1/t2 sweeps above never leave the tangent plane, so
# this is the only condition that moves the pole along the normal.
FACE_CENTER_M = 0.020
for axis in (1, 2):
    tilt = math.radians(10.0)
    normal_part = FACE_CENTER_M * math.cos(tilt)
    tangent_part = FACE_CENTER_M * math.sin(tilt)
    if axis == 1:
        rc = (0.0, -tangent_part, normal_part)
    else:
        rc = (tangent_part, 0.0, normal_part)
    overrides = [
        pair for pair in main_gain_overrides(
            10.0 if axis == 1 else 0.0,
            10.0 if axis == 2 else 0.0,
        )
        if pair[0] != "use_coupled_stiffness"
    ] + [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_use_direct_rc_surface", "1"),
        ("coupled_rc_tangent1", f"{rc[0]:.6f}"),
        ("coupled_rc_tangent2", f"{rc[1]:.6f}"),
        ("coupled_rc_normal", f"{rc[2]:.6f}"),
    ]
    add(
        f"MAIN_D3_t{axis}_rc_face_centre",
        f"Case D: compliance centre at the grinding-face centre, +10 deg "
        f"about t{axis}. r_c = -20 mm along the tool axis from p_EE.",
        "Compare against the matched p000 run, which places the pole at p_EE "
        "instead. A difference is the 20 mm of tool the p000 convention "
        "ignores; no difference means the normal offset does not matter and "
        "only the tangential lever does.",
        overrides,
        repeats=3,
    )


# Case H: is there one pole that serves every tilt direction?
#
# The press force is normal, f = -F n, so the moment it makes about the TCP is
#
#   m = f x r_c = F (r_c,t2, -r_c,t1, 0).
#
# Only the tangential lever turns the tool, and it turns it perpendicular to
# itself. A tilt of theta about u = (cos a, sin a) in the tangent plane needs a
# corrective moment along -u, so the lever it needs is
#
#   r_c,t = rho (sin a, -cos a),
#
# perpendicular to the tilt axis and rotating with it. Case D measured the two
# ends of that rule and both agree: a = 0 needs r_c,t2 = -60 (7.0 -> 0.9 deg),
# a = 90 needs r_c,t1 = +60 (8.5 -> 2.1 deg), and the same levers on the wrong
# axis or with the wrong sign removed nothing at all.
#
# The rule says no single lever can serve every direction, which is what a
# general pole would have to do. H1 tests the rule at the two directions Case D
# did not measure; H2 holds one pole fixed while the direction turns, which is
# the general-pole claim itself; H3 asks the remaining question, whether the
# pole belongs above the plane, in it, or under it.
#
# Directions are named in the tool frame, as Case E named them: the twist puts
# the 120 mm long axis 25.05 deg from t2, so a tilt about Y_EE leads with the
# long edge and one about X_EE with the short edge.
# Shared by Cases H and E: both tilt the tool by this much, so their
# conditions are matched and the pole levers can be compared across them.
TOOL_TILT_DEG = 10.0
POLE_RHO_MM = 60.0
POLE_DIRECTIONS = (
    ("yEE", 0.0, "about Y_EE, the 120 mm edge leading"),
    ("diag_m45", -45.0, "45 deg between the tool axes"),
    ("xEE", -90.0, "about X_EE, the 40 mm edge leading"),
    ("diag_p45", 45.0, "45 deg between the tool axes, the other way"),
)


def pole_direction(axis_offset_deg):
    """Commanded tilt and the lever the moment rule asks for, in mm."""
    a = math.radians(MAIN_TWIST_DEG + axis_offset_deg)
    tilt = (TOOL_TILT_DEG * math.cos(a), TOOL_TILT_DEG * math.sin(a))
    lever = (POLE_RHO_MM * math.sin(a), -POLE_RHO_MM * math.cos(a))
    return tilt, lever


def direct_pole_overrides(tilt_deg, rc_mm):
    """Case A gains, plus a directly commanded r_c in the surface frame."""
    return [
        pair for pair in main_gain_overrides(tilt_deg[0], tilt_deg[1])
        if pair[0] != "use_coupled_stiffness"
    ] + [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_use_direct_rc_surface", "1"),
        ("coupled_rc_tangent1", f"{rc_mm[0] / 1000.0:.6f}"),
        ("coupled_rc_tangent2", f"{rc_mm[1] / 1000.0:.6f}"),
        ("coupled_rc_normal", f"{rc_mm[2] / 1000.0:.6f}"),
    ]


# H1: the lever follows the tilt direction, as the rule says it must.
for name, axis_offset_deg, description in POLE_DIRECTIONS:
    tilt, lever = pole_direction(axis_offset_deg)
    add(
        f"MAIN_H1_rot_{name}",
        f"Case H: {TOOL_TILT_DEG:.0f} deg tilt {description}, with the lever "
        f"the moment rule asks for: r_c,t = ({lever[0]:+.1f}, {lever[1]:+.1f}) "
        f"mm, {POLE_RHO_MM:.0f} mm perpendicular to the tilt axis.",
        "The fraction removed must be the same at every direction. If it is, "
        "the rule holds off the surface axes too and the pole is a function of "
        "the tilt, not a constant. A direction that removes markedly less is "
        "the face aspect ratio entering, and is the result to report.",
        direct_pole_overrides(tilt, (lever[0], lever[1], 0.0)),
        repeats=3,
    )

# H2: one pole, every direction. This is the general-pole claim, tested by
# holding the Y_EE lever while the tilt turns away from it.
_, FIXED_LEVER = pole_direction(0.0)
for name, axis_offset_deg, description in POLE_DIRECTIONS[1:]:
    tilt, _ = pole_direction(axis_offset_deg)
    add(
        f"MAIN_H2_fix_{name}",
        f"Case H: the same fixed lever as MAIN_H1_rot_yEE, "
        f"r_c,t = ({FIXED_LEVER[0]:+.1f}, {FIXED_LEVER[1]:+.1f}) mm, against a "
        f"{TOOL_TILT_DEG:.0f} deg tilt {description}.",
        "Against its matched H1 run: how much a fixed pole loses when the tilt "
        "turns away from it. The rule predicts the loss follows the cosine of "
        "the direction change and reaches nothing at 90 deg, where Case D "
        "already measured a wrong-axis lever removing 0.0 deg.",
        direct_pole_overrides(tilt, (FIXED_LEVER[0], FIXED_LEVER[1], 0.0)),
        repeats=3,
    )

# H3: above the plane, in it, or under it. The normal lever makes no moment
# against a normal press -- it drops out of f x r_c entirely -- and enters only
# as K_p,t r_n^2 of extra rotational stiffness, which resists the correction.
# So the prediction is a symmetric loss about r_n = 0 that only becomes
# visible past |r_n| ~ sqrt(K_R / K_p,t) = sqrt(5/2000) = 50 mm. MAIN_D3
# tested +20 mm and changed nothing, which is consistent but far too small to
# separate the prediction from no effect at all.
#
# r_c = p_TCP - p_c, so a positive normal lever puts the pole BELOW the TCP.
# The TCP stands about 20 mm off the plane at contact, so +20 is the pole in
# the plane, +60 and +120 are under it, and -60 is 80 mm above it.
H3_TILT, H3_LEVER = pole_direction(0.0)
for rc_n_mm, where in ((-60.0, "80 mm above the plane"),
                       (20.0, "in the plane, at the contact face"),
                       (60.0, "40 mm under the plane"),
                       (120.0, "100 mm under the plane")):
    tag = f"{'m' if rc_n_mm < 0 else 'p'}{abs(int(rc_n_mm)):03d}"
    add(
        f"MAIN_H3_rcn_{tag}",
        f"Case H: the H1 Y_EE condition with the pole moved along the normal "
        f"to r_c,n = {rc_n_mm:+.0f} mm -- {where}.",
        "Against MAIN_H1_rot_yEE, which is the same lever with r_c,n = 0. The "
        "moment rule predicts a symmetric loss in the sign of r_c,n, growing "
        "with r_c,n^2 and worth about K_p,t r_n^2 against a 5 N m/rad K_R. An "
        "asymmetry between above and under is not in the rule and would be "
        "the contact, not the spring.",
        direct_pole_overrides(H3_TILT, (H3_LEVER[0], H3_LEVER[1], rc_n_mm)),
        repeats=3,
    )


# Case E: tilt about the tool's own axes rather than the surface's. The
# commanded twist puts the 120 mm long axis 25.05 deg away from t2, so every
# tilt so far has tipped the long and short axes together in some mixture and
# none has tipped one alone. That leaves the campaign unable to say whether
# t2 correcting more than t1 is a property of the plane or of a face three
# times longer than it is wide. A tilt about a tool axis is the same command
# resolved onto the surface axes it is oblique to.
for name, axis_offset_deg, edge_mm in (("y_long", 0.0, 120), ("x_short", -90.0, 40)):
    a = math.radians(MAIN_TWIST_DEG + axis_offset_deg)
    t1_deg = TOOL_TILT_DEG * math.cos(a)
    t2_deg = TOOL_TILT_DEG * math.sin(a)
    add(
        f"MAIN_E1_tilt_about_{name}",
        f"Tilt {TOOL_TILT_DEG:.0f} deg about the tool's own "
        f"{name.split('_')[0].upper()}_EE axis, so the {edge_mm} mm edge leads. "
        f"Resolved onto the surface axes as t1 {t1_deg:+.2f}, t2 {t2_deg:+.2f} deg.",
        "Compare against A2 and A4, which tilt the same 10 deg about the "
        "surface axes. If the t1/t2 asymmetry follows the tool axes rather "
        "than the surface ones, it is the face aspect ratio and not the plane. "
        "Also the campaign's first negative commanded tilt component.",
        main_gain_overrides(t1_deg, t2_deg),
        repeats=3,
    )


# Case F: the null-space terms, isolated. This is a hold experiment, not a
# contact sequence: the arm holds one pose while a smooth point-force command
# displaces a point fixed to link 3, and the recovery is what is compared.
#
# The modes separate cleanly in the controller. Mode 1 returns projected
# damping alone; mode 2 zeroes that damping and applies only the
# smallest-singular-value bias; mode 3 is their sum and is therefore not run,
# because it cannot show anything the two single-term sweeps do not.
#
# Both gains are swept a factor of two either side of the configured value.
# k_sigma is bracketed deliberately: 1.0 is below the value at which the probe
# was observed to move the arm, 2.0 is about where motion becomes visible, and
# 4.0 is clearly driven. A setting that commands torque without moving the
# joints is not optimisation and must not be reported as such.
NULLSPACE_LEVELS = (1.0, 2.0, 4.0)


def nullspace_common(mode):
    return [
        pair for pair in main_gain_overrides(0.0, 0.0)
        if pair[0] != "nullspace_mode"
    ] + [
        ("nullspace_mode", f"{mode}"),
        ("disturbance_cues_enabled", "0"),
        ("disturbance_auto_enabled", "1"),
        ("disturbance_link", "3"),
        ("disturbance_point_link_x", "0.0"),
        ("disturbance_point_link_y", "0.0"),
        ("disturbance_point_link_z", "0.100"),
        ("disturbance_force", "20.0"),
        ("disturbance_direction_sign", "1.0"),
        ("disturbance_max_tau_norm", "2.0"),
        ("disturbance_release_ramp_time", "1.0"),
        ("disturbance_push_time", "5.0"),
        ("disturbance_hold_time", "7.0"),
        ("disturbance_release_time", "8.0"),
        # Displace, hold still, release, then recover. Ten seconds of recovery
        # and not the twenty-two first used: across the archived mode-1 runs
        # all null-space motion ceased within 2.6 s of the release cue and the
        # arm then sat idle for the remaining twenty, which is dead time in
        # every repetition. Ten still covers the sigma drive, which ran about
        # twelve seconds in the earlier conditioning runs.
        ("experiment_duration", "18.0"),
        # Terminal only. The sigma rows still reach the log; printing four
        # lines every debug period buries the operator cues in a 30 s hold.
        ("print_sigma_debug", "0"),
    ]


add(
    "PILOT_F_disturbance_10N",
    "Automatic null-space disturbance pilot: 10 N at link 3 with a +100 mm "
    "local-z lever, mode 0.",
    "The disturbance must produce measurable redundant motion without torque "
    "clipping, a reflex, or more than 2 mm Cartesian drift.",
    nullspace_common(0) + [
        ("disturbance_link", "3"),
        ("disturbance_point_link_z", "0.100"),
        ("disturbance_force", "10.0"),
    ],
    repeats=1,
)

add(
    "PILOT_F_disturbance_20N",
    "Automatic null-space disturbance pilot: 20 N at link 3 with a +100 mm "
    "local-z lever, mode 0.",
    "The disturbance must break joint friction without torque clipping, a "
    "reflex, or more than 2 mm Cartesian drift.",
    nullspace_common(0) + [
        ("disturbance_link", "3"),
        ("disturbance_point_link_z", "0.100"),
        ("disturbance_force", "20.0"),
    ],
    repeats=1,
)


add(
    "MAIN_F0_baseline",
    "Null-space mode 0 under the 20 N link-3 point-force disturbance: no "
    "null-space control torque.",
    "The reference the two swept modes are read against. Recovery here is "
    "whatever the arm does unaided along the redundant axis. The logged "
    "force and applied joint torque must reach the configured waveform.",
    nullspace_common(0),
    repeats=3,
)

for level in NULLSPACE_LEVELS:
    tag = f"{level:.1f}".replace(".", "p")
    add(
        f"MAIN_F1_damping_{tag}",
        f"Null-space mode 1 under the 20 N link-3 point-force disturbance: "
        f"projected "
        f"damping only, nullspace_damping = {level}.",
        "Compare recovery, Cartesian task drift and joint motion against the "
        "mode-0 reference. Damping alone has no preferred posture, so the arm "
        "settles wherever the redundant axis stops.",
        nullspace_common(1) + [("nullspace_damping", f"{level}")],
        repeats=3,
    )

for level in NULLSPACE_LEVELS:
    tag = f"{level:.1f}".replace(".", "p")
    add(
        f"MAIN_F2_ksigma_{tag}",
        f"Null-space mode 2 under the 20 N link-3 point-force disturbance: "
        f"smallest-singular-value bias only, k_sigma = {level}.",
        "Accept a setting only if the commanded null-space direction is "
        "actually followed by joint motion and the Cartesian task drift stays "
        "within its registered limit. Torque without motion is below the "
        "friction threshold, not optimisation.",
        nullspace_common(2) + [("nullspace_k_sigma", f"{level}")],
        repeats=1 if level == 4.0 else 3,
    )


# Stronger follow-on requested after the first automatic campaign.  These use
# new run IDs so the archived +100 mm Case-F records are never reinterpreted as
# having received the larger moment.  The +150 mm virtual point gives the same
# 20 N force a larger lever about link 3, while the 2.5 Nm ceiling limits the
# increase before any hardware pilot is accepted.  Sigma settings below
# 1.5 Nm are omitted because the first campaign showed that 1 Nm mostly loaded
# joint friction without producing a useful conditioning response.
NULLSPACE_STRONG_SIGMA_LEVELS = (1.5, 2.0)


def nullspace_strong_common(mode):
    return merged_overrides(nullspace_common(mode), [
        ("disturbance_point_link_z", "0.150"),
        ("disturbance_max_tau_norm", "2.5"),
    ])


add(
    "PILOT_F_disturbance_20N_150mm",
    "Stronger automatic null-space disturbance pilot: 20 N at link 3 with "
    "a +150 mm local-z lever, mode 0.",
    "Accept only if the larger moment produces visible redundant motion "
    "without torque clipping, a reflex, or more than 2 mm Cartesian drift.",
    nullspace_strong_common(0),
    repeats=1,
)

add(
    "MAIN_F3_baseline_150mm",
    "Mode-0 reference for the stronger 20 N, +150 mm link-3 point-force "
    "disturbance.",
    "Quantify the increased redundant excursion and verify the force and "
    "joint-torque waveform before comparing any null-space gain.",
    nullspace_strong_common(0),
    repeats=3,
)

add(
    "MAIN_F3_damping_2p0_150mm",
    "Projected damping only under the stronger disturbance, "
    "nullspace_damping = 2.0 Nms/rad.",
    "Compare with the stronger mode-0 reference; damping should reduce the "
    "excursion without imposing a preferred posture.",
    nullspace_strong_common(1) + [("nullspace_damping", "2.0")],
    repeats=3,
)

for level in NULLSPACE_STRONG_SIGMA_LEVELS:
    tag = f"{level:.1f}".replace(".", "p")
    add(
        f"MAIN_F4_ksigma_{tag}_150mm",
        f"Sigma-only response under the stronger disturbance, "
        f"k_sigma = {level} Nm.",
        "The setting must produce observable motion along the selected "
        "conditioning direction while the Cartesian position error remains "
        "within 2 mm.",
        nullspace_strong_common(2) + [
            ("nullspace_k_sigma", f"{level}"),
        ],
        repeats=3,
    )


# Balanced follow-on after the 40 N, +200 mm pilot clipped at 5 Nm and exceeded
# the Cartesian-error gate.  Keeping the 200 mm point but halving the force
# gives a static prediction of about 2.17 Nm at the Case-F start pose, with
# margin below the 3 Nm safety ceiling.
def nullspace_balanced_common(mode):
    return merged_overrides(nullspace_common(mode), [
        ("disturbance_point_link_z", "0.200"),
        ("disturbance_force", "20.0"),
        ("disturbance_max_tau_norm", "3.0"),
    ])


add(
    "PILOT_F_disturbance_20N_200mm",
    "Balanced automatic null-space disturbance pilot: 20 N at link 3 with "
    "a +200 mm local-z lever, mode 0.",
    "Accept only if it exceeds the original disturbance without clipping, a "
    "reflex, or more than 2 mm Cartesian error.",
    nullspace_balanced_common(0),
    repeats=1,
)

add(
    "MAIN_F7_baseline_20N_200mm",
    "Mode-0 reference for the balanced 20 N, +200 mm link-3 point-force "
    "disturbance.",
    "Quantify the increased excursion and verify the complete force and "
    "joint-torque waveform before comparing null-space gains.",
    nullspace_balanced_common(0),
    repeats=3,
)

add(
    "MAIN_F7_damping_2p0_20N_200mm",
    "Projected damping only under the balanced disturbance, "
    "nullspace_damping = 2.0 Nms/rad.",
    "Compare with the balanced mode-0 reference; damping should reduce the "
    "excursion without imposing a preferred posture.",
    nullspace_balanced_common(1) + [("nullspace_damping", "2.0")],
    repeats=3,
)

for level in NULLSPACE_STRONG_SIGMA_LEVELS:
    tag = f"{level:.1f}".replace(".", "p")
    add(
        f"MAIN_F8_ksigma_{tag}_20N_200mm",
        f"Sigma-only response under the balanced disturbance, "
        f"k_sigma = {level} Nm.",
        "The setting must produce observable motion along the selected "
        "conditioning direction while the Cartesian position error remains "
        "within 2 mm.",
        nullspace_balanced_common(2) + [
            ("nullspace_k_sigma", f"{level}"),
        ],
        repeats=3,
    )


# Preserved rejected candidate: the 40 N, +200 mm pilot reached its 5 Nm
# ceiling, reduced the requested force waveform, and exceeded the Cartesian
# error gate.  It remains generated for provenance but is not scheduled by the
# automatic runner; the active follow-on is the balanced 20 N setup above.
def nullspace_stronger_common(mode):
    return merged_overrides(nullspace_common(mode), [
        ("disturbance_point_link_z", "0.200"),
        ("disturbance_force", "40.0"),
        ("disturbance_max_tau_norm", "5.0"),
    ])


add(
    "PILOT_F_disturbance_40N_200mm",
    "Stronger automatic null-space disturbance pilot: 40 N at link 3 with "
    "a +200 mm local-z lever, mode 0.",
    "Accept only if the larger moment produces visible redundant motion "
    "without torque clipping, a reflex, or more than 2 mm Cartesian drift.",
    nullspace_stronger_common(0),
    repeats=1,
)

add(
    "MAIN_F5_baseline_40N_200mm",
    "Mode-0 reference for the stronger 40 N, +200 mm link-3 point-force "
    "disturbance.",
    "Quantify the increased redundant excursion and verify the force and "
    "joint-torque waveform before comparing any null-space gain.",
    nullspace_stronger_common(0),
    repeats=3,
)

add(
    "MAIN_F5_damping_2p0_40N_200mm",
    "Projected damping only under the stronger disturbance, "
    "nullspace_damping = 2.0 Nms/rad.",
    "Compare with the stronger mode-0 reference; damping should reduce the "
    "excursion without imposing a preferred posture.",
    nullspace_stronger_common(1) + [("nullspace_damping", "2.0")],
    repeats=3,
)

for level in NULLSPACE_STRONG_SIGMA_LEVELS:
    tag = f"{level:.1f}".replace(".", "p")
    add(
        f"MAIN_F6_ksigma_{tag}_40N_200mm",
        f"Sigma-only response under the stronger disturbance, "
        f"k_sigma = {level} Nm.",
        "The setting must produce observable motion along the selected "
        "conditioning direction while the Cartesian position error remains "
        "within 2 mm.",
        nullspace_stronger_common(2) + [
            ("nullspace_k_sigma", f"{level}"),
        ],
        repeats=3,
    )


# Compact absolute-orientation validation on the separately calibrated tilted
# plane. These reproduce the three baseline angle cases from horizontal Case A.
# A final tuned condition is added only after horizontal Cases A--D select it.
for run_id, a1, a2 in (
    ("VALID_T0_00deg", 0.0, 0.0),
    ("VALID_T1_t1_10deg", 10.0, 0.0),
    ("VALID_T2_t2_10deg", 0.0, 10.0),
):
    add(
        run_id,
        f"Tilted-plane validation at the baseline gains with independent "
        f"tool offsets (t1={a1:+.0f} deg, t2={a2:+.0f} deg).",
        "Compare with the matched horizontal Case-A result using the measured "
        "first-contact angle. This is a frame-transfer check, not a second "
        "parameter sweep.",
        main_gain_overrides(
            a1, a2, q_init_case="tilted_tool"
        ),
        repeats=3,
    )

# ---- Series B: centre of compliance / pole ---------------------------------
# NOTE ON THE NAME. This setup was originally called B1_pole_at_tcp and claimed
# a zero lever. It does not have one: main.cpp computes
#     r_c = tcp_ref - (edge_ref + coupled_pole_from_edge)
# so zeroing the parameter puts the pole on the CONTACT EDGE, and the measured
# lever is r_c = [-13.8, 60.7, 41.4] mm, about 75 mm. The runs recorded under
# the old name are a valid pole-on-edge measurement and are kept; the zero-lever
# gate it was meant to be is B1b below.
add(
    "B1_pole_on_contact_edge",
    "Coupled law with the pole on the contact edge (coupled_pole_from_edge = 0).",
    "Historically the pole-on-edge case suppressed alignment almost entirely. "
    "Also serves as a held-out check of the quadratic pole model fitted on "
    "B2/B3/B4: predicted +2.44 deg, measured +2.30 +/- 0.07 deg.",
    [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_pole_freeze_at_contact", "1"),
    ]
    + pole_keys([0.0, 0.0, 0.0]),
)

# The lever measured under the old B1 was repeatable to 0.1 mm across three
# runs, so cancelling it with a fixed offset puts the pole within a tenth of a
# millimetre of the TCP -- close enough for Ad to reduce to the identity.
add(
    "B1b_pole_at_tcp",
    "Coupled law with the pole AT the TCP: the offset cancels the measured "
    "TCP-to-edge lever, giving r_c ~ 0.",
    "Second correctness gate. With a zero lever the adjoint reduces to the "
    "identity, so this must reproduce the decoupled baseline "
    "(2.458 +/- 0.111 deg tip, 59.37 +/- 0.11 N). Check the printed r_c is "
    "within a millimetre of zero before trusting the run.",
    [
        ("use_coupled_stiffness", "1"),
        ("coupled_use_block_diagonal", "0"),
        ("coupled_pole_manual", "1"),
        ("coupled_pole_freeze_at_contact", "1"),
    ]
    + pole_keys([-0.0138, 0.0607, 0.0414]),
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


# A setup declares its pole convention if it names any of these, or if it turns
# the coupled spring off or onto the block diagonal, where no pole is read.
POLE_CONVENTION_KEYS = {
    "coupled_use_pole_ee",
    "coupled_use_direct_rc_surface",
    "coupled_pole_from_edge_x",
    "coupled_pole_from_edge_y",
    "coupled_pole_from_edge_z",
    "use_coupled_stiffness",
    "coupled_use_block_diagonal",
}

ARCHIVED_POLE_CONVENTION = """
# The pole convention these trials were archived with. Pinned here because the
# nominal set now defaults to the surface convention Case H used; without this
# the same setup would press about a different lever than its archive.
coupled_use_pole_ee = 0
coupled_use_direct_rc_surface = 0
coupled_pole_from_edge_x = -0.04
coupled_pole_from_edge_y = 0.08
coupled_pole_from_edge_z = 0.0
"""


def write_setups():
    os.makedirs(SETUPS, exist_ok=True)
    index = []
    for run_id, purpose, criterion, overrides, repeats in SPEC:
        d = os.path.join(SETUPS, run_id)
        os.makedirs(d, exist_ok=True)
        plane_profile = None
        tool_profile = None
        if run_id.startswith(("MAIN_", "PILOT_COC_")):
            plane_profile = "horizontal"
            tool_profile = "grinding_tool"
        elif run_id.startswith("VALID_T"):
            plane_profile = "tilted"
            tool_profile = "grinding_tool"

        # A setup that says nothing about the pole would follow whatever the
        # nominal set defaults to, which is the surface convention Case H
        # introduced. These trials were archived pressing about the edge pole,
        # so the convention they ran with is written out rather than inherited.
        keys = {key for key, _ in overrides}
        inherits_pole = not (keys & POLE_CONVENTION_KEYS)

        with open(os.path.join(d, "overlay.txt"), "w") as f:
            f.write(f"# {run_id}\n")
            f.write("# Applied on top of surface_grinding_controller/params/.\n")
            f.write("# Only keys listed here differ from the nominal set.\n")
            if not overrides and not inherits_pole:
                f.write("# (nominal configuration, no overrides)\n")
            for key, value in overrides:
                f.write(f"{key} = {value}\n")
            if inherits_pole:
                f.write(ARCHIVED_POLE_CONVENTION)

        # Which startup key drives this setup. Case F holds a pose and is
        # driven with h; every contact case runs the sequence with s. The
        # runner and the unattended driver both read this rather than guessing
        # from the run id.
        startup_mode = (
            "h" if run_id.startswith(("MAIN_F", "PILOT_F")) else "s"
        )
        with open(os.path.join(d, "startup_mode.txt"), "w") as f:
            f.write(startup_mode + "\n")

        with open(os.path.join(d, "about.txt"), "w") as f:
            f.write(f"run_id:   {run_id}\n")
            f.write(f"repeats:  {repeats}\n\n")
            if plane_profile:
                f.write(f"plane profile: {plane_profile}\n\n")
            if tool_profile:
                f.write(f"tool profile: {tool_profile}\n\n")
            f.write("purpose:\n  " + purpose.replace("\n", "\n  ") + "\n\n")
            f.write("pass criterion:\n  " + criterion.replace("\n", "\n  ") + "\n")

        profile_path = os.path.join(d, "plane_profile.txt")
        if plane_profile:
            with open(profile_path, "w") as f:
                f.write(plane_profile + "\n")
        elif os.path.exists(profile_path):
            os.unlink(profile_path)

        tool_profile_path = os.path.join(d, "tool_profile.txt")
        if tool_profile:
            with open(tool_profile_path, "w") as f:
                f.write(tool_profile + "\n")
        elif os.path.exists(tool_profile_path):
            os.unlink(tool_profile_path)

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
