#pragma once

#include "controller_common.h"

// A sequence run walks these phases in order. The three numbered phases are the
// experiment itself; kHold and kManualGuide are the two standalone modes the
// startup menu can pick instead.
//
//   1. approach  -- orient the tool, then descend to the clearance height.
//   2. set up    -- press the contact edge into the plane until the tool seats
//                   flat on it (the rotational spring stays soft so real
//                   contact moment does the tipping).
//   3. grind     -- keep that press constant and sweep along a surface tangent.
enum class ControlPhase {
  kApproachOrient,
  kApproachDescend,
  kSetUp,
  kGrind,
  kHold,
  kManualGuide
};

enum class NullspaceMode {
  kOff = 0,
  kPostureOnly = 1,
  kSigmaOnly = 2,
  kPostureAndSigma = 3
};

struct Parameters {
  // ================================================================
  // Robot connection, logging and terminal debug
  // ================================================================
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 0.0;  // <= 0 runs until e + Enter
  std::string csv_file_name = "general_axis_constraint_nullspace_sigma_only_open_collision_log.csv";
  int log_every_n_cycles = 5;
  int max_log_rows = 120000;
  double debug_period = 0.20;
  bool print_hold_debug = true;
  bool print_grind_debug = true;
  bool print_coupled_diagnostics = true;

  // ================================================================
  // Gripper action performed once after q_init
  // ================================================================
  bool open_gripper_before_run = true;
  bool require_gripper_open = true;
  // 1 = close onto the tool held in the hand, 0 = open to gripper_open_width.
  bool gripper_grasp_on_tool = false;
  double gripper_open_width = 0.08;
  double gripper_open_speed = 0.05;
  double gripper_grasp_width = 0.02;
  double gripper_grasp_speed = 0.05;
  double gripper_grasp_force = 40.0;
  double gripper_grasp_epsilon_inner = 0.005;
  double gripper_grasp_epsilon_outer = 0.010;
  // Runtime-only (never read from file): set when the user works the gripper
  // from the startup menu, which suppresses the automatic action at q_init so
  // an explicit open/grasp is left exactly as the user left it.
  bool startup_gripper_manual = false;

  // ================================================================
  // Run mode
  // ================================================================
  // 1 = run the phase sequence (approach -> set up -> grind).
  // 0 = hold at the start pose.
  bool use_phase_sequence = true;
  // 0 = skip the orient step and start the sequence at the descend step.
  bool use_approach_orient = true;
  bool use_manual_guidance_start = false;
  double manual_guidance_damping = 0.5;

  // ================================================================
  // Hold mode gains (isotropic, base frame)
  // ================================================================
  double hold_Kp = 300.0;
  double hold_Dp = 50.0;
  double hold_KR = 40.0;
  double hold_DR = 8.0;  // applied directly; hold rotation is not auto-damped
  bool hold_auto_damping = true;
  double hold_auto_damping_factor = 1.0;

  // ================================================================
  // Surface plane and tool geometry
  // ================================================================
  bool constraint_enabled = true;
  bool use_start_as_surface_point = true;
  Vec3 surface_point = Vec3(0.0, 0.0, 0.0);
  Vec3 alignment_target_normal = Vec3(1.0, 0.0, 0.0);
  bool use_alignment_target_tilt_angle = false;
  // 1 = derive the two tilt angles FROM the normal vector instead of reading
  // them: a = -asin(n_y), b = atan2(n_x, n_z).
  bool derive_tilt_angles_from_plane_normal = false;
  double alignment_target_tilt_angle_deg = 0.0;    // a, about base x
  double alignment_target_tilt_angle_y_deg = 0.0;  // b, about base y
  Vec3 alignment_target_tangent1 = Vec3(0.0, 1.0, 0.0);
  Vec3 tool_axis_ee = Vec3(0.0, 0.0, 1.0);
  double tool_axis_target_sign = -1.0;
  bool use_tool_contact_point_control = true;
  bool auto_select_tool_contact_edge = true;
  Vec3 tool_contact_point_ee = Vec3(0.0, 0.0, 0.0);
  // Per-axis rotational spring mask in the alignment-target frame. 0 = no
  // spring on that component (damping still acts unless its DR is 0 too).
  bool constrain_rotation_about_alignment_normal = true;
  bool constrain_rotation_about_alignment_tangent1 = true;
  bool constrain_rotation_about_alignment_tangent2 = true;

  // ================================================================
  // Phase 1: approach (orient, then descend)
  // ================================================================
  double approach_orient_min_time = 0.5;
  double approach_orient_error_threshold = 0.03;
  // One impedance for BOTH approach steps, alignment-target frame
  // [tangent1, tangent2, normal]. Position stays soft; KR_tangent must be stiff
  // enough that orient converges below approach_orient_error_threshold.
  Vec3 approach_Kp_diag = Vec3(150.0, 150.0, 150.0);
  Vec3 approach_KR_diag = Vec3(90.0, 90.0, 8.0);
  Vec3 approach_Dp_diag = Vec3(20.0, 20.0, 20.0);
  Vec3 approach_DR_diag = Vec3(12.0, 12.0, 12.0);
  bool approach_auto_damping = false;
  double approach_auto_damping_factor = 1.0;
  // 1 = descend along -alignment_target_normal, 0 = along descend_direction.
  bool descend_use_alignment_target_normal = true;
  Vec3 descend_direction = Vec3(0.0, 0.0, -1.0);
  double descend_speed = 0.005;
  double descend_max_distance = 0.02;
  // The descend step ends when the active tool contact point is this far above
  // the plane; the set-up press takes over from there.
  double descend_surface_clearance = 0.020;

  // ================================================================
  // Phase 2: set up (press the edge down until the tool sits flat)
  // ================================================================
  double setup_min_time = 0.3;       // before the moment early-exit can fire
  double setup_duration = 15.0;      // hard time limit for the phase
  double setup_moment_threshold = 60.0;
  // The press is a position preload ramped into the plane. It starts at
  // -descend_surface_clearance (the tool is still that far above the plane) and
  // grows at setup_push_speed until setup_max_push. Grind inherits its final
  // value, so this ramp sets the press for BOTH phase 2 and phase 3.
  double setup_push_speed = 0.0;
  double setup_max_push = 0.0;
  // Translational spring, base frame [x, y, z]. Firm normal for the press,
  // soft tangents so the tool is free to rotate about the pressed edge.
  Vec3 setup_Kp_diag = Vec3(40.0, 40.0, 5500.0);
  Vec3 setup_Dp_diag = Vec3(10.0, 10.0, 175.0);
  // Rotational spring, alignment-target frame [tangent1, tangent2, normal].
  // Normal (yaw) stays constrained; the tangents stay very soft so contact
  // moment can tip the tool flat.
  Vec3 setup_KR_diag = Vec3(0.0, 0.0, 8.0);
  Vec3 setup_DR_diag = Vec3(0.01, 0.01, 4.0);
  bool setup_auto_damping = false;
  double setup_auto_damping_factor = 1.0;

  // ================================================================
  // Phase 3: grind (constant press, shared gains with phase 2)
  // ================================================================
  // 1 = sweep the contact side-to-side along a surface tangent (tangentially
  //     stiff, so the tool tracks the sweep path).
  // 0 = free-slide hold: only the normal direction is sprung, so the pressed
  //     tool can be pushed along the plane by hand.
  bool grind_sweep_enabled = false;
  int grind_axis = 1;               // 1 = tangent1, 2 = tangent2
  double grind_amplitude_m = 0.03;  // sweep half-amplitude A [m]
  double grind_frequency_hz = 0.2;  // one full back-and-forth cycle [Hz]

  // ================================================================
  // Enter gates between phases (bare Enter continues, e + Enter stops)
  // ================================================================
  bool pause_before_set_up = false;  // hold at the clearance height
  bool pause_before_grind = false;   // hold the seated/pressed pose
  // Stiff isotropic position hold used only while paused, so the tool locks in
  // place instead of holding with the soft approach gains. Position only --
  // a stiff rotational lock here drove a limit-cycle wiggle.
  double pause_hold_Kp = 5000.0;
  double pause_hold_Dp = 200.0;

  // ================================================================
  // Coupled (pole-based) stiffness for the set-up phase
  // ================================================================
  // The decoupled law commands f = Kp*e_p + Dp*(pdot_d - pdot) and
  // m = KR*e_R - DR*omega, i.e. two independent 3x3 springs.
  //
  // The coupled law instead commands ONE 6x6 spring built by moving the same
  // diagonal spring from a chosen pole out to the TCP through the adjoint:
  //
  //   K_TCP = Ad(r_c)^T * blockdiag(Kp, KR) * Ad(r_c),   r_c = p_TCP - pole
  //
  // The off-diagonal quadrants of K_TCP are the lever-arm coupling: rotation
  // then produces force and translation produces moment. The pole is the only
  // knob in the adjoint, so sweeping it moves the effective rotation center.
  bool use_coupled_stiffness = false;   // command the 6x6 spring
  bool eval_coupled_stiffness = false;  // only compute it side-by-side
  // Source of the 6x6 K/D when use_coupled_stiffness = 1, in priority order:
  //   coupled_use_block_diagonal = 1 -> the plain block-diagonal set-up gains
  //     (no coupling). Fed through the 6x6 path this reproduces the decoupled
  //     wrench exactly, so it is the sanity check that the path is correct.
  //   coupled_pole_manual = 1 -> rebuild K_TCP/D_TCP from
  //     pole = contact edge + coupled_pole_from_edge.
  //   otherwise -> the saved matrices below.
  bool coupled_use_block_diagonal = false;
  bool coupled_pole_manual = false;
  Vec3 coupled_pole_from_edge = Vec3::Zero();
  // 1 = pin the pole to the FIRST contact pose, so K_TCP/D_TCP are computed
  // once and stay constant for the whole phase (predictable when sweeping).
  // 0 = track the live moving edge, refreshing the matrices every cycle.
  bool coupled_pole_freeze_at_contact = true;
  // Auto-written after a set-up phase with a valid finite screw axis.
  bool coupled_gains_saved = false;
  Mat6x6 coupled_K_tcp = Mat6x6::Zero();
  Mat6x6 coupled_D_tcp = Mat6x6::Zero();

  // ================================================================
  // Nullspace optimization
  // ================================================================
  bool use_nullspace_optimization = true;
  NullspaceMode nullspace_mode = NullspaceMode::kPostureAndSigma;
  double nullspace_k_start = 1.0;
  double nullspace_damping = 1.0;
  double nullspace_k_sigma = 0.5;
  double nullspace_alpha = 0.01;

  // Upper clamp on any auto-damping value computed as D = factor*2*sqrt(M*K).
  double auto_damping_max = 8000.0;
  // 1 = treat each group's manual Dp/DR as a per-axis FLOOR under the computed
  // value, instead of a pure fallback: D_i = max(computed_i, manual_i). This
  // matters on very soft axes, where the formula returns near zero and leaves
  // that axis almost undamped -- for example the soft set-up tipping axes,
  // where the computed damping lands far below the manual entry. 0 = use the
  // computed value as-is, and fall back to the manual one only when the online
  // inertia estimate is unavailable.
  bool auto_damping_min_from_manual = false;
  // 1 = print the computed damping next to the manual entry each time a phase
  // group's auto-damping is (re)computed, so the two can be compared on the
  // real robot before deciding whether the floor is needed.
  bool print_auto_damping = true;

  // ================================================================
  // Start pose and collision thresholds
  // ================================================================
  Array7 q_init = {{
      0.0,
      -M_PI_4,
      0.0,
      -3.0 * M_PI_4,
      0.0,
      M_PI_2,
      0.0
  }};
  std::string q_init_case = "horizontal_tool";

  bool use_custom_collision_behavior = false;
  double collision_torque_acc = 80.0;
  double collision_torque_nom = 80.0;
  double collision_force_acc = 80.0;
  double collision_force_nom = 80.0;
};

struct LogData {
  double time;
  int phase;

  Vec3 p_EE;
  Vec3 p_d;
  Vec3 tool_contact_point;
  Vec3 first_contact_tcp;
  Vec3 first_contact_point;
  Vec3 edge_target;
  Vec3 tool_contact_offset_ee;

  Vec3 e_p;
  Vec3 e_R;

  Vec3 pdot;
  Vec3 pdot_d;
  Vec3 omega;

  Vec3 f;
  Vec3 m;
  Vec3 external_force;
  Vec3 external_moment;
  Vec3 contact_force_bias;
  Vec3 contact_moment_bias;
  double push;

  Vec7 tau_cmd;
};

struct DesiredMotion {
  Vec3 p_d;
  Vec3 pdot_d;
};
