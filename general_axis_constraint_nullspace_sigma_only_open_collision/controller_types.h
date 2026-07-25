#pragma once

#include "controller_common.h"

// Sequence phases plus the two standalone startup modes.
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
  // Robot connection, logging and terminal debug.
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 0.0;  // <= 0 runs until e + Enter
  std::string csv_file_name = "general_axis_constraint_nullspace_sigma_only_open_collision_log.csv";
  int log_every_n_cycles = 5;
  int max_log_rows = 120000;
  double debug_period = 0.20;
  bool print_hold_debug = true;
  bool print_grind_debug = true;
  bool print_coupled_diagnostics = true;

  // Gripper action performed once after q_init.
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
  // Runtime-only: set when the startup menu already handled the gripper.
  bool startup_gripper_manual = false;

  // Run mode.
  // 1 = sequence, 0 = hold.
  bool use_phase_sequence = true;
  // 0 = skip the orient step and start the sequence at the descend step.
  bool use_approach_orient = true;
  bool use_manual_guidance_start = false;
  double manual_guidance_damping = 0.5;

  // Hold mode gains (isotropic, base frame).
  double hold_Kp = 300.0;
  double hold_Dp = 50.0;
  double hold_KR = 40.0;
  double hold_DR = 8.0;  // applied directly; hold rotation is not auto-damped
  bool hold_auto_damping = true;
  double hold_auto_damping_factor = 1.0;

  // Surface plane and tool geometry.
  bool constraint_enabled = true;
  bool use_start_as_surface_point = true;
  Vec3 surface_point = Vec3(0.0, 0.0, 0.0);
  Vec3 alignment_target_normal = Vec3(1.0, 0.0, 0.0);
  bool use_alignment_target_tilt_angle = false;
  // 1 = derive tilt angles from the normal vector during parameter loading.
  bool derive_tilt_angles_from_plane_normal = false;
  double alignment_target_tilt_angle_deg = 0.0;    // a, about base x
  double alignment_target_tilt_angle_y_deg = 0.0;  // b, about base y
  Vec3 alignment_target_tangent1 = Vec3(0.0, 1.0, 0.0);
  Vec3 tool_axis_ee = Vec3(0.0, 0.0, 1.0);
  double tool_axis_target_sign = -1.0;
  bool use_tool_contact_point_control = true;
  bool auto_select_tool_contact_edge = true;
  Vec3 tool_contact_point_ee = Vec3(0.0, 0.0, 0.0);
  // Rotational spring mask in the alignment-target frame.
  bool constrain_rotation_about_alignment_normal = true;
  bool constrain_rotation_about_alignment_tangent1 = true;
  bool constrain_rotation_about_alignment_tangent2 = true;

  // Phase 1: approach (orient, then descend).
  double approach_orient_min_time = 0.5;
  double approach_orient_error_threshold = 0.03;
  // One impedance for both approach steps, in [tangent1, tangent2, normal].
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
  // Clearance above the surface before set up takes over.
  double descend_surface_clearance = 0.020;

  // Phase 2: set up.
  double setup_min_time = 0.3;       // before the moment early-exit can fire
  double setup_duration = 15.0;      // hard time limit for the phase
  double setup_moment_threshold = 60.0;
  // Position preload ramped into the surface; grind inherits the final value.
  double setup_push_speed = 0.0;
  double setup_max_push = 0.0;
  // Translational spring, base frame [x, y, z].
  Vec3 setup_Kp_diag = Vec3(40.0, 40.0, 5500.0);
  Vec3 setup_Dp_diag = Vec3(10.0, 10.0, 175.0);
  // Rotational spring, alignment-target frame [tangent1, tangent2, normal].
  Vec3 setup_KR_diag = Vec3(0.0, 0.0, 8.0);
  Vec3 setup_DR_diag = Vec3(0.01, 0.01, 4.0);
  bool setup_auto_damping = false;
  double setup_auto_damping_factor = 1.0;

  // Phase 3: grind (constant press, shared gains with phase 2).
  // 1 = commanded sweep, 0 = free-slide press hold.
  bool grind_sweep_enabled = false;
  int grind_axis = 1;               // 1 = tangent1, 2 = tangent2
  double grind_amplitude_m = 0.03;  // sweep half-amplitude A [m]
  double grind_frequency_hz = 0.2;  // one full back-and-forth cycle [Hz]

  // Enter gates between phases.
  bool pause_before_set_up = false;  // hold at the clearance height
  bool pause_before_grind = false;   // hold the seated/pressed pose
  // Position-only stiff hold while paused.
  double pause_hold_Kp = 5000.0;
  double pause_hold_Dp = 200.0;

  // Coupled (pole-based) stiffness for the set-up phase.
  bool use_coupled_stiffness = false;   // command the 6x6 spring
  bool eval_coupled_stiffness = false;  // only compute it side-by-side
  // K/D source: block diagonal, manual pole, or saved matrices.
  bool coupled_use_block_diagonal = false;
  bool coupled_pole_manual = false;
  Vec3 coupled_pole_from_edge = Vec3::Zero();
  // 1 = freeze pole at first contact, 0 = track the live moving edge.
  bool coupled_pole_freeze_at_contact = true;
  // Auto-written after a set-up phase with a valid finite screw axis.
  bool coupled_gains_saved = false;
  Mat6x6 coupled_K_tcp = Mat6x6::Zero();
  Mat6x6 coupled_D_tcp = Mat6x6::Zero();

  // Nullspace optimization.
  bool use_nullspace_optimization = true;
  NullspaceMode nullspace_mode = NullspaceMode::kPostureAndSigma;
  double nullspace_k_start = 1.0;
  double nullspace_damping = 1.0;
  double nullspace_k_sigma = 0.5;
  double nullspace_alpha = 0.01;

  // Auto-damping options.
  double auto_damping_max = 8000.0;
  // 1 = manual Dp/DR values are per-axis floors, not only fallbacks.
  bool auto_damping_min_from_manual = false;
  bool print_auto_damping = true;

  // Start pose and collision thresholds.
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
