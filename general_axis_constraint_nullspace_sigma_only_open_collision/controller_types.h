#pragma once

#include "controller_common.h"

struct Parameters {
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 10.0;
  std::string csv_file_name = "general_axis_constraint_nullspace_sigma_only_open_collision_log.csv";
  int log_every_n_cycles = 5;
  int max_log_rows = 120000;
  double debug_period = 0.20;
  bool print_impedance_debug = true;
  bool open_gripper_before_run = true;
  bool require_gripper_open = true;
  double gripper_open_width = 0.08;
  double gripper_open_speed = 0.05;

  bool hold_mode = true;

  bool constraint_enabled = true;
  bool use_start_as_surface_point = true;
  Vec3 surface_point = Vec3(0.0, 0.0, 0.0);
  Vec3 alignment_target_normal = Vec3(1.0, 0.0, 0.0);
  bool use_alignment_target_tilt_angle = false;
  double alignment_target_tilt_angle_deg = 0.0;
  Vec3 alignment_target_tangent1 = Vec3(0.0, 1.0, 0.0);
  Vec3 tool_axis_ee = Vec3(0.0, 0.0, 1.0);
  double tool_axis_target_sign = -1.0;
  bool use_tool_contact_point_control = true;
  bool auto_select_tool_contact_edge = true;
  Vec3 tool_contact_point_ee = Vec3(0.0, 0.0, 0.0);
  bool align_orientation_to_surface_after_contact = false;
  bool orientation_test_only = false;
  double orientation_test_extra_tilt_deg = 0.0;
  bool use_phase_sequence = true;
  double orient_phase_min_time = 0.5;
  double orient_phase_error_threshold = 0.03;
  double post_contact_align_min_time = 0.3;
  double post_contact_align_duration = 15.0;
  double post_contact_best_axis_min_time = 0.60;
  double post_contact_best_axis_min_omega = 0.12;
  double post_contact_moment_threshold = 60.0;
  double post_contact_normal_push = 0.0;
  double post_contact_push_speed = 0.0;
  double post_contact_max_push = 0.0;
  bool post_contact_eval_method2_tcp_wrench = false;
  bool post_contact_apply_method2_tcp_wrench = false;
  bool method2_tcp_wrench_saved = false;
  Mat6x6 method2_K_tcp_base = Mat6x6::Zero();
  Mat6x6 method2_D_tcp_base = Mat6x6::Zero();
  bool use_search_direction_surface_after_alignment = true;
  bool use_virtual_center_after_contact = false;
  double vcr_offset = 0.0;
  bool print_gain_suggestion_diagnostics = true;
  Vec3 last_best_axis_from_edge = Vec3(0.0144, 0.0094, -0.0064);
  Vec3 last_best_axis_dir = Vec3(-0.919, -0.390, -0.053);
  double last_best_axis_pitch = -0.0146;
  Vec3 last_chasles_axis_from_edge = Vec3(0.0144, 0.0094, -0.0064);
  Vec3 last_chasles_axis_dir = Vec3(-0.919, -0.390, -0.053);
  double last_chasles_axis_pitch = -0.0146;
  double suggested_gain_omega_ref = 0.20;
  double suggested_gain_angle_ref = 0.10;
  double suggested_gain_min = 0.01;
  double suggested_gain_max = 8000.0;
  Vec3 quasi_force_limit = Vec3(15.0, 10.0, 10.0);
  Vec3 quasi_displacement_limit = Vec3(0.005, 0.010, 0.010);
  Vec3 quasi_moment_limit = Vec3(0.75, 0.75, 0.75);
  Vec3 quasi_angle_limit = Vec3(0.1745, 0.1745, 0.1745);
  Vec3 quasi_effective_mass = Vec3(1.0, 1.0, 1.0);
  Vec3 quasi_effective_inertia = Vec3(1.0, 1.0, 1.0);
  double quasi_damping_ratio = 1.0;
  double effective_moment_fit_ridge = 1e-8;

  bool use_contact_search = false;
  bool contact_search_use_alignment_target_normal = true;
  Vec3 contact_search_direction = Vec3(0.0, 0.0, -1.0);
  double contact_search_speed = 0.005;
  double contact_search_max_distance = 0.02;
  double contact_search_min_distance = 0.0;
  double contact_search_first_touch_min_distance = 0.0;
  bool contact_search_use_directional_force = true;
  double contact_search_confirm_time = 0.05;
  double contact_force_threshold = 5.0;
  bool detect_contact_during_alignment = true;
  double alignment_contact_force_threshold = 5.0;
  Vec3 contact_search_Kp_diag = Vec3(150.0, 150.0, 150.0);
  Vec3 contact_search_Dp_diag = Vec3(25.0, 25.0, 25.0);
  Vec3 post_contact_Kp_diag = Vec3(40.0, 40.0, 5500.0);
  Vec3 post_contact_Dp_diag = Vec3(10.0, 10.0, 175.0);
  Vec3 post_contact_KR_diag = Vec3(8.0, 0.0, 0.0);
  Vec3 post_contact_DR_diag = Vec3(4.0, 0.01, 0.01);

  bool constrain_rotation_about_alignment_normal = true;
  bool constrain_rotation_about_alignment_tangent1 = true;
  bool constrain_rotation_about_alignment_tangent2 = true;

  bool use_nullspace_optimization = true;
  double nullspace_k_start = 1.0;
  double nullspace_damping = 1.0;
  double nullspace_k_sigma = 0.5;
  double nullspace_alpha = 0.01;

  Vec3 delta_p = Vec3(0.005, 0.0, 0.0);
  double trajectory_duration = 8.0;

  Vec3 Kp_diag = Vec3(150.0, 100.0, 100.0);
  Vec3 Dp_diag = Vec3(25.0, 20.0, 20.0);

  Vec3 KR_diag = Vec3(3.0, 3.0, 3.0);
  Vec3 DR_diag = Vec3(3.5, 3.5, 3.5);

  double collision_torque_acc = 80.0;
  double collision_torque_nom = 80.0;
  double collision_force_acc = 80.0;
  double collision_force_nom = 80.0;

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

  Array7 collision_torque_lower_acc = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  Array7 collision_torque_upper_acc = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  Array7 collision_torque_lower_nom = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  Array7 collision_torque_upper_nom = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};

  Array6 collision_force_lower_acc = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  Array6 collision_force_upper_acc = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  Array6 collision_force_lower_nom = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  Array6 collision_force_upper_nom = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
};

struct LogData {
  double time;
  int phase;

  Vec3 p_EE;
  Vec3 p_d;
  Vec3 p_end;
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
  double post_contact_push;

  Vec7 tau_cmd;
};

struct DesiredMotion {
  Vec3 p_d;
  Vec3 pdot_d;
};
