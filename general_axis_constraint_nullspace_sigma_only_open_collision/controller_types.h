#pragma once

#include "controller_common.h"

struct Parameters {
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 10.0;
  std::string csv_file_name = "general_axis_constraint_nullspace_sigma_only_open_collision_log.csv";

  bool hold_mode = true;

  bool constraint_enabled = true;
  bool use_start_as_surface_point = true;
  Vec3 surface_point = Vec3(0.0, 0.0, 0.0);
  Vec3 surface_normal = Vec3(1.0, 0.0, 0.0);
  Vec3 surface_tangent_hint = Vec3(0.0, 1.0, 0.0);
  bool align_orientation_to_surface_after_contact = false;
  bool orientation_test_only = false;
  double orientation_test_extra_tilt_deg = 0.0;
  bool use_phase_sequence = true;
  double orient_phase_min_time = 0.5;
  double orient_phase_error_threshold = 0.03;
  double post_contact_align_duration = 3.0;
  bool use_virtual_center_after_contact = false;
  double vcr_offset = 0.0;

  bool use_contact_search = false;
  Vec3 contact_search_direction = Vec3(0.0, 0.0, -1.0);
  double contact_search_speed = 0.005;
  double contact_search_max_distance = 0.02;
  double contact_search_min_distance = 0.0;
  double contact_force_threshold = 5.0;
  Vec3 contact_search_Kp_diag = Vec3(150.0, 150.0, 150.0);
  Vec3 contact_search_Dp_diag = Vec3(25.0, 25.0, 25.0);
  Vec3 contact_search_KR_diag = Vec3(0.5, 0.5, 0.5);
  Vec3 contact_search_DR_diag = Vec3(1.0, 1.0, 1.0);

  bool fix_R_x = true;
  bool fix_R_y = true;
  bool fix_R_z = true;

  bool use_nullspace_optimization = true;
  double nullspace_k_start = 1.0;
  double nullspace_damping = 1.0;
  double nullspace_k_sigma = 0.5;
  double nullspace_alpha = 0.01;
  double nullspace_tau_max = 2.0;

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

  Vec3 p_EE;
  Vec3 p_d;
  Vec3 p_end;

  Vec3 e_p;
  Vec3 e_R;

  Vec3 pdot;
  Vec3 pdot_d;
  Vec3 omega;

  Vec3 f;
  Vec3 m;

  Vec7 tau_cmd;
};

struct DesiredMotion {
  Vec3 p_d;
  Vec3 pdot_d;
};
