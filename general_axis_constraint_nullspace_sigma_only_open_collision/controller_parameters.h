#pragma once

#include "controller_types.h"

inline std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

inline std::string removeSpaces(std::string value) {
  value.erase(
      std::remove_if(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c);
      }),
      value.end());
  return value;
}

inline double parseDoubleValue(const std::string& input) {
  std::string value = removeSpaces(input);

  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  const std::string pi = "pi";
  const std::size_t pi_pos = value.find(pi);
  if (pi_pos == std::string::npos) {
    return std::stod(value);
  }

  double sign = 1.0;
  if (!value.empty() && value[0] == '-') {
    sign = -1.0;
    value = value.substr(1);
  } else if (!value.empty() && value[0] == '+') {
    value = value.substr(1);
  }

  double numerator = 1.0;
  if (value.find("*pi") != std::string::npos) {
    numerator = std::stod(value.substr(0, value.find("*pi")));
  } else if (value.find(pi) != 0) {
    numerator = std::stod(value.substr(0, value.find(pi)));
  }

  double denominator = 1.0;
  const std::size_t slash_pos = value.find('/');
  if (slash_pos != std::string::npos) {
    denominator = std::stod(value.substr(slash_pos + 1));
  }

  return sign * numerator * M_PI / denominator;
}

inline Parameters readParameters(const std::string& filename) {
  Parameters p;
  std::ifstream file(filename);

  if (!file.is_open()) {
    printf("Could not open %s. Using defaults.\n", filename.c_str());
    return p;
  }

  std::map<std::string, std::string> values;
  std::string line;

  while (std::getline(file, line)) {
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    const auto eq_pos = line.find('=');
    if (eq_pos == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, eq_pos));
    const std::string value = trim(line.substr(eq_pos + 1));

    if (!key.empty() && !value.empty()) {
      values[key] = value;
    }
  }

  auto getString = [&](const std::string& key, const std::string& def) {
    return values.count(key) ? values[key] : def;
  };
  auto getDouble = [&](const std::string& key, double def) {
    return values.count(key) ? parseDoubleValue(values[key]) : def;
  };
  auto getBool = [&](const std::string& key, bool def) {
    return values.count(key) ? (std::stoi(values[key]) != 0) : def;
  };
  auto getDoubleAlias = [&](const std::string& new_key, const std::string& old_key, double def) {
    return getDouble(new_key, getDouble(old_key, def));
  };
  auto getBoolAlias = [&](const std::string& new_key, const std::string& old_key, bool def) {
    return getBool(new_key, getBool(old_key, def));
  };

  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);

  p.hold_mode = getBool("hold_mode", p.hold_mode);
  p.hold_mode = getBool("use_current_pose", p.hold_mode);
  p.constraint_enabled = getBool("constraint_enabled", p.constraint_enabled);
  p.use_start_as_surface_point = getBool("use_start_as_surface_point", p.use_start_as_surface_point);
  p.surface_point(0) = getDouble("surface_point_x", p.surface_point(0));
  p.surface_point(1) = getDouble("surface_point_y", p.surface_point(1));
  p.surface_point(2) = getDouble("surface_point_z", p.surface_point(2));
  p.surface_normal(0) = getDouble("surface_normal_x", p.surface_normal(0));
  p.surface_normal(1) = getDouble("surface_normal_y", p.surface_normal(1));
  p.surface_normal(2) = getDouble("surface_normal_z", p.surface_normal(2));
  p.surface_tangent1(0) =
      getDoubleAlias("surface_tangent1_x", "surface_tangent_hint_x", p.surface_tangent1(0));
  p.surface_tangent1(1) =
      getDoubleAlias("surface_tangent1_y", "surface_tangent_hint_y", p.surface_tangent1(1));
  p.surface_tangent1(2) =
      getDoubleAlias("surface_tangent1_z", "surface_tangent_hint_z", p.surface_tangent1(2));
  p.tool_axis_ee(0) = getDouble("tool_axis_ee_x", p.tool_axis_ee(0));
  p.tool_axis_ee(1) = getDouble("tool_axis_ee_y", p.tool_axis_ee(1));
  p.tool_axis_ee(2) = getDouble("tool_axis_ee_z", p.tool_axis_ee(2));
  p.tool_axis_target_sign =
      getDouble("tool_axis_target_sign", p.tool_axis_target_sign);
  p.align_orientation_to_surface_after_contact =
      getBool("align_orientation_to_surface_after_contact",
              p.align_orientation_to_surface_after_contact);
  p.orientation_test_only = getBool("orientation_test_only", p.orientation_test_only);
  p.orientation_test_extra_tilt_deg =
      getDouble("orientation_test_extra_tilt_deg", p.orientation_test_extra_tilt_deg);
  p.use_phase_sequence = getBool("use_phase_sequence", p.use_phase_sequence);
  p.orient_phase_min_time = getDouble("orient_phase_min_time", p.orient_phase_min_time);
  p.orient_phase_error_threshold =
      getDouble("orient_phase_error_threshold", p.orient_phase_error_threshold);
  p.post_contact_align_duration =
      getDouble("post_contact_align_duration", p.post_contact_align_duration);
  p.use_virtual_center_after_contact =
      getBool("use_virtual_center_after_contact", p.use_virtual_center_after_contact);
  p.vcr_offset = getDouble("vcr_offset", p.vcr_offset);

  p.use_contact_search = getBool("use_contact_search", p.use_contact_search);
  p.contact_search_direction(0) = getDouble("contact_search_direction_x", p.contact_search_direction(0));
  p.contact_search_direction(1) = getDouble("contact_search_direction_y", p.contact_search_direction(1));
  p.contact_search_direction(2) = getDouble("contact_search_direction_z", p.contact_search_direction(2));
  p.contact_search_speed = getDouble("contact_search_speed", p.contact_search_speed);
  p.contact_search_max_distance =
      getDouble("contact_search_max_distance", p.contact_search_max_distance);
  p.contact_search_min_distance =
      getDouble("contact_search_min_distance", p.contact_search_min_distance);
  p.contact_force_threshold = getDouble("contact_force_threshold", p.contact_force_threshold);
  p.contact_moment_threshold =
      getDouble("contact_moment_threshold", p.contact_moment_threshold);
  p.contact_require_force_and_moment =
      getBool("contact_require_force_and_moment", p.contact_require_force_and_moment);
  p.detect_contact_during_alignment =
      getBool("detect_contact_during_alignment", p.detect_contact_during_alignment);
  p.alignment_contact_force_threshold =
      getDouble("alignment_contact_force_threshold", p.alignment_contact_force_threshold);
  p.alignment_contact_moment_threshold =
      getDouble("alignment_contact_moment_threshold", p.alignment_contact_moment_threshold);
  p.alignment_contact_require_force_and_moment =
      getBool("alignment_contact_require_force_and_moment",
              p.alignment_contact_require_force_and_moment);
  p.contact_search_Kp_diag(0) = getDouble("contact_search_Kp_x", p.contact_search_Kp_diag(0));
  p.contact_search_Kp_diag(1) = getDouble("contact_search_Kp_y", p.contact_search_Kp_diag(1));
  p.contact_search_Kp_diag(2) = getDouble("contact_search_Kp_z", p.contact_search_Kp_diag(2));
  p.contact_search_Dp_diag(0) = getDouble("contact_search_Dp_x", p.contact_search_Dp_diag(0));
  p.contact_search_Dp_diag(1) = getDouble("contact_search_Dp_y", p.contact_search_Dp_diag(1));
  p.contact_search_Dp_diag(2) = getDouble("contact_search_Dp_z", p.contact_search_Dp_diag(2));
  p.contact_search_KR_diag(0) = getDouble("contact_search_KR_x", p.contact_search_KR_diag(0));
  p.contact_search_KR_diag(1) = getDouble("contact_search_KR_y", p.contact_search_KR_diag(1));
  p.contact_search_KR_diag(2) = getDouble("contact_search_KR_z", p.contact_search_KR_diag(2));
  p.contact_search_DR_diag(0) = getDouble("contact_search_DR_x", p.contact_search_DR_diag(0));
  p.contact_search_DR_diag(1) = getDouble("contact_search_DR_y", p.contact_search_DR_diag(1));
  p.contact_search_DR_diag(2) = getDouble("contact_search_DR_z", p.contact_search_DR_diag(2));

  p.constrain_rotation_about_surface_normal =
      getBoolAlias("constrain_rotation_about_surface_normal",
                   "fix_R_x",
                   p.constrain_rotation_about_surface_normal);
  p.constrain_rotation_about_surface_tangent1 =
      getBoolAlias("constrain_rotation_about_surface_tangent1",
                   "fix_R_y",
                   p.constrain_rotation_about_surface_tangent1);
  p.constrain_rotation_about_surface_tangent2 =
      getBoolAlias("constrain_rotation_about_surface_tangent2",
                   "fix_R_z",
                   p.constrain_rotation_about_surface_tangent2);

  p.use_nullspace_optimization = getBool("use_nullspace_optimization", p.use_nullspace_optimization);
  p.nullspace_k_start = getDouble("nullspace_k_start", p.nullspace_k_start);
  p.nullspace_damping = getDouble("nullspace_damping", p.nullspace_damping);
  p.nullspace_k_sigma = getDouble("nullspace_k_sigma", p.nullspace_k_sigma);
  p.nullspace_alpha = getDouble("nullspace_alpha", p.nullspace_alpha);
  p.nullspace_tau_max = getDouble("nullspace_tau_max", p.nullspace_tau_max);

  p.delta_p(0) = getDouble("delta_p_x", p.delta_p(0));
  p.delta_p(1) = getDouble("delta_p_y", p.delta_p(1));
  p.delta_p(2) = getDouble("delta_p_z", p.delta_p(2));
  p.trajectory_duration = getDouble("trajectory_duration", p.trajectory_duration);

  p.Kp_diag(0) = getDoubleAlias("Kp_normal", "Kp_x", p.Kp_diag(0));
  p.Kp_diag(1) = getDoubleAlias("Kp_tangent1", "Kp_y", p.Kp_diag(1));
  p.Kp_diag(2) = getDoubleAlias("Kp_tangent2", "Kp_z", p.Kp_diag(2));

  p.Dp_diag(0) = getDoubleAlias("Dp_normal", "Dp_x", p.Dp_diag(0));
  p.Dp_diag(1) = getDoubleAlias("Dp_tangent1", "Dp_y", p.Dp_diag(1));
  p.Dp_diag(2) = getDoubleAlias("Dp_tangent2", "Dp_z", p.Dp_diag(2));

  p.KR_diag(0) = getDoubleAlias("KR_normal", "KR_x", p.KR_diag(0));
  p.KR_diag(1) = getDoubleAlias("KR_tangent1", "KR_y", p.KR_diag(1));
  p.KR_diag(2) = getDoubleAlias("KR_tangent2", "KR_z", p.KR_diag(2));

  p.DR_diag(0) = getDoubleAlias("DR_normal", "DR_x", p.DR_diag(0));
  p.DR_diag(1) = getDoubleAlias("DR_tangent1", "DR_y", p.DR_diag(1));
  p.DR_diag(2) = getDoubleAlias("DR_tangent2", "DR_z", p.DR_diag(2));

  p.collision_torque_acc = getDouble("collision_torque_acc", p.collision_torque_acc);
  p.collision_torque_nom = getDouble("collision_torque_nom", p.collision_torque_nom);
  p.collision_force_acc = getDouble("collision_force_acc", p.collision_force_acc);
  p.collision_force_nom = getDouble("collision_force_nom", p.collision_force_nom);

  p.q_init_case = getString("q_init_case", p.q_init_case);
  if (p.q_init_case == "horizontal_table_search") {
    p.q_init[0] = getDouble("q_init_table_1", p.q_init[0]);
    p.q_init[1] = getDouble("q_init_table_2", p.q_init[1]);
    p.q_init[2] = getDouble("q_init_table_3", p.q_init[2]);
    p.q_init[3] = getDouble("q_init_table_4", p.q_init[3]);
    p.q_init[4] = getDouble("q_init_table_5", p.q_init[4]);
    p.q_init[5] = getDouble("q_init_table_6", p.q_init[5]);
    p.q_init[6] = getDouble("q_init_table_7", p.q_init[6]);
  } else if (p.q_init_case == "tilted_tool") {
    p.q_init[0] = getDouble("q_init_tilted_1", p.q_init[0]);
    p.q_init[1] = getDouble("q_init_tilted_2", p.q_init[1]);
    p.q_init[2] = getDouble("q_init_tilted_3", p.q_init[2]);
    p.q_init[3] = getDouble("q_init_tilted_4", p.q_init[3]);
    p.q_init[4] = getDouble("q_init_tilted_5", p.q_init[4]);
    p.q_init[5] = getDouble("q_init_tilted_6", p.q_init[5]);
    p.q_init[6] = getDouble("q_init_tilted_7", p.q_init[6]);
  } else {
    p.q_init[0] = getDouble("q_init_horizontal_1", p.q_init[0]);
    p.q_init[1] = getDouble("q_init_horizontal_2", p.q_init[1]);
    p.q_init[2] = getDouble("q_init_horizontal_3", p.q_init[2]);
    p.q_init[3] = getDouble("q_init_horizontal_4", p.q_init[3]);
    p.q_init[4] = getDouble("q_init_horizontal_5", p.q_init[4]);
    p.q_init[5] = getDouble("q_init_horizontal_6", p.q_init[5]);
    p.q_init[6] = getDouble("q_init_horizontal_7", p.q_init[6]);
  }

  p.q_init[0] = getDouble("q_goal_1", p.q_init[0]);
  p.q_init[1] = getDouble("q_goal_2", p.q_init[1]);
  p.q_init[2] = getDouble("q_goal_3", p.q_init[2]);
  p.q_init[3] = getDouble("q_goal_4", p.q_init[3]);
  p.q_init[4] = getDouble("q_goal_5", p.q_init[4]);
  p.q_init[5] = getDouble("q_goal_6", p.q_init[5]);
  p.q_init[6] = getDouble("q_goal_7", p.q_init[6]);
  p.q_init[0] = getDouble("q_init_1", p.q_init[0]);
  p.q_init[1] = getDouble("q_init_2", p.q_init[1]);
  p.q_init[2] = getDouble("q_init_3", p.q_init[2]);
  p.q_init[3] = getDouble("q_init_4", p.q_init[3]);
  p.q_init[4] = getDouble("q_init_5", p.q_init[4]);
  p.q_init[5] = getDouble("q_init_6", p.q_init[5]);
  p.q_init[6] = getDouble("q_init_7", p.q_init[6]);

  p.use_custom_collision_behavior = getBool("use_custom_collision_behavior", p.use_custom_collision_behavior);

  const double collision_joint_torque = getDouble("collision_joint_torque_threshold", 30.0);
  for (int i = 0; i < 7; ++i) {
    p.collision_torque_lower_acc[i] = collision_joint_torque;
    p.collision_torque_upper_acc[i] = collision_joint_torque;
    p.collision_torque_lower_nom[i] = collision_joint_torque;
    p.collision_torque_upper_nom[i] = collision_joint_torque;
  }

  const double collision_cart_force = getDouble("collision_cartesian_force_threshold", 30.0);
  for (int i = 0; i < 3; ++i) {
    p.collision_force_lower_acc[i] = collision_cart_force;
    p.collision_force_upper_acc[i] = collision_cart_force;
    p.collision_force_lower_nom[i] = collision_cart_force;
    p.collision_force_upper_nom[i] = collision_cart_force;
  }

  const double collision_cart_moment = getDouble("collision_cartesian_moment_threshold", 20.0);
  for (int i = 3; i < 6; ++i) {
    p.collision_force_lower_acc[i] = collision_cart_moment;
    p.collision_force_upper_acc[i] = collision_cart_moment;
    p.collision_force_lower_nom[i] = collision_cart_moment;
    p.collision_force_upper_nom[i] = collision_cart_moment;
  }

  return p;
}
