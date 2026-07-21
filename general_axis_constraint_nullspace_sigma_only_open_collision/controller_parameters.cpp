#include "controller_parameters.h"

std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

std::string removeSpaces(std::string value) {
  value.erase(
      std::remove_if(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c);
      }),
      value.end());
  return value;
}

double parseDoubleValue(const std::string& input) {
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

NullspaceMode parseNullspaceMode(const std::string& input, NullspaceMode fallback) {
  std::string value = removeSpaces(input);
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (value == "0" || value == "off" || value == "none" || value == "no") {
    return NullspaceMode::kOff;
  }
  if (value == "1" || value == "posture" || value == "tau_nullspace" ||
      value == "tau-nullspace" || value == "nullspace") {
    return NullspaceMode::kPostureOnly;
  }
  if (value == "2" || value == "sigma" || value == "tau_sigma" ||
      value == "tau-sigma") {
    return NullspaceMode::kSigmaOnly;
  }
  if (value == "3" || value == "both" || value == "combined" ||
      value == "postureandsigma" || value == "posture+sigma") {
    return NullspaceMode::kPostureAndSigma;
  }

  return fallback;
}

ContactSearchMode parseContactSearchMode(const std::string& input, ContactSearchMode fallback) {
  std::string value = removeSpaces(input);
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (value == "0" || value == "force" || value == "force_threshold" ||
      value == "external_force") {
    return ContactSearchMode::kForce;
  }
  if (value == "1" || value == "pre_surface" || value == "presurface" ||
      value == "surface_clearance" || value == "geometry" || value == "geometric") {
    return ContactSearchMode::kPreSurface;
  }

  return fallback;
}

Parameters readParameters(const std::string& filename) {
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
  auto getNullspaceMode = [&](const std::string& key, NullspaceMode def) {
    return values.count(key) ? parseNullspaceMode(values[key], def) : def;
  };
  auto getContactSearchMode = [&](const std::string& key, ContactSearchMode def) {
    return values.count(key) ? parseContactSearchMode(values[key], def) : def;
  };
  auto getDoubleAlias = [&](const std::string& new_key, const std::string& old_key, double def) {
    return getDouble(new_key, getDouble(old_key, def));
  };
  auto getBoolAlias = [&](const std::string& new_key, const std::string& old_key, bool def) {
    return getBool(new_key, getBool(old_key, def));
  };
  auto getMat6 = [&](const std::string& prefix, const Mat6x6& def) {
    Mat6x6 matrix = def;
    char key[64];
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        snprintf(key, sizeof(key), "%s_%d%d", prefix.c_str(), r, c);
        matrix(r, c) = getDouble(key, matrix(r, c));
      }
    }
    return matrix;
  };

  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);
  p.log_every_n_cycles =
      std::max(1, static_cast<int>(getDouble("log_every_n_cycles", p.log_every_n_cycles)));
  p.max_log_rows =
      std::max(0, static_cast<int>(getDouble("max_log_rows", p.max_log_rows)));
  p.debug_period = getDouble("debug_period", p.debug_period);
  p.print_impedance_debug = getBool("print_impedance_debug", p.print_impedance_debug);
  p.open_gripper_before_run = getBool("open_gripper_before_run", p.open_gripper_before_run);
  p.require_gripper_open = getBool("require_gripper_open", p.require_gripper_open);
  p.gripper_open_width = getDouble("gripper_open_width", p.gripper_open_width);
  p.gripper_open_speed = getDouble("gripper_open_speed", p.gripper_open_speed);
  p.gripper_grasp_on_tool = getBool("gripper_grasp_on_tool", p.gripper_grasp_on_tool);
  p.gripper_grasp_width = getDouble("gripper_grasp_width", p.gripper_grasp_width);
  p.gripper_grasp_speed = getDouble("gripper_grasp_speed", p.gripper_grasp_speed);
  p.gripper_grasp_force = getDouble("gripper_grasp_force", p.gripper_grasp_force);
  p.gripper_grasp_epsilon_inner =
      getDouble("gripper_grasp_epsilon_inner", p.gripper_grasp_epsilon_inner);
  p.gripper_grasp_epsilon_outer =
      getDouble("gripper_grasp_epsilon_outer", p.gripper_grasp_epsilon_outer);

  p.hold_mode = getBool("hold_mode", p.hold_mode);
  p.hold_mode = getBool("use_current_pose", p.hold_mode);
  p.hold_Kp = getDouble("hold_Kp", p.hold_Kp);
  p.hold_Dp = getDouble("hold_Dp", p.hold_Dp);
  p.hold_auto_damping = getBool("hold_auto_damping", p.hold_auto_damping);
  p.hold_auto_damping_factor =
      getDouble("hold_auto_damping_factor", p.hold_auto_damping_factor);
  p.use_manual_guidance_start =
      getBool("use_manual_guidance_start", p.use_manual_guidance_start);
  p.manual_guidance_damping =
      getDouble("manual_guidance_damping", p.manual_guidance_damping);
  p.constraint_enabled = getBool("constraint_enabled", p.constraint_enabled);
  p.use_start_as_surface_point = getBool("use_start_as_surface_point", p.use_start_as_surface_point);
  p.surface_point(0) = getDouble("surface_point_x", p.surface_point(0));
  p.surface_point(1) = getDouble("surface_point_y", p.surface_point(1));
  p.surface_point(2) = getDouble("surface_point_z", p.surface_point(2));
  p.alignment_target_normal(0) =
      getDoubleAlias("alignment_target_normal_x", "surface_normal_x", p.alignment_target_normal(0));
  p.alignment_target_normal(1) =
      getDoubleAlias("alignment_target_normal_y", "surface_normal_y", p.alignment_target_normal(1));
  p.alignment_target_normal(2) =
      getDoubleAlias("alignment_target_normal_z", "surface_normal_z", p.alignment_target_normal(2));
  p.use_alignment_target_tilt_angle =
      getBoolAlias("use_alignment_target_tilt_angle",
                   "use_surface_tilt_angle",
                   p.use_alignment_target_tilt_angle);
  p.alignment_target_tilt_angle_deg =
      getDoubleAlias("alignment_target_tilt_angle_deg",
                     "surface_tilt_angle_deg",
                     p.alignment_target_tilt_angle_deg);
  p.alignment_target_tilt_angle_y_deg =
      getDouble("alignment_target_tilt_angle_y_deg",
                p.alignment_target_tilt_angle_y_deg);
  p.derive_tilt_angles_from_plane_normal =
      getBool("derive_tilt_angles_from_plane_normal",
              p.derive_tilt_angles_from_plane_normal);
  if (p.derive_tilt_angles_from_plane_normal) {
    // Invert n = [sin(b)cos(a), -sin(a), cos(b)cos(a)] to get the two tilt
    // angles from the virtual-plane normal vector the user provided above:
    //   a = -asin(n_y),  b = atan2(n_x, n_z).
    // Then rebuild the normal from those angles so everything downstream stays
    // consistent (the round trip returns the same unit normal).
    Vec3 n = p.alignment_target_normal;
    n = (n.norm() > 1e-9) ? n.normalized() : Vec3(0.0, 0.0, 1.0);
    const double ny = std::max(-1.0, std::min(1.0, n(1)));
    const double ax = std::asin(-ny);
    const double ay = std::atan2(n(0), n(2));
    p.alignment_target_tilt_angle_deg = ax * 180.0 / M_PI;
    p.alignment_target_tilt_angle_y_deg = ay * 180.0 / M_PI;
    p.alignment_target_normal = Vec3(std::sin(ay) * std::cos(ax),
                                     -std::sin(ax),
                                     std::cos(ay) * std::cos(ax));
    p.alignment_target_normal.normalize();
  } else if (p.use_alignment_target_tilt_angle) {
    // n = R_y(b) * R_x(a) * [0,0,1]
    //   = [sin(b)cos(a), -sin(a), cos(b)cos(a)].
    // a = tilt about base x (legacy single-angle meaning, b=0), b = tilt about base y.
    const double ax = p.alignment_target_tilt_angle_deg * M_PI / 180.0;
    const double ay = p.alignment_target_tilt_angle_y_deg * M_PI / 180.0;
    p.alignment_target_normal = Vec3(std::sin(ay) * std::cos(ax),
                                     -std::sin(ax),
                                     std::cos(ay) * std::cos(ax));
    p.alignment_target_normal.normalize();
  }
  p.alignment_target_tangent1(0) =
      getDouble("alignment_target_tangent1_x",
                getDoubleAlias("surface_tangent1_x",
                               "surface_tangent_hint_x",
                               p.alignment_target_tangent1(0)));
  p.alignment_target_tangent1(1) =
      getDouble("alignment_target_tangent1_y",
                getDoubleAlias("surface_tangent1_y",
                               "surface_tangent_hint_y",
                               p.alignment_target_tangent1(1)));
  p.alignment_target_tangent1(2) =
      getDouble("alignment_target_tangent1_z",
                getDoubleAlias("surface_tangent1_z",
                               "surface_tangent_hint_z",
                               p.alignment_target_tangent1(2)));
  p.tool_axis_ee(0) = getDouble("tool_axis_ee_x", p.tool_axis_ee(0));
  p.tool_axis_ee(1) = getDouble("tool_axis_ee_y", p.tool_axis_ee(1));
  p.tool_axis_ee(2) = getDouble("tool_axis_ee_z", p.tool_axis_ee(2));
  p.tool_axis_target_sign =
      getDouble("tool_axis_target_sign", p.tool_axis_target_sign);
  p.use_tool_contact_point_control =
      getBool("use_tool_contact_point_control", p.use_tool_contact_point_control);
  p.auto_select_tool_contact_edge =
      getBool("auto_select_tool_contact_edge", p.auto_select_tool_contact_edge);
  p.tool_contact_point_ee(0) =
      getDouble("tool_contact_point_ee_x", p.tool_contact_point_ee(0));
  p.tool_contact_point_ee(1) =
      getDouble("tool_contact_point_ee_y", p.tool_contact_point_ee(1));
  p.tool_contact_point_ee(2) =
      getDouble("tool_contact_point_ee_z", p.tool_contact_point_ee(2));
  p.align_orientation_to_surface_after_contact =
      getBool("align_orientation_to_surface_after_contact",
              p.align_orientation_to_surface_after_contact);
  p.orientation_test_only = getBool("orientation_test_only", p.orientation_test_only);
  p.orientation_test_extra_tilt_deg =
      getDouble("orientation_test_extra_tilt_deg", p.orientation_test_extra_tilt_deg);
  p.use_phase_sequence = getBool("use_phase_sequence", p.use_phase_sequence);
  p.use_orientation_phase = getBool("use_orientation_phase", p.use_orientation_phase);
  p.orient_phase_min_time = getDouble("orient_phase_min_time", p.orient_phase_min_time);
  p.orient_phase_error_threshold =
      getDouble("orient_phase_error_threshold", p.orient_phase_error_threshold);
  // Diag order is [tangent1, tangent2, normal] (matches R column order).
  p.approach_Kp_diag(0) = getDouble("approach_Kp_tangent1", p.approach_Kp_diag(0));
  p.approach_Kp_diag(1) = getDouble("approach_Kp_tangent2", p.approach_Kp_diag(1));
  p.approach_Kp_diag(2) = getDouble("approach_Kp_normal", p.approach_Kp_diag(2));
  p.approach_KR_diag(0) = getDouble("approach_KR_tangent1", p.approach_KR_diag(0));
  p.approach_KR_diag(1) = getDouble("approach_KR_tangent2", p.approach_KR_diag(1));
  p.approach_KR_diag(2) = getDouble("approach_KR_normal", p.approach_KR_diag(2));
  p.approach_Dp_diag(0) = getDouble("approach_Dp_tangent1", p.approach_Dp_diag(0));
  p.approach_Dp_diag(1) = getDouble("approach_Dp_tangent2", p.approach_Dp_diag(1));
  p.approach_Dp_diag(2) = getDouble("approach_Dp_normal", p.approach_Dp_diag(2));
  p.approach_DR_diag(0) = getDouble("approach_DR_tangent1", p.approach_DR_diag(0));
  p.approach_DR_diag(1) = getDouble("approach_DR_tangent2", p.approach_DR_diag(1));
  p.approach_DR_diag(2) = getDouble("approach_DR_normal", p.approach_DR_diag(2));
  p.approach_auto_damping = getBool("approach_auto_damping", p.approach_auto_damping);
  p.approach_auto_damping_factor =
      getDouble("approach_auto_damping_factor", p.approach_auto_damping_factor);
  p.post_contact_align_min_time =
      getDouble("post_contact_align_min_time", p.post_contact_align_min_time);
  p.post_contact_align_duration =
      getDouble("post_contact_align_duration", p.post_contact_align_duration);
  p.post_contact_best_axis_min_time =
      std::max(0.0, getDouble("post_contact_best_axis_min_time",
                              p.post_contact_best_axis_min_time));
  p.post_contact_best_axis_min_omega =
      std::max(0.0, getDouble("post_contact_best_axis_min_omega",
                              p.post_contact_best_axis_min_omega));
  p.post_contact_moment_threshold =
      getDoubleAlias("post_contact_moment_threshold",
                     "alignment_contact_moment_threshold",
                     p.post_contact_moment_threshold);
  p.post_contact_normal_push =
      getDouble("post_contact_normal_push", p.post_contact_normal_push);
  p.post_contact_push_speed =
      getDouble("post_contact_push_speed", p.post_contact_push_speed);
  p.post_contact_max_push =
      getDouble("post_contact_max_push", p.post_contact_max_push);
  p.post_contact_hold_after_align =
      getBool("post_contact_hold_after_align",
              p.post_contact_hold_after_align);
  p.post_contact_eval_method2_tcp_wrench =
      getBoolAlias("post_contact_eval_method2_tcp_wrench",
                   "post_contact_use_method2_tcp_wrench",
                   p.post_contact_eval_method2_tcp_wrench);
  p.post_contact_apply_method2_tcp_wrench =
      getBool("post_contact_apply_method2_tcp_wrench",
              p.post_contact_apply_method2_tcp_wrench);
  p.use_in_method2_k_d_diag =
      getBool("use_in_method2_k_d_diag", p.use_in_method2_k_d_diag);
  p.use_manual_method2_pole =
      getBool("use_manual_method2_pole", p.use_manual_method2_pole);
  p.manual_method2_pole_from_edge(0) =
      getDouble("manual_method2_pole_from_edge_x", p.manual_method2_pole_from_edge(0));
  p.manual_method2_pole_from_edge(1) =
      getDouble("manual_method2_pole_from_edge_y", p.manual_method2_pole_from_edge(1));
  p.manual_method2_pole_from_edge(2) =
      getDouble("manual_method2_pole_from_edge_z", p.manual_method2_pole_from_edge(2));
  p.method2_pole_freeze_at_contact =
      getBool("method2_pole_freeze_at_contact", p.method2_pole_freeze_at_contact);
  p.method2_tcp_wrench_saved =
      getBool("method2_tcp_wrench_saved", p.method2_tcp_wrench_saved);
  p.method2_K_tcp_base = getMat6("method2_K_tcp", p.method2_K_tcp_base);
  p.method2_D_tcp_base = getMat6("method2_D_tcp", p.method2_D_tcp_base);
  p.use_search_direction_surface_after_alignment =
      getBool("use_search_direction_surface_after_alignment",
              p.use_search_direction_surface_after_alignment);
  p.use_virtual_center_after_contact =
      getBool("use_virtual_center_after_contact", p.use_virtual_center_after_contact);
  p.vcr_offset = getDouble("vcr_offset", p.vcr_offset);
  p.print_gain_suggestion_diagnostics =
      getBoolAlias("print_gain_suggestion_diagnostics",
                   "suggest_gains_from_desired_axis",
                   p.print_gain_suggestion_diagnostics);
  p.print_method1_diagnostics =
      getBool("print_method1_diagnostics", p.print_method1_diagnostics);
  p.print_method3_diagnostics =
      getBool("print_method3_diagnostics", p.print_method3_diagnostics);
  p.last_best_axis_from_edge(0) =
      getDoubleAlias("last_best_axis_from_edge_x",
                     "desired_axis_from_edge_x",
                     p.last_best_axis_from_edge(0));
  p.last_best_axis_from_edge(1) =
      getDoubleAlias("last_best_axis_from_edge_y",
                     "desired_axis_from_edge_y",
                     p.last_best_axis_from_edge(1));
  p.last_best_axis_from_edge(2) =
      getDoubleAlias("last_best_axis_from_edge_z",
                     "desired_axis_from_edge_z",
                     p.last_best_axis_from_edge(2));
  p.last_best_axis_dir(0) =
      getDoubleAlias("last_best_axis_dir_x", "desired_axis_dir_x", p.last_best_axis_dir(0));
  p.last_best_axis_dir(1) =
      getDoubleAlias("last_best_axis_dir_y", "desired_axis_dir_y", p.last_best_axis_dir(1));
  p.last_best_axis_dir(2) =
      getDoubleAlias("last_best_axis_dir_z", "desired_axis_dir_z", p.last_best_axis_dir(2));
  p.last_best_axis_pitch =
      getDoubleAlias("last_best_axis_pitch", "desired_axis_pitch", p.last_best_axis_pitch);
  p.last_chasles_axis_from_edge(0) =
      getDoubleAlias("last_chasles_axis_from_edge_x",
                     "desired_axis_from_edge_x",
                     p.last_chasles_axis_from_edge(0));
  p.last_chasles_axis_from_edge(1) =
      getDoubleAlias("last_chasles_axis_from_edge_y",
                     "desired_axis_from_edge_y",
                     p.last_chasles_axis_from_edge(1));
  p.last_chasles_axis_from_edge(2) =
      getDoubleAlias("last_chasles_axis_from_edge_z",
                     "desired_axis_from_edge_z",
                     p.last_chasles_axis_from_edge(2));
  p.last_chasles_axis_dir(0) =
      getDoubleAlias("last_chasles_axis_dir_x", "desired_axis_dir_x", p.last_chasles_axis_dir(0));
  p.last_chasles_axis_dir(1) =
      getDoubleAlias("last_chasles_axis_dir_y", "desired_axis_dir_y", p.last_chasles_axis_dir(1));
  p.last_chasles_axis_dir(2) =
      getDoubleAlias("last_chasles_axis_dir_z", "desired_axis_dir_z", p.last_chasles_axis_dir(2));
  p.last_chasles_axis_pitch =
      getDoubleAlias("last_chasles_axis_pitch", "desired_axis_pitch", p.last_chasles_axis_pitch);
  p.suggested_gain_omega_ref =
      getDouble("suggested_gain_omega_ref", p.suggested_gain_omega_ref);
  p.suggested_gain_angle_ref =
      getDouble("suggested_gain_angle_ref", p.suggested_gain_angle_ref);
  p.suggested_gain_min = getDouble("suggested_gain_min", p.suggested_gain_min);
  p.suggested_gain_max = getDouble("suggested_gain_max", p.suggested_gain_max);

  // Diag order is [tangent1, tangent2, normal] (matches R column order).
  p.quasi_force_limit(0) = getDouble("quasi_force_limit_tangent1", p.quasi_force_limit(0));
  p.quasi_force_limit(1) = getDouble("quasi_force_limit_tangent2", p.quasi_force_limit(1));
  p.quasi_force_limit(2) = getDouble("quasi_force_limit_normal", p.quasi_force_limit(2));
  p.quasi_displacement_limit(0) =
      getDouble("quasi_displacement_limit_tangent1", p.quasi_displacement_limit(0));
  p.quasi_displacement_limit(1) =
      getDouble("quasi_displacement_limit_tangent2", p.quasi_displacement_limit(1));
  p.quasi_displacement_limit(2) =
      getDouble("quasi_displacement_limit_normal", p.quasi_displacement_limit(2));
  p.quasi_moment_limit(0) = getDouble("quasi_moment_limit_tangent1", p.quasi_moment_limit(0));
  p.quasi_moment_limit(1) = getDouble("quasi_moment_limit_tangent2", p.quasi_moment_limit(1));
  p.quasi_moment_limit(2) = getDouble("quasi_moment_limit_normal", p.quasi_moment_limit(2));
  p.quasi_angle_limit(0) = getDouble("quasi_angle_limit_tangent1", p.quasi_angle_limit(0));
  p.quasi_angle_limit(1) = getDouble("quasi_angle_limit_tangent2", p.quasi_angle_limit(1));
  p.quasi_angle_limit(2) = getDouble("quasi_angle_limit_normal", p.quasi_angle_limit(2));
  p.quasi_effective_mass(0) = getDouble("quasi_effective_mass_tangent1", p.quasi_effective_mass(0));
  p.quasi_effective_mass(1) = getDouble("quasi_effective_mass_tangent2", p.quasi_effective_mass(1));
  p.quasi_effective_mass(2) = getDouble("quasi_effective_mass_normal", p.quasi_effective_mass(2));
  p.quasi_effective_inertia(0) =
      getDouble("quasi_effective_inertia_tangent1", p.quasi_effective_inertia(0));
  p.quasi_effective_inertia(1) =
      getDouble("quasi_effective_inertia_tangent2", p.quasi_effective_inertia(1));
  p.quasi_effective_inertia(2) =
      getDouble("quasi_effective_inertia_normal", p.quasi_effective_inertia(2));
  p.quasi_damping_ratio = getDouble("quasi_damping_ratio", p.quasi_damping_ratio);
  p.effective_moment_fit_ridge =
      getDouble("effective_moment_fit_ridge", p.effective_moment_fit_ridge);

  p.use_contact_search = getBool("use_contact_search", p.use_contact_search);
  p.contact_search_mode =
      getContactSearchMode("contact_search_mode", p.contact_search_mode);
  p.contact_search_surface_clearance =
      getDouble("contact_search_surface_clearance", p.contact_search_surface_clearance);
  p.contact_search_use_alignment_target_normal =
      getBoolAlias("contact_search_use_alignment_target_normal",
                   "contact_search_use_surface_normal",
                   p.contact_search_use_alignment_target_normal);
  p.contact_search_direction(0) = getDouble("contact_search_direction_x", p.contact_search_direction(0));
  p.contact_search_direction(1) = getDouble("contact_search_direction_y", p.contact_search_direction(1));
  p.contact_search_direction(2) = getDouble("contact_search_direction_z", p.contact_search_direction(2));
  p.contact_search_speed = getDouble("contact_search_speed", p.contact_search_speed);
  p.contact_search_max_distance =
      getDouble("contact_search_max_distance", p.contact_search_max_distance);
  p.contact_search_min_distance =
      getDouble("contact_search_min_distance", p.contact_search_min_distance);
  p.contact_search_first_touch_min_distance =
      getDouble("contact_search_first_touch_min_distance",
                p.contact_search_first_touch_min_distance);
  p.contact_search_use_directional_force =
      getBool("contact_search_use_directional_force", p.contact_search_use_directional_force);
  p.contact_search_confirm_time =
      std::max(0.0, getDouble("contact_search_confirm_time", p.contact_search_confirm_time));
  p.contact_force_threshold = getDouble("contact_force_threshold", p.contact_force_threshold);
  p.detect_contact_during_alignment =
      getBool("detect_contact_during_alignment", p.detect_contact_during_alignment);
  p.alignment_contact_force_threshold =
      getDouble("alignment_contact_force_threshold", p.alignment_contact_force_threshold);
  p.post_contact_Kp_diag(0) = getDouble("post_contact_Kp_x", p.post_contact_Kp_diag(0));
  p.post_contact_Kp_diag(1) = getDouble("post_contact_Kp_y", p.post_contact_Kp_diag(1));
  p.post_contact_Kp_diag(2) = getDouble("post_contact_Kp_z", p.post_contact_Kp_diag(2));
  p.post_contact_pre_surface_Kp_z =
      getDouble("post_contact_pre_surface_Kp_z", p.post_contact_pre_surface_Kp_z);
  p.post_contact_Dp_diag(0) = getDouble("post_contact_Dp_x", p.post_contact_Dp_diag(0));
  p.post_contact_Dp_diag(1) = getDouble("post_contact_Dp_y", p.post_contact_Dp_diag(1));
  p.post_contact_Dp_diag(2) = getDouble("post_contact_Dp_z", p.post_contact_Dp_diag(2));
  // Diag order [tangent1, tangent2, normal] (matches R column order).
  p.post_contact_KR_diag(0) = getDouble("post_contact_KR_tangent1", p.post_contact_KR_diag(0));
  p.post_contact_KR_diag(1) = getDouble("post_contact_KR_tangent2", p.post_contact_KR_diag(1));
  p.post_contact_KR_diag(2) = getDouble("post_contact_KR_normal", p.post_contact_KR_diag(2));
  p.post_contact_DR_diag(0) = getDouble("post_contact_DR_tangent1", p.post_contact_DR_diag(0));
  p.post_contact_DR_diag(1) = getDouble("post_contact_DR_tangent2", p.post_contact_DR_diag(1));
  p.post_contact_DR_diag(2) = getDouble("post_contact_DR_normal", p.post_contact_DR_diag(2));
  p.post_contact_auto_damping =
      getBool("post_contact_auto_damping", p.post_contact_auto_damping);
  p.post_contact_auto_damping_factor =
      getDouble("post_contact_auto_damping_factor", p.post_contact_auto_damping_factor);

  p.post_contact_grind_mode = getBool("post_contact_grind_mode", p.post_contact_grind_mode);
  p.print_grind_debug = getBool("print_grind_debug", p.print_grind_debug);
  p.grind_axis = static_cast<int>(getDouble("grind_axis", p.grind_axis));
  p.grind_amplitude_m = getDouble("grind_amplitude_m", p.grind_amplitude_m);
  p.grind_frequency_hz = getDouble("grind_frequency_hz", p.grind_frequency_hz);

  p.constrain_rotation_about_alignment_normal =
      getBool("constrain_rotation_about_alignment_normal",
              getBoolAlias("constrain_rotation_about_surface_normal",
                           "fix_R_x",
                           p.constrain_rotation_about_alignment_normal));
  p.constrain_rotation_about_alignment_tangent1 =
      getBool("constrain_rotation_about_alignment_tangent1",
              getBoolAlias("constrain_rotation_about_surface_tangent1",
                           "fix_R_y",
                           p.constrain_rotation_about_alignment_tangent1));
  p.constrain_rotation_about_alignment_tangent2 =
      getBool("constrain_rotation_about_alignment_tangent2",
              getBoolAlias("constrain_rotation_about_surface_tangent2",
                           "fix_R_z",
                           p.constrain_rotation_about_alignment_tangent2));

  p.use_nullspace_optimization = getBool("use_nullspace_optimization", p.use_nullspace_optimization);
  p.nullspace_mode = p.use_nullspace_optimization
      ? NullspaceMode::kPostureAndSigma
      : NullspaceMode::kOff;
  p.nullspace_mode = getNullspaceMode("nullspace_mode", p.nullspace_mode);
  p.use_nullspace_optimization = (p.nullspace_mode != NullspaceMode::kOff);
  p.nullspace_k_start = getDouble("nullspace_k_start", p.nullspace_k_start);
  p.nullspace_damping = getDouble("nullspace_damping", p.nullspace_damping);
  p.nullspace_k_sigma = getDouble("nullspace_k_sigma", p.nullspace_k_sigma);
  p.nullspace_alpha = getDouble("nullspace_alpha", p.nullspace_alpha);

  p.delta_p(0) = getDouble("delta_p_x", p.delta_p(0));
  p.delta_p(1) = getDouble("delta_p_y", p.delta_p(1));
  p.delta_p(2) = getDouble("delta_p_z", p.delta_p(2));
  p.trajectory_duration = getDouble("trajectory_duration", p.trajectory_duration);

  // Diag order [tangent1, tangent2, normal] (matches R column order).
  p.Kp_diag(0) = getDoubleAlias("Kp_tangent1", "Kp_y", p.Kp_diag(0));
  p.Kp_diag(1) = getDoubleAlias("Kp_tangent2", "Kp_z", p.Kp_diag(1));
  p.Kp_diag(2) = getDoubleAlias("Kp_normal", "Kp_x", p.Kp_diag(2));

  p.Dp_diag(0) = getDoubleAlias("Dp_tangent1", "Dp_y", p.Dp_diag(0));
  p.Dp_diag(1) = getDoubleAlias("Dp_tangent2", "Dp_z", p.Dp_diag(1));
  p.Dp_diag(2) = getDoubleAlias("Dp_normal", "Dp_x", p.Dp_diag(2));

  p.KR_diag(0) = getDoubleAlias("KR_tangent1", "KR_y", p.KR_diag(0));
  p.KR_diag(1) = getDoubleAlias("KR_tangent2", "KR_z", p.KR_diag(1));
  p.KR_diag(2) = getDoubleAlias("KR_normal", "KR_x", p.KR_diag(2));

  p.DR_diag(0) = getDoubleAlias("DR_tangent1", "DR_y", p.DR_diag(0));
  p.DR_diag(1) = getDoubleAlias("DR_tangent2", "DR_z", p.DR_diag(1));
  p.DR_diag(2) = getDoubleAlias("DR_normal", "DR_x", p.DR_diag(2));

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
  } else if (p.q_init_case == "tilted_close") {
    p.q_init[0] = getDouble("q_init_tilted_close_1", p.q_init[0]);
    p.q_init[1] = getDouble("q_init_tilted_close_2", p.q_init[1]);
    p.q_init[2] = getDouble("q_init_tilted_close_3", p.q_init[2]);
    p.q_init[3] = getDouble("q_init_tilted_close_4", p.q_init[3]);
    p.q_init[4] = getDouble("q_init_tilted_close_5", p.q_init[4]);
    p.q_init[5] = getDouble("q_init_tilted_close_6", p.q_init[5]);
    p.q_init[6] = getDouble("q_init_tilted_close_7", p.q_init[6]);
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
