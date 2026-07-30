// ====================================================================
// Parameter loading
// ====================================================================
// Reads the params/ topic files into one Parameters struct. 
// Values accept plain numbers and simple pi expressions.
#include "controller.h"

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

// Accepts plain numbers and simple pi expressions ("pi/2", "-3*pi/4").
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
      value == "nullspace" || value == "damping") {
    return NullspaceMode::kDampingOnly;
  }
  if (value == "2" || value == "sigma" || value == "tau_sigma") {
    return NullspaceMode::kSigmaOnly;
  }
  if (value == "3" || value == "both" || value == "combined") {
    return NullspaceMode::kDampingAndSigma;
  }

  return fallback;
}

std::vector<std::string> parameterFiles(const std::string& dir) {
  const std::string prefix =
      (dir.empty() || dir.back() == '/') ? dir : dir + "/";
  return {
      prefix + "Run_Settings.txt",
      prefix + "safety.txt",
      prefix + "Gripper_Action.txt",
      prefix + "Auto_Damping.txt",
      prefix + "Plane_Definition.txt",
      prefix + "Tool_Orientation.txt",
      prefix + "Tool_Geometry.txt",
      prefix + "Q_Init.txt",
      prefix + "Approach_Phase.txt",
      prefix + "Clearance_Gate.txt",
      prefix + "SetUp_Phase.txt",
      prefix + "Grind_Phase.txt",
      prefix + "Nullspace.txt",
      prefix + "hold.txt",
      prefix + "guidance.txt",
  };
}

Parameters readParameters(const std::vector<std::string>& filenames) {
  Parameters p;
  std::map<std::string, std::string> values;

  int files_opened = 0;
  for (const std::string& filename : filenames) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      printf("Could not open %s. Skipping.\n", filename.c_str());
      continue;
    }
    ++files_opened;

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
        values[key] = value;  // later files override earlier on duplicate keys
      }
    }
  }

  if (files_opened == 0) {
    printf("No parameter files could be opened. Using defaults.\n");
    return p;
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
  // Reads "<key>_x/_y/_z" into a base-frame vector.
  auto getVec3Xyz = [&](const std::string& key, const Vec3& def) {
    return Vec3(getDouble(key + "_x", def(0)),
                getDouble(key + "_y", def(1)),
                getDouble(key + "_z", def(2)));
  };
  // Reads "<key>_tangent1/_tangent2/_normal" into a task-frame diagonal,
  // stored [tangent1, tangent2, normal] like the alignment-frame columns.
  auto getVec3Task = [&](const std::string& key, const Vec3& def) {
    return Vec3(getDouble(key + "_tangent1", def(0)),
                getDouble(key + "_tangent2", def(1)),
                getDouble(key + "_normal", def(2)));
  };
  // ---- robot, logging, debug ----
  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);
  p.log_every_n_cycles =
      std::max(1, static_cast<int>(getDouble("log_every_n_cycles", p.log_every_n_cycles)));
  p.max_log_rows =
      std::max(0, static_cast<int>(getDouble("max_log_rows", p.max_log_rows)));
  p.sigma_debug_csv_file_name =
      getString("sigma_debug_csv_file_name", p.sigma_debug_csv_file_name);
  p.sigma_debug_log_period =
      std::max(0.001,
               getDouble("sigma_debug_log_period", p.sigma_debug_log_period));
  p.max_sigma_debug_rows =
      std::max(0, static_cast<int>(
                      getDouble("max_sigma_debug_rows",
                                p.max_sigma_debug_rows)));
  p.debug_period = getDouble("debug_period", p.debug_period);
  p.print_hold_debug = getBool("print_hold_debug", p.print_hold_debug);
  p.print_grind_debug = getBool("print_grind_debug", p.print_grind_debug);
  p.print_sigma_debug = getBool("print_sigma_debug", p.print_sigma_debug);
  p.print_coupled_diagnostics =
      getBool("print_coupled_diagnostics", p.print_coupled_diagnostics);

  // ---- gripper ----
  p.open_gripper_before_run = getBool("open_gripper_before_run", p.open_gripper_before_run);
  p.require_gripper_open = getBool("require_gripper_open", p.require_gripper_open);
  p.gripper_grasp_on_tool = getBool("gripper_grasp_on_tool", p.gripper_grasp_on_tool);
  p.gripper_open_width = getDouble("gripper_open_width", p.gripper_open_width);
  p.gripper_open_speed = getDouble("gripper_open_speed", p.gripper_open_speed);
  p.gripper_grasp_width = getDouble("gripper_grasp_width", p.gripper_grasp_width);
  p.gripper_grasp_speed = getDouble("gripper_grasp_speed", p.gripper_grasp_speed);
  p.gripper_grasp_force = getDouble("gripper_grasp_force", p.gripper_grasp_force);
  p.gripper_grasp_epsilon_inner =
      getDouble("gripper_grasp_epsilon_inner", p.gripper_grasp_epsilon_inner);
  p.gripper_grasp_epsilon_outer =
      getDouble("gripper_grasp_epsilon_outer", p.gripper_grasp_epsilon_outer);

  // ---- run mode ----
  p.use_approach_orient = getBool("use_approach_orient", p.use_approach_orient);
  p.use_manual_guidance_start =
      getBool("use_manual_guidance_start", p.use_manual_guidance_start);
  p.manual_guidance_damping =
      getDouble("manual_guidance_damping", p.manual_guidance_damping);

  // ---- hold ----
  p.hold_Kp_diag = getVec3Xyz("hold_Kp", p.hold_Kp_diag);
  p.hold_Dp_diag = getVec3Xyz("hold_Dp", p.hold_Dp_diag);
  p.hold_KR_diag = getVec3Xyz("hold_KR", p.hold_KR_diag);
  p.hold_DR_diag = getVec3Xyz("hold_DR", p.hold_DR_diag);
  p.hold_auto_damping = getBool("hold_auto_damping", p.hold_auto_damping);
  p.hold_auto_match_manual_damping =
      getBool("hold_auto_match_manual_damping",
              p.hold_auto_match_manual_damping);
  p.hold_auto_damping_factor =
      getDouble("hold_auto_damping_factor", p.hold_auto_damping_factor);

  // ---- surface plane and tool geometry ----
  p.use_start_as_surface_point =
      getBool("use_start_as_surface_point", p.use_start_as_surface_point);
  p.surface_point = getVec3Xyz("surface_point", p.surface_point);
  p.alignment_target_tilt_angle_deg =
      getDouble("alignment_target_tilt_angle_deg", p.alignment_target_tilt_angle_deg);
  p.alignment_target_tilt_angle_y_deg =
      getDouble("alignment_target_tilt_angle_y_deg", p.alignment_target_tilt_angle_y_deg);
  const double ax = p.alignment_target_tilt_angle_deg * M_PI / 180.0;
  const double ay = p.alignment_target_tilt_angle_y_deg * M_PI / 180.0;
  p.alignment_target_normal =
      Vec3(std::sin(ay) * std::cos(ax), -std::sin(ax), std::cos(ay) * std::cos(ax));
  p.alignment_target_normal.normalize();
  p.alignment_target_tangent1 =
      getVec3Xyz("alignment_target_tangent1", p.alignment_target_tangent1);
  p.tool_target_offset_tangent1_deg =
      getDouble("tool_target_offset_tangent1_deg",
                p.tool_target_offset_tangent1_deg);
  p.tool_target_offset_tangent2_deg =
      getDouble("tool_target_offset_tangent2_deg",
                p.tool_target_offset_tangent2_deg);
  p.tool_axis_ee = getVec3Xyz("tool_axis_ee", p.tool_axis_ee);
  p.tool_axis_target_sign = getDouble("tool_axis_target_sign", p.tool_axis_target_sign);
  p.use_tool_contact_point_control =
      getBool("use_tool_contact_point_control", p.use_tool_contact_point_control);
  p.auto_select_tool_contact_edge =
      getBool("auto_select_tool_contact_edge", p.auto_select_tool_contact_edge);
  p.tool_contact_face_center_ee =
      getVec3Xyz("tool_contact_face_center_ee", p.tool_contact_face_center_ee);
  p.tool_contact_half_width_ee =
      getVec3Xyz("tool_contact_half_width_ee", p.tool_contact_half_width_ee);
  p.tool_contact_half_length_ee =
      getVec3Xyz("tool_contact_half_length_ee", p.tool_contact_half_length_ee);
  p.tool_contact_feature_tie_tolerance =
      getDouble("tool_contact_feature_tie_tolerance",
                p.tool_contact_feature_tie_tolerance);
  p.constrain_rotation_about_alignment_normal =
      getBool("constrain_rotation_about_alignment_normal",
              p.constrain_rotation_about_alignment_normal);
  p.constrain_rotation_about_alignment_tangent1 =
      getBool("constrain_rotation_about_alignment_tangent1",
              p.constrain_rotation_about_alignment_tangent1);
  p.constrain_rotation_about_alignment_tangent2 =
      getBool("constrain_rotation_about_alignment_tangent2",
              p.constrain_rotation_about_alignment_tangent2);

  // ---- phase 1: approach ----
  p.approach_orient_min_time =
      getDouble("approach_orient_min_time", p.approach_orient_min_time);
  p.approach_orient_error_threshold =
      getDouble("approach_orient_error_threshold", p.approach_orient_error_threshold);
  p.approach_Kp_diag = getVec3Task("approach_Kp", p.approach_Kp_diag);
  p.approach_KR_diag = getVec3Task("approach_KR", p.approach_KR_diag);
  p.approach_Dp_diag = getVec3Task("approach_Dp", p.approach_Dp_diag);
  p.approach_DR_diag = getVec3Task("approach_DR", p.approach_DR_diag);
  p.approach_auto_damping = getBool("approach_auto_damping", p.approach_auto_damping);
  p.approach_auto_damping_factor =
      getDouble("approach_auto_damping_factor", p.approach_auto_damping_factor);
  p.descend_speed = getDouble("descend_speed", p.descend_speed);
  p.descend_max_distance = getDouble("descend_max_distance", p.descend_max_distance);
  p.descend_surface_clearance =
      getDouble("descend_surface_clearance", p.descend_surface_clearance);

  // ---- phase 2: set up ----
  p.setup_min_time = getDouble("setup_min_time", p.setup_min_time);
  p.setup_timeout = getDouble("setup_timeout", p.setup_timeout);
  p.setup_moment_threshold = getDouble("setup_moment_threshold", p.setup_moment_threshold);
  p.setup_push_speed = getDouble("setup_push_speed", p.setup_push_speed);
  p.setup_push_end = getDouble("setup_push_end", p.setup_push_end);
  p.setup_Kp_diag = getVec3Xyz("setup_Kp", p.setup_Kp_diag);
  p.setup_Dp_diag = getVec3Xyz("setup_Dp", p.setup_Dp_diag);
  p.setup_translation_surface_frame =
      getBool("setup_translation_surface_frame", p.setup_translation_surface_frame);
  p.setup_Kp_surface_diag =
      getVec3Task("setup_Kp_surface", p.setup_Kp_surface_diag);
  p.setup_Dp_surface_diag =
      getVec3Task("setup_Dp_surface", p.setup_Dp_surface_diag);
  p.setup_KR_diag = getVec3Task("setup_KR", p.setup_KR_diag);
  p.setup_DR_diag = getVec3Task("setup_DR", p.setup_DR_diag);
  p.setup_auto_damping = getBool("setup_auto_damping", p.setup_auto_damping);
  p.setup_auto_damping_factor =
      getDouble("setup_auto_damping_factor", p.setup_auto_damping_factor);

  // ---- phase 3: grind ----
  p.grind_sweep_enabled = getBool("grind_sweep_enabled", p.grind_sweep_enabled);
  p.grind_axis = static_cast<int>(getDouble("grind_axis", p.grind_axis));
  p.grind_amplitude_m = getDouble("grind_amplitude_m", p.grind_amplitude_m);
  p.grind_frequency_hz = getDouble("grind_frequency_hz", p.grind_frequency_hz);

  // ---- phase gates ----
  p.pause_before_set_up = getBool("pause_before_set_up", p.pause_before_set_up);
  p.pause_before_grind = getBool("pause_before_grind", p.pause_before_grind);
  p.pause_hold_Kp_diag = getVec3Xyz("pause_hold_Kp", p.pause_hold_Kp_diag);
  p.pause_hold_Dp_diag = getVec3Xyz("pause_hold_Dp", p.pause_hold_Dp_diag);
  p.pause_hold_KR_diag = getVec3Task("pause_hold_KR", p.pause_hold_KR_diag);
  p.pause_hold_DR_diag = getVec3Task("pause_hold_DR", p.pause_hold_DR_diag);
  p.pause_hold_auto_damping =
      getBool("pause_hold_auto_damping", p.pause_hold_auto_damping);

  // ---- coupled stiffness ----
  p.use_coupled_stiffness = getBool("use_coupled_stiffness", p.use_coupled_stiffness);
  p.coupled_use_block_diagonal =
      getBool("coupled_use_block_diagonal", p.coupled_use_block_diagonal);
  p.coupled_pole_manual = getBool("coupled_pole_manual", p.coupled_pole_manual);
  p.coupled_use_direct_rc_surface =
      getBool("coupled_use_direct_rc_surface", p.coupled_use_direct_rc_surface);
  p.coupled_rc_surface = getVec3Task("coupled_rc", p.coupled_rc_surface);
  p.coupled_pole_from_edge = getVec3Xyz("coupled_pole_from_edge", p.coupled_pole_from_edge);
  p.coupled_pole_freeze_at_contact =
      getBool("coupled_pole_freeze_at_contact", p.coupled_pole_freeze_at_contact);

  // ---- nullspace ----
  p.use_nullspace_optimization =
      getBool("use_nullspace_optimization", p.use_nullspace_optimization);
  p.nullspace_mode = p.use_nullspace_optimization
      ? NullspaceMode::kDampingAndSigma
      : NullspaceMode::kOff;
  if (values.count("nullspace_mode")) {
    p.nullspace_mode = parseNullspaceMode(values["nullspace_mode"], p.nullspace_mode);
  }
  p.use_nullspace_optimization = (p.nullspace_mode != NullspaceMode::kOff);
  p.nullspace_damping = getDouble("nullspace_damping", p.nullspace_damping);
  p.nullspace_k_sigma = getDouble("nullspace_k_sigma", p.nullspace_k_sigma);
  p.nullspace_alpha = getDouble("nullspace_alpha", p.nullspace_alpha);
  p.nullspace_sigma_deadband =
      getDouble("nullspace_sigma_deadband", p.nullspace_sigma_deadband);
  p.nullspace_svd_relative_tolerance =
      getDouble("nullspace_svd_relative_tolerance",
                p.nullspace_svd_relative_tolerance);

  p.auto_damping_max = getDouble("auto_damping_max", p.auto_damping_max);
  p.auto_damping_min_from_manual =
      getBool("auto_damping_min_from_manual", p.auto_damping_min_from_manual);
  p.print_auto_damping = getBool("print_auto_damping", p.print_auto_damping);

  // ---- start pose ----
  p.q_init_case = getString("q_init_case", p.q_init_case);
  const char* q_init_prefix = "q_init_horizontal";
  if (p.q_init_case == "horizontal_table_search") {
    q_init_prefix = "q_init_table";
  } else if (p.q_init_case == "tilted_tool") {
    q_init_prefix = "q_init_tilted";
  } else if (p.q_init_case == "tilted_close") {
    q_init_prefix = "q_init_tilted_close";
  }
  char q_key[64];
  for (int i = 0; i < 7; ++i) {
    snprintf(q_key, sizeof(q_key), "%s_%d", q_init_prefix, i + 1);
    p.q_init[i] = getDouble(q_key, p.q_init[i]);
  }
  // Optional direct override, bypassing q_init_case.
  for (int i = 0; i < 7; ++i) {
    snprintf(q_key, sizeof(q_key), "q_init_%d", i + 1);
    p.q_init[i] = getDouble(q_key, p.q_init[i]);
  }

  // ---- tool pickup posture ----
  p.use_tool_pickup = getBool("use_tool_pickup", p.use_tool_pickup);
  p.pickup_standoff = getDouble("pickup_standoff", p.pickup_standoff);
  p.pickup_descend_speed_factor =
      getDouble("pickup_descend_speed_factor", p.pickup_descend_speed_factor);
  for (int i = 0; i < 7; ++i) {
    snprintf(q_key, sizeof(q_key), "q_pickup_%d", i + 1);
    p.q_pickup[i] = getDouble(q_key, p.q_pickup[i]);
  }

  // ---- collision thresholds ----
  p.use_custom_collision_behavior =
      getBool("use_custom_collision_behavior", p.use_custom_collision_behavior);
  p.collision_torque_acc = getDouble("collision_torque_acc", p.collision_torque_acc);
  p.collision_torque_nom = getDouble("collision_torque_nom", p.collision_torque_nom);
  p.collision_force_acc = getDouble("collision_force_acc", p.collision_force_acc);
  p.collision_force_nom = getDouble("collision_force_nom", p.collision_force_nom);

  return p;
}
