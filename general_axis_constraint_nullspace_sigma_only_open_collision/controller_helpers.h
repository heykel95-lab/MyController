#pragma once

#include "controller_types.h"

inline Array7 vec7ToArray(const Vec7& v) {
  Array7 array{};
  for (int i = 0; i < 7; ++i) {
    array[i] = v(i);
  }
  return array;
}

inline double smallestSingularValue(const Mat6x7& J) {
  Eigen::JacobiSVD<Mat6x7> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.singularValues().minCoeff();
}

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
  p.surface_tangent_hint(0) = getDouble("surface_tangent_hint_x", p.surface_tangent_hint(0));
  p.surface_tangent_hint(1) = getDouble("surface_tangent_hint_y", p.surface_tangent_hint(1));
  p.surface_tangent_hint(2) = getDouble("surface_tangent_hint_z", p.surface_tangent_hint(2));
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

  p.fix_R_x = getBool("fix_R_x", p.fix_R_x);
  p.fix_R_y = getBool("fix_R_y", p.fix_R_y);
  p.fix_R_z = getBool("fix_R_z", p.fix_R_z);

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

  p.Kp_diag(0) = getDouble("Kp_x", p.Kp_diag(0));
  p.Kp_diag(1) = getDouble("Kp_y", p.Kp_diag(1));
  p.Kp_diag(2) = getDouble("Kp_z", p.Kp_diag(2));

  p.Dp_diag(0) = getDouble("Dp_x", p.Dp_diag(0));
  p.Dp_diag(1) = getDouble("Dp_y", p.Dp_diag(1));
  p.Dp_diag(2) = getDouble("Dp_z", p.Dp_diag(2));

  p.KR_diag(0) = getDouble("KR_x", p.KR_diag(0));
  p.KR_diag(1) = getDouble("KR_y", p.KR_diag(1));
  p.KR_diag(2) = getDouble("KR_z", p.KR_diag(2));

  p.DR_diag(0) = getDouble("DR_x", p.DR_diag(0));
  p.DR_diag(1) = getDouble("DR_y", p.DR_diag(1));
  p.DR_diag(2) = getDouble("DR_z", p.DR_diag(2));

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

inline Vec7 limitJointTorqueVectorNorm(const Vec7& v, double max_norm) {
  if (max_norm <= 0.0) {
    return v;
  }

  const double norm = v.norm();
  if (norm > max_norm && norm > 1e-12) {
    return max_norm * v / norm;
  }
  return v;
}

inline double smoothStep(double r) {
  r = std::max(0.0, std::min(1.0, r));
  return 10.0 * std::pow(r, 3) - 15.0 * std::pow(r, 4) + 6.0 * std::pow(r, 5);
}

inline double smoothStepDerivative(double r, double T) {
  r = std::max(0.0, std::min(1.0, r));
  if (r <= 0.0 || r >= 1.0) {
    return 0.0;
  }

  const double ds_dr =
      30.0 * std::pow(r, 2)
    - 60.0 * std::pow(r, 3)
    + 30.0 * std::pow(r, 4);
  return ds_dr / T;
}

inline Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired) {
  Mat3 R_error = R_current.transpose() * R_desired;
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Vec3::Zero();
  }
  return R_current * angle_axis.axis() * angle_axis.angle();
}

inline Array7 filledArray7(double value) {
  Array7 array{};
  array.fill(value);
  return array;
}

inline Array6 filledArray6(double value) {
  Array6 array{};
  array.fill(value);
  return array;
}

inline void printVec3(const char* label, const Vec3& v) {
  printf("%s = [%.6f, %.6f, %.6f]\n", label, v(0), v(1), v(2));
}

inline void printVec3Mm(const char* label, const Vec3& v) {
  printf("%s = [%.1f, %.1f, %.1f] mm\n",
         label,
         1000.0 * v(0),
         1000.0 * v(1),
         1000.0 * v(2));
}

inline void printVec3Deg(const char* label, const Vec3& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.2f, %.2f, %.2f] deg\n",
         label,
         rad_to_deg * v(0),
         rad_to_deg * v(1),
         rad_to_deg * v(2));
}

inline void printVec7(const char* label, const Vec7& v) {
  printf("%s = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
         label,
         v(0),
         v(1),
         v(2),
         v(3),
         v(4),
         v(5),
         v(6));
}

inline void printVec7Deg(const char* label, const Vec7& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg\n",
         label,
         rad_to_deg * v(0),
         rad_to_deg * v(1),
         rad_to_deg * v(2),
         rad_to_deg * v(3),
         rad_to_deg * v(4),
         rad_to_deg * v(5),
         rad_to_deg * v(6));
}

inline void printMat3(const char* label, const Mat3& m) {
  printf("%s = [\n", label);
  printf("  %.6f, %.6f, %.6f;\n", m(0, 0), m(0, 1), m(0, 2));
  printf("  %.6f, %.6f, %.6f;\n", m(1, 0), m(1, 1), m(1, 2));
  printf("  %.6f, %.6f, %.6f\n", m(2, 0), m(2, 1), m(2, 2));
  printf("]\n");
}

inline void printMat4x4(const char* label, const Mat4x4& m) {
  printf("%s = [\n", label);
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(0, 0), m(0, 1), m(0, 2), m(0, 3));
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(1, 0), m(1, 1), m(1, 2), m(1, 3));
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(2, 0), m(2, 1), m(2, 2), m(2, 3));
  printf("  %.6f, %.6f, %.6f, %.6f\n", m(3, 0), m(3, 1), m(3, 2), m(3, 3));
  printf("]\n");
}

inline void printJointStartEndTable(const Vec7& q_start, const Vec7& q_final) {
  printf("\nq_start_q_final_delta = [\n");
  printf("  %% joint, q_start_rad, q_final_rad, delta_rad\n");
  for (int i = 0; i < 7; ++i) {
    printf("  %d, %.6f, %.6f, %.6f%s\n",
           i + 1,
           q_start(i),
           q_final(i),
           q_final(i) - q_start(i),
           (i == 6) ? "" : ";");
  }
  printf("]\n");
}

inline void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("\nJoint motion [deg]:\n");
  printf("joint | start | final | delta\n");
  for (int i = 0; i < 7; ++i) {
    printf("q%d    | %7.1f | %7.1f | %+7.1f\n",
           i + 1,
           rad_to_deg * q_start(i),
           rad_to_deg * q_final(i),
           rad_to_deg * (q_final(i) - q_start(i)));
  }
}

inline void printParameters(const Parameters& params) {
  printf("\n=== Controller setup ===\n");
  if (params.experiment_duration <= 0.0) {
    printf("time: indefinite, stop with e + Enter\n");
  } else {
    printf("time: %.1f s, or stop with e + Enter\n", params.experiment_duration);
  }
  printf("q_init: %s | contact: %s | orientation_test: %s\n",
         params.q_init_case.c_str(),
         params.use_contact_search ? "on" : "off",
         params.orientation_test_only ? "on" : "off");
  printf("phases: %s | virtual_center: %s | vcr_offset: %.1f mm\n",
         params.use_phase_sequence ? "on" : "off",
         params.use_virtual_center_after_contact ? "on" : "off",
         1000.0 * params.vcr_offset);
  printVec3("surface_n", params.surface_normal);
  printf("rot_axes [normal, tangent1, tangent2] = [%d, %d, %d]\n",
         params.fix_R_x ? 1 : 0,
         params.fix_R_y ? 1 : 0,
         params.fix_R_z ? 1 : 0);
  printf("orient_tol: %.1f deg | post_align: %.1f s\n",
         (180.0 / M_PI) * params.orient_phase_error_threshold,
         params.post_contact_align_duration);
  printVec3("Kp", params.Kp_diag);
  printVec3("Dp", params.Dp_diag);
  printVec3("KR", params.KR_diag);
  printVec3("DR", params.DR_diag);
  printf("nullspace: k_start=%.2f damping=%.2f k_sigma=%.2f tau_max=%.2f Nm\n",
         params.nullspace_k_start,
         params.nullspace_damping,
         params.nullspace_k_sigma,
         params.nullspace_tau_max);
}

inline Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback) {
  if (v.norm() > 1e-9) {
    return v.normalized();
  }
  return fallback.normalized();
}

inline Mat3 makeSurfaceFrame(const Parameters& params) {
  const Vec3 normal = normalizedOrFallback(params.surface_normal, Vec3(1.0, 0.0, 0.0));
  Vec3 tangent1 = params.surface_tangent_hint - normal * normal.dot(params.surface_tangent_hint);

  if (tangent1.norm() <= 1e-9) {
    Vec3 fallback(0.0, 1.0, 0.0);
    if (std::abs(normal.dot(fallback)) > 0.95) {
      fallback = Vec3(0.0, 0.0, 1.0);
    }
    tangent1 = fallback - normal * normal.dot(fallback);
  }

  tangent1.normalize();
  Vec3 tangent2 = normal.cross(tangent1);
  tangent2.normalize();

  Mat3 R_surface;
  R_surface.col(0) = normal;
  R_surface.col(1) = tangent1;
  R_surface.col(2) = tangent2;
  return R_surface;
}

inline Mat3 makeToolOrientationParallelToSurface(const Mat3& R_surface, const Mat3& R_start) {
  const Vec3 normal = R_surface.col(0);
  Vec3 x_axis = R_start.col(0) - normal * normal.dot(R_start.col(0));

  if (x_axis.norm() <= 1e-9) {
    x_axis = R_surface.col(1);
  }
  x_axis.normalize();

  Mat3 R_tool;
  R_tool.col(0) = x_axis;
  R_tool.col(2) = -normal;
  R_tool.col(1) = R_tool.col(2).cross(R_tool.col(0));
  R_tool.col(1).normalize();
  return R_tool;
}

inline Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_surface_frame, const Mat3& R_surface) {
  return R_surface * diagonal_in_surface_frame.asDiagonal() * R_surface.transpose();
}

inline void startKeyboardStopThread(
    const Parameters& params,
    std::atomic<bool>& stop_requested) {
  printf("Press e+Enter to stop the Control algorithm before the remaining duration expires.\n");
  if (params.experiment_duration <= 0.0) {
    printf("experiment_duration <= 0: the Control algorithm is running indefinitely until e + Enter.\n");
  }

  std::thread keyboard_thread([&stop_requested]() {
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line == "e" || line == "E") {
        stop_requested.store(true);
        break;
      }
    }
  });
  keyboard_thread.detach();
}

inline void configureCollisionBehavior(Robot& robot, const Parameters& params) {
  setDefaultBehavior(robot);

  if (!params.use_custom_collision_behavior) {
    printf("Collision: default thresholds.\n");
    return;
  }

  printf("Collision: custom thresholds.\n");

  const Array7 collision_torque_acc = filledArray7(params.collision_torque_acc);
  const Array7 collision_torque_nom = filledArray7(params.collision_torque_nom);
  const Array6 collision_force_acc = filledArray6(params.collision_force_acc);
  const Array6 collision_force_nom = filledArray6(params.collision_force_nom);

  robot.setCollisionBehavior(
      collision_torque_acc,
      collision_torque_acc,
      collision_torque_nom,
      collision_torque_nom,
      collision_force_acc,
      collision_force_acc,
      collision_force_nom,
      collision_force_nom);
}

inline DesiredMotion computeDesiredMotion(
    const Parameters& params,
    double time,
    const Vec3& p_start,
    const Vec3& p_EE,
    const Mat3& R_surface,
    const Vec3& plane_point) {
  DesiredMotion desired{p_start, Vec3::Zero()};

  if (!params.hold_mode) {
    const double r = time / params.trajectory_duration;
    const double s = smoothStep(r);
    const double s_dot = smoothStepDerivative(r, params.trajectory_duration);
    desired.p_d = p_start + s * params.delta_p;
    desired.pdot_d = s_dot * params.delta_p;
  }

  if (params.constraint_enabled) {
    const Vec3 normal = R_surface.col(0);

    // Plane constraint for any surface orientation:
    //
    //   signed_distance [m] = n^T * (p_plane - p_EE)
    //   p_d [m]            = p_EE + signed_distance * n
    //
    // The same surface-frame mode covers axis-aligned and inclined planes:
    //   YZ plane -> surface_normal = [1, 0, 0]
    //   XZ plane -> surface_normal = [0, 1, 0]
    //   XY plane -> surface_normal = [0, 0, 1]
    //   inclined plane -> any normalized surface_normal
    const double signed_distance = normal.dot(plane_point - p_EE);
    desired.p_d = p_EE + signed_distance * normal;
    desired.pdot_d.setZero();
  }

  return desired;
}

inline Vec3 applyRotationalAxisMask(const Parameters& params, Vec3 e_R, const Mat3& R_surface) {
  if (!params.constraint_enabled) {
    return e_R;
  }

  Vec3 e_R_task = R_surface.transpose() * e_R;
  e_R_task(0) = params.fix_R_x ? e_R_task(0) : 0.0;
  e_R_task(1) = params.fix_R_y ? e_R_task(1) : 0.0;
  e_R_task(2) = params.fix_R_z ? e_R_task(2) : 0.0;

  return R_surface * e_R_task;
}

inline Vec7 computeNullspaceTorque(
    const Parameters& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq,
    const Vec7& q_start) {
  Vec7 tau_nullspace = Vec7::Zero();

  // If nullspace optimization is disabled, return zero nullspace torque.
  if (!params.use_nullspace_optimization) {
    return tau_nullspace;
  }
  // else, compute the nullspace torque to optimize the smallest singular value of the Jacobian.
  // Map the current joint positions to a 7 elements vector.
  Map<const Vec7> q_current(state.q.data());

  // Identity matrices for the pseudo-inverse calculation:
  const Mat7x7 I7 = Mat7x7::Identity();
  const Mat6x6 I6 = Mat6x6::Identity();
  // Damping factor for the pseudo-inverse to improve numerical stability near singularities.
  const double lambda = 0.05;
  // Damped pseudo-inverse of the Jacobian: JJt_damped = J*J^T + lambda^2 I in units of [m^2/s^2] or [rad^2/s^2], depending on the row of J.
  const Mat6x6 JJt_damped = J * J.transpose() + lambda * lambda * I6;

  // Joint nullspace projector:
  //
  //   N = I - J^T * (JJt_damped)^(-1) * J
  //
  // Units:
  //   J              maps dq [rad/s] to xdot [m/s, rad/s]
  //   tau_posture    is joint torque [Nm]
  //   nullspace term keeps motion from changing the end-effector task.
  // JJt_damped.ldlt().solve(J) computes (JJt_damped)^(-1) * J in a numerically stable way.
  // It solves the linear system JJt_damped * X = J for X, which is equivalent to X = (JJt_damped)^(-1) * J,
  // but more efficient and stable than explicitly computing the inverse of JJt_damped.
  const Mat7x7 N = I7 - J.transpose() * JJt_damped.ldlt().solve(J);

  // Restoring spring in the nullspace:
  //
  //   tau_return [Nm] = N * (k_start * (q_start - q) - d * dq)
  //
  // This prevents the robot from slowly rotating/drifting in the nullspace
  // when the TCP is moved back and forth at the same Cartesian place.
  tau_nullspace = N * (params.nullspace_k_start * (q_start - q_current) - params.nullspace_damping * dq);

  Eigen::JacobiSVD<Mat6x7> svd_current(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec7 n = svd_current.matrixV().col(6);

  if (n.norm() <= 1e-9) {
    return limitJointTorqueVectorNorm(tau_nullspace, params.nullspace_tau_max);
  }

  n.normalize();

  const Vec7 q_plus = q_current + params.nullspace_alpha * n;
  const Vec7 q_minus = q_current - params.nullspace_alpha * n;

  const Array7 q_plus_array = vec7ToArray(q_plus);
  const Array7 q_minus_array = vec7ToArray(q_minus);

  const std::array<double, 42> J_plus_array =
      model.zeroJacobian(
          Frame::kEndEffector,
          q_plus_array,
          state.F_T_EE,
          state.EE_T_K);

  const std::array<double, 42> J_minus_array =
      model.zeroJacobian(
          Frame::kEndEffector,
          q_minus_array,
          state.F_T_EE,
          state.EE_T_K);

  Map<const Mat6x7> J_plus(J_plus_array.data());
  Map<const Mat6x7> J_minus(J_minus_array.data());

  const double sigma_min_plus = smallestSingularValue(J_plus);
  const double sigma_min_minus = smallestSingularValue(J_minus);
  const double sigma_direction = (sigma_min_plus > sigma_min_minus) ? 1.0 : -1.0;

  const Vec7 tau_sigma =
      N * (params.nullspace_k_sigma * sigma_direction * params.nullspace_alpha * n);

  return limitJointTorqueVectorNorm(tau_nullspace + tau_sigma, params.nullspace_tau_max);
}

inline LogData makeLogRow(
    double time,
    const Vec3& p_EE,
    const Vec3& p_d,
    const Vec3& p_end,
    const Vec3& e_p,
    const Vec3& e_R,
    const Vec3& pdot,
    const Vec3& pdot_d,
    const Vec3& omega,
    const Vec3& f,
    const Vec3& m,
    const Vec7& tau_cmd) {
  LogData row;
  row.time = time;
  row.p_EE = p_EE;
  row.p_d = p_d;
  row.p_end = p_end;
  row.e_p = e_p;
  row.e_R = e_R;
  row.pdot = pdot;
  row.pdot_d = pdot_d;
  row.omega = omega;
  row.f = f;
  row.m = m;
  row.tau_cmd = tau_cmd;
  return row;
}

inline void printFinalSummary(
    const Vec3& final_p_d,
    const Vec3& final_p_EE,
    const Vec3& final_e_p,
    const Vec3& final_e_R,
    const std::string& csv_file_name) {
  printf("\n=== Final result ===\n");
  printVec3Mm("p_d", final_p_d);
  printVec3Mm("p_EE", final_p_EE);
  printVec3Mm("e_p", final_e_p);
  printf("position_error = %.2f mm\n", 1000.0 * final_e_p.norm());
  printVec3Deg("e_R", final_e_R);
  printf("rotation_error = %.2f deg\n", (180.0 / M_PI) * final_e_R.norm());
  printf("csv: %s\n", csv_file_name.c_str());
}

inline void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name) {
  std::ofstream log_file(csv_file_name);

  log_file << "time,"
           << "p_EE_x,p_EE_y,p_EE_z,"
           << "p_d_x,p_d_y,p_d_z,"
           << "p_end_x,p_end_y,p_end_z,"
           << "e_p_x,e_p_y,e_p_z,"
           << "e_R_x,e_R_y,e_R_z,"
           << "pdot_x,pdot_y,pdot_z,"
           << "pdot_d_x,pdot_d_y,pdot_d_z,"
           << "omega_x,omega_y,omega_z,"
           << "f_x,f_y,f_z,"
           << "m_x,m_y,m_z,"
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  for (const auto& row : log_data) {
    log_file << std::fixed << std::setprecision(6)
             << row.time << ","
             << row.p_EE(0) << "," << row.p_EE(1) << "," << row.p_EE(2) << ","
             << row.p_d(0) << "," << row.p_d(1) << "," << row.p_d(2) << ","
             << row.p_end(0) << "," << row.p_end(1) << "," << row.p_end(2) << ","
             << row.e_p(0) << "," << row.e_p(1) << "," << row.e_p(2) << ","
             << row.e_R(0) << "," << row.e_R(1) << "," << row.e_R(2) << ","
             << row.pdot(0) << "," << row.pdot(1) << "," << row.pdot(2) << ","
             << row.pdot_d(0) << "," << row.pdot_d(1) << "," << row.pdot_d(2) << ","
             << row.omega(0) << "," << row.omega(1) << "," << row.omega(2) << ","
             << row.f(0) << "," << row.f(1) << "," << row.f(2) << ","
             << row.m(0) << "," << row.m(1) << "," << row.m(2) << ","
             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
}
