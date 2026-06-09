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
    return values.count(key) ? std::stod(values[key]) : def;
  };
  auto getBool = [&](const std::string& key, bool def) {
    return values.count(key) ? (std::stoi(values[key]) != 0) : def;
  };

  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);

  p.use_coriolis = getBool("use_coriolis", p.use_coriolis);
  p.use_current_pose = getBool("use_current_pose", p.use_current_pose);
  p.axis_constraint_mode = getBool("axis_constraint_mode", p.axis_constraint_mode);

  p.fix_p_x = getBool("fix_p_x", p.fix_p_x);
  p.fix_p_y = getBool("fix_p_y", p.fix_p_y);
  p.fix_p_z = getBool("fix_p_z", p.fix_p_z);

  p.use_surface_constraint = getBool("use_surface_constraint", p.use_surface_constraint);
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

  p.use_contact_search = getBool("use_contact_search", p.use_contact_search);
  p.contact_search_direction(0) = getDouble("contact_search_direction_x", p.contact_search_direction(0));
  p.contact_search_direction(1) = getDouble("contact_search_direction_y", p.contact_search_direction(1));
  p.contact_search_direction(2) = getDouble("contact_search_direction_z", p.contact_search_direction(2));
  p.contact_search_speed = getDouble("contact_search_speed", p.contact_search_speed);
  p.contact_search_max_distance =
      getDouble("contact_search_max_distance", p.contact_search_max_distance);
  p.contact_force_threshold = getDouble("contact_force_threshold", p.contact_force_threshold);
  p.contact_search_Kp_diag(0) = getDouble("contact_search_Kp_x", p.contact_search_Kp_diag(0));
  p.contact_search_Kp_diag(1) = getDouble("contact_search_Kp_y", p.contact_search_Kp_diag(1));
  p.contact_search_Kp_diag(2) = getDouble("contact_search_Kp_z", p.contact_search_Kp_diag(2));
  p.contact_search_Dp_diag(0) = getDouble("contact_search_Dp_x", p.contact_search_Dp_diag(0));
  p.contact_search_Dp_diag(1) = getDouble("contact_search_Dp_y", p.contact_search_Dp_diag(1));
  p.contact_search_Dp_diag(2) = getDouble("contact_search_Dp_z", p.contact_search_Dp_diag(2));

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

  p.q_goal[0] = getDouble("q_goal_1", p.q_goal[0]);
  p.q_goal[1] = getDouble("q_goal_2", p.q_goal[1]);
  p.q_goal[2] = getDouble("q_goal_3", p.q_goal[2]);
  p.q_goal[3] = getDouble("q_goal_4", p.q_goal[3]);
  p.q_goal[4] = getDouble("q_goal_5", p.q_goal[4]);
  p.q_goal[5] = getDouble("q_goal_6", p.q_goal[5]);
  p.q_goal[6] = getDouble("q_goal_7", p.q_goal[6]);

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
  printf("%s %.6f %.6f %.6f\n", label, v(0), v(1), v(2));
}

inline void printParameters(const Parameters& params) {
  printf("\nController summary:\n");
  printf("experiment_duration [s]: %.6f\n", params.experiment_duration);
  if (params.experiment_duration <= 0.0) {
    printf("time mode: indefinite run, stop with e + Enter\n");
  } else {
    printf("time mode: automatic stop after experiment_duration, or earlier with e + Enter\n");
  }

  printf("axis_constraint_mode: %d\n", params.axis_constraint_mode ? 1 : 0);
  printf("use_surface_constraint: %d\n", params.use_surface_constraint ? 1 : 0);
  printf("use_contact_search: %d\n", params.use_contact_search ? 1 : 0);
  printVec3("surface_normal:", params.surface_normal);
  printVec3("surface_tangent_hint:", params.surface_tangent_hint);
  printf("fix_R in surface frame [normal tangent1 tangent2]: %d %d %d\n",
         params.fix_R_x ? 1 : 0,
         params.fix_R_y ? 1 : 0,
         params.fix_R_z ? 1 : 0);
  printVec3("Kp [N/m]:", params.Kp_diag);
  printVec3("Dp [Ns/m]:", params.Dp_diag);
  printVec3("KR [Nm/rad]:", params.KR_diag);
  printVec3("DR [Nms/rad]:", params.DR_diag);
  printf("nullspace gains: k_start=%.6f damping=%.6f k_sigma=%.6f tau_max=%.6f\n",
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

inline Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_surface_frame, const Mat3& R_surface) {
  return R_surface * diagonal_in_surface_frame.asDiagonal() * R_surface.transpose();
}

inline void configureCollisionBehavior(franka::Robot& robot, const Parameters& params) {
  setDefaultBehavior(robot);

  if (!params.use_custom_collision_behavior) {
    printf("Using setDefaultBehavior(robot) collision thresholds only.\n");
    return;
  }

  printf("Applying custom robot.setCollisionBehavior(...) thresholds.\n");

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

  if (!params.use_current_pose) {
    const double r = time / params.trajectory_duration;
    const double s = smoothStep(r);
    const double s_dot = smoothStepDerivative(r, params.trajectory_duration);
    desired.p_d = p_start + s * params.delta_p;
    desired.pdot_d = s_dot * params.delta_p;
  }

  if (params.axis_constraint_mode) {
    if (params.use_surface_constraint) {
      const Vec3 normal = R_surface.col(0);

      // Plane constraint for any surface orientation:
      //
      //   signed_distance [m] = n^T * (p_plane - p_EE)
      //   p_d [m]            = p_EE + signed_distance * n
      //
      // This keeps only the normal direction constrained. The two tangent
      // directions stay compliant because their position error is zero.
      const double signed_distance = normal.dot(plane_point - p_EE);
      desired.p_d = p_EE + signed_distance * normal;
    } else {
      desired.p_d(0) = params.fix_p_x ? p_start(0) : p_EE(0);
      desired.p_d(1) = params.fix_p_y ? p_start(1) : p_EE(1);
      desired.p_d(2) = params.fix_p_z ? p_start(2) : p_EE(2);
    }
    desired.pdot_d.setZero();
  }

  return desired;
}

inline Vec3 applyRotationalAxisMask(const Parameters& params, Vec3 e_R, const Mat3& R_surface) {
  if (!params.axis_constraint_mode) {
    return e_R;
  }

  Vec3 e_R_task = params.use_surface_constraint ? R_surface.transpose() * e_R : e_R;
  e_R_task(0) = params.fix_R_x ? e_R_task(0) : 0.0;
  e_R_task(1) = params.fix_R_y ? e_R_task(1) : 0.0;
  e_R_task(2) = params.fix_R_z ? e_R_task(2) : 0.0;

  return params.use_surface_constraint ? R_surface * e_R_task : e_R_task;
}

inline Vec6 makeWrench(const Vec3& f, const Vec3& m) {
  Vec6 wrench;
  wrench.head<3>() = f;
  wrench.tail<3>() = m;
  return wrench;
}

inline Vec7 computeNullspaceTorque(
    const Parameters& params,
    const franka::Model& model,
    const franka::RobotState& state,
    const Mat6x7& J,
    const Vec7& dq,
    const Vec7& q_start) {
  Vec7 tau_nullspace = Vec7::Zero();

  if (!params.use_nullspace_optimization) {
    return tau_nullspace;
  }

  Eigen::Map<const Vec7> q_current(state.q.data());

  const Eigen::Matrix<double, 7, 7> I7 =
      Eigen::Matrix<double, 7, 7>::Identity();
  const Eigen::Matrix<double, 6, 6> I6 =
      Eigen::Matrix<double, 6, 6>::Identity();
  const double lambda = 0.05;
  const Eigen::Matrix<double, 6, 6> JJt_damped =
      J * J.transpose() + lambda * lambda * I6;

  // Joint nullspace projector:
  //
  //   N = I - J^T * (J*J^T + lambda^2 I)^(-1) * J
  //
  // Units:
  //   J              maps dq [rad/s] to xdot [m/s, rad/s]
  //   tau_posture    is joint torque [Nm]
  //   nullspace term keeps motion from changing the end-effector task.
  const Eigen::Matrix<double, 7, 7> N =
      I7 - J.transpose() * JJt_damped.ldlt().solve(J);

  // Restoring spring in the nullspace:
  //
  //   tau_return [Nm] = N * (k_start * (q_start - q) - d * dq)
  //
  // This prevents the robot from slowly rotating/drifting in the nullspace
  // when the TCP is moved back and forth at the same Cartesian place.
  tau_nullspace =
      N * (params.nullspace_k_start * (q_start - q_current)
           - params.nullspace_damping * dq);

  Eigen::JacobiSVD<Mat6x7> svd_current(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);
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
          franka::Frame::kEndEffector,
          q_plus_array,
          state.F_T_EE,
          state.EE_T_K);

  const std::array<double, 42> J_minus_array =
      model.zeroJacobian(
          franka::Frame::kEndEffector,
          q_minus_array,
          state.F_T_EE,
          state.EE_T_K);

  Eigen::Map<const Mat6x7> J_plus(J_plus_array.data());
  Eigen::Map<const Mat6x7> J_minus(J_minus_array.data());

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
    const Vec7& tau_raw,
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
  row.tau_raw = tau_raw;
  row.tau_cmd = tau_cmd;
  return row;
}

inline void printFinalSummary(
    const Vec3& final_p_d,
    const Vec3& final_p_EE,
    const Vec3& final_e_p,
    const Vec3& final_e_R,
    const std::string& csv_file_name) {
  printf("\nExperiment finished.\n");
  printVec3("Final desired position p_d [m]: ", final_p_d);
  printVec3("Final reached position p_EE [m]:", final_p_EE);
  printVec3("Final position error e_p [m]:   ", final_e_p);
  printf("Final position error norm [m]:   %.6f\n", final_e_p.norm());
  printf("Final position error norm [mm]:  %.6f\n", 1000.0 * final_e_p.norm());
  printVec3("Final rotation error e_R [rad]: ", final_e_R);
  printf("Final rotation error norm [rad]: %.6f\n", final_e_R.norm());
  printf("Final rotation error norm [deg]: %.6f\n", (180.0 / M_PI) * final_e_R.norm());
  printf("CSV log written to: %s\n", csv_file_name.c_str());
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
           << "tau_raw_1,tau_raw_2,tau_raw_3,tau_raw_4,tau_raw_5,tau_raw_6,tau_raw_7,"
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
             << row.tau_raw(0) << "," << row.tau_raw(1) << ","
             << row.tau_raw(2) << "," << row.tau_raw(3) << ","
             << row.tau_raw(4) << "," << row.tau_raw(5) << ","
             << row.tau_raw(6) << ","
             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
}
