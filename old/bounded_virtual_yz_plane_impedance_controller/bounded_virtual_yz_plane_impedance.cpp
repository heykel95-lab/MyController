/*
  Bounded Virtual Y-Z Plane Cartesian Impedance Controller
  -------------------------------------------------------------------------
  This controller is positional and rotational.

  It computes:
      f = Kp * e_p + Dp * (pdot_d - pdot)
      m = KR * e_R - DR * omega

  Then it builds the full Cartesian wrench:
      wrench = [f_x, f_y, f_z, m_x, m_y, m_z]^T

  and maps it to joint torques:
      tau_task = J^T * wrench

  Final command:
      tau_cmd = tau_task + coriolis

  Gravity compensation is not added, following the working project.

  New in this version:
  - Direction-dependent friction-compensation force can be applied in all 3 positional axes.
  - Direction-dependent friction-compensation moment can be applied in all 3 rotational axes.
  - Force and moment saturation are still applied after compensation.
  - Torque-rate limiting is still applied.
  - CSV logging is buffered.
  - Semi-automatic recovery is included.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"

struct Parameters {
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 10.0;
  std::string csv_file_name = "bounded_virtual_yz_plane_impedance_log.csv";

  bool use_current_pose = true;

  // Virtual plane mode:
  // If enabled, the desired x position is fixed at x_start,
  // while desired y and z are continuously set to the current y and z.
  // This creates a virtual Y-Z plane: x is constrained, y and z are allowed.
  bool virtual_yz_plane_mode = true;

  // Workspace bounds inside the virtual Y-Z plane.
  // Inside these limits y and z are free.
  // Outside these limits the controller creates a restoring force back to the boundary.
  double y_limit_neg = 0.06;   // allowed motion in negative y direction [m]
  double y_limit_pos = 0.06;   // allowed motion in positive y direction [m]
  double z_limit_down = 0.06;  // allowed motion downward [m]
  double z_limit_up = 0.04;    // allowed motion upward [m]

  Eigen::Vector3d delta_p = Eigen::Vector3d(0.005, 0.0, 0.0);
  double trajectory_duration = 8.0;

  Eigen::Vector3d Kp_diag = Eigen::Vector3d(150.0, 100.0, 100.0);
  Eigen::Vector3d Dp_diag = Eigen::Vector3d(25.0, 20.0, 20.0);

  Eigen::Vector3d KR_diag = Eigen::Vector3d(3.0, 3.0, 3.0);
  Eigen::Vector3d DR_diag = Eigen::Vector3d(3.5, 3.5, 3.5);

  double f_max = 10.0;
  double m_max = 2.0;
  double delta_tau_max = 2.0;

  // Initial joint configuration used before starting impedance control.
  // It is read from parameters.txt so the start pose can be tuned without recompiling.
  std::array<double, 7> q_goal = {{
      0.0,
      -M_PI_4,
      0.0,
      -3.0 * M_PI_4,
      0.0,
      M_PI_2,
      0.0
  }};

  // Optional robot collision/contact thresholds.
  // These affect Franka collision/contact reflex thresholds.
  // They do not disable joint velocity limits.
  bool use_custom_collision_behavior = false;

  std::array<double, 7> collision_torque_lower_acc = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  std::array<double, 7> collision_torque_upper_acc = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  std::array<double, 7> collision_torque_lower_nom = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  std::array<double, 7> collision_torque_upper_nom = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};

  std::array<double, 6> collision_force_lower_acc = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  std::array<double, 6> collision_force_upper_acc = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  std::array<double, 6> collision_force_lower_nom = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  std::array<double, 6> collision_force_upper_nom = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};

  Eigen::Vector3d e_thresh_p = Eigen::Vector3d(0.003, 0.003, 0.003);

  // Direction-dependent positional compensation.
  // f_fric_pos is used when e_p(i) > +threshold.
  // f_fric_neg is used when e_p(i) < -threshold.
  Eigen::Vector3d f_fric_pos = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d f_fric_neg = Eigen::Vector3d(0.5, 0.5, 0.5);

  Eigen::Vector3d e_thresh_R = Eigen::Vector3d(0.03, 0.03, 0.03);

  // Direction-dependent rotational compensation.
  // m_fric_pos is used when e_R(i) > +threshold.
  // m_fric_neg is used when e_R(i) < -threshold.
  Eigen::Vector3d m_fric_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d m_fric_neg = Eigen::Vector3d(0.0, 0.0, 0.0);
};

struct LogData {
  double time;

  Eigen::Vector3d p_EE;
  Eigen::Vector3d p_d;
  Eigen::Vector3d p_end;

  Eigen::Vector3d e_p;
  Eigen::Vector3d e_R;

  Eigen::Vector3d pdot;
  Eigen::Vector3d pdot_d;
  Eigen::Vector3d omega;

  Eigen::Vector3d f_before_deadzone;
  Eigen::Vector3d f_after_deadzone;

  Eigen::Vector3d m_before_deadzone;
  Eigen::Vector3d m_after_deadzone;

  Eigen::Matrix<double, 7, 1> tau_raw;
  Eigen::Matrix<double, 7, 1> tau_limited;
};

std::array<double, 7> eigenToArray(const Eigen::Matrix<double, 7, 1>& tau) {
  std::array<double, 7> tau_array{};
  for (int i = 0; i < 7; ++i) {
    tau_array[i] = tau(i);
  }
  return tau_array;
}

std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

double clampValue(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

Parameters readParameters(const std::string& filename) {
  Parameters p;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cout << "Could not open " << filename << ". Using defaults." << std::endl;
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

  p.use_current_pose = getBool("use_current_pose", p.use_current_pose);
  p.virtual_yz_plane_mode = getBool("virtual_yz_plane_mode", p.virtual_yz_plane_mode);

  p.y_limit_neg = getDouble("y_limit_neg", p.y_limit_neg);
  p.y_limit_pos = getDouble("y_limit_pos", p.y_limit_pos);
  p.z_limit_down = getDouble("z_limit_down", p.z_limit_down);
  p.z_limit_up = getDouble("z_limit_up", p.z_limit_up);

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

  p.f_max = getDouble("f_max", p.f_max);
  p.m_max = getDouble("m_max", p.m_max);
  p.delta_tau_max = getDouble("delta_tau_max", p.delta_tau_max);

  p.q_goal[0] = getDouble("q_goal_1", p.q_goal[0]);
  p.q_goal[1] = getDouble("q_goal_2", p.q_goal[1]);
  p.q_goal[2] = getDouble("q_goal_3", p.q_goal[2]);
  p.q_goal[3] = getDouble("q_goal_4", p.q_goal[3]);
  p.q_goal[4] = getDouble("q_goal_5", p.q_goal[4]);
  p.q_goal[5] = getDouble("q_goal_6", p.q_goal[5]);
  p.q_goal[6] = getDouble("q_goal_7", p.q_goal[6]);

  p.use_custom_collision_behavior = getBool("use_custom_collision_behavior", p.use_custom_collision_behavior);

  // One common value for all 7 joint torque thresholds [Nm].
  const double collision_joint_torque = getDouble("collision_joint_torque_threshold", 30.0);
  for (int i = 0; i < 7; ++i) {
    p.collision_torque_lower_acc[i] = collision_joint_torque;
    p.collision_torque_upper_acc[i] = collision_joint_torque;
    p.collision_torque_lower_nom[i] = collision_joint_torque;
    p.collision_torque_upper_nom[i] = collision_joint_torque;
  }

  // One common value for translational Cartesian force thresholds [N].
  const double collision_cart_force = getDouble("collision_cartesian_force_threshold", 30.0);
  for (int i = 0; i < 3; ++i) {
    p.collision_force_lower_acc[i] = collision_cart_force;
    p.collision_force_upper_acc[i] = collision_cart_force;
    p.collision_force_lower_nom[i] = collision_cart_force;
    p.collision_force_upper_nom[i] = collision_cart_force;
  }

  // One common value for rotational Cartesian moment thresholds [Nm].
  const double collision_cart_moment = getDouble("collision_cartesian_moment_threshold", 20.0);
  for (int i = 3; i < 6; ++i) {
    p.collision_force_lower_acc[i] = collision_cart_moment;
    p.collision_force_upper_acc[i] = collision_cart_moment;
    p.collision_force_lower_nom[i] = collision_cart_moment;
    p.collision_force_upper_nom[i] = collision_cart_moment;
  }

  p.e_thresh_p(0) = getDouble("e_thresh_p_x", p.e_thresh_p(0));
  p.e_thresh_p(1) = getDouble("e_thresh_p_y", p.e_thresh_p(1));
  p.e_thresh_p(2) = getDouble("e_thresh_p_z", p.e_thresh_p(2));

  // New asymmetric names.
  p.f_fric_pos(0) = getDouble("f_fric_x_pos", p.f_fric_pos(0));
  p.f_fric_pos(1) = getDouble("f_fric_y_pos", p.f_fric_pos(1));
  p.f_fric_pos(2) = getDouble("f_fric_z_pos", p.f_fric_pos(2));

  p.f_fric_neg(0) = getDouble("f_fric_x_neg", p.f_fric_neg(0));
  p.f_fric_neg(1) = getDouble("f_fric_y_neg", p.f_fric_neg(1));
  p.f_fric_neg(2) = getDouble("f_fric_z_neg", p.f_fric_neg(2));

  // Backward compatibility: if old symmetric names are still present, use them for both signs.
  if (values.count("f_fric_x")) {
    p.f_fric_pos(0) = getDouble("f_fric_x", p.f_fric_pos(0));
    p.f_fric_neg(0) = getDouble("f_fric_x", p.f_fric_neg(0));
  }
  if (values.count("f_fric_y")) {
    p.f_fric_pos(1) = getDouble("f_fric_y", p.f_fric_pos(1));
    p.f_fric_neg(1) = getDouble("f_fric_y", p.f_fric_neg(1));
  }
  if (values.count("f_fric_z")) {
    p.f_fric_pos(2) = getDouble("f_fric_z", p.f_fric_pos(2));
    p.f_fric_neg(2) = getDouble("f_fric_z", p.f_fric_neg(2));
  }

  p.e_thresh_R(0) = getDouble("e_thresh_R_x", p.e_thresh_R(0));
  p.e_thresh_R(1) = getDouble("e_thresh_R_y", p.e_thresh_R(1));
  p.e_thresh_R(2) = getDouble("e_thresh_R_z", p.e_thresh_R(2));

  // New asymmetric names.
  p.m_fric_pos(0) = getDouble("m_fric_x_pos", p.m_fric_pos(0));
  p.m_fric_pos(1) = getDouble("m_fric_y_pos", p.m_fric_pos(1));
  p.m_fric_pos(2) = getDouble("m_fric_z_pos", p.m_fric_pos(2));

  p.m_fric_neg(0) = getDouble("m_fric_x_neg", p.m_fric_neg(0));
  p.m_fric_neg(1) = getDouble("m_fric_y_neg", p.m_fric_neg(1));
  p.m_fric_neg(2) = getDouble("m_fric_z_neg", p.m_fric_neg(2));

  // Backward compatibility: if old symmetric names are still present, use them for both signs.
  if (values.count("m_fric_x")) {
    p.m_fric_pos(0) = getDouble("m_fric_x", p.m_fric_pos(0));
    p.m_fric_neg(0) = getDouble("m_fric_x", p.m_fric_neg(0));
  }
  if (values.count("m_fric_y")) {
    p.m_fric_pos(1) = getDouble("m_fric_y", p.m_fric_pos(1));
    p.m_fric_neg(1) = getDouble("m_fric_y", p.m_fric_neg(1));
  }
  if (values.count("m_fric_z")) {
    p.m_fric_pos(2) = getDouble("m_fric_z", p.m_fric_pos(2));
    p.m_fric_neg(2) = getDouble("m_fric_z", p.m_fric_neg(2));
  }

  return p;
}

Eigen::Vector3d limitVectorNorm(const Eigen::Vector3d& v, double max_norm) {
  if (max_norm <= 0.0) {
    return v;
  }

  const double norm = v.norm();

  if (norm > max_norm && norm > 1e-12) {
    return max_norm * v / norm;
  }

  return v;
}

Eigen::Matrix<double, 7, 1> limitTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_desired,
    const Eigen::Matrix<double, 7, 1>& tau_previous,
    double delta_tau_max) {

  if (delta_tau_max <= 0.0) {
    return tau_desired;
  }

  Eigen::Matrix<double, 7, 1> tau_limited;

  for (int i = 0; i < 7; ++i) {
    double delta_tau = tau_desired(i) - tau_previous(i);
    delta_tau = std::max(-delta_tau_max, std::min(delta_tau_max, delta_tau));
    tau_limited(i) = tau_previous(i) + delta_tau;
  }

  return tau_limited;
}

double smoothStep(double r) {
  r = std::max(0.0, std::min(1.0, r));
  return 10.0 * std::pow(r, 3) - 15.0 * std::pow(r, 4) + 6.0 * std::pow(r, 5);
}

double smoothStepDerivative(double r, double T) {
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

Eigen::Vector3d orientationError(
    const Eigen::Matrix3d& R_current,
    const Eigen::Matrix3d& R_desired) {

  Eigen::Matrix3d R_error = R_current.transpose() * R_desired;
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  return R_current * angle_axis.axis() * angle_axis.angle();
}

// Add a small direction-dependent compensation component.
// For position, this returns a force.
// For rotation, this returns a moment.
//
// Important sign convention:
// e = desired - measured.
// If e_i > 0, the restoring command must act in the positive axis direction.
// If e_i < 0, the restoring command must act in the negative axis direction.
//
// Therefore:
//   e_i > +threshold -> add +fric_pos_i
//   e_i < -threshold -> add -fric_neg_i
Eigen::Vector3d addAsymmetricRestoringComponent(
    const Eigen::Vector3d& command,
    const Eigen::Vector3d& error,
    const Eigen::Vector3d& threshold,
    const Eigen::Vector3d& friction_pos,
    const Eigen::Vector3d& friction_neg) {

  Eigen::Vector3d result = command;

  for (int i = 0; i < 3; ++i) {
    if (error(i) > threshold(i) && friction_pos(i) > 0.0) {
      result(i) += friction_pos(i);
    } else if (error(i) < -threshold(i) && friction_neg(i) > 0.0) {
      result(i) -= friction_neg(i);
    }
  }

  return result;
}

void writeLogToCsv(
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
           << "f_before_deadzone_x,f_before_deadzone_y,f_before_deadzone_z,"
           << "f_x,f_y,f_z,"
           << "m_before_deadzone_x,m_before_deadzone_y,m_before_deadzone_z,"
           << "m_x,m_y,m_z,"
           << "tau_raw_1,tau_raw_2,tau_raw_3,tau_raw_4,tau_raw_5,tau_raw_6,tau_raw_7,"
           << "tau_limited_1,tau_limited_2,tau_limited_3,tau_limited_4,"
           << "tau_limited_5,tau_limited_6,tau_limited_7"
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

             << row.f_before_deadzone(0) << "," << row.f_before_deadzone(1) << "," << row.f_before_deadzone(2) << ","
             << row.f_after_deadzone(0) << "," << row.f_after_deadzone(1) << "," << row.f_after_deadzone(2) << ","

             << row.m_before_deadzone(0) << "," << row.m_before_deadzone(1) << "," << row.m_before_deadzone(2) << ","
             << row.m_after_deadzone(0) << "," << row.m_after_deadzone(1) << "," << row.m_after_deadzone(2) << ","

             << row.tau_raw(0) << "," << row.tau_raw(1) << ","
             << row.tau_raw(2) << "," << row.tau_raw(3) << ","
             << row.tau_raw(4) << "," << row.tau_raw(5) << ","
             << row.tau_raw(6) << ","

             << row.tau_limited(0) << "," << row.tau_limited(1) << ","
             << row.tau_limited(2) << "," << row.tau_limited(3) << ","
             << row.tau_limited(4) << "," << row.tau_limited(5) << ","
             << row.tau_limited(6)
             << "\n";
  }
}

int main() {
  try {
    Parameters params = readParameters("parameters.txt");

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Robot IP: " << params.robot_ip << std::endl;
    std::cout << "Experiment duration: " << params.experiment_duration << " s" << std::endl;
    std::cout << "CSV file: " << params.csv_file_name << std::endl;
    std::cout << "use_current_pose: " << params.use_current_pose << std::endl;
    std::cout << "virtual_yz_plane_mode: " << params.virtual_yz_plane_mode << std::endl;
    if (params.virtual_yz_plane_mode) {
      std::cout << "bounded Y-Z workspace [m]: "
                << "y in [-" << params.y_limit_neg << ", +" << params.y_limit_pos << "], "
                << "z in [-" << params.z_limit_down << ", +" << params.z_limit_up << "] relative to start"
                << std::endl;
    }
    std::cout << "delta_p [m]: " << params.delta_p.transpose() << std::endl;
    std::cout << "f_max [N]: " << params.f_max << std::endl;
    std::cout << "m_max [Nm]: " << params.m_max << std::endl;
    std::cout << "delta_tau_max [Nm/cycle]: " << params.delta_tau_max << std::endl;
    std::cout << "e_thresh_p [m]: " << params.e_thresh_p.transpose() << std::endl;
    std::cout << "f_fric_pos [N] for e_p > threshold:  " << params.f_fric_pos.transpose() << std::endl;
    std::cout << "f_fric_neg [N] for e_p < -threshold: " << params.f_fric_neg.transpose() << std::endl;
    std::cout << "e_thresh_R [rad]: " << params.e_thresh_R.transpose() << std::endl;
    std::cout << "m_fric_pos [Nm] for e_R > threshold:  " << params.m_fric_pos.transpose() << std::endl;
    std::cout << "m_fric_neg [Nm] for e_R < -threshold: " << params.m_fric_neg.transpose() << std::endl;

    franka::Robot robot(params.robot_ip);

    std::cout << "\nRecovery step:" << std::endl;
    std::cout << "If the robot is in an error/reflex state, automatic recovery will be attempted." << std::endl;
    std::cout << "Make sure the workspace is clear and the emergency stop is reachable." << std::endl;
    std::cout << "Press Enter to attempt recovery and continue..." << std::endl;
    std::cin.ignore();

    try {
      robot.automaticErrorRecovery();
      std::cout << "Automatic error recovery finished or was not necessary." << std::endl;
    } catch (const franka::Exception& e) {
      std::cerr << "Automatic error recovery failed: " << e.what() << std::endl;
      std::cerr << "Please recover/unlock the robot manually in Franka Desk." << std::endl;
      return -1;
    }

    setDefaultBehavior(robot);

    if (params.use_custom_collision_behavior) {
      std::cout << "Applying custom robot.setCollisionBehavior(...) thresholds." << std::endl;
      std::cout << "Important: this changes contact/collision thresholds, not joint velocity limits." << std::endl;

      robot.setCollisionBehavior(
          params.collision_torque_lower_acc,
          params.collision_torque_upper_acc,
          params.collision_torque_lower_nom,
          params.collision_torque_upper_nom,
          params.collision_force_lower_acc,
          params.collision_force_upper_acc,
          params.collision_force_lower_nom,
          params.collision_force_upper_nom);
    } else {
      std::cout << "Using setDefaultBehavior(robot) collision thresholds only." << std::endl;
    }

    // Move to the initial joint configuration from parameters.txt.
    // The speed factor is intentionally conservative for safe testing.
    MotionGenerator motion_generator(0.4, params.q_goal);

    std::cout << "\nWARNING: The robot will move to the initial joint configuration." << std::endl;
    std::cout << "Make sure the workspace is free and the emergency stop is available." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    robot.control(motion_generator);

    std::cout << "Finished moving to initial joint configuration." << std::endl;

    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_initial(
        initial_state.O_T_EE.data());

    Eigen::Vector3d p_start = T_initial.block<3, 1>(0, 3);
    Eigen::Matrix3d R_d = T_initial.block<3, 3>(0, 0);

    // Columns of R_d are the local end-effector axes expressed in the robot base frame.
    Eigen::Vector3d tool_x_axis = R_d.col(0);
    Eigen::Vector3d tool_y_axis = R_d.col(1);
    Eigen::Vector3d tool_z_axis = R_d.col(2);

    Eigen::Vector3d p_end = p_start;

    if (!params.use_current_pose) {
      p_end = p_start + params.delta_p;
    }

    std::cout << "Initial position p_start [m]: " << p_start.transpose() << std::endl;
    std::cout << "Final target p_end [m]:       " << p_end.transpose() << std::endl;
    std::cout << "Tool x-axis in base frame:    " << tool_x_axis.transpose() << std::endl;
    std::cout << "Tool y-axis in base frame:    " << tool_y_axis.transpose() << std::endl;
    std::cout << "Tool z-axis in base frame:    " << tool_z_axis.transpose() << std::endl;
    if (params.virtual_yz_plane_mode) {
      std::cout << "Bounded virtual Y-Z plane active: x is constrained; y/z are free only inside workspace bounds." << std::endl;
    }

    Eigen::Matrix3d Kp = params.Kp_diag.asDiagonal();
    Eigen::Matrix3d Dp = params.Dp_diag.asDiagonal();

    Eigen::Matrix3d KR = params.KR_diag.asDiagonal();
    Eigen::Matrix3d DR = params.DR_diag.asDiagonal();

    std::vector<LogData> log_data;
    log_data.reserve(static_cast<std::size_t>(params.experiment_duration * 1500.0));

    double time = 0.0;

    Eigen::Matrix<double, 7, 1> tau_previous;
    tau_previous.setZero();

    Eigen::Vector3d final_p_EE = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_p_d = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_R = Eigen::Vector3d::Zero();

    std::cout << "Starting bounded virtual Y-Z plane impedance controller." << std::endl;

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      time += period.toSec();

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          dq(state.dq.data());

      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>>
          J(jacobian_array.data());

      Eigen::Matrix<double, 6, 1> xdot = J * dq;

      Eigen::Vector3d pdot = xdot.head<3>();
      Eigen::Vector3d omega = xdot.tail<3>();

      Eigen::Map<const Eigen::Matrix<double, 4, 4>>
          T_EE(state.O_T_EE.data());

      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);
      Eigen::Matrix3d R_EE = T_EE.block<3, 3>(0, 0);

      Eigen::Vector3d p_d = p_start;
      Eigen::Vector3d pdot_d = Eigen::Vector3d::Zero();

      if (!params.use_current_pose) {
        double r = time / params.trajectory_duration;
        double s = smoothStep(r);
        double s_dot = smoothStepDerivative(r, params.trajectory_duration);

        p_d = p_start + s * params.delta_p;
        pdot_d = s_dot * params.delta_p;
      }

      if (params.virtual_yz_plane_mode) {
        // Bounded virtual Y-Z plane:
        // x is always constrained to x_start.
        p_d(0) = p_start(0);

        // y and z are allowed only inside a box around the start pose.
        // Inside the box:
        //   p_d_y = p_EE_y and p_d_z = p_EE_z, so e_y = e_z = 0.
        // Outside the box:
        //   p_d_y or p_d_z is clamped to the boundary, so the controller pushes back.
        const double y_min = p_start(1) - params.y_limit_neg;
        const double y_max = p_start(1) + params.y_limit_pos;
        const double z_min = p_start(2) - params.z_limit_down;
        const double z_max = p_start(2) + params.z_limit_up;

        p_d(1) = clampValue(p_EE(1), y_min, y_max);
        p_d(2) = clampValue(p_EE(2), z_min, z_max);

        // Desired velocity is zero. This gives damping in all directions.
        pdot_d(0) = 0.0;
        pdot_d(1) = 0.0;
        pdot_d(2) = 0.0;
      }

      Eigen::Vector3d e_p = p_d - p_EE;
      Eigen::Vector3d e_R = orientationError(R_EE, R_d);

      Eigen::Vector3d f =
          Kp * e_p + Dp * (pdot_d - pdot);

      Eigen::Vector3d m =
          KR * e_R - DR * omega;

      Eigen::Vector3d f_before_deadzone = f;
      Eigen::Vector3d m_before_deadzone = m;

      f = addAsymmetricRestoringComponent(
          f, e_p, params.e_thresh_p, params.f_fric_pos, params.f_fric_neg);

      m = addAsymmetricRestoringComponent(
          m, e_R, params.e_thresh_R, params.m_fric_pos, params.m_fric_neg);

      f = limitVectorNorm(f, params.f_max);
      m = limitVectorNorm(m, params.m_max);

      Eigen::Matrix<double, 6, 1> wrench;
      wrench.head<3>() = f;
      wrench.tail<3>() = m;

      Eigen::Matrix<double, 7, 1> tau_task =
          J.transpose() * wrench;

      std::array<double, 7> coriolis_array =
          model.coriolis(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          coriolis(coriolis_array.data());

      Eigen::Matrix<double, 7, 1> tau_raw =
          tau_task + coriolis;

      Eigen::Matrix<double, 7, 1> tau_limited =
          limitTorqueRate(tau_raw, tau_previous, params.delta_tau_max);

      tau_previous = tau_limited;

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

      row.f_before_deadzone = f_before_deadzone;
      row.f_after_deadzone = f;

      row.m_before_deadzone = m_before_deadzone;
      row.m_after_deadzone = m;

      row.tau_raw = tau_raw;
      row.tau_limited = tau_limited;

      log_data.push_back(row);

      final_p_EE = p_EE;
      final_p_d = p_d;
      final_e_p = e_p;
      final_e_R = e_R;

      std::array<double, 7> tau_array = eigenToArray(tau_limited);

      if (time >= params.experiment_duration) {
        return franka::MotionFinished(franka::Torques(tau_array));
      }

      return franka::Torques(tau_array);
    });

    writeLogToCsv(log_data, params.csv_file_name);

    std::cout << "\nExperiment finished." << std::endl;
    std::cout << "Final desired position p_d [m]:  " << final_p_d.transpose() << std::endl;
    std::cout << "Final reached position p_EE [m]: " << final_p_EE.transpose() << std::endl;
    std::cout << "Final position error e_p [m]:    " << final_e_p.transpose() << std::endl;
    std::cout << "Final position error norm [m]:   " << final_e_p.norm() << std::endl;
    std::cout << "Final rotation error e_R [rad]:  " << final_e_R.transpose() << std::endl;
    std::cout << "Final rotation error norm [rad]: " << final_e_R.norm() << std::endl;
    std::cout << "CSV log written to: " << params.csv_file_name << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    std::cerr << "If the robot is still in an error/reflex state, recover it manually in Franka Desk." << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
