/*
  General Axis-Constraint Cartesian Impedance Controller
  ------------------------------------------------------
  This file runs a Cartesian impedance controller on a Franka Panda.

  Frames and vector ordering:
    - All Cartesian positions are expressed in the robot base frame O.
    - Franka's end-effector pose O_T_EE is a 4x4 transform from base O to EE.
    - The geometric Jacobian J has size 6x7 and maps joint velocity to twist:

          xdot = J(q) * dq

      where:
          dq    [rad/s]         = joint velocity
          xdot  [m/s, rad/s]    = [pdot_x, pdot_y, pdot_z,
                                   omega_x, omega_y, omega_z]^T

  Basic impedance law:

      e_p = p_d - p_EE                         [m]
      e_R = orientation error                  [rad]

      f = Kp * e_p + Dp * (pdot_d - pdot)      [N]
      m = KR * e_R - DR * omega                [Nm]

      Kp [N/m], Dp [Ns/m], KR [Nm/rad], DR [Nms/rad]

  Wrench-to-torque mapping:

      wrench   = [f_x, f_y, f_z, m_x, m_y, m_z]^T  [N, Nm]
      tau_task = J^T * wrench                      [Nm]

  Optional additions:
    - Axis constraints: selected Cartesian axes are held, while free axes follow
      the measured pose so they do not generate spring force.
    - Virtual-center self-alignment: orientation error can be converted into a
      coupled position error around a virtual center p_c:

          p_c = surface_point + vcr_offset * surface_normal      [m]
          r_c = p_EE - p_c                                       [m]
          e_p <- e_p - e_R x r_c                                 [m]

      This makes the end-effector behave as if it rotates around p_c.
    - Asymmetric residual/friction compensation: a constant force/moment can be
      added only when an error threshold is exceeded.
    - Active-task nullspace damping and sigma_min preference.
    - Controller-side force, moment and torque-rate limits.

  Safety note:
    Controller-side limits can be disabled for testing, but Franka internal
    safety/reflex limits remain active and can still abort motion.
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

#include <atomic>
#include <thread>
#include "examples_common.h"

struct Parameters {
  // Franka controller network address.
  std::string robot_ip = "172.16.0.2";

  // Impedance-loop runtime [s]. If <= 0, the loop runs until the user types e.
  double experiment_duration = 10.0;

  // Output CSV path, relative to the directory where the executable runs.
  std::string csv_file_name = "general_axis_constraint_active_nullspace_sigma_only_no_limits_no_limits_log.csv";

  // Test switch:
  //   true  -> tau_cmd = tau_task + tau_nullspace + coriolis  [Nm]
  //   false -> tau_cmd = tau_task + tau_nullspace             [Nm]
  bool use_coriolis = true;

  // true  -> hold the measured start pose, modified only by axis constraints.
  // false -> follow a smooth relative trajectory p_start -> p_start + delta_p.
  bool use_current_pose = true;

  // General axis constraint mode:
  // Translation:
  //   fix_p_i = true  -> p_d_i = p_start_i, axis is fixed/constrained
  //   fix_p_i = false -> p_d_i = p_EE_i, axis is free in position
  //
  // Examples:
  //   fix_p_x=1, fix_p_y=0, fix_p_z=0 -> Y-Z plane
  //   fix_p_x=0, fix_p_y=1, fix_p_z=0 -> X-Z plane
  //   fix_p_x=0, fix_p_y=0, fix_p_z=1 -> X-Y plane
  bool axis_constraint_mode = true;
  bool fix_p_x = true;
  bool fix_p_y = false;
  bool fix_p_z = false;

  // Rotation:
  //   fix_R_i = true  -> keep rotational stiffness in this error component
  //   fix_R_i = false -> set e_R_i = 0, no rotational spring in this component
  //
  // Damping still acts if DR_i > 0. For a fully free rotational component,
  // set fix_R_i = 0 and DR_i = 0.
  bool fix_R_x = true;
  bool fix_R_y = true;
  bool fix_R_z = true;

  // Virtual-center self-alignment.
  // When enabled, the desired tool z-axis is aligned with surface_normal and
  // rotational error is coupled into translational error around
  // p_c = surface_point + vcr_offset * surface_normal [m].
  bool use_virtual_center = false;

  // A point on the surface in the robot base frame [m].
  Eigen::Vector3d surface_point = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Surface normal in the robot base frame [-]. It is normalized before use.
  Eigen::Vector3d surface_normal = Eigen::Vector3d(0.0, 0.0, 1.0);

  // Signed virtual-center offset along surface_normal [m].
  //   0.0 -> center on the surface
  //   >0  -> center above the surface along surface_normal
  //   <0  -> center below the surface opposite surface_normal
  double vcr_offset = 0.0;

  // If true, use the measured start pose p_start as the surface point. This
  // keeps r_c small during early tests and reduces self-collision reflex risk.
  bool use_start_as_surface_point = true;

  // Maximum allowed distance between configured surface_point and p_start [m].
  // If exceeded, p_start is used as the active surface point.
  double max_surface_point_start_distance = 0.10;

  // Nullspace/SVD optimization.
  // The 6x7 Jacobian has one redundant joint-space direction.
  // This term damps motion inside the active-task nullspace and can bias motion
  // toward a larger smallest singular value sigma_min(J_active).
  bool use_nullspace_optimization = true;

  // Nullspace damping gain [Nm/(rad/s)] in tau_ns = -damping * N * dq.
  double nullspace_damping = 1.0;

  // Weight for the sigma_min preference term [Nm/rad approximately].
  double nullspace_k_sigma = 0.5;

  // Small finite-difference joint step used for sigma_min comparison [rad].
  double nullspace_alpha = 0.01;

  // Maximum Euclidean norm of added nullspace torque [Nm].
  double nullspace_tau_max = 2.0;

  // Damping factor for damped least-squares active-task pseudoinverse [-].
  double active_nullspace_lambda = 0.05;

  // Relative Cartesian trajectory displacement [m], used only when
  // use_current_pose == false.
  Eigen::Vector3d delta_p = Eigen::Vector3d(0.005, 0.0, 0.0);

  // Duration of the smooth relative trajectory [s].
  double trajectory_duration = 8.0;

  // Translational stiffness diagonal [N/m].
  Eigen::Vector3d Kp_diag = Eigen::Vector3d(150.0, 100.0, 100.0);

  // Translational damping diagonal [Ns/m].
  Eigen::Vector3d Dp_diag = Eigen::Vector3d(25.0, 20.0, 20.0);

  // Rotational stiffness diagonal [Nm/rad].
  Eigen::Vector3d KR_diag = Eigen::Vector3d(3.0, 3.0, 3.0);

  // Rotational damping diagonal [Nms/rad].
  Eigen::Vector3d DR_diag = Eigen::Vector3d(3.5, 3.5, 3.5);

  // Controller-side force saturation limit ||f|| <= f_max [N].
  double f_max = 10.0;

  // Controller-side moment saturation limit ||m|| <= m_max [Nm].
  double m_max = 2.0;

  // Per-joint torque-rate step limit per 1 kHz control cycle [Nm/cycle].
  double delta_tau_max = 2.0;

  // true bypasses f_max, m_max and delta_tau_max in this controller.
  bool disable_controller_side_limits = true;

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

  // Joint torque reflex thresholds for acceleration and nominal phases [Nm].
  std::array<double, 7> collision_torque_lower_acc = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  std::array<double, 7> collision_torque_upper_acc = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  std::array<double, 7> collision_torque_lower_nom = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};
  std::array<double, 7> collision_torque_upper_nom = {{30.0, 30.0, 28.0, 28.0, 25.0, 25.0, 25.0}};

  // Cartesian collision thresholds. First three entries are forces [N];
  // last three entries are moments [Nm].
  std::array<double, 6> collision_force_lower_acc = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  std::array<double, 6> collision_force_upper_acc = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  std::array<double, 6> collision_force_lower_nom = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};
  std::array<double, 6> collision_force_upper_nom = {{30.0, 30.0, 30.0, 20.0, 20.0, 20.0}};

  // Position-error threshold before residual compensation is added [m].
  Eigen::Vector3d e_thresh_p = Eigen::Vector3d(0.003, 0.003, 0.003);

  // Direction-dependent positional compensation.
  // f_fric_pos is used when e_p(i) > +threshold [N].
  // f_fric_neg is used when e_p(i) < -threshold [N].
  Eigen::Vector3d f_fric_pos = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d f_fric_neg = Eigen::Vector3d(0.5, 0.5, 0.5);

  // Rotation-error threshold before residual compensation is added [rad].
  Eigen::Vector3d e_thresh_R = Eigen::Vector3d(0.03, 0.03, 0.03);

  // Direction-dependent rotational compensation.
  // m_fric_pos is used when e_R(i) > +threshold [Nm].
  // m_fric_neg is used when e_R(i) < -threshold [Nm].
  Eigen::Vector3d m_fric_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d m_fric_neg = Eigen::Vector3d(0.0, 0.0, 0.0);
};

struct LogData {
  // One CSV sample from the control loop. Units match the variable comments
  // below and the CSV header written in writeLogToCsv().
  double time;

  // Current, desired, and final target position in base frame [m].
  Eigen::Vector3d p_EE;
  Eigen::Vector3d p_d;
  Eigen::Vector3d p_end;

  // Translational and rotational errors [m] and [rad].
  Eigen::Vector3d e_p;
  Eigen::Vector3d e_R;

  // Current and desired linear velocity [m/s], current angular velocity [rad/s].
  Eigen::Vector3d pdot;
  Eigen::Vector3d pdot_d;
  Eigen::Vector3d omega;

  // Cartesian force before and after residual compensation/saturation [N].
  Eigen::Vector3d f_before_deadzone;
  Eigen::Vector3d f_after_deadzone;

  // Cartesian moment before and after residual compensation/saturation [Nm].
  Eigen::Vector3d m_before_deadzone;
  Eigen::Vector3d m_after_deadzone;

  // Joint torque before and after controller-side torque-rate limiting [Nm].
  Eigen::Matrix<double, 7, 1> tau_raw;
  Eigen::Matrix<double, 7, 1> tau_limited;
};

std::array<double, 7> eigenToArray(const Eigen::Matrix<double, 7, 1>& tau) {
  // libfranka expects std::array<double, 7>; the controller math uses Eigen.
  std::array<double, 7> tau_array{};
  for (int i = 0; i < 7; ++i) {
    tau_array[i] = tau(i);
  }
  return tau_array;
}


std::array<double, 7> eigenQToArray(const Eigen::Matrix<double, 7, 1>& q) {
  // Convert Eigen joint vector q [rad] for libfranka model calls.
  std::array<double, 7> q_array{};
  for (int i = 0; i < 7; ++i) {
    q_array[i] = q(i);
  }
  return q_array;
}

double smallestSingularValue(const Eigen::Matrix<double, 6, 7>& J) {
  // sigma_min(J) [-] is a manipulability/singularity indicator. Larger values
  // mean the task Jacobian is farther from losing rank.
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.singularValues().minCoeff();
}

Eigen::Matrix3d desiredOrientationFromSurfaceNormal(
    const Eigen::Vector3d& surface_normal,
    const Eigen::Matrix3d& fallback_orientation) {
  // Build a desired rotation matrix R_d_surface from a surface normal.
  // Convention used here:
  //   z_d = normalized surface normal [-]
  //   x_d = any unit vector perpendicular to z_d [-]
  //   y_d = z_d x x_d [-]
  // The resulting R_d_surface columns are [x_d y_d z_d].
  const double normal_norm = surface_normal.norm();
  if (normal_norm < 1e-9) {
    // Degenerate normal: keep the original desired orientation.
    return fallback_orientation;
  }

  Eigen::Vector3d z_d = surface_normal / normal_norm;

  // Choose a reference axis that is not almost parallel to z_d. This prevents
  // the projection below from becoming numerically tiny.
  Eigen::Vector3d reference_axis(0.0, 0.0, 1.0);
  if (std::abs(z_d.dot(reference_axis)) > 0.95) {
    reference_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  // Project reference_axis into the plane perpendicular to z_d:
  //   x_d = a - z_d * (z_d^T a)
  Eigen::Vector3d x_d = reference_axis - z_d * z_d.dot(reference_axis);
  x_d.normalize();

  Eigen::Vector3d y_d = z_d.cross(x_d);
  y_d.normalize();

  Eigen::Matrix3d R_d_surface;
  R_d_surface.col(0) = x_d;
  R_d_surface.col(1) = y_d;
  R_d_surface.col(2) = z_d;
  return R_d_surface;
}


Eigen::MatrixXd buildActiveTaskJacobian(
    const Eigen::Matrix<double, 6, 7>& J,
    const Parameters& params) {
  // Build J_active by keeping only constrained task rows.
  // Row order in Franka's Jacobian:
  //   rows 0..2 -> translational velocity [m/s]
  //   rows 3..5 -> angular velocity [rad/s]
  // J_active is used for the nullspace projector, so free axes are excluded.
  int rows = 0;
  if (params.fix_p_x) ++rows;
  if (params.fix_p_y) ++rows;
  if (params.fix_p_z) ++rows;
  if (params.fix_R_x) ++rows;
  if (params.fix_R_y) ++rows;
  if (params.fix_R_z) ++rows;

  Eigen::MatrixXd J_active(rows, 7);

  int r = 0;
  if (params.fix_p_x) {
    J_active.row(r++) = J.row(0);
  }
  if (params.fix_p_y) {
    J_active.row(r++) = J.row(1);
  }
  if (params.fix_p_z) {
    J_active.row(r++) = J.row(2);
  }
  if (params.fix_R_x) {
    J_active.row(r++) = J.row(3);
  }
  if (params.fix_R_y) {
    J_active.row(r++) = J.row(4);
  }
  if (params.fix_R_z) {
    J_active.row(r++) = J.row(5);
  }

  return J_active;
}

Eigen::Matrix<double, 7, 7> activeTaskNullspaceProjector(
    const Eigen::MatrixXd& J_active,
    double damping_lambda) {
  // Damped least-squares pseudoinverse:
  //
  //   J_active# = J_active^T * (J_active * J_active^T + lambda^2 I)^-1
  //
  // Nullspace projector:
  //
  //   N = I - J_active# * J_active
  //
  // N maps joint torques/velocities into directions that minimally disturb
  // the active constrained Cartesian task.
  Eigen::Matrix<double, 7, 7> I =
      Eigen::Matrix<double, 7, 7>::Identity();

  if (J_active.rows() == 0) {
    return I;
  }

  Eigen::MatrixXd A =
      J_active * J_active.transpose()
      + damping_lambda * damping_lambda
            * Eigen::MatrixXd::Identity(J_active.rows(), J_active.rows());

  Eigen::MatrixXd J_pseudo =
      J_active.transpose() * A.inverse();

  Eigen::Matrix<double, 7, 7> N =
      I - J_pseudo * J_active;

  return N;
}

std::string trim(const std::string& input) {
  // Remove leading/trailing whitespace from key=value parameter strings.
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

Parameters readParameters(const std::string& filename) {
  // Read a simple parameters.txt file with lines of the form:
  //   key = value
  // Inline comments after '#' are ignored.
  Parameters p;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cout << "Could not open " << filename << ". Using defaults." << std::endl;
    return p;
  }

  std::map<std::string, std::string> values;
  std::string line;

  while (std::getline(file, line)) {
    // Strip comments before parsing.
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    // Skip blank or malformed lines.
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
    // Keep the default if the key is missing.
    return values.count(key) ? values[key] : def;
  };
  auto getDouble = [&](const std::string& key, double def) {
    // Invalid numeric strings intentionally throw, so bad parameters are visible.
    return values.count(key) ? std::stod(values[key]) : def;
  };
  auto getBool = [&](const std::string& key, bool def) {
    // Boolean parameters are written as 0 or 1.
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

  p.fix_R_x = getBool("fix_R_x", p.fix_R_x);
  p.fix_R_y = getBool("fix_R_y", p.fix_R_y);
  p.fix_R_z = getBool("fix_R_z", p.fix_R_z);

  p.use_virtual_center = getBool("use_virtual_center", p.use_virtual_center);
  p.surface_point(0) = getDouble("surface_point_x", p.surface_point(0));
  p.surface_point(1) = getDouble("surface_point_y", p.surface_point(1));
  p.surface_point(2) = getDouble("surface_point_z", p.surface_point(2));
  p.surface_normal(0) = getDouble("surface_normal_x", p.surface_normal(0));
  p.surface_normal(1) = getDouble("surface_normal_y", p.surface_normal(1));
  p.surface_normal(2) = getDouble("surface_normal_z", p.surface_normal(2));
  p.vcr_offset = getDouble("vcr_offset", p.vcr_offset);
  p.use_start_as_surface_point =
      getBool("use_start_as_surface_point", p.use_start_as_surface_point);
  p.max_surface_point_start_distance =
      getDouble("max_surface_point_start_distance", p.max_surface_point_start_distance);

  p.use_nullspace_optimization = getBool("use_nullspace_optimization", p.use_nullspace_optimization);
  p.nullspace_damping = getDouble("nullspace_damping", p.nullspace_damping);
  p.nullspace_k_sigma = getDouble("nullspace_k_sigma", p.nullspace_k_sigma);
  p.nullspace_alpha = getDouble("nullspace_alpha", p.nullspace_alpha);
  p.nullspace_tau_max = getDouble("nullspace_tau_max", p.nullspace_tau_max);
  p.active_nullspace_lambda = getDouble("active_nullspace_lambda", p.active_nullspace_lambda);

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
  p.disable_controller_side_limits = getBool("disable_controller_side_limits", p.disable_controller_side_limits);

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
  // Saturate a 3D vector by Euclidean norm:
  //
  //   if ||v|| > max_norm: v_limited = max_norm * v / ||v||
  //
  // Used for Cartesian force [N] and Cartesian moment [Nm].
  if (max_norm <= 0.0) {
    return v;
  }

  const double norm = v.norm();

  if (norm > max_norm && norm > 1e-12) {
    return max_norm * v / norm;
  }

  return v;
}

Eigen::Matrix<double, 7, 1> limitJointTorqueVectorNorm(
    const Eigen::Matrix<double, 7, 1>& v,
    double max_norm) {
  // Saturate the norm of a 7D joint torque vector [Nm]. This limits the added
  // nullspace torque as a whole, without changing its direction.
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
  // Per-joint torque-rate limiter for the discrete 1 kHz control loop:
  //
  //   delta_tau_i = tau_desired_i - tau_previous_i              [Nm]
  //   delta_tau_i <- clamp(delta_tau_i, -delta_tau_max, +delta_tau_max)
  //   tau_limited_i = tau_previous_i + delta_tau_i              [Nm]
  //
  // delta_tau_max has units [Nm/cycle], not [Nm/s].

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
  // Quintic smoothstep trajectory scalar:
  //
  //   s(r) = 10 r^3 - 15 r^4 + 6 r^5,  r in [0, 1]
  //
  // It satisfies s(0)=0, s(1)=1, s_dot(0)=s_dot(1)=0.
  r = std::max(0.0, std::min(1.0, r));
  return 10.0 * std::pow(r, 3) - 15.0 * std::pow(r, 4) + 6.0 * std::pow(r, 5);
}

double smoothStepDerivative(double r, double T) {
  // Time derivative of the smoothstep scalar [1/s]:
  //
  //   ds/dr = 30 r^2 - 60 r^3 + 30 r^4
  //   ds/dt = (ds/dr) / T
  //
  // Multiplying ds/dt by delta_p [m] gives pdot_d [m/s].
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

  // Relative rotation from current orientation to desired orientation:
  //
  //   R_error = R_current^T * R_desired
  //
  // Convert to angle-axis, then express the rotation vector in the base frame:
  //
  //   e_R = R_current * axis(R_error) * angle(R_error)     [rad]
  //
  // The result is compatible with omega from J*dq, which is also base-frame.
  Eigen::Matrix3d R_error = R_current.transpose() * R_desired;
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  return R_current * angle_axis.axis() * angle_axis.angle();
}

// Add a small direction-dependent compensation component.
// For position, this returns a force [N].
// For rotation, this returns a moment [Nm].
//
// Important sign convention:
// e = desired - measured.
// If e_i > 0, the restoring command must act in the positive axis direction.
// If e_i < 0, the restoring command must act in the negative axis direction.
//
// Therefore:
//   e_i > +threshold -> add +fric_pos_i
//   e_i < -threshold -> add -fric_neg_i
//
// Units:
//   position mode: error [m], threshold [m], friction_* [N]
//   rotation mode: error [rad], threshold [rad], friction_* [Nm]
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

  // Write buffered log data after the real-time control loop exits. Avoiding
  // file I/O inside robot.control() keeps the callback lighter and more stable.
  std::ofstream log_file(csv_file_name);

  // CSV columns and units:
  //   time [s]
  //   p_* and e_p [m]
  //   e_R [rad]
  //   pdot/pdot_d [m/s], omega [rad/s]
  //   f_* [N], m_* [Nm], tau_* [Nm]
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
    // Load all controller and experiment settings from parameters.txt.
    Parameters params = readParameters("parameters.txt");

    // Print the resolved parameters before the robot moves. This is useful
    // because missing parameters silently keep their defaults.
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Robot IP: " << params.robot_ip << std::endl;
    std::cout << "Experiment duration: " << params.experiment_duration << " s" << std::endl;
    if (params.experiment_duration <= 0.0) {
      std::cout << "Time mode: indefinite run, stop with e + Enter." << std::endl;
    } else {
      std::cout << "Time mode: automatic stop after experiment_duration, or earlier with e + Enter." << std::endl;
    }
    std::cout << "CSV file: " << params.csv_file_name << std::endl;
    std::cout << "use_coriolis: " << params.use_coriolis << std::endl;
    std::cout << "use_nullspace_optimization: " << params.use_nullspace_optimization << std::endl;

    if (params.use_coriolis && params.use_nullspace_optimization) {
      std::cout << "Torque command mode: tau_cmd = J^T * F + tau_active_nullspace + coriolis" << std::endl;
    } else if (params.use_coriolis && !params.use_nullspace_optimization) {
      std::cout << "Torque command mode: tau_cmd = J^T * F + coriolis" << std::endl;
    } else if (!params.use_coriolis && params.use_nullspace_optimization) {
      std::cout << "Torque command mode: tau_cmd = J^T * F + tau_active_nullspace  (NO coriolis)" << std::endl;
    } else {
      std::cout << "Torque command mode: tau_cmd = J^T * F  (NO coriolis, NO nullspace)" << std::endl;
    }
    std::cout << "use_current_pose: " << params.use_current_pose << std::endl;
    std::cout << "axis_constraint_mode: " << params.axis_constraint_mode << std::endl;
    std::cout << "fix_p [x y z]: " << params.fix_p_x << " " << params.fix_p_y << " " << params.fix_p_z << std::endl;
    std::cout << "fix_R [x y z]: " << params.fix_R_x << " " << params.fix_R_y << " " << params.fix_R_z << std::endl;
    std::cout << "use_virtual_center: " << params.use_virtual_center << std::endl;
    if (params.use_virtual_center) {
      std::cout << "surface_point [m]: " << params.surface_point.transpose() << std::endl;
      std::cout << "surface_normal: " << params.surface_normal.transpose() << std::endl;
      std::cout << "vcr_offset [m]: " << params.vcr_offset << std::endl;
      std::cout << "use_start_as_surface_point: " << params.use_start_as_surface_point << std::endl;
      std::cout << "max_surface_point_start_distance [m]: "
                << params.max_surface_point_start_distance << std::endl;
    }
    std::cout << "use_nullspace_optimization: " << params.use_nullspace_optimization << std::endl;
    std::cout << "nullspace_damping: " << params.nullspace_damping << std::endl;
    std::cout << "nullspace_k_sigma: " << params.nullspace_k_sigma << std::endl;
    std::cout << "nullspace_alpha [rad]: " << params.nullspace_alpha << std::endl;
    std::cout << "nullspace_tau_max [Nm]: " << params.nullspace_tau_max << std::endl;
    std::cout << "active_nullspace_lambda: " << params.active_nullspace_lambda << std::endl;
    std::cout << "delta_p [m]: " << params.delta_p.transpose() << std::endl;
    std::cout << "f_max [N]: " << params.f_max << std::endl;
    std::cout << "m_max [Nm]: " << params.m_max << std::endl;
    std::cout << "delta_tau_max [Nm/cycle]: " << params.delta_tau_max << std::endl;
    std::cout << "disable_controller_side_limits: " << params.disable_controller_side_limits << std::endl;
    if (params.disable_controller_side_limits) {
      std::cout << "Controller-side f_max, m_max and delta_tau_max limiters are DISABLED." << std::endl;
      std::cout << "Franka internal safety/reflex limits are still active." << std::endl;
    }
    std::cout << "e_thresh_p [m]: " << params.e_thresh_p.transpose() << std::endl;
    std::cout << "f_fric_pos [N] for e_p > threshold:  " << params.f_fric_pos.transpose() << std::endl;
    std::cout << "f_fric_neg [N] for e_p < -threshold: " << params.f_fric_neg.transpose() << std::endl;
    std::cout << "e_thresh_R [rad]: " << params.e_thresh_R.transpose() << std::endl;
    std::cout << "m_fric_pos [Nm] for e_R > threshold:  " << params.m_fric_pos.transpose() << std::endl;
    std::cout << "m_fric_neg [Nm] for e_R < -threshold: " << params.m_fric_neg.transpose() << std::endl;

    franka::Robot robot(params.robot_ip);

    // The robot may remain in a reflex/error state after a previous abort.
    // automaticErrorRecovery() attempts to clear that state before motion.
    std::cout << "\nStartup confirmation:" << std::endl;
    std::cout << "If the robot is in an error/reflex state, automatic recovery will be attempted." << std::endl;
    std::cout << "After recovery, the robot will move to the initial joint configuration." << std::endl;
    std::cout << "Make sure the workspace is clear and the emergency stop is reachable." << std::endl;
    std::cout << "Press Enter once to recover if needed and move to the initial joint configuration..." << std::endl;
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

    std::cout << "\nMoving to the initial joint configuration after the single startup confirmation..." << std::endl;

    robot.control(motion_generator);

    std::cout << "Finished moving to initial joint configuration." << std::endl;

    franka::Model model = robot.loadModel();

    // Read the actual end-effector pose after the joint-space move. This pose
    // defines p_start [m] and R_d [-], the nominal desired pose.
    franka::RobotState initial_state = robot.readOnce();
    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_initial(
        initial_state.O_T_EE.data());

    Eigen::Vector3d p_start = T_initial.block<3, 1>(0, 3);
    Eigen::Matrix3d R_d = T_initial.block<3, 3>(0, 0);

    // Columns of R_d are the local end-effector axes expressed in the robot base frame.
    Eigen::Vector3d tool_x_axis = R_d.col(0);
    Eigen::Vector3d tool_y_axis = R_d.col(1);
    Eigen::Vector3d tool_z_axis = R_d.col(2);

    // p_end is only used for printing/logging. In static holding mode it equals
    // p_start; in trajectory mode it is p_start + delta_p [m].
    Eigen::Vector3d p_end = p_start;

    if (!params.use_current_pose) {
      p_end = p_start + params.delta_p;
    }

    std::cout << "Initial position p_start [m]: " << p_start.transpose() << std::endl;
    std::cout << "Final target p_end [m]:       " << p_end.transpose() << std::endl;
    std::cout << "Tool x-axis in base frame:    " << tool_x_axis.transpose() << std::endl;
    std::cout << "Tool y-axis in base frame:    " << tool_y_axis.transpose() << std::endl;
    std::cout << "Tool z-axis in base frame:    " << tool_z_axis.transpose() << std::endl;
    if (params.axis_constraint_mode) {
      std::cout << "General axis constraint active." << std::endl;
      std::cout << "Translation: fixed axes return to start; free axes follow current position." << std::endl;
      std::cout << "Rotation: fixed components keep rotational stiffness; free components have no rotational spring." << std::endl;
    }

    Eigen::Vector3d active_surface_point = params.surface_point;
    if (params.use_virtual_center) {
      // A far-away surface point creates a large lever arm:
      //
      //   r_c = p_EE - p_c       [m]
      //   e_p <- e_p - e_R x r_c [m]
      //
      // Large r_c can turn a moderate rotation error into a large translational
      // correction, which may trigger Franka reflexes. Therefore the controller
      // can choose p_start as the active surface point for safer first tests.
      const double surface_point_distance =
          (params.surface_point - p_start).norm();

      if (params.use_start_as_surface_point ||
          surface_point_distance > params.max_surface_point_start_distance) {
        active_surface_point = p_start;
        std::cout << "Using p_start as virtual-center surface point to avoid a large lever arm." << std::endl;
      } else {
        std::cout << "Using configured virtual-center surface point." << std::endl;
      }

      std::cout << "Active surface_point [m]: " << active_surface_point.transpose() << std::endl;
      std::cout << "Configured surface point distance from p_start [m]: "
                << surface_point_distance << std::endl;
    }

    // Diagonal gain matrices:
    //   Kp [N/m], Dp [Ns/m], KR [Nm/rad], DR [Nms/rad]
    Eigen::Matrix3d Kp = params.Kp_diag.asDiagonal();
    Eigen::Matrix3d Dp = params.Dp_diag.asDiagonal();

    Eigen::Matrix3d KR = params.KR_diag.asDiagonal();
    Eigen::Matrix3d DR = params.DR_diag.asDiagonal();

    // Store log data in memory during the control loop and write it afterwards.
    std::vector<LogData> log_data;
    log_data.reserve(static_cast<std::size_t>(params.experiment_duration * 1500.0));

    double time = 0.0;

    Eigen::Matrix<double, 7, 1> tau_previous;
    tau_previous.setZero();

    Eigen::Vector3d final_p_EE = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_p_d = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_R = Eigen::Vector3d::Zero();

    std::cout << "NO-LIMITS TEST VERSION: controller-side f_max, m_max and delta_tau_max can be disabled." << std::endl;
    std::cout << "Keep the emergency stop reachable and test carefully." << std::endl;
    std::cout << "Starting general axis-constraint impedance controller with active-task sigma-only nullspace, Coriolis switch and e+Enter stop." << std::endl;
    if (params.use_nullspace_optimization) {
      std::cout << "Active-task sigma-only nullspace is enabled: tau_active_nullspace is added to the task torque." << std::endl;
    }

    
    // A detached keyboard thread sets this flag when the user types e + Enter.
    std::atomic<bool> stop_requested(false);

    std::cout << std::endl;
    std::cout << "Early stop option:" << std::endl;
    std::cout << "Type e and press Enter during the impedance run to stop the control loop safely." << std::endl;
    if (params.experiment_duration <= 0.0) {
      std::cout << "experiment_duration <= 0: running indefinitely until e + Enter." << std::endl;
    } else {
      std::cout << "Otherwise, the experiment stops automatically after experiment_duration, or earlier with e + Enter." << std::endl;
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

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      // Elapsed experiment time [s] from libfranka's measured callback period.
      time += period.toSec();

      // Current joint velocity dq [rad/s].
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          dq(state.dq.data());

      // End-effector geometric Jacobian J(q) [6x7].
      // It maps dq to base-frame twist:
      //   [pdot; omega] = J * dq
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>>
          J(jacobian_array.data());

      Eigen::Matrix<double, 6, 1> xdot = J * dq;

      // Split twist into linear velocity [m/s] and angular velocity [rad/s].
      Eigen::Vector3d pdot = xdot.head<3>();
      Eigen::Vector3d omega = xdot.tail<3>();

      // Current end-effector transform O_T_EE and pose in base frame.
      Eigen::Map<const Eigen::Matrix<double, 4, 4>>
          T_EE(state.O_T_EE.data());

      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);
      Eigen::Matrix3d R_EE = T_EE.block<3, 3>(0, 0);

      // Desired position p_d [m] and desired linear velocity pdot_d [m/s].
      // In holding mode, p_d starts as p_start and pdot_d is zero.
      Eigen::Vector3d p_d = p_start;
      Eigen::Vector3d pdot_d = Eigen::Vector3d::Zero();

      if (!params.use_current_pose) {
        // Smooth relative trajectory:
        //   r = t / T                         [-]
        //   p_d = p_start + s(r) * delta_p    [m]
        //   pdot_d = s_dot(r) * delta_p       [m/s]
        double r = time / params.trajectory_duration;
        double s = smoothStep(r);
        double s_dot = smoothStepDerivative(r, params.trajectory_duration);

        p_d = p_start + s * params.delta_p;
        pdot_d = s_dot * params.delta_p;
      }

      if (params.axis_constraint_mode) {
        // General translational axis constraint:
        // fixed axis: desired position is the start value, so it generates
        //             spring force if the robot leaves that coordinate.
        // free axis: desired position follows the current measured position,
        //            so e_p on that axis is zero and no spring force is made.
        //
        // This creates:
        //   one fixed axis  -> virtual plane
        //   two fixed axes  -> virtual line
        //   three fixed axes -> normal return-to-point behavior
        p_d(0) = params.fix_p_x ? p_start(0) : p_EE(0);
        p_d(1) = params.fix_p_y ? p_start(1) : p_EE(1);
        p_d(2) = params.fix_p_z ? p_start(2) : p_EE(2);

        // Desired velocity is zero. Free axes therefore have damping but no spring.
        // For a completely free axis, set the corresponding Dp_i = 0.
        pdot_d(0) = 0.0;
        pdot_d(1) = 0.0;
        pdot_d(2) = 0.0;
      }

      // Translational error:
      //   e_p = p_d - p_EE [m]
      Eigen::Vector3d e_p = p_d - p_EE;
      Eigen::Vector3d e_R;
      Eigen::Matrix3d R_d_used = R_d;

      if (params.use_virtual_center) {
        // Normalize the configured surface normal n_s [-].
        Eigen::Vector3d n_s = params.surface_normal;
        const double n_norm = n_s.norm();
        if (n_norm > 1e-9) {
          n_s /= n_norm;
        } else {
          n_s = R_d.col(2);
        }

        // Virtual center:
        //   p_c = active_surface_point + vcr_offset * n_s [m]
        //
        // vcr_offset = 0   -> on the surface
        // vcr_offset > 0   -> above the surface along n_s
        // vcr_offset < 0   -> below the surface opposite n_s
        const Eigen::Vector3d p_c =
            active_surface_point + params.vcr_offset * n_s;

        // Self-align the tool z-axis with the selected surface normal.
        R_d_used = desiredOrientationFromSurfaceNormal(n_s, R_d);
        e_R = orientationError(R_EE, R_d_used);

        // Coupled virtual-center position error:
        //
        //   r_c = p_EE - p_c             [m]
        //   e_p <- e_p - e_R x r_c       [m]
        //
        // Since e_R is in [rad] and radians are dimensionless in SI, e_R x r_c
        // has units [m]. This term makes rotations produce the translation that
        // would be needed if the tool rotated around p_c.
        const Eigen::Vector3d r_c = p_EE - p_c;
        e_p -= e_R.cross(r_c);
      } else {
        e_R = orientationError(R_EE, R_d_used);
      }

      if (params.axis_constraint_mode) {
        // Release selected rotational error components.
        // Fixed components keep their orientation spring.
        // Free components have e_R_i = 0, so no rotational stiffness in that component.
        e_R(0) = params.fix_R_x ? e_R(0) : 0.0;
        e_R(1) = params.fix_R_y ? e_R(1) : 0.0;
        e_R(2) = params.fix_R_z ? e_R(2) : 0.0;
      }

      // Cartesian impedance:
      //
      //   f = Kp * e_p + Dp * (pdot_d - pdot) [N]
      //
      // Kp [N/m] * e_p [m] = [N]
      // Dp [Ns/m] * velocity_error [m/s] = [N]
      Eigen::Vector3d f =
          Kp * e_p + Dp * (pdot_d - pdot);

      // Rotational impedance:
      //
      //   m = KR * e_R - DR * omega [Nm]
      //
      // KR [Nm/rad] * e_R [rad] = [Nm]
      // DR [Nms/rad] * omega [rad/s] = [Nm]
      Eigen::Vector3d m =
          KR * e_R - DR * omega;

      Eigen::Vector3d f_before_deadzone = f;
      Eigen::Vector3d m_before_deadzone = m;

      f = addAsymmetricRestoringComponent(
          f, e_p, params.e_thresh_p, params.f_fric_pos, params.f_fric_neg);

      m = addAsymmetricRestoringComponent(
          m, e_R, params.e_thresh_R, params.m_fric_pos, params.m_fric_neg);

      if (!params.disable_controller_side_limits) {
        // Controller-side force saturation: ||f|| <= f_max [N].
        f = limitVectorNorm(f, params.f_max);
      }
      if (!params.disable_controller_side_limits) {
        // Controller-side moment saturation: ||m|| <= m_max [Nm].
        m = limitVectorNorm(m, params.m_max);
      }

      // Spatial wrench ordered like Franka's Jacobian rows:
      //   wrench = [f_x, f_y, f_z, m_x, m_y, m_z]^T [N, Nm]
      Eigen::Matrix<double, 6, 1> wrench;
      wrench.head<3>() = f;
      wrench.tail<3>() = m;

      // Virtual work torque mapping:
      //   tau_task = J^T * wrench [Nm]
      Eigen::Matrix<double, 7, 1> tau_task =
          J.transpose() * wrench;

      Eigen::Matrix<double, 7, 1> tau_nullspace;
      tau_nullspace.setZero();

      if (params.use_nullspace_optimization) {
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>
            q_current(state.q.data());

        // Build the nullspace from the actually constrained task rows.
        // Example:
        //   fix_p = [1 0 0], fix_R = [0 1 1]
        //   -> J_active contains rows x, Ry, Rz only.
        Eigen::MatrixXd J_active =
            buildActiveTaskJacobian(J, params);

        // Active-task nullspace projector:
        //   N = I - J_active# * J_active     [-]
        //
        // Joint velocity projected by N should not create velocity in the
        // constrained Cartesian task directions.
        Eigen::Matrix<double, 7, 7> N =
            activeTaskNullspaceProjector(
                J_active,
                params.active_nullspace_lambda);

        // No return-to-start joint posture term is used.
        // This keeps the allowed virtual-plane motion free.
        // Only damping in the active-task nullspace and the optional
        // sigma_min optimization term are applied.
        //
        // Nullspace damping:
        //   tau_ns_damp = -d_ns * N * dq     [Nm]
        //
        // d_ns [Nm/(rad/s)], dq [rad/s].
        tau_nullspace = -params.nullspace_damping * (N * dq);

        // Optional small SVD/sigma_min preference is kept as a secondary term.
        // It is projected with the same active-task nullspace projector.
        if (params.nullspace_k_sigma > 0.0) {
          // SVD of J_active gives the joint-space direction n associated with
          // the smallest singular direction. Moving a small amount +/- alpha*n
          // estimates whether sigma_min improves in the +n or -n direction.
          Eigen::JacobiSVD<Eigen::MatrixXd> svd_active(
              J_active, Eigen::ComputeFullU | Eigen::ComputeFullV);

          Eigen::Matrix<double, 7, 1> n =
              svd_active.matrixV().col(6);

          if (n.norm() > 1e-9) {
            n.normalize();

            Eigen::Matrix<double, 7, 1> q_plus =
                q_current + params.nullspace_alpha * n;
            Eigen::Matrix<double, 7, 1> q_minus =
                q_current - params.nullspace_alpha * n;

            std::array<double, 7> q_plus_array = eigenQToArray(q_plus);
            std::array<double, 7> q_minus_array = eigenQToArray(q_minus);

            std::array<double, 42> J_plus_array =
                model.zeroJacobian(
                    franka::Frame::kEndEffector,
                    q_plus_array,
                    state.F_T_EE,
                    state.EE_T_K);

            std::array<double, 42> J_minus_array =
                model.zeroJacobian(
                    franka::Frame::kEndEffector,
                    q_minus_array,
                    state.F_T_EE,
                    state.EE_T_K);

            Eigen::Map<const Eigen::Matrix<double, 6, 7>>
                J_plus_full(J_plus_array.data());

            Eigen::Map<const Eigen::Matrix<double, 6, 7>>
                J_minus_full(J_minus_array.data());

            Eigen::MatrixXd J_plus_active =
                buildActiveTaskJacobian(J_plus_full, params);

            Eigen::MatrixXd J_minus_active =
                buildActiveTaskJacobian(J_minus_full, params);

            const double sigma_min_plus =
                J_plus_active.rows() > 0
                    ? Eigen::JacobiSVD<Eigen::MatrixXd>(
                          J_plus_active,
                          Eigen::ComputeThinU | Eigen::ComputeThinV)
                          .singularValues()
                          .minCoeff()
                    : 0.0;

            const double sigma_min_minus =
                J_minus_active.rows() > 0
                    ? Eigen::JacobiSVD<Eigen::MatrixXd>(
                          J_minus_active,
                          Eigen::ComputeThinU | Eigen::ComputeThinV)
                          .singularValues()
                          .minCoeff()
                    : 0.0;

            const double sigma_direction =
                (sigma_min_plus > sigma_min_minus) ? 1.0 : -1.0;

            // Add a small projected torque in the direction that increased
            // sigma_min in the finite-difference test:
            //
            //   tau_ns_sigma = N * k_sigma * sign * alpha * n    [Nm]
            tau_nullspace +=
                N * (params.nullspace_k_sigma
                     * sigma_direction
                     * params.nullspace_alpha
                     * n);
          }
        }

        tau_nullspace =
            limitJointTorqueVectorNorm(
                tau_nullspace,
                params.nullspace_tau_max);
      }

      // Coriolis compensation from the Franka model [Nm].
      std::array<double, 7> coriolis_array =
          model.coriolis(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          coriolis(coriolis_array.data());

      // Final torque before optional torque-rate limiting:
      //
      //   tau_raw = tau_task + tau_nullspace              [Nm]
      //   tau_raw = tau_raw + coriolis, if enabled        [Nm]
      Eigen::Matrix<double, 7, 1> tau_raw =
          tau_task + tau_nullspace;
      if (params.use_coriolis) {
        tau_raw += coriolis;
      }

      Eigen::Matrix<double, 7, 1> tau_limited = tau_raw;
      if (!params.disable_controller_side_limits) {
        // Controller-side torque-rate limiting [Nm/cycle].
        tau_limited = limitTorqueRate(tau_raw, tau_previous, params.delta_tau_max);
      }

      // Store the sent torque so the next control cycle can limit the step.
      tau_previous = tau_limited;

      // Save one row for CSV plotting and offline analysis.
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

      // Stop conditions:
      //   - experiment_duration > 0 and elapsed time reached it
      //   - user typed e + Enter
      if (((params.experiment_duration > 0.0) && (time >= params.experiment_duration)) || stop_requested.load()) {
        if (stop_requested.load()) {
          std::cout << std::endl << "Stop requested with e + Enter. Finishing control loop..." << std::endl;
        }
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
    std::cout << "Final position error norm [mm]:  " << 1000.0 * final_e_p.norm() << std::endl;
    std::cout << "Final rotation error e_R [rad]:  " << final_e_R.transpose() << std::endl;
    std::cout << "Final rotation error norm [rad]: " << final_e_R.norm() << std::endl;
    std::cout << "Final rotation error norm [deg]: " << (180.0 / M_PI) * final_e_R.norm() << std::endl;
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
