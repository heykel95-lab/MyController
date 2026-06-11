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
