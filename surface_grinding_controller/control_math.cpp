#include "controller.h"

// ====================================================================
// Small numeric helpers
// ====================================================================

Array7 vec7ToArray(const Vec7& v) {
  Array7 array{};
  for (int i = 0; i < 7; ++i) {
    array[i] = v(i);
  }
  return array;
}

Array7 filledArray7(double value) {
  Array7 array{};
  array.fill(value);
  return array;
}

Array6 filledArray6(double value) {
  Array6 array{};
  array.fill(value);
  return array;
}

double smallestSingularValue(const Mat6x7& J) {
  Eigen::JacobiSVD<Mat6x7> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.singularValues().minCoeff();
}

Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback) {
  if (v.norm() > 1e-9) {
    return v.normalized();
  }
  return fallback.normalized();
}

Mat3 skewMatrix(const Vec3& v) {
  Mat3 s;
  s << 0.0, -v(2), v(1),
       v(2), 0.0, -v(0),
       -v(1), v(0), 0.0;
  return s;
}

// ====================================================================
// Trajectory primitives
// ====================================================================

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

void grindSweep(double t, double amplitude, double stroke_duration,
                double& s, double& s_dot) {
  if (stroke_duration <= 1e-9) {
    s = 0.0;
    s_dot = 0.0;
    return;
  }
  // Endpoints in units of amplitude: stroke 0 goes center -> +A, then ping-pong
  // +A <-> -A. smoothStep gives zero velocity at both ends of every stroke, so
  // the reversals are smooth.
  const double tau = std::max(0.0, t) / stroke_duration;
  const int k = static_cast<int>(std::floor(tau));
  const double r = tau - std::floor(tau);
  auto endpoint = [](int i) -> double {
    if (i <= 0) return 0.0;
    return (i % 2 == 1) ? 1.0 : -1.0;
  };
  const double a0 = endpoint(k);
  const double a1 = endpoint(k + 1);
  s = amplitude * (a0 + (a1 - a0) * smoothStep(r));
  s_dot = amplitude * (a1 - a0) * smoothStepDerivative(r, stroke_duration);
}

double grindStrokeDuration(const Parameters& params) {
  return (params.grind_frequency_hz > 1e-9) ? (0.5 / params.grind_frequency_hz) : 0.0;
}

double setUpPush(const Parameters& params,
                 double phase_time,
                 double start_push,
                 double& push_speed) {
  const double delta = params.setup_push_end - start_push;
  const double speed_magnitude = std::abs(params.setup_push_speed);
  if (std::abs(delta) <= 1e-12 || speed_magnitude <= 1e-12) {
    push_speed = 0.0;
    return (std::abs(delta) <= 1e-12) ? params.setup_push_end : start_push;
  }

  const double direction = (delta >= 0.0) ? 1.0 : -1.0;
  const double signed_speed = direction * speed_magnitude;
  const double unclamped = start_push + signed_speed * std::max(0.0, phase_time);
  const bool end_reached = (direction > 0.0)
                               ? (unclamped >= params.setup_push_end)
                               : (unclamped <= params.setup_push_end);
  push_speed = end_reached ? 0.0 : signed_speed;
  return end_reached ? params.setup_push_end : unclamped;
}

// ====================================================================
// Screw-axis geometry
// ====================================================================

Vec3 nearestPointOnAxis(
    const Vec3& point,
    const Vec3& axis_point,
    const Vec3& axis_direction) {
  const double axis_norm = axis_direction.norm();
  if (axis_norm <= 1e-8) {
    return axis_point;
  }
  const Vec3 axis_unit = axis_direction / axis_norm;
  return axis_point + (point - axis_point).dot(axis_unit) * axis_unit;
}

FiniteScrewAxis computeFiniteScrewAxis(
    const Vec3& p_start,
    const Mat3& R_start,
    const Vec3& p_end,
    const Mat3& R_end) {
  FiniteScrewAxis result;
  // R_rel maps a vector expressed in the start orientation to the same
  // body-fixed vector expressed in the end orientation, in base coordinates:
  // (p_end - axis_point) = R_rel * (p_start - axis_point).
  const Mat3 R_rel = R_end * R_start.transpose();
  const Eigen::AngleAxisd angle_axis(R_rel);
  const double theta = angle_axis.angle();
  result.angle = theta;
  // Below this angle the net rotation is too small to fix an axis location: the
  // displacement is dominated by translation, and the formula below divides by
  // sin(theta/2).
  constexpr double kMinUsefulAngle = 1e-3;
  if (theta < kMinUsefulAngle) {
    return result;
  }
  const Vec3 n_hat = angle_axis.axis();
  const Vec3 d = p_end - p_start;
  const double h_theta = n_hat.dot(d);
  result.pitch = h_theta / theta;
  // g is the component of the displacement perpendicular to the axis -- the
  // part that has to come from rotating about an axis offset from p_start,
  // not from translation along the axis itself.
  const Vec3 g = d - h_theta * n_hat;
  result.axis_point_from_start =
      0.5 * g + 0.5 * (std::cos(0.5 * theta) / std::sin(0.5 * theta)) * n_hat.cross(g);
  result.axis_dir = n_hat;
  result.valid = true;
  return result;
}

// ====================================================================
// Spatial (6x6) gains
// ====================================================================

Mat6x6 blockDiagonal(const Mat3& translational, const Mat3& rotational) {
  Mat6x6 gain = Mat6x6::Zero();
  gain.block<3, 3>(0, 0) = translational;
  gain.block<3, 3>(3, 3) = rotational;
  return gain;
}

Mat6x6 offsetAdjoint(const Vec3& r_c) {
  Mat6x6 adjoint = Mat6x6::Zero();
  adjoint.block<3, 3>(0, 0) = Mat3::Identity();
  adjoint.block<3, 3>(0, 3) = skewMatrix(r_c);
  adjoint.block<3, 3>(3, 3) = Mat3::Identity();
  return adjoint;
}

Mat6x6 adjointTransformedGain(const Mat6x6& pole_gain, const Vec3& r_c) {
  const Mat6x6 adjoint = offsetAdjoint(r_c);
  return adjoint.transpose() * pole_gain * adjoint;
}

Mat6x6 blockDiagonalRotation(const Mat3& R) {
  Mat6x6 T = Mat6x6::Zero();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 3>(3, 3) = R;
  return T;
}

Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_task_frame, const Mat3& R_task) {
  return R_task * diagonal_in_task_frame.asDiagonal() * R_task.transpose();
}

// ====================================================================
// Task-space inertia and auto-damping
// ====================================================================

CartesianInertiaEstimate computeCartesianInertiaEstimate(
    const Mat7x7& joint_mass,
    const Mat6x7& J,
    const Mat3& R_task) {
  CartesianInertiaEstimate result;

  Eigen::LDLT<Mat7x7> mass_ldlt(joint_mass);
  if (mass_ldlt.info() != Eigen::Success) {
    return result;
  }

  const Mat7x6 Minv_Jt = mass_ldlt.solve(J.transpose());
  if (!Minv_Jt.allFinite()) {
    return result;
  }

  Mat6x6 lambda_inv = J * Minv_Jt;
  lambda_inv = 0.5 * (lambda_inv + lambda_inv.transpose());
  if (!lambda_inv.allFinite()) {
    return result;
  }

  Mat6x6 lambda_inv_damped = lambda_inv;
  lambda_inv_damped.diagonal().array() += 1e-9;
  Eigen::LDLT<Mat6x6> lambda_ldlt(lambda_inv_damped);
  if (lambda_ldlt.info() != Eigen::Success) {
    return result;
  }

  Mat6x6 Lambda_base = lambda_ldlt.solve(Mat6x6::Identity());
  if (!Lambda_base.allFinite()) {
    return result;
  }
  Lambda_base = 0.5 * (Lambda_base + Lambda_base.transpose());

  const Mat6x6 T_task = blockDiagonalRotation(R_task);
  Mat6x6 Lambda_task = T_task.transpose() * Lambda_base * T_task;
  Lambda_task = 0.5 * (Lambda_task + Lambda_task.transpose());
  if (!Lambda_task.allFinite()) {
    return result;
  }

  for (int i = 0; i < 3; ++i) {
    const double m = Lambda_task(i, i);
    const double I = Lambda_task(i + 3, i + 3);
    if (m <= 0.0 || I <= 0.0) {
      return result;
    }
    result.translational(i) = m;
    result.rotational(i) = I;
  }
  result.valid = true;
  return result;
}

Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                  const Vec3& stiffness,
                                  double damping_ratio,
                                  const Vec3& min_damping,
                                  double max_damping) {
  Vec3 damping = Vec3::Zero();
  const double zeta = std::max(0.0, damping_ratio);
  for (int i = 0; i < 3; ++i) {
    const double m = std::max(0.0, inertia(i));
    const double k = std::max(0.0, stiffness(i));
    const double critical = 2.0 * zeta * std::sqrt(m * k);
    damping(i) = std::min(max_damping, std::max(std::max(0.0, min_damping(i)), critical));
  }
  return damping;
}

// ====================================================================
// Task frames and orientation
// ====================================================================

Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired) {
  Mat3 R_error = R_current.transpose() * R_desired;
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Vec3::Zero();
  }
  return R_current * angle_axis.axis() * angle_axis.angle();
}

Mat3 makeSurfaceFrameFromNormalTangent(const Vec3& normal_input, const Vec3& tangent1_input) {
  const Vec3 normal = normalizedOrFallback(normal_input, Vec3(1.0, 0.0, 0.0));
  Vec3 tangent1 = tangent1_input - normal * normal.dot(tangent1_input);

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

  Mat3 R_alignment_target;
  R_alignment_target.col(0) = tangent1;
  R_alignment_target.col(1) = tangent2;
  R_alignment_target.col(2) = normal;
  return R_alignment_target;
}

Mat3 makeAlignmentTargetFrame(const Parameters& params) {
  return makeSurfaceFrameFromNormalTangent(params.alignment_target_normal,
                                           params.alignment_target_tangent1);
}

Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit) {
  const double dot = std::max(-1.0, std::min(1.0, from_unit.dot(to_unit)));
  if (dot > 1.0 - 1e-9) {
    return Mat3::Identity();
  }

  if (dot < -1.0 + 1e-9) {
    Vec3 axis = from_unit.cross(Vec3::UnitX());
    if (axis.norm() <= 1e-9) {
      axis = from_unit.cross(Vec3::UnitY());
    }
    axis.normalize();
    return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
  }

  Vec3 axis = from_unit.cross(to_unit);
  axis.normalize();
  return Eigen::AngleAxisd(std::acos(dot), axis).toRotationMatrix();
}

Vec3 desiredToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target) {
  const double sign = (params.tool_axis_target_sign >= 0.0) ? 1.0 : -1.0;
  return sign * R_alignment_target.col(2);  // normal = 3rd column
}

Vec3 currentToolAxisInBase(const Parameters& params, const Mat3& R_EE) {
  return R_EE * normalizedOrFallback(params.tool_axis_ee, Vec3(0.0, 0.0, 1.0));
}

Mat3 makeToolOrientationForAlignmentTarget(
    const Parameters& params,
    const Mat3& R_alignment_target,
    const Mat3& R_start) {
  const Vec3 tool_axis_start =
      currentToolAxisInBase(params, R_start).normalized();
  const Vec3 tool_axis_target =
      desiredToolAxisInBase(params, R_alignment_target).normalized();

  // Rotate the current physical tool axis onto +-alignment_target_normal while
  // keeping the remaining orientation twist as close as possible to the start.
  return rotationBetweenUnitVectors(tool_axis_start, tool_axis_target) * R_start;
}

Vec3 applyRotationalAxisMask(const Parameters& params, Vec3 e_R, const Mat3& R_alignment_target) {
  // e_R_task components follow R's column order [tangent1, tangent2, normal].
  Vec3 e_R_task = R_alignment_target.transpose() * e_R;
  e_R_task(0) =
      params.constrain_rotation_about_alignment_tangent1 ? e_R_task(0) : 0.0;
  e_R_task(1) =
      params.constrain_rotation_about_alignment_tangent2 ? e_R_task(1) : 0.0;
  e_R_task(2) =
      params.constrain_rotation_about_alignment_normal ? e_R_task(2) : 0.0;

  return R_alignment_target * e_R_task;
}

// ====================================================================
// Robot interaction
// ====================================================================

void startKeyboardStopThread(
    const Parameters& params,
    std::atomic<bool>& stop_requested,
    std::atomic<bool>& proceed_requested,
    std::atomic<bool>& guide_requested,
    std::atomic<char>& guidance_menu_key,
    std::atomic<bool>& guided_hold_selector_pending,
    std::atomic<bool>& gate_continue) {
  printf("Press e+Enter to stop the controller before the duration expires.\n");
  if (params.use_manual_guidance_start) {
    printf("During startup guidance: use s+Enter for sequence or h+Enter for hold.\n");
  }
  printf("During hold: press g+Enter to hand-guide the tool, then p+Enter to recapture and resume hold.\n");
  if (params.experiment_duration <= 0.0) {
    printf("experiment_duration <= 0: running indefinitely until e + Enter.\n");
  }

  std::thread keyboard_thread([&stop_requested, &proceed_requested,
                               &guide_requested, &guidance_menu_key,
                               &guided_hold_selector_pending,
                               &gate_continue]() {
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) {
        // Bare Enter = continue past a phase gate.
        gate_continue.store(true);
      } else if (line == "e" || line == "E") {
        stop_requested.store(true);
        break;
      } else if (line == "p" || line == "P") {
        proceed_requested.store(true);
      } else if (line == "g" || line == "G") {
        guide_requested.store(true);
      } else if (line == "o" || line == "O") {
        guidance_menu_key.store('o');
      } else if (line == "c" || line == "C") {
        guidance_menu_key.store('c');
      } else if (line == "s" || line == "S") {
        guidance_menu_key.store('s');
      } else if (line == "h" || line == "H") {
        guidance_menu_key.store('h');
        // During startup guidance, hand stdin to the synchronous 0/1/2 hold
        // selector after this key ends the robot control loop. Without this
        // handoff, both threads could consume the next input line.
        while (guided_hold_selector_pending.load() &&
               !stop_requested.load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (stop_requested.load()) {
          break;
        }
      }
    }
  });
  keyboard_thread.detach();
}

void configureCollisionBehavior(Robot& robot, const Parameters& params) {
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

Vec7 computeNullspaceTorque(
    const Parameters& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq,
    SigmaDiagnostics& sigma) {
  sigma = SigmaDiagnostics{};
  sigma.alpha = std::abs(params.nullspace_alpha);
  sigma.k_sigma = params.nullspace_k_sigma;
  sigma.deadband = std::max(0.0, params.nullspace_sigma_deadband);

  if (!params.use_nullspace_optimization ||
      params.nullspace_mode == NullspaceMode::kOff) {
    return Vec7::Zero();
  }

  Map<const Vec7> q_current(state.q.data());

  // Moore-Penrose pseudo-inverse from the Jacobian singular values:
  //   J^+ = sum_i (1/sigma_i) v_i u_i^T
  // Singular directions below the relative cutoff are omitted. Unlike the
  // previous damped normal-equation inverse, this produces an exact projector
  // onto the retained SVD nullspace.
  Eigen::JacobiSVD<Mat6x7> svd_current(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix<double, 6, 1> singular_values =
      svd_current.singularValues();
  sigma.sigma_current = singular_values.minCoeff();
  const double sigma_max = singular_values.maxCoeff();
  const double svd_cutoff =
      std::max(0.0, params.nullspace_svd_relative_tolerance) * sigma_max;

  Mat7x6 J_pinv = Mat7x6::Zero();
  for (int i = 0; i < 6; ++i) {
    if (singular_values(i) > svd_cutoff) {
      J_pinv +=
          (1.0 / singular_values(i)) *
          svd_current.matrixV().col(i) *
          svd_current.matrixU().col(i).transpose();
    }
  }

  const Mat7x7 I7 = Mat7x7::Identity();
  // Torque nullspace projector N_tau = (I - J^+ J)^T. For the
  // Moore-Penrose inverse it is symmetric; symmetrize only for roundoff.
  Mat7x7 N_tau = I7 - J.transpose() * J_pinv.transpose();
  N_tau = 0.5 * (N_tau + N_tau.transpose());
  const Vec7 dq_nullspace = N_tau * dq;
  sigma.nullspace_velocity = dq_nullspace;
  sigma.nullspace_speed = dq_nullspace.norm();
  if (sigma.nullspace_speed > 1e-12) {
    Eigen::Index dominant_velocity_index = 0;
    dq_nullspace.cwiseAbs().maxCoeff(&dominant_velocity_index);
    sigma.dominant_velocity_joint =
        static_cast<int>(dominant_velocity_index) + 1;
    const double dominant_velocity =
        dq_nullspace(dominant_velocity_index);
    sigma.dominant_velocity_fraction =
        (dominant_velocity * dominant_velocity) /
        dq_nullspace.squaredNorm();
  }

  // Mode 1 has no q_start spring: it only dissipates joint velocity projected
  // onto the nullspace, so the arm settles wherever the redundant axis stops.
  const bool damping_enabled =
      params.nullspace_mode == NullspaceMode::kDampingOnly ||
      params.nullspace_mode == NullspaceMode::kDampingAndSigma;
  Vec7 tau_damping = Vec7::Zero();
  if (damping_enabled) {
    tau_damping.noalias() =
        -params.nullspace_damping * dq_nullspace;
  }
  if (params.nullspace_mode == NullspaceMode::kDampingOnly) {
    return tau_damping;
  }
  const bool sigma_only =
      params.nullspace_mode == NullspaceMode::kSigmaOnly;
  const Vec7 sigma_fallback =
      sigma_only ? Vec7::Zero() : tau_damping;

  // Sigma-min term: step along the 1D nullspace direction n of the 6x7
  // Jacobian in whichever sign increases the smallest singular value.
  Vec7 n = svd_current.matrixV().col(6);

  if (n.norm() <= 1e-9) {
    return sigma_fallback;
  }

  n.normalize();

  const double alpha = sigma.alpha;
  if (alpha <= 1e-12) {
    return sigma_fallback;
  }
  const Array7 q_plus_array = vec7ToArray(Vec7(q_current + alpha * n));
  const Array7 q_minus_array = vec7ToArray(Vec7(q_current - alpha * n));

  const std::array<double, 42> J_plus_array =
      model.zeroJacobian(Frame::kEndEffector, q_plus_array, state.F_T_EE, state.EE_T_K);
  const std::array<double, 42> J_minus_array =
      model.zeroJacobian(Frame::kEndEffector, q_minus_array, state.F_T_EE, state.EE_T_K);

  Map<const Mat6x7> J_plus(J_plus_array.data());
  Map<const Mat6x7> J_minus(J_minus_array.data());

  const double sigma_plus = smallestSingularValue(J_plus);
  const double sigma_minus = smallestSingularValue(J_minus);
  if (!std::isfinite(sigma_plus) || !std::isfinite(sigma_minus)) {
    return sigma_fallback;
  }
  const double sigma_difference = sigma_plus - sigma_minus;
  sigma.samples_valid = true;
  sigma.sigma_plus = sigma_plus;
  sigma.sigma_minus = sigma_minus;
  sigma.sigma_difference = sigma_difference;
  if (std::abs(sigma_difference) <= sigma.deadband) {
    return sigma_fallback;
  }

  const double sigma_direction = (sigma_difference > 0.0) ? 1.0 : -1.0;
  const Vec7 best_direction = sigma_direction * n;
  sigma.direction_valid = true;
  sigma.direction_sign = sigma_direction;
  sigma.best_direction = best_direction;
  sigma.speed_toward_better = best_direction.dot(dq_nullspace);
  Eigen::Index dominant_direction_index = 0;
  best_direction.cwiseAbs().maxCoeff(&dominant_direction_index);
  sigma.dominant_direction_joint =
      static_cast<int>(dominant_direction_index) + 1;
  const double dominant_direction =
      best_direction(dominant_direction_index);
  sigma.dominant_direction_fraction =
      dominant_direction * dominant_direction;
  sigma.jacobian_null_residual = (J * best_direction).norm();
  // alpha determines only where sigma is sampled. k_sigma is the commanded
  // torque magnitude along the better (+n or -n) nullspace direction.
  const Vec7 tau_sigma =
      params.nullspace_k_sigma * N_tau * best_direction;
  sigma.tau_sigma = tau_sigma;
  sigma.tau_sigma_norm = tau_sigma.norm();
  sigma.push_active = sigma.tau_sigma_norm > 0.0;

  return sigma_only ? tau_sigma : tau_damping + tau_sigma;
}
