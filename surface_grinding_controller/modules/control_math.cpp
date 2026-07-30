// ====================================================================
// Control mathematics
// ====================================================================
// The pure computations behind the controller: trajectory shaping, spatial
// gain matrices, task-space inertia, task frames and alignment errors, and
// the nullspace sigma-min torque. No robot state is owned here.
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
  // Stroke 0 runs center -> +A, then it ping-pongs +A <-> -A. smoothStep has
  // zero velocity at both ends, so the reversals stay smooth.
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

Vec3 surfaceToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target) {
  const double sign = (params.tool_axis_target_sign >= 0.0) ? 1.0 : -1.0;
  return sign * R_alignment_target.col(2);  // normal = 3rd column
}

Vec3 desiredToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target) {
  const Vec3 flat_axis = surfaceToolAxisInBase(params, R_alignment_target);
  const double deg_to_rad = M_PI / 180.0;
  const Vec3 rotation_vector =
      deg_to_rad *
      (params.tool_target_offset_tangent1_deg * R_alignment_target.col(0) +
       params.tool_target_offset_tangent2_deg * R_alignment_target.col(1));
  const double angle = rotation_vector.norm();
  if (angle <= 1e-12) {
    return flat_axis;
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle) * flat_axis;
}

Vec3 currentToolAxisInBase(const Parameters& params, const Mat3& R_EE) {
  return R_EE * normalizedOrFallback(params.tool_axis_ee, Vec3(0.0, 0.0, 1.0));
}

Vec3 toolSurfaceAlignmentErrorInBase(
    const Parameters& params,
    const Mat3& R_EE,
    const Mat3& R_alignment_target) {
  const Vec3 tool_axis = currentToolAxisInBase(params, R_EE).normalized();
  const Vec3 flat_axis =
      surfaceToolAxisInBase(params, R_alignment_target).normalized();
  const Eigen::AngleAxisd error(
      rotationBetweenUnitVectors(tool_axis, flat_axis));
  if (std::abs(error.angle()) < 1e-12) {
    return Vec3::Zero();
  }
  return error.axis() * error.angle();
}

double toolSurfaceMisalignmentAngle(
    const Parameters& params,
    const Mat3& R_EE,
    const Mat3& R_alignment_target) {
  // Alignment quality [rad]: zero when the tool face lies flat on the plane.
  // Not the logged e_R, which measures how far the tool turned instead.
  return toolSurfaceAlignmentErrorInBase(params, R_EE, R_alignment_target).norm();
}

Mat3 makeToolOrientationForAlignmentTarget(
    const Parameters& params,
    const Mat3& R_alignment_target,
    const Mat3& R_start) {
  const Vec3 tool_axis_start =
      currentToolAxisInBase(params, R_start).normalized();
  const Vec3 tool_axis_target =
      desiredToolAxisInBase(params, R_alignment_target).normalized();

  // Rotate the tool axis onto the offset command axis, keeping the remaining
  // twist as close to the start as possible.
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
// Control law
// ====================================================================

Vec6 computeSpringWrench(const Parameters& params,
                         ControlPhase phase,
                         const Mat3& Kp,
                         const Mat3& Dp,
                         const Mat3& KR,
                         const Mat3& DR,
                         const Mat3& R_alignment_target,
                         const Vec6& dx,
                         const Vec6& dv,
                         const ContactReference& contact) {
  // The set-up spring is also what the menu's t mode holds with, so the
  // coupled path is reachable from a hold as well as from phase 2.
  const bool coupled =
      params.use_coupled_stiffness &&
      (phase == ControlPhase::kSetUp ||
       (phase == ControlPhase::kHold && params.hold_with_setup_gains));
  if (!coupled) {
    // Two independent springs: force from position error, moment from
    // orientation error. dv.tail is -omega, so the damping signs match.
    Vec6 wrench;
    wrench.head<3>() = Kp * dx.head<3>() + Dp * dv.head<3>();
    wrench.tail<3>() = KR * dx.tail<3>() + DR * dv.tail<3>();
    return wrench;
  }

  Mat6x6 K_tcp;
  Mat6x6 D_tcp;
  if (params.coupled_use_block_diagonal) {
    // Reproduces the decoupled wrench through the 6x6 path: the check that
    // the path itself is right.
    K_tcp = blockDiagonal(Kp, KR);
    D_tcp = blockDiagonal(Dp, DR);
  } else if (params.coupled_pole_manual) {
    Vec3 r_c;
    if (params.coupled_use_direct_rc_surface) {
      r_c = R_alignment_target * params.coupled_rc_surface;
    } else {
      // Legacy convention retained only for archived setup files.
      const Vec3 edge_ref = params.coupled_pole_freeze_at_contact
                                ? contact.edge_at_contact
                                : contact.edge;
      const Vec3 tcp_ref = params.coupled_pole_freeze_at_contact
                               ? contact.tcp_at_contact
                               : contact.tcp;
      r_c = tcp_ref - (edge_ref + params.coupled_pole_from_edge);
    }
    K_tcp = adjointTransformedGain(blockDiagonal(Kp, KR), r_c);
    D_tcp = adjointTransformedGain(blockDiagonal(Dp, DR), r_c);
  } else {
    // This invalid selection is rejected before the control loop.
    K_tcp = blockDiagonal(Kp, KR);
    D_tcp = blockDiagonal(Dp, DR);
  }
  return K_tcp * dx + D_tcp * dv;
}

// ====================================================================
// Robot interaction
// ====================================================================

// Joint limits of the Panda, with a small margin kept clear.
bool withinJointLimits(const Array7& q, int& joint_out) {
  static const std::array<double, 7> lower = {
      {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
  static const std::array<double, 7> upper = {
      {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
  const double margin = 0.05;
  for (int i = 0; i < 7; ++i) {
    if (q[i] < lower[i] + margin || q[i] > upper[i] - margin) {
      joint_out = i + 1;
      return false;
    }
  }
  joint_out = 0;
  return true;
}

bool solveStandoffPosture(const Model& model,
                          const RobotState& state,
                          const Array7& q_target,
                          double standoff,
                          Array7& q_standoff) {
  const std::array<double, 16> pose_target =
      model.pose(Frame::kEndEffector, q_target, state.F_T_EE, state.EE_T_K);
  Map<const Mat4x4> T_target(pose_target.data());
  const Mat3 R_target = T_target.block<3, 3>(0, 0);
  // -Z_EE points back out of the fingers, so this retreats along the tool
  // axis instead of anywhere in the base frame.
  const Vec3 p_goal =
      T_target.block<3, 1>(0, 3) - standoff * (R_target * Vec3(0.0, 0.0, 1.0));

  Array7 q = q_target;
  for (int iteration = 0; iteration < 200; ++iteration) {
    const std::array<double, 16> pose =
        model.pose(Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
    Map<const Mat4x4> T(pose.data());
    Vec6 dx;
    dx.head<3>() = p_goal - T.block<3, 1>(0, 3);
    dx.tail<3>() = orientationError(T.block<3, 3>(0, 0), R_target);
    if (dx.head<3>().norm() < 1e-6 && dx.tail<3>().norm() < 1e-6) {
      q_standoff = q;
      return true;
    }

    const std::array<double, 42> jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
    Map<const Mat6x7> J(jacobian_array.data());
    Mat6x6 damped = J * J.transpose();
    damped.diagonal().array() += 1e-6;
    const Vec7 dq = J.transpose() * damped.ldlt().solve(dx);
    if (!dq.allFinite()) {
      return false;
    }
    // Cap the step so the linearization stays valid over a few centimetres.
    const double scale = std::min(1.0, 0.05 / std::max(1e-9, dq.norm()));
    for (int i = 0; i < 7; ++i) {
      q[i] += scale * dq(i);
    }
  }
  return false;
}

void startKeyboardStopThread(const Parameters& /*params*/,
                             KeyboardSignals& signals) {
  std::atomic<bool>& stop_requested = signals.stop_requested;
  std::atomic<bool>& proceed_requested = signals.proceed_requested;
  std::atomic<bool>& guide_requested = signals.guide_requested;
  std::atomic<char>& guidance_menu_key = signals.guidance_menu_key;
  std::atomic<bool>& guided_hold_selector_pending =
      signals.guided_hold_selector_pending;
  std::atomic<bool>& gate_continue = signals.gate_continue;
  std::atomic<bool>& menu_requested = signals.menu_requested;

  // One line only. What can be typed in a given mode is printed by that
  // mode's own block, where it is in context.
  printf("keys: e stop | m menu | g hand-guide\n");

  std::thread keyboard_thread([&signals, &stop_requested, &proceed_requested,
                               &guide_requested, &guidance_menu_key,
                               &guided_hold_selector_pending,
                               &gate_continue, &menu_requested]() {
    std::string line;
    while (true) {
      // Parked while the startup menu owns stdin, so the two never consume
      // the same input line. main clears the flag when the menu is done.
      while (menu_requested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (!std::getline(std::cin, line)) {
        break;
      }
      // Set-up impedance for the t mode: "kp2 1500", "kr3 40", "r1 -20".
      // Checked before the single-letter keys, which would otherwise swallow
      // the leading k.
      {
        int index = 0;
        double value = 0.0;
        std::array<std::atomic<double>, 3>* target = nullptr;
        if (std::sscanf(line.c_str(), "kp%d %lf", &index, &value) == 2) {
          target = &signals.setup_kp_request;
        } else if (std::sscanf(line.c_str(), "kr%d %lf", &index, &value) == 2) {
          target = &signals.setup_kr_request;
        } else if (std::sscanf(line.c_str(), "r%d %lf", &index, &value) == 2) {
          target = &signals.setup_rc_mm_request;
        }
        if (target != nullptr) {
          if (index >= 1 && index <= 3 && std::isfinite(value)) {
            (*target)[index - 1].store(value);
          } else {
            printf("Index must be 1, 2 or 3 and the value finite.\n");
          }
          continue;
        }
      }

      // "d <value>" and "k <value>" tune the nullspace live while holding.
      // Checked before the single-letter keys so a bare d or k still falls
      // through to "unknown".
      if (line.size() > 1 && (line[0] == 'd' || line[0] == 'D' ||
                              line[0] == 'k' || line[0] == 'K' ||
                              line[0] == 'a' || line[0] == 'A')) {
        double value = 0.0;
        if (std::sscanf(line.c_str() + 1, "%lf", &value) == 1 &&
            std::isfinite(value) && value >= 0.0) {
          if (line[0] == 'd' || line[0] == 'D') {
            signals.nullspace_damping_request.store(value);
          } else if (line[0] == 'k' || line[0] == 'K') {
            signals.nullspace_k_sigma_request.store(value);
          } else {
            // alpha is typed in degrees; the loop converts it to radians.
            signals.nullspace_alpha_deg_request.store(value);
          }
          continue;
        }
        printf("Expected 'd <Nms/rad>', 'k <Nm>' or 'a <deg>', value >= 0.\n");
        continue;
      }
      if (line.size() == 1 && line[0] >= '0' && line[0] <= '3') {
        // Live nullspace mode switch while holding.
        signals.nullspace_mode_request.store(line[0] - '0');
        continue;
      }
      if (line.empty()) {
        // Bare Enter = continue past a phase gate.
        gate_continue.store(true);
      } else if (line == "e" || line == "E") {
        stop_requested.store(true);
        break;
      } else if (line == "m" || line == "M") {
        // Back to the startup menu: end this run, then park until main has
        // finished asking.
        menu_requested.store(true);
        stop_requested.store(true);
      } else if (line == "p" || line == "P") {
        proceed_requested.store(true);
      } else if (line == "g" || line == "G") {
        guide_requested.store(true);
      } else if (line == "o" || line == "O") {
        guidance_menu_key.store('o');
      } else if (line == "c" || line == "C") {
        guidance_menu_key.store('c');
      } else if (line == "recal" || line == "RECAL") {
        // The full word is the confirmation: recalibrating opens the fingers
        // fully and may drop a tool. It runs after torque control returns.
        guidance_menu_key.store('r');
      } else if (line == "w" || line == "W") {
        guidance_menu_key.store('w');
      } else if (line == "s" || line == "S") {
        guidance_menu_key.store('s');
      } else if (line == "h" || line == "H") {
        guidance_menu_key.store('h');
        // Hand stdin to the hold selector once this key ends the control
        // loop, so both threads never consume the same input line.
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

  // Silent in the normal case; the surprising one is worth a line.
  if (!params.use_custom_collision_behavior) {
    printf("Collision: Franka default thresholds (safety.txt is off).\n");
    return;
  }

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

  // Moore-Penrose pseudo-inverse J^+ = sum_i (1/sigma_i) v_i u_i^T, dropping
  // directions below the cutoff. Gives an exact projector onto the nullspace.
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
