#pragma once

#include "controller_types.h"

inline std::string trimWhitespace(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

// Rewrites specific "key = value" lines of a parameters file in place,
// preserving every other line (comments, formatting, unrelated keys)
// exactly as-is. Used to auto-update desired_axis_* with the just-measured
// best axis when consecutive runs share the same post-contact gains.
inline void updateParameterValues(
    const std::string& filename,
    const std::vector<std::pair<std::string, std::string>>& updates) {
  std::ifstream in(filename);
  if (!in.is_open()) {
    return;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) {
    lines.push_back(line);
  }
  in.close();

  for (auto& l : lines) {
    const auto comment_pos = l.find('#');
    const std::string code_part =
        (comment_pos != std::string::npos) ? l.substr(0, comment_pos) : l;
    const auto eq_pos = code_part.find('=');
    if (eq_pos == std::string::npos) {
      continue;
    }
    const std::string key = trimWhitespace(code_part.substr(0, eq_pos));
    for (const auto& update : updates) {
      if (key == update.first) {
        const std::string comment_part =
            (comment_pos != std::string::npos) ? l.substr(comment_pos) : "";
        l = code_part.substr(0, eq_pos) + "= " + update.second +
            (comment_part.empty() ? "" : ("  " + comment_part));
        break;
      }
    }
  }

  std::ofstream out(filename, std::ios::trunc);
  for (const auto& out_line : lines) {
    out << out_line << "\n";
  }
}

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

inline double postContactPush(const Parameters& params, double post_align_time) {
  double push =
      params.post_contact_normal_push + params.post_contact_push_speed * post_align_time;
  if (params.post_contact_max_push > 0.0) {
    push = std::min(params.post_contact_max_push, push);
  }
  return push;
}

inline bool computeInstantaneousPoleFromTcp(
    const Vec3& v,
    const Vec3& omega,
    Vec3* pole_from_tcp) {
  const double omega_norm_squared = omega.squaredNorm();
  constexpr double kMinUsefulAngularSpeed = 0.02;
  if (omega_norm_squared <= kMinUsefulAngularSpeed * kMinUsefulAngularSpeed) {
    pole_from_tcp->setZero();
    return false;
  }

  *pole_from_tcp = omega.cross(v) / omega_norm_squared;
  return true;
}

inline double instantaneousScrewPitch(const Vec3& v, const Vec3& omega) {
  const double omega_norm_squared = omega.squaredNorm();
  if (omega_norm_squared <= 1e-8) {
    return 0.0;
  }
  return omega.dot(v) / omega_norm_squared;
}

inline double pointDistanceToAxis(
    const Vec3& point,
    const Vec3& axis_point,
    const Vec3& axis_direction) {
  const double axis_norm = axis_direction.norm();
  if (axis_norm <= 1e-8) {
    return 0.0;
  }

  return (point - axis_point).cross(axis_direction / axis_norm).norm();
}

// The point on the line through axis_point (direction axis_direction)
// closest to "point": project (point - axis_point) onto the unit axis
// direction, then step that far along the axis from axis_point.
inline Vec3 nearestPointOnAxis(
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

// The single screw axis (Chasles' theorem) that exactly describes a finite
// rigid-body displacement between two poses of the same body-fixed
// reference point -- as opposed to an instantaneous pole from one cycle's
// velocity, which only describes the motion at that instant and is
// sensitive to per-cycle velocity noise. This depends only on the start and
// end configuration of the whole motion, so it is the rigorous version of
// "one axis that describes the whole motion even though it changes":
// rotating by angle about axis_dir through axis_point, then translating by
// pitch*angle along axis_dir, taking p_start/R_start exactly to p_end/R_end.
struct FiniteScrewAxis {
  Vec3 axis_point_from_start = Vec3::Zero();
  Vec3 axis_dir = Vec3::Zero();
  double pitch = 0.0;
  double angle = 0.0;
  bool valid = false;
};

inline FiniteScrewAxis computeFiniteScrewAxis(
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
  // Below this angle the net rotation is too small to fix an axis location;
  // the displacement is dominated by translation, not rotation, and the
  // formula below divides by sin(theta/2).
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

inline double suggestedPositiveGain(
    double wrench_component,
    double motion_component,
    double min_gain,
    double max_gain) {
  const double min_motion = 1e-6;
  double gain = max_gain;
  if (std::abs(motion_component) > min_motion) {
    gain = std::abs(wrench_component / motion_component);
  }
  return std::max(min_gain, std::min(max_gain, gain));
}

inline Vec3 suggestedPositiveGains(
    const Vec3& wrench,
    const Vec3& motion,
    double min_gain,
    double max_gain) {
  return Vec3(
      suggestedPositiveGain(wrench(0), motion(0), min_gain, max_gain),
      suggestedPositiveGain(wrench(1), motion(1), min_gain, max_gain),
      suggestedPositiveGain(wrench(2), motion(2), min_gain, max_gain));
}

// Running sums for an ordinary-least-squares fit of wrench = K*x + D*v over
// many samples collected while a phase runs, instead of reading K and D off
// a single (wrench, x, v) snapshot. A single sample can only ever produce a
// residual of exactly zero for the gain it was fit from, which is why a
// one-shot fit cannot identify damping; many samples taken at different
// points in the transient can.
struct LinearFitAccumulator {
  Vec3 Sxx = Vec3::Zero();
  Vec3 Sxv = Vec3::Zero();
  Vec3 Svv = Vec3::Zero();
  Vec3 Sxf = Vec3::Zero();
  Vec3 Svf = Vec3::Zero();
  long sample_count = 0;

  void addSample(const Vec3& x, const Vec3& v, const Vec3& f) {
    Sxx += x.cwiseProduct(x);
    Sxv += x.cwiseProduct(v);
    Svv += v.cwiseProduct(v);
    Sxf += x.cwiseProduct(f);
    Svf += v.cwiseProduct(f);
    ++sample_count;
  }
};

// Solves the per-axis 2x2 normal equations for K (paired with x) and D
// (paired with v). x and v must vary independently over the sampled window
// for K and D to be separable: if x(t) and v(t) are proportional throughout
// (e.g. a single free decay mode, as in a passively settling rotation with
// no forced excitation), the system is singular and K/D cannot be told
// apart from this data at all, no matter how many samples are collected.
// min_r_squared gates on exactly that: it is 1 - (correlation between x and
// v)^2, so it is ~0 when x and v are nearly collinear.
inline void fitStiffnessDamping(
    const LinearFitAccumulator& fit,
    double min_r_squared,
    Vec3* K,
    Vec3* D,
    bool valid[3]) {
  for (int i = 0; i < 3; ++i) {
    const double Sxx = fit.Sxx(i);
    const double Sxv = fit.Sxv(i);
    const double Svv = fit.Svv(i);
    const double scale = Sxx * Svv;
    const double det = scale - Sxv * Sxv;
    if (scale <= 1e-18 || det / scale < min_r_squared) {
      (*K)(i) = 0.0;
      (*D)(i) = 0.0;
      valid[i] = false;
      continue;
    }
    (*K)(i) = (fit.Sxf(i) * Svv - fit.Svf(i) * Sxv) / det;
    (*D)(i) = (Sxx * fit.Svf(i) - Sxv * fit.Sxf(i)) / det;
    valid[i] = true;
  }
}

struct DiagonalGainSet {
  Vec3 Kp = Vec3::Zero();
  Vec3 Dp = Vec3::Zero();
  Vec3 KR = Vec3::Zero();
  Vec3 DR = Vec3::Zero();
};

inline double clampedLimitGain(double numerator,
                               double denominator,
                               double min_gain,
                               double max_gain) {
  if (std::abs(denominator) <= 1e-12) {
    return max_gain;
  }
  const double gain = std::abs(numerator) / std::abs(denominator);
  return std::max(min_gain, std::min(max_gain, gain));
}

inline Vec3 stiffnessFromLimits(const Vec3& max_wrench,
                                const Vec3& max_motion,
                                double min_gain,
                                double max_gain) {
  Vec3 stiffness = Vec3::Zero();
  for (int i = 0; i < 3; ++i) {
    stiffness(i) = clampedLimitGain(max_wrench(i), max_motion(i), min_gain, max_gain);
  }
  return stiffness;
}

inline Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                         const Vec3& stiffness,
                                         double damping_ratio,
                                         double max_gain) {
  Vec3 damping = Vec3::Zero();
  const double zeta = std::max(0.0, damping_ratio);
  for (int i = 0; i < 3; ++i) {
    const double m = std::max(0.0, inertia(i));
    const double k = std::max(0.0, stiffness(i));
    damping(i) = std::min(max_gain, 2.0 * zeta * std::sqrt(m * k));
  }
  return damping;
}

inline DiagonalGainSet computeQuasiStaticGains(const Parameters& params) {
  DiagonalGainSet gains;
  gains.Kp = stiffnessFromLimits(params.quasi_force_limit,
                                 params.quasi_displacement_limit,
                                 params.suggested_gain_min,
                                 params.suggested_gain_max);
  gains.KR = stiffnessFromLimits(params.quasi_moment_limit,
                                 params.quasi_angle_limit,
                                 params.suggested_gain_min,
                                 params.suggested_gain_max);
  gains.Dp = criticalDampingFromStiffness(params.quasi_effective_mass,
                                         gains.Kp,
                                         params.quasi_damping_ratio,
                                         params.suggested_gain_max);
  gains.DR = criticalDampingFromStiffness(params.quasi_effective_inertia,
                                         gains.KR,
                                         params.quasi_damping_ratio,
                                         params.suggested_gain_max);
  return gains;
}

inline Mat3 skewMatrix(const Vec3& v) {
  Mat3 s;
  s << 0.0, -v(2), v(1),
       v(2), 0.0, -v(0),
       -v(1), v(0), 0.0;
  return s;
}

inline Mat6x6 blockDiagonalGain(const Vec3& translational, const Vec3& rotational) {
  Mat6x6 gain = Mat6x6::Zero();
  gain.block<3, 3>(0, 0) = translational.asDiagonal();
  gain.block<3, 3>(3, 3) = rotational.asDiagonal();
  return gain;
}

inline Mat6x6 offsetAdjoint(const Vec3& r_c) {
  Mat6x6 adjoint = Mat6x6::Zero();
  adjoint.block<3, 3>(0, 0) = Mat3::Identity();
  adjoint.block<3, 3>(0, 3) = skewMatrix(r_c);
  adjoint.block<3, 3>(3, 3) = Mat3::Identity();
  return adjoint;
}

inline Mat6x6 adjointTransformedGain(const Mat6x6& pole_gain, const Vec3& r_c) {
  const Mat6x6 adjoint = offsetAdjoint(r_c);
  return adjoint.transpose() * pole_gain * adjoint;
}

struct EffectiveMomentFitAccumulator {
  Mat12x12 H = Mat12x12::Zero();
  Mat12x3 B = Mat12x3::Zero();
  double y_squared_sum = 0.0;
  long sample_count = 0;

  void addSample(const Vec3& contact_displacement,
                 const Vec3& contact_velocity,
                 const Vec3& rotation_displacement,
                 const Vec3& angular_velocity,
                 const Vec3& contact_moment) {
    Vec12 phi;
    phi << contact_displacement, contact_velocity, rotation_displacement, angular_velocity;
    H.noalias() += phi * phi.transpose();
    B.noalias() += phi * contact_moment.transpose();
    y_squared_sum += contact_moment.squaredNorm();
    ++sample_count;
  }
};

struct EffectiveMomentFit {
  Mat3 K_rt = Mat3::Zero();
  Mat3 D_rt = Mat3::Zero();
  Mat3 K_R = Mat3::Zero();
  Mat3 D_R = Mat3::Zero();
  double rms_error = 0.0;
  long sample_count = 0;
  bool valid = false;
};

inline EffectiveMomentFit fitEffectiveMomentModel(
    const EffectiveMomentFitAccumulator& fit,
    double ridge) {
  EffectiveMomentFit result;
  result.sample_count = fit.sample_count;
  if (fit.sample_count < 12) {
    return result;
  }

  Mat12x12 H_damped = fit.H;
  H_damped.diagonal().array() += std::max(0.0, ridge);
  Eigen::LDLT<Mat12x12> ldlt(H_damped);
  if (ldlt.info() != Eigen::Success) {
    return result;
  }

  const Mat12x3 x = ldlt.solve(fit.B);
  if (ldlt.info() != Eigen::Success || !x.allFinite()) {
    return result;
  }

  const Mat3x12 A = x.transpose();
  result.K_rt = A.block<3, 3>(0, 0);
  result.D_rt = A.block<3, 3>(0, 3);
  result.K_R = A.block<3, 3>(0, 6);
  result.D_R = A.block<3, 3>(0, 9);

  const double cross_term = (x.transpose() * fit.B).trace();
  const double model_term = (x.transpose() * fit.H * x).trace();
  const double sse = std::max(0.0, fit.y_squared_sum - 2.0 * cross_term + model_term);
  result.rms_error = std::sqrt(sse / static_cast<double>(fit.sample_count));
  result.valid = true;
  return result;
}

inline Vec3 desiredAxisLinearMotionFromTcp(
    const Vec3& axis_from_tcp,
    const Vec3& axis_dir,
    double pitch) {
  const Vec3 axis_unit =
      (axis_dir.norm() > 1e-9) ? axis_dir.normalized() : Vec3(1.0, 0.0, 0.0);
  return -axis_unit.cross(axis_from_tcp) + pitch * axis_unit;
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

inline Mat3 makeSurfaceFrameFromNormalTangent(const Vec3& normal_input, const Vec3& tangent1_input) {
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

  Mat3 R_surface;
  R_surface.col(0) = normal;
  R_surface.col(1) = tangent1;
  R_surface.col(2) = tangent2;
  return R_surface;
}

inline Mat3 makeSurfaceFrame(const Parameters& params) {
  return makeSurfaceFrameFromNormalTangent(params.surface_normal, params.surface_tangent1);
}

inline Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit) {
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

inline Vec3 desiredToolAxisInBase(const Parameters& params, const Mat3& R_surface) {
  const double sign = (params.tool_axis_target_sign >= 0.0) ? 1.0 : -1.0;
  return sign * R_surface.col(0);
}

inline Vec3 currentToolAxisInBase(const Parameters& params, const Mat3& R_EE) {
  return R_EE * normalizedOrFallback(params.tool_axis_ee, Vec3(0.0, 0.0, 1.0));
}

inline Mat3 makeToolOrientationParallelToSurface(
    const Parameters& params,
    const Mat3& R_surface,
    const Mat3& R_start) {
  const Vec3 tool_axis_start =
      currentToolAxisInBase(params, R_start).normalized();
  const Vec3 tool_axis_target =
      desiredToolAxisInBase(params, R_surface).normalized();

  // Rotate the current physical tool axis onto +-surface_normal while keeping
  // the remaining orientation twist as close as possible to the start pose.
  return rotationBetweenUnitVectors(tool_axis_start, tool_axis_target) * R_start;
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
  e_R_task(0) =
      params.constrain_rotation_about_surface_normal ? e_R_task(0) : 0.0;
  e_R_task(1) =
      params.constrain_rotation_about_surface_tangent1 ? e_R_task(1) : 0.0;
  e_R_task(2) =
      params.constrain_rotation_about_surface_tangent2 ? e_R_task(2) : 0.0;

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
    return tau_nullspace;
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

  return tau_nullspace + tau_sigma;
}
