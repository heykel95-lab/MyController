#pragma once

#include "controller_types.h"

std::string trimWhitespace(const std::string& input);

// Rewrites specific "key = value" lines of a parameters file in place,
// preserving every other line (comments, formatting, unrelated keys)
// exactly as-is. Used to auto-update desired_axis_* with the just-measured
// best axis when consecutive runs share the same post-contact gains.
void updateParameterValues(
    const std::string& filename,
    const std::vector<std::pair<std::string, std::string>>& updates);

void appendMat6ParameterUpdates(
    std::vector<std::pair<std::string, std::string>>& updates,
    const std::string& prefix,
    const Mat6x6& matrix);

Array7 vec7ToArray(const Vec7& v);

double smallestSingularValue(const Mat6x7& J);

double smoothStep(double r);

double smoothStepDerivative(double r, double T);

// Side-to-side sweep offset built from the smoothStep trajectory primitive:
// scalar position s [m] and velocity s_dot [m/s] of a ping-pong between -A and
// +A, each stroke lasting stroke_duration [s]. Starts at center (s=0) and eases
// with zero velocity at every reversal. Used by the grind hold.
void grindSweep(double t, double amplitude, double stroke_duration,
                double& s, double& s_dot);

// One grind stroke (one direction) = half a full back-and-forth cycle, derived
// from grind_frequency_hz. Returns 0 (no sweep) when the frequency is 0.
double grindStrokeDuration(const Parameters& params);

double postContactPush(const Parameters& params, double post_align_time);
double postContactPush(
    const Parameters& params,
    double post_align_time,
    double start_push);

bool computeInstantaneousPoleFromTcp(
    const Vec3& v,
    const Vec3& omega,
    Vec3* pole_from_tcp);

double instantaneousScrewPitch(const Vec3& v, const Vec3& omega);

double pointDistanceToAxis(
    const Vec3& point,
    const Vec3& axis_point,
    const Vec3& axis_direction);

// The point on the line through axis_point (direction axis_direction)
// closest to "point": project (point - axis_point) onto the unit axis
// direction, then step that far along the axis from axis_point.
Vec3 nearestPointOnAxis(
    const Vec3& point,
    const Vec3& axis_point,
    const Vec3& axis_direction);

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

FiniteScrewAxis computeFiniteScrewAxis(
    const Vec3& p_start,
    const Mat3& R_start,
    const Vec3& p_end,
    const Mat3& R_end);

double suggestedPositiveGain(
    double wrench_component,
    double motion_component,
    double min_gain,
    double max_gain);

Vec3 suggestedPositiveGains(
    const Vec3& wrench,
    const Vec3& motion,
    double min_gain,
    double max_gain);

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
void fitStiffnessDamping(
    const LinearFitAccumulator& fit,
    double min_r_squared,
    Vec3* K,
    Vec3* D,
    bool valid[3]);

struct DiagonalGainSet {
  Vec3 Kp = Vec3::Zero();
  Vec3 Dp = Vec3::Zero();
  Vec3 KR = Vec3::Zero();
  Vec3 DR = Vec3::Zero();
};

double clampedLimitGain(double numerator,
                        double denominator,
                        double min_gain,
                        double max_gain);

Vec3 stiffnessFromLimits(const Vec3& max_wrench,
                         const Vec3& max_motion,
                         double min_gain,
                         double max_gain);

Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                  const Vec3& stiffness,
                                  double damping_ratio,
                                  double max_gain);

DiagonalGainSet computeQuasiStaticGains(const Parameters& params,
                                        const Vec3& translational_inertia,
                                        const Vec3& rotational_inertia);

DiagonalGainSet computeQuasiStaticGains(const Parameters& params);

Mat3 skewMatrix(const Vec3& v);

Mat6x6 blockDiagonalGain(const Vec3& translational, const Vec3& rotational);

Mat6x6 offsetAdjoint(const Vec3& r_c);

Mat6x6 adjointTransformedGain(const Mat6x6& pole_gain, const Vec3& r_c);

struct CartesianInertiaEstimate {
  Vec3 translational = Vec3::Ones();
  Vec3 rotational = Vec3::Ones();
  Mat6x6 Lambda_task = Mat6x6::Zero();
  bool valid = false;
};

Mat6x6 blockDiagonalRotation(const Mat3& R);

CartesianInertiaEstimate computeCartesianInertiaEstimate(
    const Mat7x7& joint_mass,
    const Mat6x7& J,
    const Mat3& R_task);

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

EffectiveMomentFit fitEffectiveMomentModel(
    const EffectiveMomentFitAccumulator& fit,
    double ridge);

Vec3 desiredAxisLinearMotionFromTcp(
    const Vec3& axis_from_tcp,
    const Vec3& axis_dir,
    double pitch);

Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired);

Array7 filledArray7(double value);

Array6 filledArray6(double value);

Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback);

Mat3 makeSurfaceFrameFromNormalTangent(const Vec3& normal_input, const Vec3& tangent1_input);

Mat3 makeAlignmentTargetFrame(const Parameters& params);

Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit);

Vec3 desiredToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target);

Vec3 currentToolAxisInBase(const Parameters& params, const Mat3& R_EE);

Mat3 makeToolOrientationForAlignmentTarget(
    const Parameters& params,
    const Mat3& R_alignment_target,
    const Mat3& R_start);

Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_task_frame, const Mat3& R_task);

void startKeyboardStopThread(
    const Parameters& params,
    std::atomic<bool>& stop_requested,
    std::atomic<bool>& proceed_requested,
    std::atomic<bool>& guide_requested,
    std::atomic<char>& guidance_menu_key,
    std::atomic<bool>& gate_continue);

void configureCollisionBehavior(Robot& robot, const Parameters& params);

DesiredMotion computeDesiredMotion(
    const Parameters& params,
    double time,
    const Vec3& p_start,
    const Vec3& p_EE,
    const Mat3& R_position_surface,
    const Vec3& plane_point);

Vec3 applyRotationalAxisMask(const Parameters& params, Vec3 e_R, const Mat3& R_alignment_target);

Vec7 computeNullspaceTorque(
    const Parameters& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq,
    const Vec7& q_start);
