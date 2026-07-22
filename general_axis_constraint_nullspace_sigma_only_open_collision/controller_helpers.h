#pragma once

#include "controller_types.h"

// ====================================================================
// Parameter-file write-back
// ====================================================================

std::string trimWhitespace(const std::string& input);

// Rewrites specific "key = value" lines of a parameter file in place, keeping
// every other line (comments, formatting, unrelated keys) exactly as-is. Only
// keys already present in the file are rewritten. Used to save the measured
// coupled K_TCP/D_TCP back into params/sequence.txt after a run.
void updateParameterValues(
    const std::string& filename,
    const std::vector<std::pair<std::string, std::string>>& updates);

void appendMat6ParameterUpdates(
    std::vector<std::pair<std::string, std::string>>& updates,
    const std::string& prefix,
    const Mat6x6& matrix);

// ====================================================================
// Small numeric helpers
// ====================================================================

Array7 vec7ToArray(const Vec7& v);

Array7 filledArray7(double value);

Array6 filledArray6(double value);

double smallestSingularValue(const Mat6x7& J);

Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback);

Mat3 skewMatrix(const Vec3& v);

// ====================================================================
// Trajectory primitives
// ====================================================================

double smoothStep(double r);

double smoothStepDerivative(double r, double T);

// Side-to-side sweep built from smoothStep: scalar position s [m] and velocity
// s_dot [m/s] of a ping-pong between -A and +A, each stroke lasting
// stroke_duration [s]. Starts centered (s = 0) and has zero velocity at every
// reversal. Used by the grind phase.
void grindSweep(double t, double amplitude, double stroke_duration,
                double& s, double& s_dot);

// One grind stroke (one direction) = half a full back-and-forth cycle, derived
// from grind_frequency_hz. Returns 0 (no sweep) when the frequency is 0.
double grindStrokeDuration(const Parameters& params);

// Position preload pressed into the plane during set up, ramped from
// start_push at setup_push_speed and clamped at setup_max_push.
double setUpPush(const Parameters& params, double phase_time, double start_push);

// ====================================================================
// Screw-axis geometry
// ====================================================================

// The point on the line through axis_point (direction axis_direction) closest
// to "point": project (point - axis_point) onto the unit axis direction, then
// step that far along the axis from axis_point.
Vec3 nearestPointOnAxis(
    const Vec3& point,
    const Vec3& axis_point,
    const Vec3& axis_direction);

// The single screw axis (Chasles' theorem) that exactly describes a finite
// rigid-body displacement between two poses of the same body-fixed point:
// rotating by "angle" about axis_dir through the axis point, then translating
// by pitch*angle along axis_dir, takes p_start/R_start to p_end/R_end.
//
// This depends only on the start and end configuration of the whole motion, so
// unlike an instantaneous pole taken from one cycle's velocity it is not
// sensitive to per-cycle velocity noise. It is the pole the set-up report uses
// to build the coupled stiffness.
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

// ====================================================================
// Spatial (6x6) gains
// ====================================================================

Mat6x6 blockDiagonal(const Mat3& translational, const Mat3& rotational);

// Ad(r_c) = [[I, skew(r_c)], [0, I]]: maps a twist at the pole to the twist at
// a point offset by r_c.
Mat6x6 offsetAdjoint(const Vec3& r_c);

// Moves a spring defined at a pole out to a point offset by r_c:
// K_offset = Ad(r_c)^T * K_pole * Ad(r_c). The congruence preserves symmetry
// and positive semi-definiteness, so the result is still a valid spring.
Mat6x6 adjointTransformedGain(const Mat6x6& pole_gain, const Vec3& r_c);

Mat6x6 blockDiagonalRotation(const Mat3& R);

Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_task_frame, const Mat3& R_task);

// ====================================================================
// Task-space inertia and auto-damping
// ====================================================================

struct CartesianInertiaEstimate {
  Vec3 translational = Vec3::Ones();
  Vec3 rotational = Vec3::Ones();
  bool valid = false;
};

// Diagonal of the libfranka task-space inertia Lambda = (J M^-1 J^T)^-1,
// expressed in the task frame R_task. Invalid near a kinematic singularity or
// if the result comes out non-finite / non-positive.
CartesianInertiaEstimate computeCartesianInertiaEstimate(
    const Mat7x7& joint_mass,
    const Mat6x7& J,
    const Mat3& R_task);

// D = 2 * damping_ratio * sqrt(M * K) per axis, held within
// [min_damping_per_axis, max_damping].
//
// The floor matters on very soft axes: with a stiffness near zero the formula
// also returns near zero, which leaves that axis almost undamped. Passing the
// manual gains as the floor turns them from a pure fallback into a lower bound.
// Pass Vec3::Zero() for no floor.
Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                  const Vec3& stiffness,
                                  double damping_ratio,
                                  const Vec3& min_damping,
                                  double max_damping);

// ====================================================================
// Task frames and orientation
// ====================================================================

Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired);

// Builds an orthonormal frame with columns [tangent1, tangent2, normal]. Every
// surface-frame gain diagonal and .col() access in the controller follows that
// order.
Mat3 makeSurfaceFrameFromNormalTangent(const Vec3& normal_input, const Vec3& tangent1_input);

Mat3 makeAlignmentTargetFrame(const Parameters& params);

Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit);

Vec3 desiredToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target);

Vec3 currentToolAxisInBase(const Parameters& params, const Mat3& R_EE);

Mat3 makeToolOrientationForAlignmentTarget(
    const Parameters& params,
    const Mat3& R_alignment_target,
    const Mat3& R_start);

// Zeroes the rotational error components whose constrain_rotation_* flag is
// off, in the alignment-target frame.
Vec3 applyRotationalAxisMask(const Parameters& params, Vec3 e_R, const Mat3& R_alignment_target);

// ====================================================================
// Robot interaction
// ====================================================================

void startKeyboardStopThread(
    const Parameters& params,
    std::atomic<bool>& stop_requested,
    std::atomic<bool>& proceed_requested,
    std::atomic<bool>& guide_requested,
    std::atomic<char>& guidance_menu_key,
    std::atomic<bool>& gate_continue);

void configureCollisionBehavior(Robot& robot, const Parameters& params);

Vec7 computeNullspaceTorque(
    const Parameters& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq,
    const Vec7& q_start);
