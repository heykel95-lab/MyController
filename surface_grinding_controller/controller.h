// ====================================================================
// Shared declarations
// ====================================================================
// Types, the Parameters struct that every module reads, and the prototypes
// that connect config / control_math / runtime_io / setup_report / main.
// One translation unit implements each group; nothing here has a definition.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"

// ====================================================================
// Types and aliases
// ====================================================================

using Array6 = std::array<double, 6>;
using Array7 = std::array<double, 7>;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat4x4 = Eigen::Matrix<double, 4, 4>;
using Mat6x6 = Eigen::Matrix<double, 6, 6>;
using Mat6x7 = Eigen::Matrix<double, 6, 7>;
using Mat7x6 = Eigen::Matrix<double, 7, 6>;
using Mat7x7 = Eigen::Matrix<double, 7, 7>;

template <typename MatrixType>
using Map = Eigen::Map<MatrixType>;

using Robot = franka::Robot;
using Gripper = franka::Gripper;
using RobotState = franka::RobotState;
using Model = franka::Model;
using Torques = franka::Torques;
using Duration = franka::Duration;
using Frame = franka::Frame;
using franka::MotionFinished;

// Sequence phases plus the two standalone startup modes.
enum class ControlPhase {
  kApproachOrient,
  kApproachDescend,
  kSetUp,
  kGrind,
  kHold,
  kManualGuide
};

enum class NullspaceMode {
  kOff = 0,
  kDampingOnly = 1,
  kSigmaOnly = 2,
  kDampingAndSigma = 3
};

// ====================================================================
// Parameters -- everything params/*.txt can set
// ====================================================================

struct Parameters {
  // Robot connection, logging and terminal debug.
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 0.0;  // <= 0 runs until e + Enter
  std::string csv_file_name = "surface_grinding_controller_log.csv";
  int log_every_n_cycles = 5;
  int max_log_rows = 120000;
  std::string sigma_debug_csv_file_name =
      "surface_grinding_controller_sigma_debug.csv";
  double sigma_debug_log_period = 0.05;
  int max_sigma_debug_rows = 20000;
  double debug_period = 0.20;
  bool print_hold_debug = true;
  bool print_grind_debug = true;
  bool print_sigma_debug = true;
  bool print_coupled_diagnostics = true;

  // Gripper action performed once after q_init.
  bool open_gripper_before_run = true;
  bool require_gripper_open = true;
  // 1 = close onto the tool held in the hand, 0 = open to gripper_open_width.
  bool gripper_grasp_on_tool = false;
  double gripper_open_width = 0.08;
  double gripper_open_speed = 0.05;
  double gripper_grasp_width = 0.02;
  double gripper_grasp_speed = 0.05;
  double gripper_grasp_force = 40.0;
  double gripper_grasp_epsilon_inner = 0.005;
  double gripper_grasp_epsilon_outer = 0.010;
  // Runtime-only: set when the startup menu already handled the gripper.
  bool startup_gripper_manual = false;

  // Run mode. Runtime-only: not read from the parameter files, because the
  // startup menu chooses sequence or hold on every run.
  bool use_phase_sequence = true;
  // Runtime-only: the menu's t choice. Holds the captured pose with the
  // set-up impedance instead of the hold gains -- the same spring phase 2
  // commands, including the coupled pole when use_coupled_stiffness = 1.
  bool hold_with_setup_gains = false;
  // 0 = skip the orient step and start the sequence at the descend step.
  bool use_approach_orient = true;
  bool use_manual_guidance_start = false;
  double manual_guidance_damping = 0.5;

  // Hold mode gains, per axis of the base frame [x, y, z].
  Vec3 hold_Kp_diag = Vec3::Constant(300.0);
  Vec3 hold_Dp_diag = Vec3::Constant(50.0);
  Vec3 hold_KR_diag = Vec3::Constant(40.0);
  Vec3 hold_DR_diag = Vec3::Constant(8.0);
  bool hold_auto_damping = true;
  bool hold_auto_match_manual_damping = true;
  double hold_auto_damping_factor = 1.0;

  // Surface plane and tool geometry.
  bool use_start_as_surface_point = true;
  Vec3 surface_point = Vec3(0.0, 0.0, 0.0);
  Vec3 alignment_target_normal = Vec3(0.0, 0.0, 1.0);
  double alignment_target_tilt_angle_deg = 0.0;    // a, about base x
  double alignment_target_tilt_angle_y_deg = 0.0;  // b, about base y
  Vec3 alignment_target_tangent1 = Vec3(0.0, 1.0, 0.0);
  // Commanded orientation offset from the surface frame [deg]. It offsets only
  // the command, not the plane used for clearance, gains or scoring. The two
  // tangents tilt the tool axis; the normal spins the face about that axis,
  // and only applies when command_tool_twist is set.
  double tool_target_offset_tangent1_deg = 0.0;
  double tool_target_offset_tangent2_deg = 0.0;
  double tool_target_offset_normal_deg = 0.0;
  bool command_tool_twist = false;
  Vec3 tool_axis_ee = Vec3(0.0, 0.0, 1.0);
  double tool_axis_target_sign = -1.0;
  bool use_tool_contact_point_control = true;
  bool auto_select_tool_contact_edge = true;
  Vec3 tool_contact_face_center_ee = Vec3(0.0, 0.0, 0.0);
  Vec3 tool_contact_half_width_ee = Vec3(0.0, 0.0, 0.0);
  Vec3 tool_contact_half_length_ee = Vec3(0.0, 0.0, 0.0);
  double tool_contact_feature_tie_tolerance = 0.0001;
  // Rotational spring mask in the alignment-target frame.
  bool constrain_rotation_about_alignment_normal = true;
  bool constrain_rotation_about_alignment_tangent1 = true;
  bool constrain_rotation_about_alignment_tangent2 = true;

  // Phase 1: approach (orient, then descend).
  double approach_orient_min_time = 0.5;
  double approach_orient_error_threshold = 0.03;
  // Slew rate for the commanded orientation [deg/s]. The target is otherwise a
  // step at t=0, and a commanded spin about the tool axis can be most of a
  // half turn, which trips the power limit against the approach KR.
  double approach_orient_max_rate_deg = 20.0;
  // One impedance for both approach steps, in [tangent1, tangent2, normal].
  Vec3 approach_Kp_diag = Vec3(150.0, 150.0, 150.0);
  Vec3 approach_KR_diag = Vec3(90.0, 90.0, 8.0);
  Vec3 approach_Dp_diag = Vec3(20.0, 20.0, 20.0);
  Vec3 approach_DR_diag = Vec3(12.0, 12.0, 12.0);
  bool approach_auto_damping = false;
  double approach_auto_damping_factor = 1.0;
  double descend_speed = 0.005;
  double descend_max_distance = 0.02;
  // Clearance above the surface before set up takes over.
  double descend_surface_clearance = 0.020;

  // Phase 2: set up.
  double setup_min_time = 0.3;       // before the moment early-exit can fire
  double setup_timeout = 15.0;       // hard time limit for the phase
  double setup_moment_threshold = 60.0;
  // Position preload ramped into the surface; grind inherits the final value.
  double setup_push_speed = 0.0;
  double setup_push_end = 0.0;
  // Translational spring, diagonal in base [x,y,z]. Setting
  // setup_translation_surface_frame=1 uses the surface-frame vectors below.
  Vec3 setup_Kp_diag = Vec3(40.0, 40.0, 5500.0);
  Vec3 setup_Dp_diag = Vec3(10.0, 10.0, 175.0);
  bool setup_translation_surface_frame = false;
  Vec3 setup_Kp_surface_diag = Vec3(2000.0, 2000.0, 360.0);
  Vec3 setup_Dp_surface_diag = Vec3(50.0, 50.0, 25.0);
  // Rotational spring, alignment-target frame [tangent1, tangent2, normal].
  Vec3 setup_KR_diag = Vec3(0.0, 0.0, 8.0);
  Vec3 setup_DR_diag = Vec3(0.01, 0.01, 4.0);
  bool setup_auto_damping = false;
  double setup_auto_damping_factor = 1.0;

  // Phase 3: grind (constant press, shared gains with phase 2).
  // 1 = commanded sweep, 0 = free-slide press hold.
  bool grind_sweep_enabled = false;
  int grind_axis = 1;               // 1 = tangent1, 2 = tangent2
  double grind_amplitude_m = 0.03;  // sweep half-amplitude A [m]
  double grind_frequency_hz = 0.2;  // one full back-and-forth cycle [Hz]

  // Enter gates between phases.
  bool pause_before_set_up = false;  // hold at the clearance height
  bool pause_before_grind = false;   // hold the seated/pressed pose
  // Stiff position hold; rotation retains the approach stiffness.
  // Gate hold: translation per base axis, rotation per surface axis.
  Vec3 pause_hold_Kp_diag = Vec3::Constant(5000.0);
  Vec3 pause_hold_Dp_diag = Vec3::Constant(200.0);
  Vec3 pause_hold_KR_diag = Vec3::Constant(90.0);
  Vec3 pause_hold_DR_diag = Vec3::Constant(12.0);
  bool pause_hold_auto_damping = true;

  // Coupled (pole-based) stiffness for the set-up phase.
  bool use_coupled_stiffness = false;
  // K/D source: block diagonal or a deliberately commanded pole.
  bool coupled_use_block_diagonal = false;
  bool coupled_pole_manual = false;
  // Direct lever convention used by the new experiments:
  // r_c = p_TCP - p_c, resolved in [tangent1,tangent2,normal].
  bool coupled_use_direct_rc_surface = false;
  Vec3 coupled_rc_surface = Vec3::Zero();
  // Legacy base-frame pole-from-edge convention retained only so archived
  // setup files remain reproducible.
  Vec3 coupled_pole_from_edge = Vec3::Zero();
  // 1 = freeze pole at first contact, 0 = track the live moving edge.
  bool coupled_pole_freeze_at_contact = true;

  // Nullspace optimization.
  bool use_nullspace_optimization = true;
  NullspaceMode nullspace_mode = NullspaceMode::kDampingAndSigma;
  double nullspace_damping = 1.0;
  double nullspace_k_sigma = 0.05;
  double nullspace_alpha = 0.03;
  double nullspace_sigma_deadband = 1e-6;
  double nullspace_svd_relative_tolerance = 1e-4;

  // Auto-damping options.
  double auto_damping_max = 8000.0;
  // 1 = manual Dp/DR values are per-axis floors, not only fallbacks.
  bool auto_damping_min_from_manual = false;
  bool print_auto_damping = true;

  // Start pose and collision thresholds.
  Array7 q_init = {{
      0.0,
      -M_PI_4,
      0.0,
      -3.0 * M_PI_4,
      0.0,
      M_PI_2,
      0.0
  }};
  std::string q_init_case = "horizontal_tool";

  // Tool pickup posture, used by the menu's t (fetch) and b (put back) keys.
  // Disabled until a measured posture is pasted in.
  bool use_tool_pickup = false;
  Array7 q_pickup = {{0.0, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, 0.0}};
  // Stand-off retreat along -Z_EE from the pickup pose [m]. The posture is
  // solved from it, so only this distance is configured.
  double pickup_standoff = 0.05;
  double pickup_descend_speed_factor = 0.15;

  bool use_custom_collision_behavior = false;
  double collision_torque_acc = 80.0;
  double collision_torque_nom = 80.0;
  double collision_force_acc = 80.0;
  double collision_force_nom = 80.0;
};

// ====================================================================
// Logging and diagnostics
// ====================================================================

struct SigmaDiagnostics {
  bool samples_valid = false;
  bool direction_valid = false;
  bool push_active = false;
  double sigma_current = 0.0;
  double sigma_plus = 0.0;
  double sigma_minus = 0.0;
  double sigma_difference = 0.0;
  double direction_sign = 0.0;
  double alpha = 0.0;
  double k_sigma = 0.0;
  double deadband = 0.0;
  double tau_sigma_norm = 0.0;
  double nullspace_speed = 0.0;
  double speed_toward_better = 0.0;
  // Sign-selected unit nullspace direction, projected joint velocity and
  // commanded sigma torque. Fixed-size vectors need explicit initialization.
  Vec7 best_direction = Vec7::Zero();
  Vec7 nullspace_velocity = Vec7::Zero();
  Vec7 tau_sigma = Vec7::Zero();
  // Joint indices are one-based for direct comparison with q1..q7; zero means
  // that no direction/motion was available.
  int dominant_direction_joint = 0;
  int dominant_velocity_joint = 0;
  double dominant_direction_fraction = 0.0;
  double dominant_velocity_fraction = 0.0;
  double jacobian_null_residual = 0.0;
};

enum class SigmaDebugEvent {
  kSample = 0,
  kHoldStart = 1,
  kManualGuideStart = 2,
  kRecapture = 3,
  kStop = 4,
  kException = 5
};

// Compact, preallocated diagnostic row for sigma-enabled hold experiments.
// Rows are buffered in the realtime loop and written only after control ends.
struct SigmaDebugRow {
  double run_time = 0.0;
  double phase_time = 0.0;
  int segment_id = 0;
  SigmaDebugEvent event = SigmaDebugEvent::kSample;

  Vec7 q = Vec7::Zero();
  Vec7 dq = Vec7::Zero();
  Vec3 e_p = Vec3::Zero();
  Vec3 e_R = Vec3::Zero();
  Vec3 pdot = Vec3::Zero();
  Vec3 omega = Vec3::Zero();
  Vec3 command_force = Vec3::Zero();
  Vec3 command_moment = Vec3::Zero();
  Vec3 external_force_delta = Vec3::Zero();
  Vec3 external_moment_delta = Vec3::Zero();
  Vec7 external_joint_torque_delta = Vec7::Zero();
  bool external_joint_torque_baseline_valid = false;
  bool joint_contact = false;
  bool cartesian_contact = false;

  SigmaDiagnostics sigma;
  double tau_task_norm = 0.0;
  double tau_nullspace_norm = 0.0;
  // Logged because it can be retuned live while holding, and the archived
  // params_effective would then no longer describe the run.
  double nullspace_damping = 0.0;
  double tau_cmd_norm = 0.0;

  // Peaks accumulated between compact samples so brief pushes are not lost.
  double peak_nullspace_speed = 0.0;
  double peak_abs_speed_toward_better = 0.0;
  double min_speed_toward_better = 0.0;
  double max_speed_toward_better = 0.0;
  double peak_position_error = 0.0;
  double peak_rotation_error = 0.0;
  double peak_external_force_delta = 0.0;
  double peak_external_moment_delta = 0.0;
  double peak_external_joint_torque_delta = 0.0;
};

struct LogData {
  double time;
  int phase;
  int nullspace_mode;

  Vec3 p_EE;
  Vec3 p_d;
  Vec3 tool_contact_point;
  Vec3 first_contact_tcp;
  Vec3 first_contact_point;
  Vec3 edge_target;
  Vec3 tool_contact_offset_ee;

  Vec3 e_p;
  Vec3 e_R;
  // Minimal current-to-flat-surface tool-axis rotation, resolved in the
  // surface frame [tangent1, tangent2, normal], [rad].
  Vec3 alignment_error_surface;
  // Residual tool-axis-to-surface angle [rad]. Unlike e_R this is referenced to
  // the calibrated plane normal, so it measures alignment quality directly.
  double alignment_angle;

  Vec3 pdot;
  Vec3 pdot_d;
  Vec3 omega;

  Vec3 f;
  Vec3 m;
  Vec3 external_force;
  Vec3 external_moment;
  Vec3 contact_force_bias;
  Vec3 contact_moment_bias;
  double push;

  SigmaDiagnostics sigma;
  double tau_nullspace_norm;
  double nullspace_damping;
  Vec7 tau_cmd;
};

// ====================================================================
// Values passed between the modules
// ====================================================================

struct DesiredMotion {
  Vec3 p_d;
  Vec3 pdot_d;
};

struct CartesianInertiaEstimate {
  Vec3 translational = Vec3::Ones();
  Vec3 rotational = Vec3::Ones();
  bool valid = false;
};

// Everything the one-shot set-up report needs, captured by the control loop at
// the moment the set-up phase ends.
struct SetUpReport {
  // How the phase ended.
  bool stopped_on_moment = false;  // false = hit the duration limit
  double phase_time = 0.0;
  double force_delta_norm = 0.0;
  double moment_delta_norm = 0.0;

  // Pose and contact wrench at the end of the phase.
  Vec3 p_EE = Vec3::Zero();
  Mat3 R_EE = Mat3::Identity();
  Vec3 tool_contact_point = Vec3::Zero();
  Vec3 external_force = Vec3::Zero();
  Vec3 contact_moment_at_edge = Vec3::Zero();

  // Reference captured when the phase started (first contact).
  Vec3 first_contact_tcp = Vec3::Zero();
  Vec3 first_contact_point = Vec3::Zero();
  Mat3 R_contact_start = Mat3::Identity();
  Vec3 contact_force_bias = Vec3::Zero();

  // The diagonal set-up spring that was active, already in base coordinates.
  Mat3 Kp = Mat3::Zero();
  Mat3 Dp = Mat3::Zero();
  Mat3 KR = Mat3::Zero();
  Mat3 DR = Mat3::Zero();
};

// ====================================================================
// Declarations: modules/
// ====================================================================

// Leading and trailing whitespace removed. Shared with the parameter parser.
std::string trim(const std::string& input);

Parameters readParameters(const std::vector<std::string>& filenames);

// The parameter files, in load order, one topic per file. dir may be given
// with or without a trailing slash.
std::vector<std::string> parameterFiles(const std::string& dir = "params");

Array7 vec7ToArray(const Vec7& v);

Array7 filledArray7(double value);

Array6 filledArray6(double value);

double smallestSingularValue(const Mat6x7& J);

Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback);

Mat3 skewMatrix(const Vec3& v);

double smoothStep(double r);

double smoothStepDerivative(double r, double T);

void grindSweep(double t, double amplitude, double stroke_duration,
                double& s, double& s_dot);

double grindStrokeDuration(const Parameters& params);

double setUpPush(const Parameters& params,
                 double phase_time,
                 double start_push,
                 double& push_speed);

Mat6x6 blockDiagonal(const Mat3& translational, const Mat3& rotational);

Mat6x6 offsetAdjoint(const Vec3& r_c);

Mat6x6 adjointTransformedGain(const Mat6x6& pole_gain, const Vec3& r_c);

Mat6x6 blockDiagonalRotation(const Mat3& R);

Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_task_frame, const Mat3& R_task);

CartesianInertiaEstimate computeCartesianInertiaEstimate(
    const Mat7x7& joint_mass,
    const Mat6x7& J,
    const Mat3& R_task);

Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                  const Vec3& stiffness,
                                  double damping_ratio,
                                  const Vec3& min_damping,
                                  double max_damping);

Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired);

Mat3 makeSurfaceFrameFromNormalTangent(const Vec3& normal_input, const Vec3& tangent1_input);

Mat3 makeAlignmentTargetFrame(const Parameters& params);

Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit);

Vec3 surfaceToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target);

Vec3 desiredToolAxisInBase(const Parameters& params, const Mat3& R_alignment_target);

Vec3 currentToolAxisInBase(const Parameters& params, const Mat3& R_EE);

// Minimal rotation vector carrying the current physical tool axis onto the
// signed surface normal. The vector is expressed in the base frame.
Vec3 toolSurfaceAlignmentErrorInBase(
    const Parameters& params,
    const Mat3& R_EE,
    const Mat3& R_alignment_target);

// Residual tool-axis-to-surface angle [rad], non-negative. Distinct from e_R,
// which is referenced to the orientation frozen at the clearance transition.
double toolSurfaceMisalignmentAngle(
    const Parameters& params,
    const Mat3& R_EE,
    const Mat3& R_alignment_target);

Mat3 makeToolOrientationForAlignmentTarget(
    const Parameters& params,
    const Mat3& R_alignment_target,
    const Mat3& R_start);

Vec3 applyRotationalAxisMask(const Parameters& params, Vec3 e_R, const Mat3& R_alignment_target);

// ====================================================================
// Run state -- one object per thing a run carries
// ====================================================================

// The keys the operator can press while a run is in progress. One object so
// the control loop takes a single signals argument instead of seven atomics.
struct KeyboardSignals {
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> proceed_requested{false};
  std::atomic<bool> guide_requested{false};
  std::atomic<char> guidance_menu_key{0};
  std::atomic<bool> guided_hold_selector_pending{false};
  std::atomic<bool> gate_continue{false};
  // Starts parked so the first startup menu owns stdin alone.
  std::atomic<bool> menu_requested{true};

  // Live nullspace tuning while holding: "d 3.5" sets the damping, "k 1.2"
  // the sigma push. NaN means no request is pending; the control loop takes
  // the value, applies it and clears it back to NaN.
  std::atomic<double> nullspace_damping_request{
      std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> nullspace_k_sigma_request{
      std::numeric_limits<double>::quiet_NaN()};
  // Live sigma probe distance, typed in degrees; the loop converts to the
  // radians the law uses. NaN means nothing pending.
  std::atomic<double> nullspace_alpha_deg_request{
      std::numeric_limits<double>::quiet_NaN()};
  // Live mode switch: 0..3 typed while holding. -1 means nothing pending.
  std::atomic<int> nullspace_mode_request{-1};

  // Live set-up impedance, for the t mode: kp1..3 [N/m], kr1..3 [Nm/rad] and
  // the pole r1..3 [mm], indexed [tangent1, tangent2, normal]. NaN = pending
  // nothing. Retuning these rebuilds the gain matrices.
  std::array<std::atomic<double>, 3> setup_kp_request;
  std::array<std::atomic<double>, 3> setup_kr_request;
  std::array<std::atomic<double>, 3> setup_rc_mm_request;

  // Atomics are not copy-initializable, so the arrays are cleared here.
  KeyboardSignals() {
    const double none = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 3; ++i) {
      setup_kp_request[i].store(none);
      setup_kr_request[i].store(none);
      setup_rc_mm_request[i].store(none);
    }
  }
};

// Where the coupled spring's pole is measured from. Both the live and the
// frozen-at-contact pair are carried so the wrench code can honour
// coupled_pole_freeze_at_contact without reaching into the run state.
struct ContactReference {
  Vec3 tcp = Vec3::Zero();              // TCP now
  Vec3 edge = Vec3::Zero();             // controlled contact point now
  Vec3 tcp_at_contact = Vec3::Zero();   // TCP frozen at first contact
  Vec3 edge_at_contact = Vec3::Zero();  // contact point frozen at first contact
};

// Every gain matrix a run uses, all derived from the parameters alone.
struct RunGains {
  Mat3 R_alignment_target = Mat3::Identity();
  Mat3 Kp_approach = Mat3::Zero();
  Mat3 Dp_approach = Mat3::Zero();
  Mat3 KR_approach = Mat3::Zero();
  Mat3 DR_approach = Mat3::Zero();
  Vec3 setup_Kp_active_diag = Vec3::Zero();
  Vec3 setup_Dp_active_diag = Vec3::Zero();
  Mat3 Kp_setup = Mat3::Zero();
  Mat3 Dp_setup = Mat3::Zero();
  Mat3 KR_setup = Mat3::Zero();
  Mat3 DR_setup = Mat3::Zero();
  Mat3 Kp_hold = Mat3::Zero();
  Mat3 Dp_hold = Mat3::Zero();
  Mat3 KR_hold = Mat3::Zero();
  Mat3 DR_hold = Mat3::Zero();
  Mat3 Kp_pause = Mat3::Zero();
  Mat3 Dp_pause = Mat3::Zero();
  Mat3 KR_pause = Mat3::Zero();
  Mat3 DR_pause = Mat3::Zero();
};

// Auto-damping is computed once per phase entry, not every cycle, and cached
// here. The manual matrices in RunGains are both the starting values and the
// fallback when the inertia estimate is unavailable.
struct DampingCache {
  bool approach_computed = false;
  bool setup_computed = false;
  bool hold_computed = false;
  bool pause_computed = false;
  Mat3 Dp_approach = Mat3::Zero();
  Mat3 DR_approach = Mat3::Zero();
  Mat3 Dp_setup = Mat3::Zero();
  Mat3 DR_setup = Mat3::Zero();
  Mat3 Dp_hold = Mat3::Zero();
  Mat3 DR_hold = Mat3::Zero();
  Mat3 Dp_pause = Mat3::Zero();
  Mat3 DR_pause = Mat3::Zero();
  // The set-up damping actually commanded, per axis in the frame it was
  // computed in, so it can be reported once instead of twice.
  bool setup_damping_valid = false;
  Vec3 setup_Dp_used = Vec3::Zero();
  Vec3 setup_DR_used = Vec3::Zero();
  bool approach_damping_valid = false;
  Vec3 approach_Dp_used = Vec3::Zero();
  Vec3 approach_DR_used = Vec3::Zero();
  bool pause_damping_valid = false;
  Vec3 pause_Dp_used = Vec3::Zero();
  Vec3 pause_DR_used = Vec3::Zero();
  bool hold_damping_valid = false;
  Vec3 hold_Dp_used = Vec3::Zero();
  Vec3 hold_DR_used = Vec3::Zero();
};

// What one run leaves behind for the report and the CSV.
struct RunResult {
  bool descend_failed = false;
  Vec7 q_start = Vec7::Zero();
  Vec7 final_q = Vec7::Zero();
  Vec3 final_p_d = Vec3::Zero();
  Vec3 final_p_EE = Vec3::Zero();
  Vec3 final_e_p = Vec3::Zero();
  Vec3 final_e_R = Vec3::Zero();
  std::vector<LogData> log;  // already in chronological order
};

void startKeyboardStopThread(const Parameters& params,
                             KeyboardSignals& signals);

void configureCollisionBehavior(Robot& robot, const Parameters& params);

// The commanded Cartesian wrench: a decoupled pair of 3x3 springs, or, during
// set up with coupled stiffness on, one 6x6 spring moved to the TCP.
//   dx = [position error; orientation error]
//   dv = [velocity error; -omega]
Vec6 computeSpringWrench(const Parameters& params,
                         ControlPhase phase,
                         const Mat3& Kp,
                         const Mat3& Dp,
                         const Mat3& KR,
                         const Mat3& DR,
                         const Mat3& R_alignment_target,
                         const Vec6& dx,
                         const Vec6& dv,
                         const ContactReference& contact);

Vec7 computeNullspaceTorque(
    const Parameters& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq,
    SigmaDiagnostics& sigma);

// True when the hand's measured stroke covers the widths this program
// commands. A homing that ran with the tool clamped measures only the travel
// left over, and every later move() and grasp() is clamped to that stroke.
bool gripperWidthCalibrated(const franka::GripperState& state,
                            const Parameters& params);

// Prints what the bad stroke means and how to clear it. No-op when the
// calibration is good.
void reportGripperCalibration(const franka::GripperState& state,
                              const Parameters& params);

bool openGripper(const Parameters& params, Gripper& gripper);

bool graspTool(const Parameters& params, Gripper& gripper);

// Franka Hand width recalibration (libfranka homing). Opens the fingers
// fully, so a held tool falls unless it is supported.
bool recalibrateGripper(const Parameters& params,
                        Gripper& gripper,
                        bool confirmation_already_received = false);

// Writes the seven joint values into the saved_qinit block of
// params/Q_Init.txt and points q_init_case at it. Returns false if the file
// could not be rewritten; nothing else is touched.
bool saveGuidedPoseAsQInit(const Vec7& q);

// Returns false when the operator chose to quit instead of starting a run.
bool askStartupRunMode(Parameters& params, Robot& robot,
                       const Model& model);

bool performStartupGripperAction(const Parameters& params);

// ====================================================================
// Declarations: run/
// ====================================================================

// run_gains.cpp: all stiffness and damping matrices, from the parameters.
RunGains buildRunGains(const Parameters& params);

// run_gains.cpp: every cached damping matrix set back to its manual value.
DampingCache manualDampingCache(const RunGains& gains);

// run_gains.cpp: recompute the damping of whichever phase group just became
// active, from the task-space inertia. Does nothing on later cycles of the
// same phase.
void updateAutoDamping(const Parameters& params,
                       const RunGains& gains,
                       const Model& model,
                       const RobotState& state,
                       const Mat6x7& J,
                       ControlPhase phase,
                       bool after_contact,
                       bool pause_hold_active,
                       DampingCache& damping);

// run_loop.cpp: one complete run, from start pose to stop.
RunResult runControlLoop(Parameters& params,
                         Robot& robot,
                         const Model& model,
                         RunGains gains,  // by value: live retuning rebuilds it
                         KeyboardSignals& signals);

// run_report.cpp: the post-run printout and the CSV.
void writeRunLogs(const Parameters& params, const RunResult& result);

// run_report.cpp: log file name for the second and later run of one program
// start, e.g. surface_grinding_controller_log_s2.csv.
std::string sessionFileName(const std::string& name, int session);

// Joint posture that puts the EE `standoff` metres back along its own -Z_EE
// from q_target, keeping the orientation. Solved offline, commands no motion.
bool solveStandoffPosture(const Model& model,
                          const RobotState& state,
                          const Array7& q_target,
                          double standoff,
                          Array7& q_standoff);

// False if any joint is outside its limit; joint_out gets the 1-based index.
bool withinJointLimits(const Array7& q, int& joint_out);

bool runManualGuidanceStart(Parameters& params,
                            Robot& robot,
                            const Model& model,
                            std::atomic<bool>& stop_requested,
                            std::atomic<char>& guidance_menu_key,
                            std::atomic<bool>& guided_hold_selector_pending);

const char* phaseName(ControlPhase phase);

const char* nullspaceModeName(NullspaceMode mode);

void printVec3Mm(const char* label, const Vec3& v);

void printVec3Deg(const char* label, const Vec3& v);

void printGainVec(const char* label, const Vec3& v);

void printVec7Deg(const char* label, const Vec7& v);

void printSpatialGain6(const char* label, const Mat6x6& M);

void printSpatialGainEigenvalues(const char* label, const Mat6x6& M);

void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final);

// The gains a phase commands, printed as it starts. The set-up phase and
// the t hold print printSetUpImpedanceLaw instead.
void printPhaseIntro(const Parameters& params,
                     const DampingCache& damping,
                     ControlPhase phase);

// The stiff lock held at a phase gate, printed with the gate message.
void printGateHold(const Parameters& params, const DampingCache& damping);

// The set-up impedance the t mode commands, and what can be retyped while it
// holds. Printed when that hold starts and after every change.
void printSetUpImpedanceLaw(const Parameters& params,
                            const DampingCache& damping,
                            bool tunable);

// The active nullspace law and what can be retyped while holding. Printed
// when a hold starts and whenever the mode is switched live.
void printNullspaceLaw(const Parameters& params);

void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point);

void printApproachOrientDebug(double phase_time,
                              double axis_error_deg,
                              double rot_error_deg);

void printApproachDescendDebug(double phase_time,
                               double distance_mm,
                               double height_mm,
                               double target_height_mm,
                               double force_n);

void printSetUpDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double edge_mm);

void printGrindDebug(double phase_time,
                     double sweep_mm,
                     double track_error_mm,
                     double press_n);

void printHoldDebug(double phase_time,
                    double force_n,
                    double pos_error_mm,
                    double rot_error_deg);

void printSigmaDebug(double phase_time,
                     const SigmaDiagnostics& sigma,
                     double sigma_rate,
                     bool sigma_rate_valid);

void printFinalSummary(const Vec3& final_p_d,
                       const Vec3& final_p_EE,
                       const Vec3& final_e_p,
                       const Vec3& final_e_R,
                       const std::string& csv_file_name);

void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name);

bool writeSigmaDebugToCsv(
    const std::vector<SigmaDebugRow>& debug_data,
    const std::string& csv_file_name);

void reportSetUpResult(const Parameters& params,
                       const Mat3& R_alignment_target,
                       const SetUpReport& report);
