// Include the controller types and helper functions for this experiment.
// The controller_common.h header is included by controller_types.h, so it is not needed here.
#include "controller_helpers.h"
#include "controller_logging.h"
#include "controller_parameters.h"
#include "controller_printing.h"

enum class ControlPhase {
  kOrientToSurface,
  kSearchFirstContact,
  kPostContactAlign,
  kSurfaceImpedance,
  kManualGuide
};

const char* phaseName(ControlPhase phase) {
  switch (phase) {
    case ControlPhase::kOrientToSurface:
      return "orient_to_surface";
    case ControlPhase::kSearchFirstContact:
      return "search_first_contact";
    case ControlPhase::kPostContactAlign:
      return "post_contact_align";
    case ControlPhase::kSurfaceImpedance:
      return "surface_impedance";
    case ControlPhase::kManualGuide:
      return "manual_guide";
  }
  return "unknown";
}

// Main function

int main() {

  try {
    //Read parameters from file, with defaults for missing values.
    Parameters params = readParameters("parameters.txt");
    // Print the parameters to the console for confirmation before starting the experiment.
    printParameters(params);
    if ((params.post_contact_eval_method2_tcp_wrench ||
         params.post_contact_apply_method2_tcp_wrench) &&
        !params.method2_tcp_wrench_saved) {
      fprintf(stderr,
              "Method 2 TCP wrench evaluation/application is enabled, but no saved K_TCP/D_TCP was found.\n");
      fprintf(stderr,
              "Run once with Method 2 evaluation/application disabled so the finite Chasles axis can save the matrices.\n");
      return -1;
    }

    // Connect to the robot. This will block until a connection is established.
    Robot robot(params.robot_ip);

    printf("\nPress Enter to recover/configure and move to q_init.\n");
    std::cin.ignore();

    // If the robot is in an error/reflex state, try to recover automatically.
    try {
      robot.automaticErrorRecovery();
      printf("Robot recovered or already ready.\n");
    } catch (const franka::Exception& e) {
      fprintf(stderr, "Automatic error recovery failed: %s\n", e.what());
      fprintf(stderr, "Please recover/unlock the robot manually in Franka Desk.\n");
      return -1;
    }

    // Configure the collision behavior according to parameters.txt.
    configureCollisionBehavior(robot, params);

    // Move to the initial joint configuration from parameters.txt.
    // the MotionGenerator creates a smooth trajectory from the current joint position to the target joint position.
    // The speed of the motion is determined by the first argument (0.4), which is the maximum joint velocity in rad/s.
    // The second argument is the target joint configuration, which is read from parameters.txt.
    MotionGenerator motion_generator(0.4, params.q_init);

    printf("Moving to q_init...\n");
    // The robot.control() function takes a lambda function as an argument, which is called at the robot control rate (normally 1 kHz).
    // The lambda function is responsible for generating the desired joint torques for each control cycle.
    // In this case, we pass the motion_generator object, which is a callable that generates the desired joint torques to move to the initial configuration.
    robot.control(motion_generator);

    printf("q_init reached.\n");

    if (params.open_gripper_before_run) {
      try {
        printf("Opening gripper after q_init to %.1f mm...\n",
               1000.0 * params.gripper_open_width);
        Gripper gripper(params.robot_ip);
        const bool gripper_opened =
            gripper.move(params.gripper_open_width, params.gripper_open_speed);
        if (!gripper_opened) {
          fprintf(stderr, "Gripper open command returned false.\n");
          if (params.require_gripper_open) {
            fprintf(stderr, "Stopping because require_gripper_open = 1.\n");
            return -1;
          }
        } else {
          printf("Gripper commanded open and holding width.\n");
        }
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper open failed: %s\n", e.what());
        if (params.require_gripper_open) {
          fprintf(stderr, "Stopping because require_gripper_open = 1.\n");
          return -1;
        }
      }
    }

    // Load the robot model, which is needed for the controller and nullspace optimization.
    // The model contains the robot's kinematics and dynamics, and is used to compute the Jacobian, Coriolis forces, etc.
    // The model is loaded once here and passed to the control loop lambda by reference, so it does not need to be reloaded every cycle.
    Model model = robot.loadModel();

    std::atomic<bool> stop_requested(false);
    std::atomic<bool> proceed_requested(false);
    std::atomic<bool> guide_requested(false);
    startKeyboardStopThread(params, stop_requested, proceed_requested, guide_requested);

    if (params.use_manual_guidance_start) {
      printf("\nphase: manual_guidance_start\n");
      printf("Move the robot by hand, then press p+Enter to capture this pose and continue.\n");
      Vec7 manual_guidance_stop_q = Vec7::Zero();
      robot.control([&](const RobotState& state, Duration /*period*/) -> Torques {
        Map<const Vec7> dq(state.dq.data());
        Array7 coriolis_array = model.coriolis(state);
        Map<const Vec7> coriolis(coriolis_array.data());
        Vec7 tau_cmd = coriolis - params.manual_guidance_damping * dq;
        Array7 tau_array = vec7ToArray(tau_cmd);
        if (proceed_requested.load()) {
          printf("\nManual guidance finished with p + Enter. Capturing start pose...\n");
          return MotionFinished(Torques(tau_array));
        }
        if (stop_requested.load()) {
          manual_guidance_stop_q = Map<const Vec7>(state.q.data());
          printf("\nStop requested during manual guidance. Finishing control loop...\n");
          return MotionFinished(Torques(tau_array));
        }
        return Torques(tau_array);
      });
      if (stop_requested.load()) {
        // Show the hand-guided joint configuration at the moment of stopping,
        // in radians (paste-ready as a q_init_* case) and degrees for reading.
        printf("\n=== Manual guidance stop pose ===\n");
        printf("q1..q7 [rad] (paste into a q_init_* case):\n");
        for (int i = 0; i < 7; ++i) {
          printf("  q_init_%d = %.6f\n", i + 1, manual_guidance_stop_q(i));
        }
        printVec7Deg("q1..q7 [deg]", manual_guidance_stop_q);
        return 0;
      }
    }

    // Reading the Initial State from the Robot after the Generated Motion to initial is completed
    // Defining the intial State object from the class Robotstate
    RobotState initial_state = robot.readOnce();


   // Importing the Initial (EE) Pose Data from The Robot Initial State and Mapping in in 4x4 T_initial Matrix
    Map<const Mat4x4> T_initial(initial_state.O_T_EE.data());

    // Extracting the Initial Position and Rotation of the (EE)
    Vec3 p_start = T_initial.block<3, 1>(0, 3);
    Mat3 R_d = T_initial.block<3, 3>(0, 0);

    // Extracting the initial joint angles Array and mapping them to a q_start vector with 7 elements (Vec7)
    Vec7 q_start = Map<const Vec7>(initial_state.q.data());

    // Extracting the initial external wrench (force and torque) estimated by the robot,
    // and storing the initial external force for use in contact search in a initial_external_wrench vector with 6 elements (Vec6).
    // The initial external force is used as a baseline for the contact search.
    // The controller will look for changes in the external force compared to this initial value to detect
    // when contact with the surface has been made.
    // The initial force/moment are the first contact-detection bias.
    Map<const Vec6> initial_external_wrench(initial_state.O_F_ext_hat_K.data());
    Vec3 external_force_start = initial_external_wrench.head<3>();
    Vec3 contact_force_bias = initial_external_wrench.head<3>();
    Vec3 contact_moment_bias = initial_external_wrench.tail<3>();

    // Alignment-target/task frame for orientation and rotational gains:
    //
    //   column 0 = alignment target normal direction
    //   column 1 = first tangent direction
    //   column 2 = second tangent direction
    //
    // In this task-frame mode the diagonal gains from parameters.txt are interpreted
    // in this frame:
    //
    //   Kp_normal   / Dp_normal   -> target-normal stiffness/damping
    //   Kp_tangent1 / Dp_tangent1 -> first tangent stiffness/damping
    //   Kp_tangent2 / Dp_tangent2 -> second tangent stiffness/damping
    //
    // The gains are then transformed to the robot base frame:
    //
    //   K_base = R_alignment_target * K_task * R_alignment_target^T
    //
    // This is the practical idea inspired by spatial impedance: choose simple
    // diagonal impedance values in a task frame, then transform them
    // into the world/base frame.

    // Defining the alignment-target frame in the robot base frame.

    const Mat3 R_alignment_target = makeAlignmentTargetFrame(params);
    // Mutable: recomputed from the re-captured pose when the sequence is
    // restarted via manual re-guidance (g then p during surface_impedance).
    Mat3 R_d_alignment_target =
        makeToolOrientationForAlignmentTarget(params, R_alignment_target, R_d);
    const double orientation_test_extra_tilt_rad =
        params.orientation_test_extra_tilt_deg * M_PI / 180.0;
    Mat3 R_d_orientation_test =
        (std::abs(orientation_test_extra_tilt_rad) > 1e-9)
            ? Eigen::AngleAxisd(orientation_test_extra_tilt_rad, R_alignment_target.col(1)).toRotationMatrix() * R_d
            : R_d_alignment_target;
    // This frame is based on the target normal and tangent hint from
    // parameters.txt. For the self-alignment test it can be a tilted tool
    // orientation target rather than the real table normal.
    Vec3 surface_point_runtime =
        params.use_start_as_surface_point ? p_start : params.surface_point;
    const Vec3 contact_search_direction =
        params.contact_search_use_alignment_target_normal
            ? -R_alignment_target.col(0)
            : normalizedOrFallback(params.contact_search_direction, -R_alignment_target.col(0));
    const Mat3 R_contact_surface =
        makeSurfaceFrameFromNormalTangent(-contact_search_direction, params.alignment_target_tangent1);
    const Mat3 Kp_search = params.contact_search_Kp_diag.asDiagonal();
    const Mat3 Dp_search = params.contact_search_Dp_diag.asDiagonal();
    const Mat3 Kp_post_contact = params.post_contact_Kp_diag.asDiagonal();
    const Mat3 Dp_post_contact = params.post_contact_Dp_diag.asDiagonal();
    // Rotational compliance used only during post_contact_align, in the
    // alignment-target frame (normal/tangent1/tangent2), same convention as the
    // global KR/DR below. Normal (yaw about the push axis) stays at a
    // constrained stiffness; tangent1/tangent2 (the tipping directions) are
    // intentionally very soft so a real contact moment at the pressed edge
    // can passively rotate the tool flat instead of being resisted by the
    // spring.
    const Mat3 KR_post_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.post_contact_KR_diag, R_alignment_target)
        : params.post_contact_KR_diag.asDiagonal();
    const Mat3 DR_post_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.post_contact_DR_diag, R_alignment_target)
        : params.post_contact_DR_diag.asDiagonal();
    ControlPhase initial_phase = ControlPhase::kSurfaceImpedance;
    if (params.orientation_test_only) {
      initial_phase = ControlPhase::kOrientToSurface;
    } else if (params.use_contact_search) {
      initial_phase = (params.use_phase_sequence && params.use_orientation_phase)
          ? ControlPhase::kOrientToSurface
          : ControlPhase::kSearchFirstContact;
    }
    ControlPhase phase = initial_phase;
    double phase_start_time = 0.0;
    bool contact_found = !params.use_contact_search;
    bool contact_search_failed = false;
    double contact_time = 0.0;
    Mat3 R_contact_start = R_d_alignment_target;
    Mat3 R_after_contact_align = R_d_alignment_target;
    Vec3 first_contact_tcp = p_start;
    Vec3 first_contact_point = p_start;
    Vec3 first_touch_candidate_force = external_force_start;
    Vec3 first_touch_candidate_moment = initial_external_wrench.tail<3>();
    Vec3 active_tool_contact_offset_ee = params.tool_contact_point_ee;
    double first_contact_search_distance = 0.0;
    double first_contact_force_delta = 0.0;
    double first_touch_candidate_time = 0.0;
    double first_touch_candidate_distance = 0.0;
    double first_touch_candidate_signal = 0.0;
    bool first_touch_candidate_saved = false;
    double contact_search_candidate_start_time = -1.0;
    double next_debug_time = 0.0;

    auto forceContactDetected = [&](const Vec3& force,
                                    double force_threshold,
                                    double* force_delta_norm) {
      const Vec3 force_delta = force - contact_force_bias;
      *force_delta_norm = force_delta.norm();
      return *force_delta_norm >= force_threshold;
    };

    // Define the desired final target position as the initial position in hold_mode = 1/true
    // If hold_mode = 0/false, use p_start + delta_p for the move mode.
    // If true do this: Which is always the case
    Vec3 p_end = p_start;
    // If false do this:
    if (!params.hold_mode) {
      p_end = p_start + params.delta_p;
    }
    printf("\n=== Start pose ===\n");
    printVec7Deg("q_start", q_start);
    printVec3Mm("p_start", p_start);
    printVec3Mm("p_target", p_end);

    // The controller gains are defined based on the parameters read from parameters.txt.
    // If the constraint is enabled, the gains are transformed from the task
    // frame to the robot base frame using the selected frame rotation.
    // If the constraint is not enabled, the gains are used as-is as diagonal matrices in the robot base frame.
    Mat3 Kp = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Kp_diag, R_alignment_target)
        : params.Kp_diag.asDiagonal();
    Mat3 Dp = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Dp_diag, R_alignment_target)
        : params.Dp_diag.asDiagonal();
    Mat3 KR = params.constraint_enabled
        ? makeSpatialGainMatrix(params.KR_diag, R_alignment_target)
        : params.KR_diag.asDiagonal();
    Mat3 DR = params.constraint_enabled
        ? makeSpatialGainMatrix(params.DR_diag, R_alignment_target)
        : params.DR_diag.asDiagonal();
    Mat3 Kp_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Kp_diag, R_contact_surface)
        : params.Kp_diag.asDiagonal();
    Mat3 Dp_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Dp_diag, R_contact_surface)
        : params.Dp_diag.asDiagonal();
    printf("\n=== Run ===\n");
    printf("phase: %s\n", phaseName(phase));

    // Keep logging bounded so the realtime callback never reallocates on long/manual runs.
    const int log_every_n_cycles = std::max(1, params.log_every_n_cycles);
    const std::size_t max_log_rows = static_cast<std::size_t>(std::max(0, params.max_log_rows));
    std::vector<LogData> log_data;
    log_data.resize(max_log_rows);
    std::size_t control_cycle_count = 0;
    std::size_t log_write_index = 0;
    std::size_t log_rows_written = 0;
    bool log_buffer_wrapped = false;
    // The time variable is used to keep track of the elapsed time since the start of the control loop, and is updated in each cycle based on the period provided by libfranka.
    double time = 0.0;

    // Defining the final values when the duration is over for the the tool position, the desired position, the position error and the rotation error, to be printed at the end of the experiment.
    Vec3 final_p_EE = Vec3::Zero();
    Vec3 final_p_d = Vec3::Zero();
    Vec3 final_e_p = Vec3::Zero();
    Vec3 final_e_R = Vec3::Zero();
    Vec3 final_pdot = Vec3::Zero();
    Vec3 final_omega = Vec3::Zero();
    Vec3 final_tool_contact_point = Vec3::Zero();
    Vec3 final_instant_pole_from_tcp = Vec3::Zero();
    Vec3 final_instant_pole_base = Vec3::Zero();
    Vec3 final_instant_pole_to_edge = Vec3::Zero();
    Vec3 final_instant_axis_dir = Vec3::Zero();
    double final_instant_screw_pitch = 0.0;
    double final_instant_edge_axis_distance = 0.0;
    double final_instant_axis_time = 0.0;
    bool final_instant_pole_valid = false;
    Vec7 final_q = q_start;
    Vec3 last_valid_post_align_axis_point_from_edge = Vec3::Zero();
    Vec3 last_valid_post_align_axis_dir = Vec3::Zero();
    Vec3 last_valid_post_align_edge_from_tcp = Vec3::Zero();
    double last_valid_post_align_axis_edge_distance = 0.0;
    double last_valid_post_align_screw_pitch = 0.0;
    double last_valid_post_align_axis_time = 0.0;
    double last_valid_post_align_omega_norm = 0.0;
    bool last_valid_post_align_axis_valid = false;
    bool last_valid_post_align_axis_stable = false;
    EffectiveMomentFitAccumulator fit_effective_moment;
    std::size_t method2_eval_samples = 0;
    double method2_eval_force_error_sq_sum = 0.0;
    double method2_eval_moment_error_sq_sum = 0.0;
    double method2_eval_force_error_max = 0.0;
    double method2_eval_moment_error_max = 0.0;
    double method2_eval_force_norm_sum = 0.0;
    double method2_eval_moment_norm_sum = 0.0;
    double method2_eval_normal_force_norm_sum = 0.0;
    double method2_eval_normal_moment_norm_sum = 0.0;
    double method2_eval_force_from_translation_norm_sum = 0.0;
    double method2_eval_force_from_rotation_norm_sum = 0.0;
    double method2_eval_moment_from_translation_norm_sum = 0.0;
    double method2_eval_moment_from_rotation_norm_sum = 0.0;
    double method2_eval_final_force_error = 0.0;
    double method2_eval_final_moment_error = 0.0;
    double method2_eval_final_method2_force_norm = 0.0;
    double method2_eval_final_method2_moment_norm = 0.0;
    double method2_eval_final_normal_force_norm = 0.0;
    double method2_eval_final_normal_moment_norm = 0.0;
    double method2_eval_final_force_from_translation_norm = 0.0;
    double method2_eval_final_force_from_rotation_norm = 0.0;
    double method2_eval_final_moment_from_translation_norm = 0.0;
    double method2_eval_final_moment_from_rotation_norm = 0.0;
    std::vector<std::pair<std::string, std::string>> pending_parameter_updates;
    pending_parameter_updates.reserve(96);

    // Print the starting message and instructions for the user before entering the control loop.
    printf("Starting impedance controller:\n");

    // libfranka calls this lambda at the robot control rate, normally 1 kHz.
    //
    // The two callback arguments are provided by libfranka:
    //
    //   state  = RobotState
    //            A structure/object filled by libfranka for this control cycle.
    //            It is refreshed every time this callback is called.
    //
    //            It contains measured/estimated robot data such as:
    //              state.q                    joint positions [rad]
    //              state.dq                   joint velocities [rad/s]
    //              state.O_T_EE               end-effector pose as 16 doubles
    //              state.tau_J                measured joint torques [Nm]
    //              state.tau_ext_hat_filtered estimated external torques [Nm]
    //              state.O_F_ext_hat_K        estimated external wrench [N, Nm]
    //              state.robot_mode           current Franka robot mode
    //              state.current_errors       current error/reflex flags
    //              state.last_motion_errors   previous motion error flags
    //              state.F_T_EE               flange-to-end-effector transform
    //              state.EE_T_K               end-effector-to-stiffness-frame transform
    //
    //   period = Duration
    //            Measured time since the previous callback [s].
    //
    // The lambda must return Torques, i.e. the 7 commanded joint torques
    // tau_cmd [Nm] for this cycle.
    robot.control([&](const RobotState& state,
                      Duration period) -> Torques {

      // increment the time variable by the period of the current control cycle, to keep track of the elapsed time since the start of the control loop.
      time += period.toSec();


      // Computing the joint Velocity vector dq with 7 elements in [rad/s].
      Map<const Vec7> dq(state.dq.data());
      Map<const Vec7> q_current(state.q.data());

      // This Jacobian maps joint velocity dq [rad/s] to end-effector velocity:
      //
      //   xdot = J_EE(q) * dq
      //
      // where xdot = [linear velocity of EE; angular velocity of EE].
      // Because the requested frame is kEndEffector, the Jacobian also uses
      // the EE/TCP frame. Therefore the tool offset is included here too when
      // the robot's F_T_EE is configured correctly.

      // Computing the Jacobian for the current joint configuration q, for the end-effector frame
      std::array<double, 42> jacobian_array = model.zeroJacobian(Frame::kEndEffector, state);
      // Mapping the Jacobian array to a 9x7 Matrix J
      Map<const Mat6x7> J(jacobian_array.data());
      // Computing the end-effector velocity xdot in [m/s] and [rad/s].
      Vec6 xdot = J * dq;
      Vec3 pdot = xdot.head<3>();
      Vec3 omega = xdot.tail<3>();

      // state.O_T_EE is refreshed by libfranka every callback cycle.
      // It is the measured base-to-end-effector transform O_T_EE.
      //
      // Franka stores it as a 16-value array; Eigen::Map views the same memory
      // as a 4x4 matrix without copying it.
      //
      // Units:
      //   p_EE = translation part of O_T_EE [m]
      //   R_EE = rotation part of O_T_EE [-]

      // Compunting the EE position and rotation for the current control cycle
      Map<const Mat4x4> T_EE(state.O_T_EE.data());
      Vec3 p_EE = T_EE.block<3, 1>(0, 3);
      Mat3 R_EE = T_EE.block<3, 3>(0, 0);
      // R_d is the initial tool orientation. R_d_alignment_target is the desired tool
      // orientation parallel to the virtual surface.
      // In orientation_test_only, R_d_orientation_test adds an extra rotation
      // around surface tangent1. For the horizontal table this is base x.
      //
      // With use_phase_sequence = 1:
      //   1. orient_to_surface: rotate at the start position until R_EE ~= R_d_alignment_target
      //   2. search_first_contact: translate along contact_search_direction
      //      while keeping R_EE ~= R_d_alignment_target
      //   3. post_contact_align: keep the first contact point as the virtual
      //      plane point and continue rotational alignment
      //   4. surface_impedance: normal surface impedance experiment
      Map<const Vec6> external_wrench(state.O_F_ext_hat_K.data());
      // The control logic is as follows:
      // If contact search is enabled, the controller starts in a mode where it moves the end-effector
      // in the specified contact search direction at a constant speed until either contact is detected
      // or the maximum search distance is reached.
      // Contact is detected when the change in external force compared to the initial external force exceeds
      // a specified threshold.
      // If contact is detected, the current end-effector position is set as the point on the virtual surface,
      //and the controller switches to impedance control mode using that surface point.
      // If the maximum search distance is reached without detecting contact, the controller stops
      // and logs the data up to that point.

      // Defining the desired motion object from the DesiredMotion Class p_d and p_dot_d, which will be computed
      // based on the current mode (searching for contact or impedance control) and the current time in the experiment.
      // The computeDesiredMotion function computes the desired position and velocity based on the parameters,
      // the elapsed time, the initial position, the current EE position, the surface frame, and the surface point.
      DesiredMotion desired;
      // The impedance_time variable is used to keep track of the time since the start of impedance control,
      // which is used for logging and for the duration condition to end the experiment.
      double impedance_time = time;
      const bool orienting_to_surface = (phase == ControlPhase::kOrientToSurface);
      const Mat3& R_d_phase =
          params.orientation_test_only ? R_d_orientation_test : R_d_alignment_target;
      const Vec3 external_force = external_wrench.head<3>();
      const Vec3 external_moment = external_wrench.tail<3>();

      // Manual re-guidance from the surface-impedance hold. Pressing g during
      // surface_impedance relaxes the tool into gravity compensation so it can
      // be moved by hand; pressing p then re-captures the pose and restarts the
      // whole phase sequence (orient/search/align/impedance) from there. e still
      // stops. Note: relaxing from a pressed contact releases the contact force,
      // so expect the tool to ease off the surface when guidance starts.
      if (phase == ControlPhase::kSurfaceImpedance && guide_requested.load()) {
        guide_requested.store(false);
        proceed_requested.store(false);
        phase = ControlPhase::kManualGuide;
        phase_start_time = time;
        printf("\nphase: manual_guide (move the tool by hand; p+Enter restarts the sequence, e+Enter stops)\n");
      }
      if (phase == ControlPhase::kManualGuide) {
        Array7 coriolis_array = model.coriolis(state);
        Map<const Vec7> coriolis(coriolis_array.data());
        Vec7 tau_cmd = coriolis - params.manual_guidance_damping * dq;
        Array7 tau_array = vec7ToArray(tau_cmd);
        if (stop_requested.load()) {
          printf("\nStop requested with e + Enter. Finishing control loop...\n");
          return MotionFinished(Torques(tau_array));
        }
        if (proceed_requested.load()) {
          proceed_requested.store(false);
          // Re-capture the hand-moved pose as the new start pose and reset all
          // pose-dependent setup plus the phase machine, so the sequence runs
          // again from here. Gains depend only on the params-defined alignment
          // frame, so they do not need recomputing.
          p_start = p_EE;
          R_d = R_EE;
          q_start = q_current;
          R_d_alignment_target =
              makeToolOrientationForAlignmentTarget(params, R_alignment_target, R_d);
          R_d_orientation_test =
              (std::abs(orientation_test_extra_tilt_rad) > 1e-9)
                  ? Eigen::AngleAxisd(orientation_test_extra_tilt_rad,
                                      R_alignment_target.col(1)).toRotationMatrix() * R_d
                  : R_d_alignment_target;
          surface_point_runtime =
              params.use_start_as_surface_point ? p_start : params.surface_point;
          p_end = params.hold_mode ? p_start : Vec3(p_start + params.delta_p);
          external_force_start = external_force;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          first_contact_tcp = p_start;
          first_contact_point = p_start;
          first_touch_candidate_force = external_force;
          first_touch_candidate_moment = external_moment;
          R_contact_start = R_d_alignment_target;
          R_after_contact_align = R_d_alignment_target;
          active_tool_contact_offset_ee = params.tool_contact_point_ee;
          // Reset phase machine and per-run accumulators (time keeps running;
          // phase-relative timers are re-zeroed to the current time).
          phase = initial_phase;
          phase_start_time = time;
          next_debug_time = time;
          contact_found = !params.use_contact_search;
          contact_search_failed = false;
          contact_time = 0.0;
          first_contact_search_distance = 0.0;
          first_contact_force_delta = 0.0;
          first_touch_candidate_time = 0.0;
          first_touch_candidate_distance = 0.0;
          first_touch_candidate_signal = 0.0;
          first_touch_candidate_saved = false;
          contact_search_candidate_start_time = -1.0;
          last_valid_post_align_axis_point_from_edge = Vec3::Zero();
          last_valid_post_align_axis_dir = Vec3::Zero();
          last_valid_post_align_edge_from_tcp = Vec3::Zero();
          last_valid_post_align_axis_edge_distance = 0.0;
          last_valid_post_align_screw_pitch = 0.0;
          last_valid_post_align_axis_time = 0.0;
          last_valid_post_align_omega_norm = 0.0;
          last_valid_post_align_axis_valid = false;
          last_valid_post_align_axis_stable = false;
          fit_effective_moment = EffectiveMomentFitAccumulator();
          // Method 2 TCP-wrench eval stats: reset so the final compare reflects
          // only the most recent run, not a blend across restarts.
          method2_eval_samples = 0;
          method2_eval_force_error_sq_sum = 0.0;
          method2_eval_moment_error_sq_sum = 0.0;
          method2_eval_force_error_max = 0.0;
          method2_eval_moment_error_max = 0.0;
          method2_eval_force_norm_sum = 0.0;
          method2_eval_moment_norm_sum = 0.0;
          method2_eval_normal_force_norm_sum = 0.0;
          method2_eval_normal_moment_norm_sum = 0.0;
          method2_eval_force_from_translation_norm_sum = 0.0;
          method2_eval_force_from_rotation_norm_sum = 0.0;
          method2_eval_moment_from_translation_norm_sum = 0.0;
          method2_eval_moment_from_rotation_norm_sum = 0.0;
          printf("\n=== Restarting sequence from re-guided pose ===\n");
          printVec7Deg("q_start", q_start);
          printVec3Mm("p_start", p_start);
          printf("phase: %s\n", phaseName(phase));
        }
        return Torques(tau_array);
      }

      Vec3 tool_contact_offset_ee = Vec3::Zero();
      if (params.use_tool_contact_point_control) {
        if (contact_found || phase == ControlPhase::kPostContactAlign) {
          tool_contact_offset_ee = active_tool_contact_offset_ee;
        } else if (params.auto_select_tool_contact_edge) {
          const Vec3 positive_edge = R_EE * params.tool_contact_point_ee;
          const Vec3 negative_edge = R_EE * (-params.tool_contact_point_ee);
          tool_contact_offset_ee =
              (positive_edge.dot(contact_search_direction) >=
               negative_edge.dot(contact_search_direction))
                  ? params.tool_contact_point_ee
                  : -params.tool_contact_point_ee;
        } else {
          tool_contact_offset_ee = params.tool_contact_point_ee;
        }
      }
      const Vec3 tool_contact_point = p_EE + R_EE * tool_contact_offset_ee;
      Vec3 edge_target_debug = first_contact_point;
      double post_contact_push_debug = 0.0;

      const Vec3 e_R_to_alignment_target =
          applyRotationalAxisMask(params, orientationError(R_EE, R_d_phase), R_alignment_target);
      const Vec3 tool_axis_current = currentToolAxisInBase(params, R_EE).normalized();
      const Vec3 tool_axis_target = desiredToolAxisInBase(params, R_alignment_target).normalized();
      const double tool_axis_dot = std::max(
          -1.0,
          std::min(1.0, tool_axis_current.dot(tool_axis_target)));
      const double tool_axis_alignment_error = std::acos(tool_axis_dot);
      if (orienting_to_surface &&
          params.debug_period > 0.0 &&
          time >= next_debug_time) {
        printOrientDebug(time - phase_start_time,
                          (180.0 / M_PI) * tool_axis_alignment_error,
                          (180.0 / M_PI) * e_R_to_alignment_target.norm());
        next_debug_time = time + params.debug_period;
      }
      double force_delta_norm = 0.0;
      const bool alignment_contact_detected =
          orienting_to_surface &&
          params.detect_contact_during_alignment &&
          !params.orientation_test_only &&
          forceContactDetected(external_force,
                               params.alignment_contact_force_threshold,
                               &force_delta_norm);

      if (alignment_contact_detected) {
        active_tool_contact_offset_ee = tool_contact_offset_ee;
        surface_point_runtime = tool_contact_point;
        first_contact_tcp = p_EE;
        first_contact_point = tool_contact_point;
        contact_found = true;
        contact_time = time;
        R_contact_start = R_EE;
        phase = params.use_phase_sequence
            ? ControlPhase::kPostContactAlign
            : ControlPhase::kSurfaceImpedance;
        phase_start_time = time;
        contact_force_bias = external_force_start;
        contact_moment_bias = external_moment;
        printf("\nContact during alignment: force=%.1f N\n", force_delta_norm);
        printContactEdgeDebug(active_tool_contact_offset_ee, first_contact_tcp, first_contact_point);
        printf("phase: %s\n", phaseName(phase));
      }

      if (orienting_to_surface &&
          !alignment_contact_detected &&
          !params.orientation_test_only &&
          (time - phase_start_time) >= params.orient_phase_min_time &&
          tool_axis_alignment_error <= params.orient_phase_error_threshold) {
        phase = params.use_contact_search
            ? ControlPhase::kSearchFirstContact
            : ControlPhase::kSurfaceImpedance;
        phase_start_time = time;
        contact_search_candidate_start_time = -1.0;
        next_debug_time = time;
        contact_force_bias = external_force;
        contact_moment_bias = external_moment;
        printf("\nOrientation reached: axis_err=%.1f deg\n",
               (180.0 / M_PI) * tool_axis_alignment_error);
        printf("phase: %s\n", phaseName(phase));
      }

      if (phase == ControlPhase::kPostContactAlign) {
        const double post_align_time = time - phase_start_time;
        const double post_moment_delta_norm = (external_moment - contact_moment_bias).norm();
        const double post_force_delta_norm = (external_force - contact_force_bias).norm();
        // Real commanded rotation error for this cycle, same convention the
        // main control law uses (orientationError(current, desired)) -- the
        // old single-sample gain fit had this backwards.
        const Vec3 e_R_post_align = applyRotationalAxisMask(
            params, orientationError(R_EE, R_contact_start), R_alignment_target);
        const Vec3 e_R_post_align_task = R_alignment_target.transpose() * e_R_post_align;
        const Vec3 omega_post_align_task = R_alignment_target.transpose() * omega;
        const Vec3 gain_force_reference =
            first_touch_candidate_saved ? first_touch_candidate_force : contact_force_bias;
        const Vec3 gain_moment_reference =
            first_touch_candidate_saved ? first_touch_candidate_moment : contact_moment_bias;
        // Thesis effective-moment fit:
        //   M_C = K_rt*dx_C + D_rt*v_C + K_R*dtheta + D_R*omega,
        // with M_C moved from the TCP to the active contact edge by
        //   M_C = m - r_C x f,  r_C = p_C - p_EE.
        const Vec3 edge_from_tcp = tool_contact_point - p_EE;
        const Vec3 force_delta = external_force - gain_force_reference;
        const Vec3 moment_delta = external_moment - gain_moment_reference;
        const Vec3 contact_moment_at_edge =
            moment_delta - edge_from_tcp.cross(force_delta);
        const Vec3 contact_displacement_task =
            R_alignment_target.transpose() * (tool_contact_point - first_contact_point);
        const Vec3 contact_velocity_task =
            R_alignment_target.transpose() * (pdot + omega.cross(edge_from_tcp));
        const Vec3 contact_moment_task =
            R_alignment_target.transpose() * contact_moment_at_edge;
        fit_effective_moment.addSample(contact_displacement_task,
                                       contact_velocity_task,
                                       e_R_post_align_task,
                                       omega_post_align_task,
                                       contact_moment_task);
        Vec3 instant_pole_from_tcp = Vec3::Zero();
        const bool instant_pole_valid =
            computeInstantaneousPoleFromTcp(pdot, omega, &instant_pole_from_tcp);
        const Vec3 instant_pole_base = p_EE + instant_pole_from_tcp;
        const Vec3 instant_axis_point_from_edge =
            instant_pole_base - tool_contact_point;
        const Vec3 instant_axis_dir =
            instant_pole_valid ? omega.normalized() : Vec3::Zero();
        const double instant_screw_pitch =
            instant_pole_valid ? instantaneousScrewPitch(pdot, omega) : 0.0;
        const double instant_edge_axis_distance =
            instant_pole_valid
                ? pointDistanceToAxis(tool_contact_point, instant_pole_base, omega)
                : 0.0;
        const double omega_norm = omega.norm();
        const bool best_axis_candidate_valid =
            instant_pole_valid &&
            omega_norm >= params.post_contact_best_axis_min_omega;
        const bool best_axis_candidate_stable =
            post_align_time >= params.post_contact_best_axis_min_time;
        bool update_best_axis = false;
        if (best_axis_candidate_valid) {
          if (best_axis_candidate_stable) {
            update_best_axis =
                !last_valid_post_align_axis_valid ||
                !last_valid_post_align_axis_stable ||
                instant_edge_axis_distance < last_valid_post_align_axis_edge_distance;
          } else if (!last_valid_post_align_axis_valid &&
                     omega_norm > last_valid_post_align_omega_norm) {
            update_best_axis = true;
          }
        }
        if (update_best_axis) {
          last_valid_post_align_axis_point_from_edge = instant_axis_point_from_edge;
          last_valid_post_align_axis_dir = instant_axis_dir;
          last_valid_post_align_edge_from_tcp = tool_contact_point - p_EE;
          last_valid_post_align_axis_edge_distance = instant_edge_axis_distance;
          last_valid_post_align_screw_pitch = instant_screw_pitch;
          last_valid_post_align_axis_time = post_align_time;
          last_valid_post_align_omega_norm = omega_norm;
          last_valid_post_align_axis_valid = true;
          last_valid_post_align_axis_stable = best_axis_candidate_stable;
        }
        if (params.debug_period > 0.0 && time >= next_debug_time) {
          // actual_tip_deg: measured rotation away from the orientation held
          // at first contact -- shows whether/how much the tool has
          // passively tipped so far, not just where it is now.
          const double actual_tip_deg =
              (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm();
          const double edge_from_contact_mm =
              1000.0 * (tool_contact_point - first_contact_point).norm();
          const Vec3 pole_nearest_edge_from_edge =
              instant_pole_valid
                  ? Vec3(nearestPointOnAxis(tool_contact_point, instant_pole_base, omega) -
                         tool_contact_point)
                  : Vec3::Zero();
          printAlignDebug(post_align_time,
                           actual_tip_deg,
                           post_force_delta_norm,
                           post_moment_delta_norm,
                           params.post_contact_moment_threshold,
                           edge_from_contact_mm,
                           instant_pole_valid,
                           1000.0 * pole_nearest_edge_from_edge,
                           1000.0 * instant_edge_axis_distance);
          next_debug_time = time + params.debug_period;
        }
        const bool moment_contact_reached =
            post_align_time >= params.post_contact_align_min_time &&
            post_moment_delta_norm >= params.post_contact_moment_threshold;
        const bool max_align_time_reached =
            post_align_time >= params.post_contact_align_duration;

        if (moment_contact_reached || max_align_time_reached) {
          const double actual_tip_deg =
              (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm();
          R_after_contact_align = R_EE;
          surface_point_runtime = p_EE;
          phase = ControlPhase::kSurfaceImpedance;
          phase_start_time = time;
          const Vec3 edge_from_contact_mm =
              1000.0 * (tool_contact_point - first_contact_point);
          const Vec3 tcp_from_contact_mm =
              1000.0 * (p_EE - first_contact_point);
          printf("\n=== Post-align result ===\n");
          printf("stop: %s | t=%.1f s | tip=%.1f deg | F=%.1f N | M=%.1f Nm\n",
                 moment_contact_reached ? "moment" : "time",
                 post_align_time,
                 actual_tip_deg,
                 post_force_delta_norm,
                 post_moment_delta_norm);
          printf("edge_from_contact = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
                 edge_from_contact_mm(0),
                 edge_from_contact_mm(1),
                 edge_from_contact_mm(2),
                 edge_from_contact_mm.norm());
          printf("tcp_from_contact  = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
                 tcp_from_contact_mm(0),
                 tcp_from_contact_mm(1),
                 tcp_from_contact_mm(2),
                 tcp_from_contact_mm.norm());
          // Steady contact wrench and motion resolved into the alignment-target
          // frame [normal, tangent1, tangent2], the same frame the post-contact
          // gains and Method 1's quasi-static limits use. This gives real
          // per-axis force-vs-displacement and moment-vs-angle pairs so
          // quasi_force_limit/quasi_displacement_limit (and the moment/angle
          // limits) can be set from measured data instead of from a target
          // ratio. Force is paired with TCP displacement (where the translational
          // spring acts); the contact moment is taken at the pressed edge and
          // paired with the tip angle. Effective stiffness is left n/a when the
          // displacement/angle is too small to divide.
          {
            const Vec3 force_surf =
                R_alignment_target.transpose() * (external_force - contact_force_bias);
            const Vec3 moment_surf =
                R_alignment_target.transpose() * contact_moment_at_edge;
            const Vec3 tcp_disp_surf =
                R_alignment_target.transpose() * (p_EE - first_contact_tcp);
            const Vec3 edge_disp_surf =
                R_alignment_target.transpose() * (tool_contact_point - first_contact_point);
            const Vec3 tip_surf =
                R_alignment_target.transpose() * orientationError(R_EE, R_contact_start);
            auto fmtRatio = [](char* out, size_t n, double num, double den,
                               double den_floor) {
              if (std::abs(den) < den_floor) {
                snprintf(out, n, "%9s", "n/a");
              } else {
                snprintf(out, n, "%9.1f", num / den);
              }
            };
            char kp[3][16];
            char kr[3][16];
            for (int i = 0; i < 3; ++i) {
              fmtRatio(kp[i], sizeof(kp[i]), force_surf(i), tcp_disp_surf(i), 5e-5);
              fmtRatio(kr[i], sizeof(kr[i]), moment_surf(i), tip_surf(i), 5e-4);
            }
            printf("\n=== Post-align surface-frame breakdown (Method 1 inputs) ===\n");
            printf("frame: alignment-target [normal, tangent1, tangent2]\n");
            printf("force        [N]      = [%+9.2f, %+9.2f, %+9.2f]\n",
                   force_surf(0), force_surf(1), force_surf(2));
            printf("tcp_disp     [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
                   1000.0 * tcp_disp_surf(0), 1000.0 * tcp_disp_surf(1),
                   1000.0 * tcp_disp_surf(2));
            printf("edge_disp    [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
                   1000.0 * edge_disp_surf(0), 1000.0 * edge_disp_surf(1),
                   1000.0 * edge_disp_surf(2));
            printf("Kp_eff=F/tcp [N/m]    = [%s, %s, %s]\n", kp[0], kp[1], kp[2]);
            printf("moment@edge  [Nm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
                   moment_surf(0), moment_surf(1), moment_surf(2));
            printf("tip_angle    [deg]    = [%+9.2f, %+9.2f, %+9.2f]\n",
                   (180.0 / M_PI) * tip_surf(0), (180.0 / M_PI) * tip_surf(1),
                   (180.0 / M_PI) * tip_surf(2));
            printf("KR_eff=M/ang [Nm/rad] = [%s, %s, %s]\n", kr[0], kr[1], kr[2]);
          }
          // One exact axis for the whole alignment motion (Chasles'
          // theorem), computed only from the start and end edge pose --
          // unlike the per-cycle instantaneous pole below, this isn't an
          // average of noisy samples and isn't sensitive to which cycle
          // happened to look "cleanest".
          const FiniteScrewAxis finite_axis = computeFiniteScrewAxis(
              first_contact_point, R_contact_start, tool_contact_point, R_EE);
          if (finite_axis.valid) {
            printf("finite_axis: angle=%.1f deg | axis_from_edge=[%+.1f, %+.1f, %+.1f] mm | dir=[%+.3f, %+.3f, %+.3f] | pitch=%+.1f mm/rad\n",
                   (180.0 / M_PI) * finite_axis.angle,
                   1000.0 * finite_axis.axis_point_from_start(0),
                   1000.0 * finite_axis.axis_point_from_start(1),
                   1000.0 * finite_axis.axis_point_from_start(2),
                   finite_axis.axis_dir(0),
                   finite_axis.axis_dir(1),
                   finite_axis.axis_dir(2),
                   1000.0 * finite_axis.pitch);
            char edge_x[32], edge_y[32], edge_z[32];
            char dir_x[32], dir_y[32], dir_z[32];
            char pitch_buf[32];
            snprintf(edge_x, sizeof(edge_x), "%.4f", finite_axis.axis_point_from_start(0));
            snprintf(edge_y, sizeof(edge_y), "%.4f", finite_axis.axis_point_from_start(1));
            snprintf(edge_z, sizeof(edge_z), "%.4f", finite_axis.axis_point_from_start(2));
            snprintf(dir_x, sizeof(dir_x), "%.3f", finite_axis.axis_dir(0));
            snprintf(dir_y, sizeof(dir_y), "%.3f", finite_axis.axis_dir(1));
            snprintf(dir_z, sizeof(dir_z), "%.3f", finite_axis.axis_dir(2));
            snprintf(pitch_buf, sizeof(pitch_buf), "%.4f", finite_axis.pitch);
            pending_parameter_updates.emplace_back("last_chasles_axis_from_edge_x", edge_x);
            pending_parameter_updates.emplace_back("last_chasles_axis_from_edge_y", edge_y);
            pending_parameter_updates.emplace_back("last_chasles_axis_from_edge_z", edge_z);
            pending_parameter_updates.emplace_back("last_chasles_axis_dir_x", dir_x);
            pending_parameter_updates.emplace_back("last_chasles_axis_dir_y", dir_y);
            pending_parameter_updates.emplace_back("last_chasles_axis_dir_z", dir_z);
            pending_parameter_updates.emplace_back("last_chasles_axis_pitch", pitch_buf);
            printf("(last_chasles_axis_* queued from this run's finite Chasles axis)\n");
          } else {
            printf("finite_axis: angle=%.1f deg, too small for a well-defined axis\n",
                   (180.0 / M_PI) * finite_axis.angle);
          }

          printf("\n=== Best axis compare ===\n");
          if (last_valid_post_align_axis_valid) {
            const Vec3 last_best_axis_dir =
                normalizedOrFallback(params.last_best_axis_dir, Vec3(1.0, 0.0, 0.0));
            const Vec3 axis_point_error =
                last_valid_post_align_axis_point_from_edge -
                params.last_best_axis_from_edge;
            const double axis_dir_dot =
                std::abs(std::max(
                    -1.0,
                    std::min(1.0, last_valid_post_align_axis_dir.dot(last_best_axis_dir))));
            const double axis_dir_error_deg =
                (180.0 / M_PI) * std::acos(axis_dir_dot);
            const double desired_axis_edge_distance =
                pointDistanceToAxis(
                    Vec3::Zero(),
                    params.last_best_axis_from_edge,
                    last_best_axis_dir);
            const double axis_edge_error =
                last_valid_post_align_axis_edge_distance -
                desired_axis_edge_distance;
            const double pitch_error =
                last_valid_post_align_screw_pitch - params.last_best_axis_pitch;
            printf("last_best_target: axis_from_edge=[%+.1f, %+.1f, %+.1f] mm | dir=[%+.3f, %+.3f, %+.3f] | pitch=%+.1f mm/rad\n",
                   1000.0 * params.last_best_axis_from_edge(0),
                   1000.0 * params.last_best_axis_from_edge(1),
                   1000.0 * params.last_best_axis_from_edge(2),
                   last_best_axis_dir(0),
                   last_best_axis_dir(1),
                   last_best_axis_dir(2),
                   1000.0 * params.last_best_axis_pitch);
            printf("measured: axis_from_edge=[%+.1f, %+.1f, %+.1f] mm | dir=[%+.3f, %+.3f, %+.3f] | pitch=%+.1f mm/rad  (%s, t=%.2fs, w=%.2f rad/s)\n",
                   1000.0 * last_valid_post_align_axis_point_from_edge(0),
                   1000.0 * last_valid_post_align_axis_point_from_edge(1),
                   1000.0 * last_valid_post_align_axis_point_from_edge(2),
                   last_valid_post_align_axis_dir(0),
                   last_valid_post_align_axis_dir(1),
                   last_valid_post_align_axis_dir(2),
                   1000.0 * last_valid_post_align_screw_pitch,
                   last_valid_post_align_axis_stable ? "stable" : "transient_fallback",
                   last_valid_post_align_axis_time,
                   last_valid_post_align_omega_norm);
            printf("diff:     point=%.1f mm | dir=%.1f deg | edge=%+.1f mm | pitch=%+.1f mm/rad\n",
                   1000.0 * axis_point_error.norm(),
                   axis_dir_error_deg,
                   1000.0 * axis_edge_error,
                   1000.0 * pitch_error);
            char edge_x[32], edge_y[32], edge_z[32];
            char dir_x[32], dir_y[32], dir_z[32];
            char pitch_buf[32];
            snprintf(edge_x, sizeof(edge_x), "%.4f", last_valid_post_align_axis_point_from_edge(0));
            snprintf(edge_y, sizeof(edge_y), "%.4f", last_valid_post_align_axis_point_from_edge(1));
            snprintf(edge_z, sizeof(edge_z), "%.4f", last_valid_post_align_axis_point_from_edge(2));
            snprintf(dir_x, sizeof(dir_x), "%.3f", last_valid_post_align_axis_dir(0));
            snprintf(dir_y, sizeof(dir_y), "%.3f", last_valid_post_align_axis_dir(1));
            snprintf(dir_z, sizeof(dir_z), "%.3f", last_valid_post_align_axis_dir(2));
            snprintf(pitch_buf, sizeof(pitch_buf), "%.4f", last_valid_post_align_screw_pitch);
            pending_parameter_updates.emplace_back("last_best_axis_from_edge_x", edge_x);
            pending_parameter_updates.emplace_back("last_best_axis_from_edge_y", edge_y);
            pending_parameter_updates.emplace_back("last_best_axis_from_edge_z", edge_z);
            pending_parameter_updates.emplace_back("last_best_axis_dir_x", dir_x);
            pending_parameter_updates.emplace_back("last_best_axis_dir_y", dir_y);
            pending_parameter_updates.emplace_back("last_best_axis_dir_z", dir_z);
            pending_parameter_updates.emplace_back("last_best_axis_pitch", pitch_buf);
            printf("(last_best_axis_* queued from this run's best instantaneous axis)\n");

          } else {
            printf("no stable rotation measured | edge_norm=%.1f mm\n", edge_from_contact_mm.norm());
          }

          if (params.print_gain_suggestion_diagnostics) {
            std::array<double, 49> mass_array = model.mass(state);
            Map<const Mat7x7> joint_mass(mass_array.data());
            const CartesianInertiaEstimate cartesian_inertia =
                computeCartesianInertiaEstimate(joint_mass, J, R_alignment_target);
            const Vec3 translational_inertia = cartesian_inertia.valid
                ? cartesian_inertia.translational
                : params.quasi_effective_mass;
            const Vec3 rotational_inertia = cartesian_inertia.valid
                ? cartesian_inertia.rotational
                : params.quasi_effective_inertia;
            const DiagonalGainSet quasi_gains = computeQuasiStaticGains(
                params, translational_inertia, rotational_inertia);
            printf("=== Gain-suggestion diagnostics only ===\n");
            printf("Reporting only: suggestions are not applied online. Edit parameters.txt manually to test them.\n");

            printf("\n=== Method 1: Quasi-static candidate gains ===\n");
            printf("formula: Kp=Fmax/dxmax, KR=Mmax/dtheta_max, D=2*zeta*sqrt(M*K)\n");
            printf("inertia_source: %s\n",
                   cartesian_inertia.valid
                       ? "libfranka Lambda task-frame diagonal"
                       : "parameters fallback");
            printGainVec("M_eff_trans", translational_inertia);
            printGainVec("I_eff_rot", rotational_inertia);
            printGainVec("Fmax_N", params.quasi_force_limit);
            printGainVec("dxmax_m", params.quasi_displacement_limit);
            printGainVec("Mmax_Nm", params.quasi_moment_limit);
            printGainVec("dtheta_max_rad", params.quasi_angle_limit);
            printGainVec("Kp_suggested", quasi_gains.Kp);
            printGainVec("Dp_suggested", quasi_gains.Dp);
            printGainVec("KR_suggested", quasi_gains.KR);
            printGainVec("DR_suggested", quasi_gains.DR);
            printGainVec("Kp_active_post_contact", params.post_contact_Kp_diag);
            printGainVec("Dp_active_post_contact", params.post_contact_Dp_diag);
            printGainVec("KR_active_post_contact", params.post_contact_KR_diag);
            printGainVec("DR_active_post_contact", params.post_contact_DR_diag);

            printf("\n=== Method 2: Adjoint pole-based candidate gains ===\n");
            // The pole for the K/D adjoint suggestion comes from the stable
            // Chasles finite screw axis (computed from the start/end edge pose),
            // not the per-cycle best instantaneous pole, which jumps between
            // cycles and can be numerically unreliable. The separate
            // "=== Best axis compare ===" block above keeps the instantaneous
            // best axis for the run-to-run evaluation.
            Vec3 pole_point;
            if (finite_axis.valid) {
              const Vec3 axis_point_base =
                  first_contact_point + finite_axis.axis_point_from_start;
              pole_point = nearestPointOnAxis(
                  tool_contact_point, axis_point_base, finite_axis.axis_dir);
            } else {
              pole_point = tool_contact_point + params.last_chasles_axis_from_edge;
            }
            const Vec3 r_c_base = p_EE - pole_point;
            // Use the post_contact_align gains from parameters.txt directly.
            // In this self-alignment experiment R_alignment_target may be an intentional
            // tilted tool-orientation target, not the real table/contact plane,
            // so do not rotate translational gains through that frame here.
            Mat6x6 K_pole = Mat6x6::Zero();
            K_pole.block<3, 3>(0, 0) = Kp_post_contact;
            K_pole.block<3, 3>(3, 3) = KR_post_contact;
            Mat6x6 D_pole = Mat6x6::Zero();
            D_pole.block<3, 3>(0, 0) = Dp_post_contact;
            D_pole.block<3, 3>(3, 3) = DR_post_contact;
            const Mat6x6 K_tcp_base = adjointTransformedGain(K_pole, r_c_base);
            const Mat6x6 D_tcp_base = adjointTransformedGain(D_pole, r_c_base);
            if (finite_axis.valid && !params.post_contact_apply_method2_tcp_wrench) {
              std::vector<std::pair<std::string, std::string>> method2_updates;
              method2_updates.emplace_back("method2_tcp_wrench_saved", "1");
              appendMat6ParameterUpdates(method2_updates, "method2_K_tcp", K_tcp_base);
              appendMat6ParameterUpdates(method2_updates, "method2_D_tcp", D_tcp_base);
              pending_parameter_updates.insert(
                  pending_parameter_updates.end(),
                  method2_updates.begin(),
                  method2_updates.end());
              printf("(Method 2 K_TCP/D_TCP queued for post_contact_eval_method2_tcp_wrench = 1)\n");
            }
            // Method 2 expresses the SAME diagonal post-contact spring at the
            // tool tip for two poles, via  K_TCP = Ad(r_c)^T K_pole Ad(r_c),
            // r_c = TCP - pole:
            //   (A) measured Chasles screw axis -> diagnostic of the motion.
            //   (B) commanded manual pole       -> the spring actually applied
            //       in manual-pole apply mode (apply=1, diag=0, manual_pole=1).
            // Each matrix is ONE 6x6 (rows fx..mz = wrench, cols tx..rz =
            // displacement); the off-diagonal quadrants are the coupling.
            const bool manual_pole_commanded =
                params.post_contact_apply_method2_tcp_wrench &&
                !params.use_in_method2_k_d_diag &&
                params.use_manual_method2_pole;
            const Vec3 edge_ref_cmd = params.method2_pole_freeze_at_contact
                                          ? first_contact_point
                                          : tool_contact_point;
            const Vec3 tcp_ref_cmd = params.method2_pole_freeze_at_contact
                                         ? first_contact_tcp
                                         : p_EE;
            const Vec3 pole_cmd =
                edge_ref_cmd + params.manual_method2_pole_from_edge;
            const Vec3 r_c_cmd = tcp_ref_cmd - pole_cmd;
            const Mat6x6 K_tcp_cmd = adjointTransformedGain(K_pole, r_c_cmd);
            const Mat6x6 D_tcp_cmd = adjointTransformedGain(D_pole, r_c_cmd);

            printf("source spring K_pole/D_pole (diagonal, from parameters):\n");
            printGainVec("  Kp [N/m]    ", params.post_contact_Kp_diag);
            printGainVec("  KR [Nm/rad] ", params.post_contact_KR_diag);
            printGainVec("  Dp [Ns/m]   ", params.post_contact_Dp_diag);
            printGainVec("  DR [Nms/rad]", params.post_contact_DR_diag);

            printf("\npoles compared (lever r_c = TCP - pole):\n");
            if (finite_axis.valid) {
              printf("  (A) Chasles axis  : angle=%.1f deg | pitch=%+.1f mm/rad\n",
                     (180.0 / M_PI) * finite_axis.angle,
                     1000.0 * finite_axis.pitch);
            } else {
              printf("  (A) Chasles axis  : not valid (rotation too small) -> saved-axis fallback\n");
            }
            printVec3Mm("      pole_from_edge", pole_point - tool_contact_point);
            printVec3Mm("      r_c           ", r_c_base);
            printf("  (B) manual pole   : %s, %s reference\n",
                   manual_pole_commanded ? "COMMANDED this run" : "comparison only",
                   params.method2_pole_freeze_at_contact ? "contact" : "live");
            printVec3Mm("      pole_from_edge", pole_cmd - tool_contact_point);
            printVec3Mm("      r_c           ", r_c_cmd);
            printVec3Mm("      r_c(B)-r_c(A) ", r_c_cmd - r_c_base);

            printf("\nK_TCP = Ad^T K_pole Ad  [rows fx..mz | cols tx..rz]:\n");
            printSpatialGain6("  (A) Chasles axis", K_tcp_base);
            printSpatialGainEigenvalues("  (A) K_TCP", K_tcp_base);
            printSpatialGain6("  (B) manual pole (commanded)", K_tcp_cmd);
            printSpatialGainEigenvalues("  (B) K_TCP", K_tcp_cmd);

            printf("\nD_TCP = Ad^T D_pole Ad:\n");
            printSpatialGain6("  (A) Chasles axis", D_tcp_base);
            printSpatialGainEigenvalues("  (A) D_TCP", D_tcp_base);
            printSpatialGain6("  (B) manual pole (commanded)", D_tcp_cmd);
            printSpatialGainEigenvalues("  (B) D_TCP", D_tcp_cmd);

            printf("\n=== Method 3: Least-squares effective moment identification ===\n");
            printf("model: M_C = K_rt*dx_C + D_rt*v_C + K_R*dtheta + D_R*omega\n");
            printf("moment transfer: M_C = m - r_C x f\n");
            const EffectiveMomentFit moment_fit =
                fitEffectiveMomentModel(fit_effective_moment, params.effective_moment_fit_ridge);
            if (moment_fit.valid) {
              printf("samples = %ld | rms_moment_error = %.4g Nm\n",
                     moment_fit.sample_count,
                     moment_fit.rms_error);
              printMat3Rows("K_rt_eff", moment_fit.K_rt);
              printMat3Rows("D_rt_eff", moment_fit.D_rt);
              printMat3Rows("K_R_eff", moment_fit.K_R);
              printMat3Rows("D_R_eff", moment_fit.D_R);
              const Vec3 diag_Krt = moment_fit.K_rt.diagonal();
              const Vec3 diag_Drt = moment_fit.D_rt.diagonal();
              const Vec3 diag_KR = moment_fit.K_R.diagonal();
              const Vec3 diag_DR = moment_fit.D_R.diagonal();
              printGainVec("diag_Krt_eff", diag_Krt);
              printGainVec("diag_Drt_eff", diag_Drt);
              printGainVec("diag_KR_eff", diag_KR);
              printGainVec("diag_DR_eff", diag_DR);
            } else {
              printf("n/a: need at least 12 post-contact samples with sufficient excitation (got %ld)\n",
                     fit_effective_moment.sample_count);
            }
          }
          printf("phase: %s\n", phaseName(phase));
        }
      }

      // Computing the desired motion for this control cycle based on the
      // active phase.
      if (phase == ControlPhase::kOrientToSurface) {
        // Rotate the tool first, but keep the TCP at the start position.
        desired.p_d = p_start;
        desired.pdot_d.setZero();
      } else if (phase == ControlPhase::kSearchFirstContact) {
        const double search_distance =
            std::min(params.contact_search_speed * (time - phase_start_time),
                     params.contact_search_max_distance);

        // During contact search, keep the full Cartesian target from the
        // original search behavior:
        //
        //   p_d [m] = p_start + search_distance [m] * search_direction [-]
        //
        // This keeps the tool path and orientation stable while moving down.
        desired.p_d = p_start + search_distance * contact_search_direction;
        desired.pdot_d = params.contact_search_speed * contact_search_direction;

        // Contact trigger only, not force control. The phase switch happens at
        // the confirmed first-touch candidate, detected from force along the
        // search direction after the minimum travel gate.
        // Moment comparison is reserved for post_contact_align, where the
        // tool rotates after the first contact.
        const bool first_touch_distance_reached =
            search_distance >= params.contact_search_first_touch_min_distance;
        const Vec3 force_delta_from_bias = external_force - contact_force_bias;
        const double force_along_search =
            force_delta_from_bias.dot(contact_search_direction);
        const double search_force_signal =
            params.contact_search_use_directional_force
                ? force_along_search
                : force_delta_from_bias.norm();
        const bool force_threshold_reached =
            search_force_signal >= params.contact_force_threshold;
        if (!first_touch_candidate_saved &&
            first_touch_distance_reached &&
            force_threshold_reached) {
          if (contact_search_candidate_start_time < 0.0) {
            contact_search_candidate_start_time = time;
          }
        } else if (!first_touch_candidate_saved) {
          contact_search_candidate_start_time = -1.0;
        }
        const double contact_search_candidate_time =
            (contact_search_candidate_start_time >= 0.0)
                ? (time - contact_search_candidate_start_time)
                : 0.0;
        bool first_touch_candidate_just_saved = false;
        if (!first_touch_candidate_saved &&
            contact_search_candidate_start_time >= 0.0 &&
            contact_search_candidate_time >= params.contact_search_confirm_time) {
          first_touch_candidate_saved = true;
          first_touch_candidate_just_saved = true;
          first_touch_candidate_time = time - phase_start_time;
          first_touch_candidate_distance = search_distance;
          first_touch_candidate_signal = search_force_signal;
          first_touch_candidate_force = external_force;
          first_touch_candidate_moment = external_moment;
          printf("\nFirst-touch candidate saved: dist=%.1f mm | force=%.1f N | confirmed=%.3f s\n",
                 1000.0 * first_touch_candidate_distance,
                 first_touch_candidate_signal,
                 contact_search_candidate_time);
        }
        const bool search_contact_detected =
            first_touch_candidate_just_saved;
        if (params.debug_period > 0.0 && time >= next_debug_time) {
          printSearchDebug(time - phase_start_time,
                            1000.0 * search_distance,
                            search_force_signal,
                            params.contact_force_threshold,
                            first_touch_candidate_saved);
          next_debug_time = time + params.debug_period;
        }
        if (search_contact_detected) {
          const double search_phase_elapsed = time - phase_start_time;
          active_tool_contact_offset_ee = tool_contact_offset_ee;
          surface_point_runtime = tool_contact_point;
          first_contact_tcp = p_EE;
          first_contact_point = tool_contact_point;
          first_contact_search_distance = search_distance;
          first_contact_force_delta = search_force_signal;
          contact_found = true;
          contact_time = time;
          R_contact_start = R_EE;
          phase = params.use_phase_sequence
              ? ControlPhase::kPostContactAlign
              : ControlPhase::kSurfaceImpedance;
          phase_start_time = time;
          next_debug_time = time;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          printf("\nContact found: dist=%.1f mm | force=%.1f N | confirmed=%.3f s\n",
                 1000.0 * search_distance,
                 search_force_signal,
                 contact_search_candidate_time);
          if (first_touch_candidate_saved) {
            printf("first_touch_reference: dist=%.1f mm | dt_before_switch=%.3f s\n",
                   1000.0 * first_touch_candidate_distance,
                   search_phase_elapsed - first_touch_candidate_time);
          }
          printContactEdgeDebug(active_tool_contact_offset_ee, first_contact_tcp, first_contact_point);
          printf("phase: %s\n", phaseName(phase));
        // If the maximum search distance is reached without detecting contact, stop the controller and log the final data.
        } else if (search_distance >= params.contact_search_max_distance) {
          contact_search_failed = true;
          stop_requested.store(true);
          desired.p_d = p_EE;
          desired.pdot_d.setZero();
        }
      } else if (phase == ControlPhase::kPostContactAlign) {
        // Keep pressing the selected tool edge/contact point into the real
        // surface while holding the orientation captured at first contact as
        // the (soft) rotational target. The controlled Cartesian point is
        // still the TCP, so convert the desired edge position back to a TCP
        // target:
        //
        //   p_edge_d = p_contact_1 + preload * search_direction
        //   p_TCP_d  = p_edge_d - R_contact_start * r_edge_EE
        //
        // This prevents the first edge from lifting away simply because the
        // TCP rotates around a different point.
        const double post_align_time = time - phase_start_time;
        const double post_contact_push = postContactPush(params, post_align_time);
        const Vec3 edge_target =
            first_contact_point + post_contact_push * contact_search_direction;
        edge_target_debug = edge_target;
        post_contact_push_debug = post_contact_push;
        desired.p_d = edge_target - R_contact_start * tool_contact_offset_ee;
        desired.pdot_d.setZero();
      } else {
        if (contact_found) {
          impedance_time = params.use_contact_search ? (time - contact_time) : time;
        }
        const Mat3& R_position_surface =
            (params.use_search_direction_surface_after_alignment &&
             phase == ControlPhase::kSurfaceImpedance &&
             contact_found)
                ? R_contact_surface
                : R_alignment_target;
        desired = computeDesiredMotion(
            params,
            impedance_time,
            p_start,
            p_EE,
            R_position_surface,
            surface_point_runtime);
      }

      // If contact is found or contact search is not enabled, compute the desired motion
      // based on the current EE position and the virtual surface point.
      // The desired position is computed based on the current EE position, the virtual surface point,
      // and the surface normal, to create a plane constraint.
      // The position error e_p and orientation error e_R are computed
      // based on the desired position and the current EE position and orientation.
      Vec3 e_p = desired.p_d - p_EE;
      const bool use_surface_orientation =
          params.orientation_test_only ||
          (params.use_phase_sequence && params.use_orientation_phase) ||
          (contact_found && params.align_orientation_to_surface_after_contact);
      const Mat3& R_d_used = (phase == ControlPhase::kPostContactAlign)
          ? R_contact_start
          : (params.orientation_test_only
                 ? R_d_orientation_test
                 : (phase == ControlPhase::kSurfaceImpedance && contact_found
                        ? R_after_contact_align
                        : (use_surface_orientation ? R_d_alignment_target : R_d)));
      Vec3 e_R = applyRotationalAxisMask(params, orientationError(R_EE, R_d_used), R_alignment_target);

      if (phase == ControlPhase::kSurfaceImpedance && params.use_virtual_center_after_contact) {
        // Virtual center of rotation for the post-contact alignment phase:
        //
        //   p_c [m] = p_contact [m] + vcr_offset [m] * n_surface [-]
        //   r_c [m] = p_EE [m] - p_c [m]
        //   e_p [m] = e_p [m] - e_R [rad] x r_c [m]
        //
        // This couples the rotational correction into the translational error
        // so the tool behaves more like it is rotating around p_c.
        const Vec3 p_c = surface_point_runtime + params.vcr_offset * R_alignment_target.col(0);
        const Vec3 r_c = p_EE - p_c;
        e_p = e_p - e_R.cross(r_c);
      }

      if (phase == ControlPhase::kSurfaceImpedance &&
          params.print_impedance_debug &&
          params.debug_period > 0.0 &&
          time >= next_debug_time) {
        printImpedanceDebug(impedance_time,
                             (external_force - contact_force_bias).norm(),
                             1000.0 * e_p.norm(),
                             (180.0 / M_PI) * e_R.norm());
        next_debug_time = time + params.debug_period;
      }

      // Defining the stiffness and damping matrices to be used in the impedance control law,
      // based on the current mode (searching for contact or impedance control).
      const bool use_soft_translation =
          phase == ControlPhase::kSearchFirstContact;
      const bool use_contact_surface_gains =
          params.use_search_direction_surface_after_alignment &&
          (phase == ControlPhase::kSurfaceImpedance) &&
          contact_found;
      const Mat3& Kp_used =
          (phase == ControlPhase::kPostContactAlign)
              ? Kp_post_contact
              : (use_soft_translation ? Kp_search : (use_contact_surface_gains ? Kp_contact : Kp));
      const Mat3& Dp_used =
          (phase == ControlPhase::kPostContactAlign)
              ? Dp_post_contact
              : (use_soft_translation ? Dp_search : (use_contact_surface_gains ? Dp_contact : Dp));
      const Mat3& KR_used =
          (phase == ControlPhase::kPostContactAlign) ? KR_post_contact : KR;
      const Mat3& DR_used =
          (phase == ControlPhase::kPostContactAlign) ? DR_post_contact : DR;
      Vec6 wrench;
      Vec3 f;
      Vec3 m;
      if (phase == ControlPhase::kPostContactAlign &&
          params.post_contact_apply_method2_tcp_wrench) {
        // Method 2 active-apply mode. The 6x6 K/D source is selected by, in
        // priority order:
        //   use_in_method2_k_d_diag = 1 -> block-diagonal post_align gains (no
        //     lever-arm coupling). Fed through the 6x6 path this reproduces the
        //     normal decoupled commanded wrench, so the motion matches baseline.
        //   use_manual_method2_pole = 1 -> recompute the coupled K_TCP/D_TCP
        //     LIVE from a chosen pole = contact edge + manual_pole_from_edge.
        //     The pole is the only knob in the adjoint, so sweeping it moves the
        //     effective rotation center to chase the desired alignment.
        //   otherwise -> the saved coupled K_TCP/D_TCP from a previous run.
        Mat6x6 K_tcp_base;
        Mat6x6 D_tcp_base;
        if (params.use_in_method2_k_d_diag) {
          K_tcp_base = Mat6x6::Zero();
          K_tcp_base.block<3, 3>(0, 0) = Kp_post_contact;
          K_tcp_base.block<3, 3>(3, 3) = KR_post_contact;
          D_tcp_base = Mat6x6::Zero();
          D_tcp_base.block<3, 3>(0, 0) = Dp_post_contact;
          D_tcp_base.block<3, 3>(3, 3) = DR_post_contact;
        } else if (params.use_manual_method2_pole) {
          Mat6x6 K_pole = Mat6x6::Zero();
          K_pole.block<3, 3>(0, 0) = Kp_post_contact;
          K_pole.block<3, 3>(3, 3) = KR_post_contact;
          Mat6x6 D_pole = Mat6x6::Zero();
          D_pole.block<3, 3>(0, 0) = Dp_post_contact;
          D_pole.block<3, 3>(3, 3) = DR_post_contact;
          // Frozen: reference the contact-time pose (constant through align), so
          // K_TCP/D_TCP are a single fixed spring. Live: reference the moving
          // edge, so they refresh each cycle.
          const Vec3 edge_ref = params.method2_pole_freeze_at_contact
                                    ? first_contact_point
                                    : tool_contact_point;
          const Vec3 tcp_ref = params.method2_pole_freeze_at_contact
                                   ? first_contact_tcp
                                   : p_EE;
          const Vec3 manual_pole = edge_ref + params.manual_method2_pole_from_edge;
          const Vec3 r_c_manual = tcp_ref - manual_pole;
          K_tcp_base = adjointTransformedGain(K_pole, r_c_manual);
          D_tcp_base = adjointTransformedGain(D_pole, r_c_manual);
        } else {
          K_tcp_base = params.method2_K_tcp_base;
          D_tcp_base = params.method2_D_tcp_base;
        }

        Vec6 dx_base;
        dx_base.head<3>() = e_p;
        dx_base.tail<3>() = e_R;
        Vec6 dv_base;
        dv_base.head<3>() = desired.pdot_d - pdot;
        dv_base.tail<3>() = -omega;

        wrench = K_tcp_base * dx_base + D_tcp_base * dv_base;
        f = wrench.head<3>();
        m = wrench.tail<3>();
      } else {
        // Computing forces and torques from the normal decoupled impedance law.
        f = Kp_used * e_p + Dp_used * (desired.pdot_d - pdot);
        m = KR_used * e_R - DR_used * omega;
        wrench.head<3>() = f;
        wrench.tail<3>() = m;
        if (phase == ControlPhase::kPostContactAlign &&
            params.post_contact_eval_method2_tcp_wrench) {
          // Passive evaluation only: compute the saved Method 2 wrench from
          // the same TCP errors, but do not command it. The normal self-align
          // wrench above remains active.
          Vec6 dx_base;
          dx_base.head<3>() = e_p;
          dx_base.tail<3>() = e_R;
          Vec6 dv_base;
          dv_base.head<3>() = desired.pdot_d - pdot;
          dv_base.tail<3>() = -omega;
          const Vec6 method2_wrench =
              params.method2_K_tcp_base * dx_base + params.method2_D_tcp_base * dv_base;
          const Vec3 method2_f_from_translation =
              params.method2_K_tcp_base.block<3, 3>(0, 0) * dx_base.head<3>() +
              params.method2_D_tcp_base.block<3, 3>(0, 0) * dv_base.head<3>();
          const Vec3 method2_f_from_rotation =
              params.method2_K_tcp_base.block<3, 3>(0, 3) * dx_base.tail<3>() +
              params.method2_D_tcp_base.block<3, 3>(0, 3) * dv_base.tail<3>();
          const Vec3 method2_m_from_translation =
              params.method2_K_tcp_base.block<3, 3>(3, 0) * dx_base.head<3>() +
              params.method2_D_tcp_base.block<3, 3>(3, 0) * dv_base.head<3>();
          const Vec3 method2_m_from_rotation =
              params.method2_K_tcp_base.block<3, 3>(3, 3) * dx_base.tail<3>() +
              params.method2_D_tcp_base.block<3, 3>(3, 3) * dv_base.tail<3>();
          const Vec3 method2_f = method2_wrench.head<3>();
          const Vec3 method2_m = method2_wrench.tail<3>();
          const double f_error = (method2_f - f).norm();
          const double m_error = (method2_m - m).norm();

          ++method2_eval_samples;
          method2_eval_force_error_sq_sum += f_error * f_error;
          method2_eval_moment_error_sq_sum += m_error * m_error;
          method2_eval_force_error_max = std::max(method2_eval_force_error_max, f_error);
          method2_eval_moment_error_max = std::max(method2_eval_moment_error_max, m_error);
          method2_eval_force_norm_sum += method2_f.norm();
          method2_eval_moment_norm_sum += method2_m.norm();
          method2_eval_normal_force_norm_sum += f.norm();
          method2_eval_normal_moment_norm_sum += m.norm();
          method2_eval_force_from_translation_norm_sum += method2_f_from_translation.norm();
          method2_eval_force_from_rotation_norm_sum += method2_f_from_rotation.norm();
          method2_eval_moment_from_translation_norm_sum += method2_m_from_translation.norm();
          method2_eval_moment_from_rotation_norm_sum += method2_m_from_rotation.norm();
          method2_eval_final_force_error = f_error;
          method2_eval_final_moment_error = m_error;
          method2_eval_final_method2_force_norm = method2_f.norm();
          method2_eval_final_method2_moment_norm = method2_m.norm();
          method2_eval_final_normal_force_norm = f.norm();
          method2_eval_final_normal_moment_norm = m.norm();
          method2_eval_final_force_from_translation_norm = method2_f_from_translation.norm();
          method2_eval_final_force_from_rotation_norm = method2_f_from_rotation.norm();
          method2_eval_final_moment_from_translation_norm = method2_m_from_translation.norm();
          method2_eval_final_moment_from_rotation_norm = method2_m_from_rotation.norm();
        }
      }
      // The joint torques for the impedance control task are computed
      // by mapping the task-space wrench through the Jacobian transpose.

      Vec7 tau_task = J.transpose() * wrench;
      // During contact search, keep the nullspace torque off.
      // The search phase should only move the TCP downward to find the table.
      // After contact is found, the normal surface impedance starts and the
      // nullspace optimization is enabled again.
      Vec7 tau_nullspace = Vec7::Zero();
      if ((phase != ControlPhase::kOrientToSurface) &&
          (phase != ControlPhase::kSearchFirstContact) &&
          !params.orientation_test_only) {
        tau_nullspace = computeNullspaceTorque(params, model, state, J, dq, q_start);
      }

      Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());

      Vec7 tau_cmd = tau_task + tau_nullspace + coriolis;

      ++control_cycle_count;
      if ((control_cycle_count % static_cast<std::size_t>(log_every_n_cycles)) == 0) {
        if (max_log_rows > 0) {
          log_data[log_write_index] = makeLogRow(
              time,
              static_cast<int>(phase),
              p_EE,
              desired.p_d,
              p_end,
              tool_contact_point,
              first_contact_tcp,
              first_contact_point,
              edge_target_debug,
              tool_contact_offset_ee,
              e_p,
              e_R,
              pdot,
              desired.pdot_d,
              omega,
              f,
              m,
              external_force,
              external_moment,
              contact_force_bias,
              contact_moment_bias,
              post_contact_push_debug,
              tau_cmd);
          log_write_index = (log_write_index + 1) % max_log_rows;
          if (log_rows_written < max_log_rows) {
            ++log_rows_written;
          } else {
            log_buffer_wrapped = true;
          }
        }
      }

      final_p_EE = p_EE;
      final_p_d = desired.p_d;
      final_e_p = e_p;
      final_e_R = e_R;
      final_pdot = pdot;
      final_omega = omega;
      final_tool_contact_point = tool_contact_point;
      final_q = q_current;

      Array7 tau_array = vec7ToArray(tau_cmd);

      // if the time ends or if the user requested to stop the experiment, return the final torques and exit the control loop.

      if (((params.experiment_duration > 0.0) && (impedance_time >= params.experiment_duration)) ||
          stop_requested.load()) {
        if (stop_requested.load()) {
          printf("\nStop requested with e + Enter. Finishing control loop...\n");
        }
        // Returning the final torques for this cycle and exiting the control loop.
        return MotionFinished(Torques(tau_array));
      }
      // else, return the computed torques for this control cycle and continue the loop.
      return Torques(tau_array);
    });

    if (!pending_parameter_updates.empty()) {
      updateParameterValues("parameters.txt", pending_parameter_updates);
      printf("parameters.txt updated after control loop (%zu queued values).\n",
             pending_parameter_updates.size());
    }

    if (params.post_contact_eval_method2_tcp_wrench && method2_eval_samples > 0) {
      const double samples = static_cast<double>(method2_eval_samples);
      const double force_rms =
          std::sqrt(method2_eval_force_error_sq_sum / samples);
      const double moment_rms =
          std::sqrt(method2_eval_moment_error_sq_sum / samples);
      printf("\n=== Method 2 eval wrench compare ===\n");
      printf("samples=%zu | commanded=normal decoupled wrench | method2=saved coupled TCP wrench (not commanded)\n",
             method2_eval_samples);
      printf("force:  normal_avg=%5.1f N | method2_avg=%5.1f N | rms_diff=%5.1f N | max_diff=%5.1f N | final_diff=%5.1f N\n",
             method2_eval_normal_force_norm_sum / samples,
             method2_eval_force_norm_sum / samples,
             force_rms,
             method2_eval_force_error_max,
             method2_eval_final_force_error);
      printf("moment: normal_avg=%5.1f Nm | method2_avg=%5.1f Nm | rms_diff=%5.1f Nm | max_diff=%5.1f Nm | final_diff=%5.1f Nm\n",
             method2_eval_normal_moment_norm_sum / samples,
             method2_eval_moment_norm_sum / samples,
             moment_rms,
             method2_eval_moment_error_max,
             method2_eval_final_moment_error);
      printf("final:  normal=(%5.1f N, %5.1f Nm) | method2=(%5.1f N, %5.1f Nm)\n",
             method2_eval_final_normal_force_norm,
             method2_eval_final_normal_moment_norm,
             method2_eval_final_method2_force_norm,
             method2_eval_final_method2_moment_norm);
      printf("decomp avg:   F(tx)=%5.1f N | F(rot)=%5.1f N | M(tx)=%5.1f Nm | M(rot)=%5.1f Nm\n",
             method2_eval_force_from_translation_norm_sum / samples,
             method2_eval_force_from_rotation_norm_sum / samples,
             method2_eval_moment_from_translation_norm_sum / samples,
             method2_eval_moment_from_rotation_norm_sum / samples);
      printf("decomp final: F(tx)=%5.1f N | F(rot)=%5.1f N | M(tx)=%5.1f Nm | M(rot)=%5.1f Nm\n",
             method2_eval_final_force_from_translation_norm,
             method2_eval_final_force_from_rotation_norm,
             method2_eval_final_moment_from_translation_norm,
             method2_eval_final_moment_from_rotation_norm);
      printf("read: small diffs mean apply=1 should behave like the baseline; large diffs mean apply=1 will drive a different self-align motion.\n");
    }

    // After exiting the control loop, check if the contact search failed and print a message if it did.
    if (contact_search_failed) {
      printf("\nContact search stopped: maximum search distance reached before contact.\n");
    }
    printJointStartEndTableDeg(q_start, final_q);

    if (last_valid_post_align_axis_valid) {
      final_instant_pole_to_edge = last_valid_post_align_axis_point_from_edge;
      final_instant_axis_dir = last_valid_post_align_axis_dir;
      final_instant_screw_pitch = last_valid_post_align_screw_pitch;
      final_instant_edge_axis_distance = last_valid_post_align_axis_edge_distance;
      final_instant_axis_time = last_valid_post_align_axis_time;
      final_instant_pole_valid = true;
    } else {
      final_instant_pole_valid =
          computeInstantaneousPoleFromTcp(final_pdot, final_omega, &final_instant_pole_from_tcp);
      final_instant_pole_base = final_p_EE + final_instant_pole_from_tcp;
      final_instant_pole_to_edge = final_instant_pole_base - final_tool_contact_point;
      final_instant_axis_dir =
          final_instant_pole_valid ? final_omega.normalized() : Vec3::Zero();
      final_instant_screw_pitch =
          final_instant_pole_valid
              ? instantaneousScrewPitch(final_pdot, final_omega)
              : 0.0;
      final_instant_edge_axis_distance =
          final_instant_pole_valid
              ? pointDistanceToAxis(
                    final_tool_contact_point,
                    final_instant_pole_base,
                    final_omega)
              : 0.0;
    }

    std::vector<LogData> ordered_log_data;
    ordered_log_data.reserve(log_rows_written);
    if (log_buffer_wrapped) {
      ordered_log_data.insert(
          ordered_log_data.end(),
          log_data.begin() + static_cast<std::ptrdiff_t>(log_write_index),
          log_data.end());
      ordered_log_data.insert(
          ordered_log_data.end(),
          log_data.begin(),
          log_data.begin() + static_cast<std::ptrdiff_t>(log_write_index));
      printf("Log buffer wrapped: kept latest %zu rows, sampled every %d control cycles.\n",
             log_rows_written,
             log_every_n_cycles);
    } else {
      ordered_log_data.insert(
          ordered_log_data.end(),
          log_data.begin(),
          log_data.begin() + static_cast<std::ptrdiff_t>(log_rows_written));
    }

    // After the control loop finishes, write the logged data to a CSV file and print the final summary of the experiment results.
    writeLogToCsv(ordered_log_data, params.csv_file_name);
    printFinalSummary(
        final_p_d,
        final_p_EE,
        final_e_p,
        final_e_R,
        final_instant_pole_to_edge,
        final_instant_axis_dir,
        params.last_best_axis_from_edge,
        params.last_best_axis_dir,
        params.last_best_axis_pitch,
        final_instant_screw_pitch,
        final_instant_edge_axis_distance,
        final_instant_axis_time,
        final_instant_pole_valid,
        params.csv_file_name);

  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    fprintf(stderr, "If the robot is still in an error/reflex state, recover it manually in Franka Desk.\n");
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }

  return 0;
}
