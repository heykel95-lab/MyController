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
  kSurfaceImpedance
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

    // Load the robot model, which is needed for the controller and nullspace optimization.
    // The model contains the robot's kinematics and dynamics, and is used to compute the Jacobian, Coriolis forces, etc.
    // The model is loaded once here and passed to the control loop lambda by reference, so it does not need to be reloaded every cycle.
    Model model = robot.loadModel();


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
    const Vec3 external_force_start = initial_external_wrench.head<3>();
    Vec3 contact_force_bias = initial_external_wrench.head<3>();
    Vec3 contact_moment_bias = initial_external_wrench.tail<3>();

    // Surface/task frame for spatial impedance:
    //
    //   column 0 = surface normal direction
    //   column 1 = first tangent direction
    //   column 2 = second tangent direction
    //
    // In surface mode the diagonal gains from parameters.txt are interpreted
    // in this frame:
    //
    //   Kp_normal   / Dp_normal   -> normal-to-surface stiffness/damping
    //   Kp_tangent1 / Dp_tangent1 -> first tangent stiffness/damping
    //   Kp_tangent2 / Dp_tangent2 -> second tangent stiffness/damping
    //
    // The gains are then transformed to the robot base frame:
    //
    //   K_base = R_surface * K_surface * R_surface^T
    //
    // This is the practical idea inspired by spatial impedance: choose simple
    // diagonal impedance values in a task/surface frame, then transform them
    // into the world/base frame.

    // Defining the Rotation matrix from the surface frame to the robot base frame

    const Mat3 R_surface = makeSurfaceFrame(params);
    const Mat3 R_d_surface = makeToolOrientationParallelToSurface(params, R_surface, R_d);
    const double orientation_test_extra_tilt_rad =
        params.orientation_test_extra_tilt_deg * M_PI / 180.0;
    const Mat3 R_d_orientation_test =
        (std::abs(orientation_test_extra_tilt_rad) > 1e-9)
            ? Eigen::AngleAxisd(orientation_test_extra_tilt_rad, R_surface.col(1)).toRotationMatrix() * R_d
            : R_d_surface;
    // The surface frame is defined based on the surface normal and tangent hint from parameters.txt.
    // The makeSurfaceFrame function constructs an orthonormal frame where the first column is the surface normal,
    // and the other two columns are tangent directions. This frame is used to define the directions of the impedance control gains.
    Vec3 surface_point_runtime =
        params.use_start_as_surface_point ? p_start : params.surface_point;
    const Vec3 contact_search_direction =
        params.contact_search_use_surface_normal
            ? -R_surface.col(0)
            : normalizedOrFallback(params.contact_search_direction, -R_surface.col(0));
    const Mat3 R_contact_surface =
        makeSurfaceFrameFromNormalTangent(-contact_search_direction, params.surface_tangent1);
    const Mat3 Kp_search = params.contact_search_Kp_diag.asDiagonal();
    const Mat3 Dp_search = params.contact_search_Dp_diag.asDiagonal();
    const Mat3 Kp_post_contact = params.post_contact_Kp_diag.asDiagonal();
    const Mat3 Dp_post_contact = params.post_contact_Dp_diag.asDiagonal();
    // Rotational compliance used only during post_contact_align, in the
    // surface frame (normal/tangent1/tangent2), same convention as the
    // global KR/DR below. Normal (yaw about the push axis) stays at a
    // constrained stiffness; tangent1/tangent2 (the tipping directions) are
    // intentionally very soft so a real contact moment at the pressed edge
    // can passively rotate the tool flat instead of being resisted by the
    // spring.
    const Mat3 KR_post_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.post_contact_KR_diag, R_surface)
        : params.post_contact_KR_diag.asDiagonal();
    const Mat3 DR_post_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.post_contact_DR_diag, R_surface)
        : params.post_contact_DR_diag.asDiagonal();
    ControlPhase phase = ControlPhase::kSurfaceImpedance;
    if (params.orientation_test_only) {
      phase = ControlPhase::kOrientToSurface;
    } else if (params.use_contact_search) {
      phase = params.use_phase_sequence
          ? ControlPhase::kOrientToSurface
          : ControlPhase::kSearchFirstContact;
    }
    double phase_start_time = 0.0;
    bool contact_found = !params.use_contact_search;
    bool contact_search_failed = false;
    double contact_time = 0.0;
    Mat3 R_contact_start = R_d_surface;
    Mat3 R_after_contact_align = R_d_surface;
    Vec3 first_contact_point = p_start;
    Vec3 active_tool_contact_offset_ee = params.tool_contact_point_ee;
    double first_contact_search_distance = 0.0;
    double first_contact_force_delta = 0.0;
    double next_post_align_debug_time = 0.0;

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
    // If the constraint is enabled, the gains are transformed from the surface frame to the robot base frame using the R_surface rotation matrix.
    // If the constraint is not enabled, the gains are used as-is as diagonal matrices in the robot base frame.
    Mat3 Kp = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Kp_diag, R_surface)
        : params.Kp_diag.asDiagonal();
    Mat3 Dp = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Dp_diag, R_surface)
        : params.Dp_diag.asDiagonal();
    Mat3 KR = params.constraint_enabled
        ? makeSpatialGainMatrix(params.KR_diag, R_surface)
        : params.KR_diag.asDiagonal();
    Mat3 DR = params.constraint_enabled
        ? makeSpatialGainMatrix(params.DR_diag, R_surface)
        : params.DR_diag.asDiagonal();
    Mat3 Kp_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Kp_diag, R_contact_surface)
        : params.Kp_diag.asDiagonal();
    Mat3 Dp_contact = params.constraint_enabled
        ? makeSpatialGainMatrix(params.Dp_diag, R_contact_surface)
        : params.Dp_diag.asDiagonal();
    printf("\n=== Run ===\n");
    printf("phase: %s\n", phaseName(phase));

    // The log_data vector will store the data for each control cycle, which will be written to a CSV file at the end of the experiment.
    std::vector<LogData> log_data;
    log_data.reserve(static_cast<std::size_t>(params.experiment_duration * 1500.0));
    // The time variable is used to keep track of the elapsed time since the start of the control loop, and is updated in each cycle based on the period provided by libfranka.
    double time = 0.0;

    // Defining the final values when the duration is over for the the tool position, the desired position, the position error and the rotation error, to be printed at the end of the experiment.
    Vec3 final_p_EE = Vec3::Zero();
    Vec3 final_p_d = Vec3::Zero();
    Vec3 final_e_p = Vec3::Zero();
    Vec3 final_e_R = Vec3::Zero();
    Vec7 final_q = q_start;
    double next_orientation_debug_time = 0.0;

    // Print the starting message and instructions for the user before entering the control loop.
    printf("Starting impedance controller:\n");

    // Stop function for the control loop, can be triggered by the user with e + Enter, or automatically when contact search fails.
    std::atomic<bool> stop_requested(false);
    startKeyboardStopThread(params, stop_requested);

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
      // R_d is the initial tool orientation. R_d_surface is the desired tool
      // orientation parallel to the virtual surface.
      // In orientation_test_only, R_d_orientation_test adds an extra rotation
      // around surface tangent1. For the horizontal table this is base x.
      //
      // With use_phase_sequence = 1:
      //   1. orient_to_surface: rotate at the start position until R_EE ~= R_d_surface
      //   2. search_first_contact: translate along contact_search_direction
      //      while keeping R_EE ~= R_d_surface
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
          params.orientation_test_only ? R_d_orientation_test : R_d_surface;
      const Vec3 external_force = external_wrench.head<3>();
      const Vec3 external_moment = external_wrench.tail<3>();
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

      const Vec3 e_R_to_surface =
          applyRotationalAxisMask(params, orientationError(R_EE, R_d_phase), R_surface);
      const Vec3 tool_axis_current = currentToolAxisInBase(params, R_EE).normalized();
      const Vec3 tool_axis_target = desiredToolAxisInBase(params, R_surface).normalized();
      const double tool_axis_dot = std::max(
          -1.0,
          std::min(1.0, tool_axis_current.dot(tool_axis_target)));
      const double tool_axis_alignment_error = std::acos(tool_axis_dot);
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
        printf("\nContact during alignment: force = %.1f N\n", force_delta_norm);
        printVec3Mm("active_edge_ee", active_tool_contact_offset_ee);
        printVec3Mm("surface_point", surface_point_runtime);
        printVec3Mm("p_contact_1", first_contact_point);
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
        contact_force_bias = external_force;
        contact_moment_bias = external_moment;
        printf("\nOrientation reached: axis_error = %.2f deg, full_error = %.2f deg\n",
               (180.0 / M_PI) * tool_axis_alignment_error,
               (180.0 / M_PI) * e_R_to_surface.norm());
        printf("phase: %s\n", phaseName(phase));
      }

      if (phase == ControlPhase::kPostContactAlign) {
        const double post_align_time = time - phase_start_time;
        const double post_moment_delta_norm = (external_moment - contact_moment_bias).norm();
        const double post_force_delta_norm = (external_force - contact_force_bias).norm();
        if (time >= next_post_align_debug_time) {
          // height_delta_mm: actual height change of the pressed CONTACT
          // EDGE since first contact (negative = moved down), comparing the
          // edge point to itself over time. Comparing p_EE (TCP origin) to
          // first_contact_point (the edge) would mix two different physical
          // points and just report their fixed geometric offset.
          const double height_delta_mm =
              1000.0 * (tool_contact_point(2) - first_contact_point(2));
          // actual_tip_deg: measured rotation away from the orientation held
          // at first contact -- shows whether/how much the tool has
          // passively tipped so far, not just where it is now.
          const double actual_tip_deg =
              (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm();
          printf("post_align_debug: t=%.1f s, height_delta=%.1f mm (negative=down), F=%.1f N, tip=%.1f deg, M=%.1f Nm\n",
                 post_align_time,
                 height_delta_mm,
                 post_force_delta_norm,
                 actual_tip_deg,
                 post_moment_delta_norm);
          next_post_align_debug_time = time + 1.0;
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
          if (moment_contact_reached) {
            printf("\nPost-align moment reached: %.2f Nm, actual_tip = %.1f deg\n",
                   post_moment_delta_norm, actual_tip_deg);
          } else {
            printf("\nPost-align time reached: %.1f s, actual_tip = %.1f deg\n",
                   post_align_time, actual_tip_deg);
          }
          printf("first_contact_search = %.1f mm, first_force = %.1f N\n",
                 1000.0 * first_contact_search_distance,
                 first_contact_force_delta);
          printVec3Mm("p_contact_1", first_contact_point);
          printVec3Mm("p_after_align", p_EE);
          printVec3Mm("p_edge_after_align", tool_contact_point);
          printVec3Mm("p_edge_after_minus_contact_1", tool_contact_point - first_contact_point);
          printVec3Mm("p_after_minus_contact_1", p_EE - first_contact_point);
          printVec3Mm("surface_point_updated", surface_point_runtime);
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

        // Contact trigger only, not force control:
        //
        //   if search_distance [m] >= contact_search_min_distance [m]
        //   and force change exceeds the threshold,
        //   the current TCP position becomes the point of the virtual surface.
        //
        // The minimum distance prevents an early false trigger before the TCP is
        // near the expected table height.
        //
        // After this one-time event, the controller switches back to pure
        // surface impedance.

        // Search is translational: the first contact point is detected by
        // force only. Moment comparison is reserved for post_contact_align,
        // where the tool rotates after the first contact.
        double search_force_delta_norm = 0.0;
        const bool contact_distance_reached =
            search_distance >= params.contact_search_min_distance;
        const Vec3 force_delta_from_start = external_force - external_force_start;
        const bool search_contact_detected =
            contact_distance_reached &&
            (force_delta_from_start.norm() >= params.contact_force_threshold);
        search_force_delta_norm = force_delta_from_start.norm();
        if (search_contact_detected) {
          active_tool_contact_offset_ee = tool_contact_offset_ee;
          surface_point_runtime = tool_contact_point;
          first_contact_point = tool_contact_point;
          first_contact_search_distance = search_distance;
          first_contact_force_delta = search_force_delta_norm;
          contact_found = true;
          contact_time = time;
          R_contact_start = R_EE;
          phase = params.use_phase_sequence
              ? ControlPhase::kPostContactAlign
              : ControlPhase::kSurfaceImpedance;
          phase_start_time = time;
          next_post_align_debug_time = time;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          printf("\nContact found: search = %.1f mm, force_delta = %.1f N\n",
                 1000.0 * search_distance,
                 search_force_delta_norm);
          printVec3Mm("active_edge_ee", active_tool_contact_offset_ee);
          printVec3Mm("p_EE_at_contact", p_EE);
          printVec3Mm("p_contact_1", first_contact_point);
          printVec3Mm("post_align_target",
                      first_contact_point +
                          params.post_contact_normal_push * contact_search_direction);
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
                : R_surface;
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
          params.use_phase_sequence ||
          (contact_found && params.align_orientation_to_surface_after_contact);
      const Mat3& R_d_used = (phase == ControlPhase::kPostContactAlign)
          ? R_contact_start
          : (params.orientation_test_only
                 ? R_d_orientation_test
                 : (phase == ControlPhase::kSurfaceImpedance && contact_found
                        ? R_after_contact_align
                        : (use_surface_orientation ? R_d_surface : R_d)));
      Vec3 e_R = applyRotationalAxisMask(params, orientationError(R_EE, R_d_used), R_surface);
      const Vec3 e_R_surface = R_surface.transpose() * e_R;

      if (phase == ControlPhase::kSurfaceImpedance && params.use_virtual_center_after_contact) {
        // Virtual center of rotation for the post-contact alignment phase:
        //
        //   p_c [m] = p_contact [m] + vcr_offset [m] * n_surface [-]
        //   r_c [m] = p_EE [m] - p_c [m]
        //   e_p [m] = e_p [m] - e_R [rad] x r_c [m]
        //
        // This couples the rotational correction into the translational error
        // so the tool behaves more like it is rotating around p_c.
        const Vec3 p_c = surface_point_runtime + params.vcr_offset * R_surface.col(0);
        const Vec3 r_c = p_EE - p_c;
        e_p = e_p - e_R.cross(r_c);
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
      // Comuting forces and torques from the impedance control law:
      Vec3 f = Kp_used * e_p + Dp_used * (desired.pdot_d - pdot);
      Vec3 m = KR_used * e_R - DR_used * omega;
      // The task-space wrench is computed by concatenating the force and torque vectors,
      Vec6 wrench;
      wrench.head<3>() = f;
      wrench.tail<3>() = m;
      // The joint torques for the impedance control task are computed
      // by mapping the task-space wrench through the Jacobian transpose.

      Vec7 tau_task = J.transpose() * wrench;
      if (params.orientation_test_only && time >= next_orientation_debug_time) {
        printf("\norientation_debug:\n");
        printf("time_s = %.3f\n", time);
        printVec3Deg("e_R_base", e_R);
        printVec3Deg("e_R_surface", e_R_surface);
        printVec3("m_cmd_Nm", m);
        printf("tau_task_norm_Nm = %.6f\n", tau_task.norm());
        next_orientation_debug_time += 0.5;
      }
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

      log_data.push_back(makeLogRow(
          time,
          p_EE,
          desired.p_d,
          p_end,
          e_p,
          e_R,
          pdot,
          desired.pdot_d,
          omega,
          f,
          m,
          tau_cmd));

      final_p_EE = p_EE;
      final_p_d = desired.p_d;
      final_e_p = e_p;
      final_e_R = e_R;
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

    // After exiting the control loop, check if the contact search failed and print a message if it did.
    if (contact_search_failed) {
      printf("\nContact search stopped: maximum search distance reached before contact.\n");
    }
    printJointStartEndTableDeg(q_start, final_q);

    // After the control loop finishes, write the logged data to a CSV file and print the final summary of the experiment results.
    writeLogToCsv(log_data, params.csv_file_name);
    printFinalSummary(
        final_p_d,
        final_p_EE,
        final_e_p,
        final_e_R,
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
