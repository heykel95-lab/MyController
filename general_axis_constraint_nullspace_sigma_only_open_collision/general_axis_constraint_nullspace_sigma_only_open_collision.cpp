// Include the controller types and helper functions for this experiment.
// The controller_common.h header is included by controller_types.h, so it is not needed here.
#include "controller_helpers.h"

// Main function

int main() {
  
  try {
    //Read parameters from file, with defaults for missing values.
    Parameters params = readParameters("parameters.txt");
    // Print the parameters to the console for confirmation before starting the experiment.
    printParameters(params);

    // Connect to the robot. This will block until a connection is established.
    franka::Robot robot(params.robot_ip);

    printf("\nPress Enter to recover the robot if needed and move to the initial joint configuration.\n");
    std::cin.ignore();

    // If the robot is in an error/reflex state, try to recover automatically.
    try {
      robot.automaticErrorRecovery();
      printf("Automatic error recovery finished or was not necessary.\n");
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
    MotionGenerator motion_generator(0.4, params.q_goal);

    printf("\nMoving to the initial joint configuration after the single startup confirmation...\n");
    // The robot.control() function takes a lambda function as an argument, which is called at the robot control rate (normally 1 kHz).
    // The lambda function is responsible for generating the desired joint torques for each control cycle.
    // In this case, we pass the motion_generator object, which is a callable that generates the desired joint torques to move to the initial configuration.
    robot.control(motion_generator);

    printf("Finished moving to initial joint configuration.\n");

    // Load the robot model, which is needed for the controller and nullspace optimization.
    // The model contains the robot's kinematics and dynamics, and is used to compute the Jacobian, Coriolis forces, etc.
    // The model is loaded once here and passed to the control loop lambda by reference, so it does not need to be reloaded every cycle.
    franka::Model model = robot.loadModel();

    // robot.readOnce() returns one franka::RobotState outside the control loop.
    // Here it is used only to capture the start pose after the joint-space move.
    franka::RobotState initial_state = robot.readOnce();

   // The initial end-effector pose is needed to compute the desired trajectory and the axis constraint directions.
   // Franka provides the end-effector pose as a 4x4 homogeneous transformation matrix O_T_EE, which is stored as a 16-value array in the RobotState.
   // Eigen::Map is used to view the 16-value array as a 4x4 matrix without copying the data. The resulting T_initial is a 4x4 matrix representing the
    Eigen::Map<const Mat4x4> T_initial(initial_state.O_T_EE.data());
    // The initial end-effector position p_start is the translation part of T_initial, and R_d is the rotation part of T_initial.
    // The tool axes in the robot base frame are given by the columns of R_d, which is the initial rotation of the end-effector.
    Vec3 p_start = T_initial.block<3, 1>(0, 3);
    Mat3 R_d = T_initial.block<3, 3>(0, 0);
    Vec7 q_start = Eigen::Map<const Vec7>(initial_state.q.data());
    Eigen::Map<const Vec6> initial_external_wrench(initial_state.O_F_ext_hat_K.data());
    Vec3 external_force_start = initial_external_wrench.head<3>();

    // Surface/task frame for spatial impedance:
    //
    //   column 0 = surface normal direction
    //   column 1 = first tangent direction
    //   column 2 = second tangent direction
    //
    // In surface mode the diagonal gains from parameters.txt are interpreted
    // in this frame:
    //
    //   Kp_x / Dp_x -> normal-to-surface stiffness/damping
    //   Kp_y / Dp_y -> first tangent stiffness/damping
    //   Kp_z / Dp_z -> second tangent stiffness/damping
    //
    // The gains are then transformed to the robot base frame:
    //
    //   K_base = R_surface * K_surface * R_surface^T
    //
    // This is the practical idea inspired by spatial impedance: choose simple
    // diagonal impedance values in a task/surface frame, then transform them
    // into the world/base frame.
    const Mat3 R_surface = makeSurfaceFrame(params);
    Vec3 surface_point_runtime =
        params.use_start_as_surface_point ? p_start : params.surface_point;
    const Vec3 contact_search_direction =
        normalizedOrFallback(params.contact_search_direction, -R_surface.col(0));
    const Mat3 Kp_search = params.contact_search_Kp_diag.asDiagonal();
    const Mat3 Dp_search = params.contact_search_Dp_diag.asDiagonal();
    bool searching_contact = params.use_contact_search;
    bool contact_found = !params.use_contact_search;
    bool contact_search_failed = false;
    double contact_time = 0.0;

    // The tool axes in the robot base frame are given by the columns of R_d, which is the initial rotation of the end-effector.
    // The general axis constraint will be defined based on these tool axes, and the desired trajectory will be defined relative to the initial pose.
  
    Vec3 tool_x_axis = R_d.col(0);
    Vec3 tool_y_axis = R_d.col(1);
    Vec3 tool_z_axis = R_d.col(2);
    // The final target position p_end is defined based on the initial position p_start and the desired change in position delta_p from parameters.txt.
    // If use_current_pose is true, then p_end is set to the current position p_start, which means the desired trajectory will be a stationary hold at the initial position.
    // If use_current_pose is false, then p_end is set to p_start + delta_p, which means the desired trajectory will be a move from the initial position to the target position defined by delta_p.
    Vec3 p_end = p_start;
    if (!params.use_current_pose) {
      p_end = p_start + params.delta_p;
    }
    // Print the initial conditions and parameters related to the trajectory and constraint for confirmation.
    printVec3("Initial position p_start [m]:", p_start);
    printVec3("Final target p_end [m]:      ", p_end);
    // The tool axes are important to print because they define the directions of the general axis constraint, which is a key part of this experiment.
    printVec3("Tool x-axis in base frame:   ", tool_x_axis);
    printVec3("Tool y-axis in base frame:   ", tool_y_axis);
    printVec3("Tool z-axis in base frame:   ", tool_z_axis);
    //
    if (params.axis_constraint_mode) {
      printf("General axis constraint active.\n");
      if (params.use_surface_constraint) {
        printf("Surface constraint active: Kp_x/Dp_x are normal gains, y/z are tangent gains.\n");
        if (params.use_contact_search) {
          printf("Contact search active: virtual surface point will be set from detected contact.\n");
        }
      } else {
        printf("Translation: fixed base axes; free axes follow current position.\n");
      }
      printf("Rotation: fixed components keep rotational stiffness; free components have no rotational spring.\n");
    }

    // The controller gains are defined as diagonal matrices based on the parameters read from parameters.txt.
    Mat3 Kp = params.use_surface_constraint
        ? makeSpatialGainMatrix(params.Kp_diag, R_surface)
        : params.Kp_diag.asDiagonal();
    Mat3 Dp = params.use_surface_constraint
        ? makeSpatialGainMatrix(params.Dp_diag, R_surface)
        : params.Dp_diag.asDiagonal();
    Mat3 KR = params.use_surface_constraint
        ? makeSpatialGainMatrix(params.KR_diag, R_surface)
        : params.KR_diag.asDiagonal();
    Mat3 DR = params.use_surface_constraint
        ? makeSpatialGainMatrix(params.DR_diag, R_surface)
        : params.DR_diag.asDiagonal();

    // The log_data vector will store the data for each control cycle, which will be written to a CSV file at the end of the experiment.
    std::vector<LogData> log_data;
    log_data.reserve(static_cast<std::size_t>(params.experiment_duration * 1500.0));
    // The time variable is used to keep track of the elapsed time since the start of the control loop, and is updated in each cycle based on the period provided by libfranka.
    double time = 0.0;
    // The final values of key variables are stored here to be printed in the final summary after the control loop finishes. They are updated in each cycle, so they will contain the values from the last cycle when the loop ends.
    Vec3 final_p_EE = Vec3::Zero();
    Vec3 final_p_d = Vec3::Zero();
    Vec3 final_e_p = Vec3::Zero();
    Vec3 final_e_R = Vec3::Zero();

    // Print instructions for early stopping the experiment, and start a separate thread to listen for the "e" key input to request stopping the control loop safely.
    printf("Starting impedance controller:\n");
  
    // The stop_requested variable is an atomic boolean that is shared between the main thread (which runs the control loop) and the keyboard thread (which listens for user input). It is used to signal when the user has requested to stop the experiment early by pressing "e" + Enter.
    std::atomic<bool> stop_requested(false);

    printf("Type e and press Enter during the impedance run to stop the control loop safely.\n");
    if (params.experiment_duration <= 0.0) {
      printf("experiment_duration <= 0: running indefinitely until e + Enter.\n");
    } else {
      printf("Otherwise, the experiment stops automatically after experiment_duration, or earlier with e + Enter.\n");
    }
    // The keyboard thread runs a loop that waits for user input. If the user types "e" or "E" and presses Enter, it sets stop_requested to true, which will signal the control loop to finish.
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

    // libfranka calls this lambda at the robot control rate, normally 1 kHz.
    //
    // The two callback arguments are provided by libfranka:
    //
    //   state  = franka::RobotState
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
    //   period = franka::Duration
    //            Measured time since the previous callback [s].
    //
    // The lambda must return franka::Torques, i.e. the 7 commanded joint torques
    // tau_cmd [Nm] for this cycle.
    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {
      time += period.toSec();

      // state.dq is the measured joint velocity from the robot [rad/s].
      Eigen::Map<const Vec7> dq(state.dq.data());

      // This Jacobian maps joint velocity dq [rad/s] to end-effector velocity:
      //
      //   xdot = J_EE(q) * dq
      //
      // where xdot = [linear velocity of EE; angular velocity of EE].
      // Because the requested frame is kEndEffector, the Jacobian also uses
      // the EE/TCP frame. Therefore the tool offset is included here too when
      // the robot's F_T_EE is configured correctly.
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);
      Eigen::Map<const Mat6x7> J(jacobian_array.data());

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
      Eigen::Map<const Mat4x4> T_EE(state.O_T_EE.data());
      Vec3 p_EE = T_EE.block<3, 1>(0, 3);
      Mat3 R_EE = T_EE.block<3, 3>(0, 0);

      Eigen::Map<const Vec6> external_wrench(state.O_F_ext_hat_K.data());
      const Vec3 external_force_delta = external_wrench.head<3>() - external_force_start;

      DesiredMotion desired;
      double impedance_time = time;

      if (searching_contact) {
        const double search_distance =
            std::min(params.contact_search_speed * time,
                     params.contact_search_max_distance);

        desired.p_d = p_start + search_distance * contact_search_direction;
        desired.pdot_d = params.contact_search_speed * contact_search_direction;

        // Contact trigger only, not force control:
        //
        //   if ||F_ext - F_ext_start|| [N] > threshold [N],
        //   the current TCP position becomes the point of the virtual surface.
        //
        // After this one-time event, the controller switches back to pure
        // surface impedance. The measured force is not used as a control input.
        if (external_force_delta.norm() >= params.contact_force_threshold) {
          surface_point_runtime = p_EE;
          searching_contact = false;
          contact_found = true;
          contact_time = time;
          printf("\nContact found. Virtual surface point set from current TCP position.\n");
          printVec3("surface_point_runtime [m]:", surface_point_runtime);
        } else if (search_distance >= params.contact_search_max_distance) {
          contact_search_failed = true;
          stop_requested.store(true);
          desired.p_d = p_EE;
          desired.pdot_d.setZero();
        }
      }

      if (contact_found) {
        impedance_time = params.use_contact_search ? (time - contact_time) : time;
        desired = computeDesiredMotion(
            params,
            impedance_time,
            p_start,
            p_EE,
            R_surface,
            surface_point_runtime);
      }

      Vec3 e_p = desired.p_d - p_EE;
      Vec3 e_R = applyRotationalAxisMask(params, orientationError(R_EE, R_d), R_surface);

      const Mat3& Kp_used = searching_contact ? Kp_search : Kp;
      const Mat3& Dp_used = searching_contact ? Dp_search : Dp;

      Vec3 f = Kp_used * e_p + Dp_used * (desired.pdot_d - pdot);
      Vec3 m = KR * e_R - DR * omega;

      Vec6 wrench = makeWrench(f, m);
      Vec7 tau_task = J.transpose() * wrench;

      Vec7 tau_nullspace =
          computeNullspaceTorque(params, model, state, J, dq, q_start);

      Array7 coriolis_array = model.coriolis(state);
      Eigen::Map<const Vec7> coriolis(coriolis_array.data());

      Vec7 tau_raw = tau_task + tau_nullspace;
      if (params.use_coriolis) {
        tau_raw += coriolis;
      }

      Vec7 tau_cmd = tau_raw;

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
          tau_raw,
          tau_cmd));

      final_p_EE = p_EE;
      final_p_d = desired.p_d;
      final_e_p = e_p;
      final_e_R = e_R;

      Array7 tau_array = vec7ToArray(tau_cmd);

      if (((params.experiment_duration > 0.0) && (impedance_time >= params.experiment_duration)) ||
          stop_requested.load()) {
        if (stop_requested.load()) {
          printf("\nStop requested with e + Enter. Finishing control loop...\n");
        }
        return franka::MotionFinished(franka::Torques(tau_array));
      }

      return franka::Torques(tau_array);
    });

    if (contact_search_failed) {
      printf("\nContact search stopped: maximum search distance reached before contact.\n");
    }

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
