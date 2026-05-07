/*
  Simplified Positional Compliance Test with Relative Desired Position
  -------------------------------------------------------------------
  This version keeps CSV logging inside the callback, but changes the desired
  position definition.

  Instead of using a fixed absolute desired position, the controller reads the
  initial end-effector position at the beginning of the torque-control callback
  and defines:

      p_d = p_start + delta_p

  This is similar to the idea used in the other project:
  the desired target is defined relative to the initial robot state.

  Important:
  - Only positional impedance is active.
  - Rotational control is disabled.
  - CSV logging is still performed inside the callback.
  - The experiment duration is shortened for a first test.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"


// ============================================================
// Experiment parameters
// ============================================================

// Fixed robot IP address.
const std::string robot_ip = "172.16.0.2";

// Relative displacement from the initial end-effector position.
// For a first test, keep this small.
// Example: move 2 cm in x-direction.
Eigen::Vector3d delta_p(0.02, 0.00, 0.00);

// Positional stiffness values.
// Lower stiffness gives softer, more compliant behavior.
double K1_p = 100.0;
double K2_p = 100.0;
double K3_p = 100.0;

// Short experiment duration in seconds.
const double experiment_duration = 3.0;

// CSV output file name.
std::string csv_file_name = "simplified_compliance_delta_log.csv";

// Torque limit used for commanded joint torques.
const double tau_max = 87.0;


// ============================================================
// Helper functions
// ============================================================

// Define function eigenToArray:
// convert an Eigen torque vector to a std::array required by libfranka.
std::array<double, 7> eigenToArray(
    const Eigen::Matrix<double, 7, 1>& tau) {

  std::array<double, 7> tau_array{};

  for (int i = 0; i < 7; ++i) {
    tau_array[i] = tau(i);
  }

  return tau_array;
}


// Define function limitTorques:
// limit each commanded joint torque to the interval [-tau_max, tau_max].
Eigen::Matrix<double, 7, 1> limitTorques(
    const Eigen::Matrix<double, 7, 1>& tau) {

  Eigen::Matrix<double, 7, 1> tau_limited;

  for (int i = 0; i < 7; ++i) {
    tau_limited(i) = std::max(-tau_max, std::min(tau_max, tau(i)));
  }

  return tau_limited;
}


// ============================================================
// Main program
// ============================================================

int main() {
  try {
    // ============================================================
    // Connect to robot
    // ============================================================

    franka::Robot robot(robot_ip);


    // ============================================================
    // Set default behavior
    // ============================================================

    setDefaultBehavior(robot);


    // ============================================================
    // Move robot to safe initial joint configuration
    // ============================================================

    std::array<double, 7> q_goal = {{
        0.0,
        -M_PI_4,
        0.0,
        -3.0 * M_PI_4,
        0.0,
        M_PI_2,
        M_PI_4
    }};

    MotionGenerator motion_generator(0.5, q_goal);

    std::cout << "WARNING: The robot will move to the initial joint configuration." << std::endl;
    std::cout << "Make sure the workspace is free and the emergency stop is available." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    robot.control(motion_generator);

    std::cout << "Finished moving to initial joint configuration." << std::endl;


    // ============================================================
    // Set collision behavior
    // ============================================================

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}
    );


    // ============================================================
    // Load robot model
    // ============================================================

    franka::Model model = robot.loadModel();


    // ============================================================
    // Define stiffness and damping matrices
    // ============================================================

    Eigen::Matrix3d Kp =
        Eigen::Vector3d(K1_p, K2_p, K3_p).asDiagonal();

    Eigen::Matrix3d Dp =
        2.0 * Eigen::Vector3d(std::sqrt(K1_p),
                              std::sqrt(K2_p),
                              std::sqrt(K3_p)).asDiagonal();


    // ============================================================
    // Open CSV file
    // ============================================================

    std::ofstream log_file(csv_file_name);

    log_file << "time,"
             << "p_EE_x,p_EE_y,p_EE_z,"
             << "p_d_x,p_d_y,p_d_z,"
             << "e_p_x,e_p_y,e_p_z,"
             << "pdot_x,pdot_y,pdot_z,"
             << "f_x,f_y,f_z,"
             << "tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7"
             << "\n";


    // ============================================================
    // Initialize experiment variables
    // ============================================================

    double time = 0.0;

    // The initial position is read at the first callback step.
    bool initial_position_set = false;
    Eigen::Vector3d p_start = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_d = Eigen::Vector3d::Zero();

    Eigen::Vector3d final_p_EE = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_p = Eigen::Vector3d::Zero();

    std::cout << "Starting simplified compliance test with relative target." << std::endl;
    std::cout << "Experiment duration: " << experiment_duration << " s" << std::endl;
    std::cout << "delta_p [m]: "
              << delta_p(0) << ", " << delta_p(1) << ", " << delta_p(2)
              << std::endl;


    // ============================================================
    // Real-time torque-control callback
    // ============================================================

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      // Map current joint velocities.
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          dq(state.dq.data());

      // Compute the geometric Jacobian at the end-effector.
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      // Map Jacobian array to a 6x7 Eigen matrix.
      Eigen::Map<const Eigen::Matrix<double, 6, 7>>
          J(jacobian_array.data());

      // Cartesian velocity xdot = J(q) * dq.
      Eigen::Matrix<double, 6, 1> xdot = J * dq;

      // Extract only the translational velocity.
      Eigen::Vector3d pdot = xdot.head<3>();

      // Read current end-effector pose.
      Eigen::Map<const Eigen::Matrix<double, 4, 4>>
          T_EE(state.O_T_EE.data());

      // Extract current end-effector position.
      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);

      // At the first callback cycle, store the start position and compute the target.
      if (!initial_position_set) {
        p_start = p_EE;
        p_d = p_start + delta_p;
        initial_position_set = true;

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Initial position p_start [m]: "
                  << p_start(0) << ", " << p_start(1) << ", " << p_start(2)
                  << std::endl;
        std::cout << "Desired position p_d = p_start + delta_p [m]: "
                  << p_d(0) << ", " << p_d(1) << ", " << p_d(2)
                  << std::endl;
      }

      // Position error.
      Eigen::Vector3d e_p = p_d - p_EE;

      // Positional impedance force.
      Eigen::Vector3d f =
          Kp * e_p - Dp * pdot;

      // Cartesian force/moment vector.
      // Moment part is zero because rotational control is disabled.
      Eigen::Matrix<double, 6, 1> F;
      F.head<3>() = f;
      F.tail<3>().setZero();

      // Task torque.
      Eigen::Matrix<double, 7, 1> tau_task =
          J.transpose() * F;

      // Gravity compensation.
      std::array<double, 7> gravity_array =
          model.gravity(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          tau_g(gravity_array.data());

      // Coriolis and centrifugal compensation.
      std::array<double, 7> coriolis_array =
          model.coriolis(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          tau_c(coriolis_array.data());

      // Final commanded torque.
      Eigen::Matrix<double, 7, 1> tau =
          tau_task + tau_g + tau_c;

      // Limit torques before returning them to libfranka.
      Eigen::Matrix<double, 7, 1> tau_limited =
          limitTorques(tau);

      // Update experiment time.
      time += period.toSec();

      final_p_EE = p_EE;
      final_e_p = e_p;

      // Keep CSV logging inside the callback as requested.
      log_file << std::fixed << std::setprecision(6)
               << time << ","
               << p_EE(0) << "," << p_EE(1) << "," << p_EE(2) << ","
               << p_d(0)  << "," << p_d(1)  << "," << p_d(2)  << ","
               << e_p(0) << "," << e_p(1) << "," << e_p(2) << ","
               << pdot(0) << "," << pdot(1) << "," << pdot(2) << ","
               << f(0) << "," << f(1) << "," << f(2) << ","
               << tau_limited(0) << "," << tau_limited(1) << ","
               << tau_limited(2) << "," << tau_limited(3) << ","
               << tau_limited(4) << "," << tau_limited(5) << ","
               << tau_limited(6)
               << "\n";

      // Stop the experiment automatically after the selected duration.
      if (time >= experiment_duration) {
        return franka::MotionFinished(
            franka::Torques(eigenToArray(tau_limited)));
      }

      return franka::Torques(eigenToArray(tau_limited));
    });

    log_file.close();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nExperiment finished." << std::endl;
    std::cout << "Initial position p_start [m]:    "
              << p_start(0) << ", " << p_start(1) << ", " << p_start(2) << std::endl;
    std::cout << "Desired position p_d [m]:        "
              << p_d(0) << ", " << p_d(1) << ", " << p_d(2) << std::endl;
    std::cout << "Final reached position p_EE [m]: "
              << final_p_EE(0) << ", " << final_p_EE(1) << ", " << final_p_EE(2) << std::endl;
    std::cout << "Final position error e_p [m]:    "
              << final_e_p(0) << ", " << final_e_p(1) << ", " << final_e_p(2) << std::endl;
    std::cout << "Final position error norm [m]:   "
              << final_e_p.norm() << std::endl;
    std::cout << "CSV log written to: " << csv_file_name << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
