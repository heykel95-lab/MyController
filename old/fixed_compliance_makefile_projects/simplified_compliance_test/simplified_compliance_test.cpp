/*
  Simplified Positional Compliance Test Controller
  ------------------------------------------------
  This controller is used for first compliance tests.

  Main idea:
  - The robot is first moved to a safe initial joint configuration.
  - Then a Cartesian positional impedance controller is started.
  - Only the translational part is controlled.
  - Rotational control is disabled by setting the moment part of F to zero.
  - The controller computes a Cartesian impedance force:
        f = Kp * e_p - Dp * pdot
  - The Cartesian force is mapped to joint torques using:
        tau_task = J^T * F
  - Gravity and Coriolis compensation are added:
        tau = tau_task + tau_g + tau_c
  - The commanded torques are limited before they are returned to libfranka.
  - The measured and computed quantities are written directly to a CSV file.

  Important:
  - tau_ext is not used in this version.
  - Rotational impedance is not active in this simplified test version.
  - The Makefile links to the local working libfranka 0.7 build.
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

// This header is found through the Makefile include path:
// -I/home/hm-panda/libfranka/examples
#include "examples_common.h"


// ============================================================
// Experiment parameters
// ============================================================

// Fixed robot IP address.
const std::string robot_ip = "172.16.0.2";

// Desired end-effector position in the robot base frame.
Eigen::Vector3d p_d(0.45, 0.00, 0.35);

// Positional stiffness values.
// Lower stiffness gives softer, more compliant behavior.
double K1_p = 500.0;
double K2_p = 500.0;
double K3_p = 500.0;

// CSV output file name.
std::string csv_file_name = "simplified_compliance_test_log.csv";

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

int main(int argc, char** argv) {
  try {
    // Connect to the robot with fixed IP.
    franka::Robot robot(robot_ip);

    // Apply standard libfranka example settings.
    setDefaultBehavior(robot);

    // Move robot to safe initial joint configuration.
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

    // Set collision and contact thresholds.
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

    // Load robot model.
    franka::Model model = robot.loadModel();

    // Positional stiffness matrix.
    Eigen::Matrix3d Kp =
        Eigen::Vector3d(K1_p, K2_p, K3_p).asDiagonal();

    // Critical damping matrix with unit virtual mass.
    Eigen::Matrix3d Dp =
        2.0 * Eigen::Vector3d(std::sqrt(K1_p),
                              std::sqrt(K2_p),
                              std::sqrt(K3_p)).asDiagonal();

    // Open CSV file before entering the control loop.
    std::ofstream log_file(csv_file_name);

    // Write CSV header.
    log_file << "time,"
             << "p_EE_x,p_EE_y,p_EE_z,"
             << "p_d_x,p_d_y,p_d_z,"
             << "e_p_x,e_p_y,e_p_z,"
             << "pdot_x,pdot_y,pdot_z,"
             << "f_x,f_y,f_z,"
             << "tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7"
             << "\n";

    // Initialize experiment time.
    double time = 0.0;

    // Real-time torque-control callback.
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

      // Position error: e_p = p_d - p_EE.
      Eigen::Vector3d e_p = p_d - p_EE;

      // Positional impedance force: f = Kp * e_p - Dp * pdot.
      Eigen::Vector3d f =
          Kp * e_p - Dp * pdot;

      // Build Cartesian force/moment vector.
      // Moment part is zero because rotational control is disabled.
      Eigen::Matrix<double, 6, 1> F;
      F.head<3>() = f;
      F.tail<3>().setZero();

      // Task torque: tau_task = J^T * F.
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

      // Log values to CSV.
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

      // Return torque command.
      return franka::Torques(eigenToArray(tau_limited));
    });

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
