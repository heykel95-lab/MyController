/*
  Simplified Positional Compliance Test Controller
  ------------------------------------------------
  Fixes added:
  - The experiment stops automatically after experiment_duration seconds.
  - The program prints the final reached end-effector position in the terminal.
  - The program prints the final desired position and final position error.
  - The program still logs values directly to CSV inside the callback.
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

const std::string robot_ip = "172.16.0.2";

// Desired end-effector position in the robot base frame.
Eigen::Vector3d p_d(0.45, 0.00, 0.35);

// Positional stiffness values.
double K1_p = 500.0;
double K2_p = 500.0;
double K3_p = 500.0;

// Experiment duration in seconds.
// After this time, the torque callback stops automatically.
const double experiment_duration = 10.0;

// CSV output file name.
std::string csv_file_name = "simplified_compliance_test_log.csv";

// Torque limit used for commanded joint torques.
const double tau_max = 87.0;


// ============================================================
// Helper functions
// ============================================================

std::array<double, 7> eigenToArray(
    const Eigen::Matrix<double, 7, 1>& tau) {

  std::array<double, 7> tau_array{};

  for (int i = 0; i < 7; ++i) {
    tau_array[i] = tau(i);
  }

  return tau_array;
}


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
    franka::Robot robot(robot_ip);

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

    franka::Model model = robot.loadModel();

    Eigen::Matrix3d Kp =
        Eigen::Vector3d(K1_p, K2_p, K3_p).asDiagonal();

    Eigen::Matrix3d Dp =
        2.0 * Eigen::Vector3d(std::sqrt(K1_p),
                              std::sqrt(K2_p),
                              std::sqrt(K3_p)).asDiagonal();

    std::ofstream log_file(csv_file_name);

    log_file << "time,"
             << "p_EE_x,p_EE_y,p_EE_z,"
             << "p_d_x,p_d_y,p_d_z,"
             << "e_p_x,e_p_y,e_p_z,"
             << "pdot_x,pdot_y,pdot_z,"
             << "f_x,f_y,f_z,"
             << "tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7"
             << "\n";

    double time = 0.0;

    // These variables are updated inside the callback and printed after the experiment.
    Eigen::Vector3d final_p_EE = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_p = Eigen::Vector3d::Zero();

    std::cout << "Starting compliance controller for "
              << experiment_duration << " seconds." << std::endl;

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          dq(state.dq.data());

      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>>
          J(jacobian_array.data());

      Eigen::Matrix<double, 6, 1> xdot = J * dq;

      Eigen::Vector3d pdot = xdot.head<3>();

      Eigen::Map<const Eigen::Matrix<double, 4, 4>>
          T_EE(state.O_T_EE.data());

      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);

      Eigen::Vector3d e_p = p_d - p_EE;

      Eigen::Vector3d f =
          Kp * e_p - Dp * pdot;

      Eigen::Matrix<double, 6, 1> F;
      F.head<3>() = f;
      F.tail<3>().setZero();

      Eigen::Matrix<double, 7, 1> tau_task =
          J.transpose() * F;

      std::array<double, 7> gravity_array =
          model.gravity(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          tau_g(gravity_array.data());

      std::array<double, 7> coriolis_array =
          model.coriolis(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          tau_c(coriolis_array.data());

      Eigen::Matrix<double, 7, 1> tau =
          tau_task + tau_g + tau_c;

      Eigen::Matrix<double, 7, 1> tau_limited =
          limitTorques(tau);

      time += period.toSec();

      final_p_EE = p_EE;
      final_e_p = e_p;

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

      if (time >= experiment_duration) {
        return franka::MotionFinished(
            franka::Torques(eigenToArray(tau_limited)));
      }

      return franka::Torques(eigenToArray(tau_limited));
    });

    log_file.close();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nExperiment finished." << std::endl;
    std::cout << "Desired position p_d [m]:       "
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
