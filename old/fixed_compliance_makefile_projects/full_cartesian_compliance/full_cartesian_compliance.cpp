/*
  Full Cartesian Impedance / Compliance Controller
  ------------------------------------------------
  This controller is used for Cartesian compliance experiments.

  Main idea:
  - The robot is first moved to a safe initial joint configuration.
  - Then a Cartesian impedance controller is started.
  - The controller computes:
        positional force:  f = Kp * e_p - Dp * pdot
        rotational moment: m = KR * e_R - DR * omega
  - Force and moment are combined into:
        F = [f, m]^T
  - The Cartesian force/moment vector is mapped to joint torques:
        tau_task = J^T * F
  - Gravity and Coriolis compensation are added:
        tau = tau_task + tau_g + tau_c
  - The commanded torques are limited before they are returned to libfranka.
  - The measured and computed quantities are written directly to a CSV file.

  Important:
  - tau_ext is not used in this version.
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
#include <Eigen/Geometry>

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

const std::string robot_ip = "172.16.0.2";

// Desired end-effector position in the robot base frame.
Eigen::Vector3d p_d(0.45, 0.00, 0.35);

// Desired orientation angles in radians.
double roll_d  = 0.0;
double pitch_d = 0.0;
double yaw_d   = 0.0;

// Positional stiffness values.
double K1_p = 800.0;
double K2_p = 800.0;
double K3_p = 800.0;

// Rotational stiffness values.
double K1_R = 30.0;
double K2_R = 30.0;
double K3_R = 30.0;

// CSV output file name.
std::string csv_file_name = "full_cartesian_compliance_log.csv";

// Torque limit.
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


// Compute desired orientation matrix using ZYX convention:
// R_d = Rz(yaw_d) * Ry(pitch_d) * Rx(roll_d).
Eigen::Matrix3d computeDesiredOrientation(
    double roll_d,
    double pitch_d,
    double yaw_d) {

  Eigen::Matrix3d R_d =
      Eigen::AngleAxisd(yaw_d, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(pitch_d, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(roll_d, Eigen::Vector3d::UnitX()).toRotationMatrix();

  return R_d;
}


// Compute rotational error vector from desired and current orientation.
Eigen::Vector3d computeOrientationError(
    const Eigen::Matrix3d& R_d,
    const Eigen::Matrix3d& R_EE) {

  Eigen::Matrix3d dR = R_d.transpose() * R_EE;

  double cos_phi = (dR.trace() - 1.0) / 2.0;
  cos_phi = std::min(1.0, std::max(-1.0, cos_phi));

  double phi = std::acos(cos_phi);

  Eigen::Vector3d e_R;

  if (phi < 1e-6) {
    e_R.setZero();
  } else {
    e_R =
        phi / (2.0 * std::sin(phi)) *
        Eigen::Vector3d(dR(2, 1) - dR(1, 2),
                        dR(0, 2) - dR(2, 0),
                        dR(1, 0) - dR(0, 1));
  }

  return e_R;
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

    // Desired orientation.
    Eigen::Matrix3d R_d =
        computeDesiredOrientation(roll_d, pitch_d, yaw_d);

    // Diagonal stiffness matrices in desired end-effector frame.
    Eigen::Matrix3d Kp_EE =
        Eigen::Vector3d(K1_p, K2_p, K3_p).asDiagonal();

    Eigen::Matrix3d KR_EE =
        Eigen::Vector3d(K1_R, K2_R, K3_R).asDiagonal();

    // Critical damping matrices with unit virtual mass.
    Eigen::Matrix3d Dp_EE =
        2.0 * Eigen::Vector3d(std::sqrt(K1_p),
                              std::sqrt(K2_p),
                              std::sqrt(K3_p)).asDiagonal();

    Eigen::Matrix3d DR_EE =
        2.0 * Eigen::Vector3d(std::sqrt(K1_R),
                              std::sqrt(K2_R),
                              std::sqrt(K3_R)).asDiagonal();

    // Express stiffness and damping matrices in the base frame.
    Eigen::Matrix3d Kp_base = R_d * Kp_EE * R_d.transpose();
    Eigen::Matrix3d Dp_base = R_d * Dp_EE * R_d.transpose();

    Eigen::Matrix3d KR_base = R_d * KR_EE * R_d.transpose();
    Eigen::Matrix3d DR_base = R_d * DR_EE * R_d.transpose();

    // Open CSV file.
    std::ofstream log_file(csv_file_name);

    log_file << "time,"
             << "p_EE_x,p_EE_y,p_EE_z,"
             << "p_d_x,p_d_y,p_d_z,"
             << "e_p_x,e_p_y,e_p_z,"
             << "e_R_x,e_R_y,e_R_z,"
             << "pdot_x,pdot_y,pdot_z,"
             << "omega_x,omega_y,omega_z,"
             << "f_x,f_y,f_z,"
             << "m_x,m_y,m_z,"
             << "tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7"
             << "\n";

    double time = 0.0;

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      // Map current joint positions and velocities.
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          q(state.q.data());

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          dq(state.dq.data());

      // Compute geometric Jacobian at the end-effector.
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>>
          J(jacobian_array.data());

      // Cartesian velocity.
      Eigen::Matrix<double, 6, 1> xdot = J * dq;

      Eigen::Vector3d pdot = xdot.head<3>();
      Eigen::Vector3d omega = xdot.tail<3>();

      // Current end-effector pose.
      Eigen::Map<const Eigen::Matrix<double, 4, 4>>
          T_EE(state.O_T_EE.data());

      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);
      Eigen::Matrix3d R_EE = T_EE.block<3, 3>(0, 0);

      // Position and orientation errors.
      Eigen::Vector3d e_p = p_d - p_EE;
      Eigen::Vector3d e_R = computeOrientationError(R_d, R_EE);

      // Cartesian impedance force and moment.
      Eigen::Vector3d f =
          Kp_base * e_p - Dp_base * pdot;

      Eigen::Vector3d m =
          KR_base * e_R - DR_base * omega;

      // Cartesian force/moment vector.
      Eigen::Matrix<double, 6, 1> F;
      F.head<3>() = f;
      F.tail<3>() = m;

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

      // Limit commanded torque.
      Eigen::Matrix<double, 7, 1> tau_limited =
          limitTorques(tau);

      // Update time.
      time += period.toSec();

      // Log row.
      log_file << std::fixed << std::setprecision(6)
               << time << ","
               << p_EE(0) << "," << p_EE(1) << "," << p_EE(2) << ","
               << p_d(0)  << "," << p_d(1)  << "," << p_d(2)  << ","
               << e_p(0) << "," << e_p(1) << "," << e_p(2) << ","
               << e_R(0) << "," << e_R(1) << "," << e_R(2) << ","
               << pdot(0) << "," << pdot(1) << "," << pdot(2) << ","
               << omega(0) << "," << omega(1) << "," << omega(2) << ","
               << f(0) << "," << f(1) << "," << f(2) << ","
               << m(0) << "," << m(1) << "," << m(2) << ","
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
