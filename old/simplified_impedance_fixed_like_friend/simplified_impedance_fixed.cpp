/*
  Simplified Impedance / Compliance Test
  --------------------------------------
  This version follows the important structure of the working project:

  Fixes included:
  1. No gravity compensation is added.
     The command is:
         tau_cmd = tau_task + coriolis

  2. CSV logging is not written inside the real-time callback.
     Data are stored in a memory buffer during the callback and written
     to CSV after the experiment.

  3. The experiment stops automatically using franka::MotionFinished.

  4. Safe first-test mode:
     If use_current_pose = 1 in parameters.txt, the current end-effector
     position and orientation are used as the desired pose. The robot should
     hold the pose instead of moving to a new target.

  5. Optional smooth relative motion:
     If use_current_pose = 0, the desired position moves smoothly from:
         p_start
     to:
         p_start + delta_p
     using a fifth-order smooth trajectory.

  Important:
  - This is a simplified impedance/compliance controller for testing.
  - It uses Cartesian impedance forces and moments.
  - It adds Coriolis compensation only.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"


// ============================================================
// Parameter structure
// ============================================================

struct Parameters {
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 5.0;
  std::string csv_file_name = "simplified_impedance_fixed_log.csv";

  bool use_current_pose = true;

  Eigen::Vector3d delta_p = Eigen::Vector3d(0.005, 0.0, 0.0);
  double trajectory_duration = 8.0;

  Eigen::Vector3d Kp_diag = Eigen::Vector3d(50.0, 50.0, 50.0);
  Eigen::Vector3d Dp_diag = Eigen::Vector3d(14.0, 14.0, 14.0);

  Eigen::Vector3d KR_diag = Eigen::Vector3d(3.0, 3.0, 3.0);
  Eigen::Vector3d DR_diag = Eigen::Vector3d(3.5, 3.5, 3.5);
};


// ============================================================
// Log data structure
// ============================================================

struct LogData {
  double time;

  Eigen::Vector3d p_EE;
  Eigen::Vector3d p_d;
  Eigen::Vector3d p_end;

  Eigen::Vector3d e_p;
  Eigen::Vector3d e_R;

  Eigen::Vector3d pdot;
  Eigen::Vector3d pdot_d;
  Eigen::Vector3d omega;

  Eigen::Vector3d f;
  Eigen::Vector3d m;

  Eigen::Matrix<double, 7, 1> tau;
};


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


// Remove comments and trim whitespace.
std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);

  if (begin == std::string::npos) {
    return "";
  }

  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}


// Read key=value parameter file.
Parameters readParameters(const std::string& filename) {
  Parameters p;

  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cout << "Could not open " << filename
              << ". Using default parameters." << std::endl;
    return p;
  }

  std::map<std::string, std::string> values;
  std::string line;

  while (std::getline(file, line)) {
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    const auto eq_pos = line.find('=');
    if (eq_pos == std::string::npos) {
      continue;
    }

    std::string key = trim(line.substr(0, eq_pos));
    std::string value = trim(line.substr(eq_pos + 1));

    if (!key.empty() && !value.empty()) {
      values[key] = value;
    }
  }

  auto getString = [&](const std::string& key, const std::string& default_value) {
    return values.count(key) ? values[key] : default_value;
  };

  auto getDouble = [&](const std::string& key, double default_value) {
    return values.count(key) ? std::stod(values[key]) : default_value;
  };

  auto getBool = [&](const std::string& key, bool default_value) {
    if (!values.count(key)) {
      return default_value;
    }
    return std::stoi(values[key]) != 0;
  };

  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);

  p.use_current_pose = getBool("use_current_pose", p.use_current_pose);

  p.delta_p(0) = getDouble("delta_p_x", p.delta_p(0));
  p.delta_p(1) = getDouble("delta_p_y", p.delta_p(1));
  p.delta_p(2) = getDouble("delta_p_z", p.delta_p(2));
  p.trajectory_duration = getDouble("trajectory_duration", p.trajectory_duration);

  p.Kp_diag(0) = getDouble("Kp_x", p.Kp_diag(0));
  p.Kp_diag(1) = getDouble("Kp_y", p.Kp_diag(1));
  p.Kp_diag(2) = getDouble("Kp_z", p.Kp_diag(2));

  p.Dp_diag(0) = getDouble("Dp_x", p.Dp_diag(0));
  p.Dp_diag(1) = getDouble("Dp_y", p.Dp_diag(1));
  p.Dp_diag(2) = getDouble("Dp_z", p.Dp_diag(2));

  p.KR_diag(0) = getDouble("KR_x", p.KR_diag(0));
  p.KR_diag(1) = getDouble("KR_y", p.KR_diag(1));
  p.KR_diag(2) = getDouble("KR_z", p.KR_diag(2));

  p.DR_diag(0) = getDouble("DR_x", p.DR_diag(0));
  p.DR_diag(1) = getDouble("DR_y", p.DR_diag(1));
  p.DR_diag(2) = getDouble("DR_z", p.DR_diag(2));

  return p;
}


// Fifth-order smooth interpolation.
// s(0)=0, s(1)=1, with zero velocity and acceleration at both ends.
double smoothStep(double r) {
  r = std::max(0.0, std::min(1.0, r));
  return 10.0 * std::pow(r, 3)
       - 15.0 * std::pow(r, 4)
       +  6.0 * std::pow(r, 5);
}


// Derivative of smoothStep with respect to time.
// r = t / T, so ds/dt = ds/dr / T.
double smoothStepDerivative(double r, double T) {
  r = std::max(0.0, std::min(1.0, r));

  if (r <= 0.0 || r >= 1.0) {
    return 0.0;
  }

  double ds_dr =
      30.0 * std::pow(r, 2)
    - 60.0 * std::pow(r, 3)
    + 30.0 * std::pow(r, 4);

  return ds_dr / T;
}


// Orientation error similar to the working impedance example.
Eigen::Vector3d orientationError(
    const Eigen::Matrix3d& R_current,
    const Eigen::Matrix3d& R_desired) {

  Eigen::Matrix3d R_error = R_current.transpose() * R_desired;
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  return R_current * angle_axis.axis() * angle_axis.angle();
}


// Write buffered data to CSV after the experiment.
void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name) {

  std::ofstream log_file(csv_file_name);

  log_file << "time,"
           << "p_EE_x,p_EE_y,p_EE_z,"
           << "p_d_x,p_d_y,p_d_z,"
           << "p_end_x,p_end_y,p_end_z,"
           << "e_p_x,e_p_y,e_p_z,"
           << "e_R_x,e_R_y,e_R_z,"
           << "pdot_x,pdot_y,pdot_z,"
           << "pdot_d_x,pdot_d_y,pdot_d_z,"
           << "omega_x,omega_y,omega_z,"
           << "f_x,f_y,f_z,"
           << "m_x,m_y,m_z,"
           << "tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7"
           << "\n";

  for (const auto& row : log_data) {
    log_file << std::fixed << std::setprecision(6)
             << row.time << ","

             << row.p_EE(0) << "," << row.p_EE(1) << "," << row.p_EE(2) << ","
             << row.p_d(0)  << "," << row.p_d(1)  << "," << row.p_d(2)  << ","
             << row.p_end(0) << "," << row.p_end(1) << "," << row.p_end(2) << ","

             << row.e_p(0) << "," << row.e_p(1) << "," << row.e_p(2) << ","
             << row.e_R(0) << "," << row.e_R(1) << "," << row.e_R(2) << ","

             << row.pdot(0) << "," << row.pdot(1) << "," << row.pdot(2) << ","
             << row.pdot_d(0) << "," << row.pdot_d(1) << "," << row.pdot_d(2) << ","
             << row.omega(0) << "," << row.omega(1) << "," << row.omega(2) << ","

             << row.f(0) << "," << row.f(1) << "," << row.f(2) << ","
             << row.m(0) << "," << row.m(1) << "," << row.m(2) << ","

             << row.tau(0) << "," << row.tau(1) << ","
             << row.tau(2) << "," << row.tau(3) << ","
             << row.tau(4) << "," << row.tau(5) << ","
             << row.tau(6)
             << "\n";
  }
}


// ============================================================
// Main program
// ============================================================

int main() {
  try {
    Parameters params = readParameters("parameters.txt");

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Robot IP: " << params.robot_ip << std::endl;
    std::cout << "Experiment duration: " << params.experiment_duration << " s" << std::endl;
    std::cout << "CSV file: " << params.csv_file_name << std::endl;
    std::cout << "use_current_pose: " << params.use_current_pose << std::endl;
    std::cout << "delta_p [m]: " << params.delta_p.transpose() << std::endl;

    franka::Robot robot(params.robot_ip);

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

    franka::Model model = robot.loadModel();

    // Read initial pose before starting torque control.
    franka::RobotState initial_state = robot.readOnce();

    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_initial(
        initial_state.O_T_EE.data());

    Eigen::Vector3d p_start = T_initial.block<3, 1>(0, 3);
    Eigen::Matrix3d R_d = T_initial.block<3, 3>(0, 0);

    Eigen::Vector3d p_end = p_start;

    if (!params.use_current_pose) {
      p_end = p_start + params.delta_p;
    }

    std::cout << "Initial position p_start [m]: " << p_start.transpose() << std::endl;
    std::cout << "Final target p_end [m]:       " << p_end.transpose() << std::endl;

    Eigen::Matrix3d Kp = params.Kp_diag.asDiagonal();
    Eigen::Matrix3d Dp = params.Dp_diag.asDiagonal();

    Eigen::Matrix3d KR = params.KR_diag.asDiagonal();
    Eigen::Matrix3d DR = params.DR_diag.asDiagonal();

    // Reserve memory for logging.
    // Example: 10 seconds at 1 ms = 10000 samples.
    const std::size_t reserve_size =
        static_cast<std::size_t>(params.experiment_duration * 1500.0);

    std::vector<LogData> log_data;
    log_data.reserve(reserve_size);

    double time = 0.0;

    Eigen::Vector3d final_p_EE = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_p_d = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_R = Eigen::Vector3d::Zero();

    std::cout << "Starting impedance controller." << std::endl;

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      time += period.toSec();

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          dq(state.dq.data());

      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>>
          J(jacobian_array.data());

      Eigen::Matrix<double, 6, 1> xdot = J * dq;

      Eigen::Vector3d pdot = xdot.head<3>();
      Eigen::Vector3d omega = xdot.tail<3>();

      Eigen::Map<const Eigen::Matrix<double, 4, 4>>
          T_EE(state.O_T_EE.data());

      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);
      Eigen::Matrix3d R_EE = T_EE.block<3, 3>(0, 0);

      Eigen::Vector3d p_d = p_start;
      Eigen::Vector3d pdot_d = Eigen::Vector3d::Zero();

      if (!params.use_current_pose) {
        double r = time / params.trajectory_duration;
        double s = smoothStep(r);
        double s_dot = smoothStepDerivative(r, params.trajectory_duration);

        p_d = p_start + s * params.delta_p;
        pdot_d = s_dot * params.delta_p;
      }

      Eigen::Vector3d e_p = p_d - p_EE;
      Eigen::Vector3d e_R = orientationError(R_EE, R_d);

      // Cartesian impedance law with desired velocity.
      Eigen::Vector3d f =
          Kp * e_p + Dp * (pdot_d - pdot);

      Eigen::Vector3d m =
          KR * e_R - DR * omega;

      Eigen::Matrix<double, 6, 1> wrench;
      wrench.head<3>() = f;
      wrench.tail<3>() = m;

      Eigen::Matrix<double, 7, 1> tau_task =
          J.transpose() * wrench;

      // Add Coriolis compensation only, like the working project.
      std::array<double, 7> coriolis_array =
          model.coriolis(state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>>
          coriolis(coriolis_array.data());

      Eigen::Matrix<double, 7, 1> tau_cmd =
          tau_task + coriolis;

      // Store data in memory only.
      LogData row;
      row.time = time;

      row.p_EE = p_EE;
      row.p_d = p_d;
      row.p_end = p_end;

      row.e_p = e_p;
      row.e_R = e_R;

      row.pdot = pdot;
      row.pdot_d = pdot_d;
      row.omega = omega;

      row.f = f;
      row.m = m;

      row.tau = tau_cmd;

      log_data.push_back(row);

      final_p_EE = p_EE;
      final_p_d = p_d;
      final_e_p = e_p;
      final_e_R = e_R;

      std::array<double, 7> tau_array = eigenToArray(tau_cmd);

      if (time >= params.experiment_duration) {
        return franka::MotionFinished(franka::Torques(tau_array));
      }

      return franka::Torques(tau_array);
    });

    writeLogToCsv(log_data, params.csv_file_name);

    std::cout << "\nExperiment finished." << std::endl;
    std::cout << "Final desired position p_d [m]:  " << final_p_d.transpose() << std::endl;
    std::cout << "Final reached position p_EE [m]: " << final_p_EE.transpose() << std::endl;
    std::cout << "Final position error e_p [m]:    " << final_e_p.transpose() << std::endl;
    std::cout << "Final position error norm [m]:   " << final_e_p.norm() << std::endl;
    std::cout << "Final rotation error norm [rad]: " << final_e_R.norm() << std::endl;
    std::cout << "CSV log written to: " << params.csv_file_name << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
