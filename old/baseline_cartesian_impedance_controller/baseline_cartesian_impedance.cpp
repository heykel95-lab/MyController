/*
  Baseline Cartesian Impedance Controller
  ---------------------------------------
  This controller is used as the baseline comparison for the thesis.

  It implements the basic Cartesian impedance law:

      f = Kp * (p_d - p_EE) - Dp * p_dot
      m = KR * e_R - DR * omega

  and maps the Cartesian wrench to joint torques:

      tau_cmd = J^T * [f; m] + coriolis

  This baseline intentionally does NOT include the extended additions:

      - Cartesian force saturation f_max
      - Cartesian moment saturation m_max
      - joint torque-rate limiting delta_tau_max
      - threshold-based friction compensation f_fric / m_fric
      - rho(E) or rho(e) energy/error shaping

  Purpose:
  It gives a clear reference controller for comparison with the extended
  safe/friction-compensated controller.

  Terminal output:
  At the end, the program prints:
      - final desired pose
      - final reached pose
      - final position error
      - maximum position error during the experiment
      - final rotation error
      - maximum rotation error during the experiment
      - final Cartesian force/moment command
      - final joint torque command

  CSV logging:
  The experiment is logged to a CSV file for plotting and evaluation.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
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

struct Parameters {
  std::string robot_ip = "172.16.0.2";
  double experiment_duration = 12.0;
  std::string csv_file_name = "baseline_cartesian_impedance_log.csv";

  // Initial joint configuration [rad].
  // Same horizontal-pose start used in the extended controller.
  std::array<double, 7> q_goal = {{
      0.0,
      -M_PI_4,
      0.0,
      -3.0 * M_PI_4,
      0.0,
      M_PI_2,
      0.0
  }};

  // Translational stiffness [N/m].
  Eigen::Vector3d Kp_diag = Eigen::Vector3d(120.0, 100.0, 100.0);

  // Translational damping [Ns/m].
  Eigen::Vector3d Dp_diag = Eigen::Vector3d(20.0, 20.0, 20.0);

  // Rotational stiffness [Nm/rad].
  Eigen::Vector3d KR_diag = Eigen::Vector3d(3.5, 2.0, 3.5);

  // Rotational damping [Nms/rad].
  Eigen::Vector3d DR_diag = Eigen::Vector3d(4.0, 2.0, 2.0);
};

struct LogData {
  double time;

  Eigen::Vector3d p_EE;
  Eigen::Vector3d p_d;
  Eigen::Vector3d e_p;
  Eigen::Vector3d e_R;

  Eigen::Vector3d pdot;
  Eigen::Vector3d omega;

  Eigen::Vector3d f;
  Eigen::Vector3d m;

  Eigen::Matrix<double, 7, 1> tau_cmd;
};

std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

Parameters readParameters(const std::string& filename) {
  Parameters p;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cout << "Could not open " << filename << ". Using default parameters." << std::endl;
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

    const std::string key = trim(line.substr(0, eq_pos));
    const std::string value = trim(line.substr(eq_pos + 1));

    if (!key.empty() && !value.empty()) {
      values[key] = value;
    }
  }

  auto getString = [&](const std::string& key, const std::string& def) {
    return values.count(key) ? values[key] : def;
  };

  auto getDouble = [&](const std::string& key, double def) {
    return values.count(key) ? std::stod(values[key]) : def;
  };

  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);

  p.q_goal[0] = getDouble("q_goal_1", p.q_goal[0]);
  p.q_goal[1] = getDouble("q_goal_2", p.q_goal[1]);
  p.q_goal[2] = getDouble("q_goal_3", p.q_goal[2]);
  p.q_goal[3] = getDouble("q_goal_4", p.q_goal[3]);
  p.q_goal[4] = getDouble("q_goal_5", p.q_goal[4]);
  p.q_goal[5] = getDouble("q_goal_6", p.q_goal[5]);
  p.q_goal[6] = getDouble("q_goal_7", p.q_goal[6]);

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

std::array<double, 7> eigenToArray(const Eigen::Matrix<double, 7, 1>& tau) {
  std::array<double, 7> tau_array{};
  for (int i = 0; i < 7; ++i) {
    tau_array[i] = tau(i);
  }
  return tau_array;
}

Eigen::Vector3d orientationError(
    const Eigen::Matrix3d& R_current,
    const Eigen::Matrix3d& R_desired) {

  Eigen::Matrix3d R_error = R_current.transpose() * R_desired;
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  // Express orientation error in the robot base frame.
  return R_current * angle_axis.axis() * angle_axis.angle();
}

void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name) {

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

  for (const auto& row : log_data) {
    log_file << std::fixed << std::setprecision(6)
             << row.time << ","

             << row.p_EE(0) << "," << row.p_EE(1) << "," << row.p_EE(2) << ","
             << row.p_d(0) << "," << row.p_d(1) << "," << row.p_d(2) << ","

             << row.e_p(0) << "," << row.e_p(1) << "," << row.e_p(2) << ","
             << row.e_R(0) << "," << row.e_R(1) << "," << row.e_R(2) << ","

             << row.pdot(0) << "," << row.pdot(1) << "," << row.pdot(2) << ","
             << row.omega(0) << "," << row.omega(1) << "," << row.omega(2) << ","

             << row.f(0) << "," << row.f(1) << "," << row.f(2) << ","
             << row.m(0) << "," << row.m(1) << "," << row.m(2) << ","

             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
}

int main() {
  try {
    Parameters params = readParameters("parameters.txt");

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Baseline Cartesian impedance controller" << std::endl;
    std::cout << "Robot IP: " << params.robot_ip << std::endl;
    std::cout << "Experiment duration: " << params.experiment_duration << " s" << std::endl;
    std::cout << "CSV file: " << params.csv_file_name << std::endl;
    std::cout << "Kp [N/m]: " << params.Kp_diag.transpose() << std::endl;
    std::cout << "Dp [Ns/m]: " << params.Dp_diag.transpose() << std::endl;
    std::cout << "KR [Nm/rad]: " << params.KR_diag.transpose() << std::endl;
    std::cout << "DR [Nms/rad]: " << params.DR_diag.transpose() << std::endl;
    std::cout << "q_goal [rad]: "
              << params.q_goal[0] << " "
              << params.q_goal[1] << " "
              << params.q_goal[2] << " "
              << params.q_goal[3] << " "
              << params.q_goal[4] << " "
              << params.q_goal[5] << " "
              << params.q_goal[6] << std::endl;

    franka::Robot robot(params.robot_ip);

    std::cout << "\nRecovery step:" << std::endl;
    std::cout << "If the robot is in an error/reflex state, automatic recovery will be attempted." << std::endl;
    std::cout << "Make sure the workspace is clear and the emergency stop is reachable." << std::endl;
    std::cout << "Press Enter to attempt recovery and continue..." << std::endl;
    std::cin.ignore();

    try {
      robot.automaticErrorRecovery();
      std::cout << "Automatic error recovery finished or was not necessary." << std::endl;
    } catch (const franka::Exception& e) {
      std::cerr << "Automatic error recovery failed: " << e.what() << std::endl;
      std::cerr << "Please recover/unlock the robot manually in Franka Desk." << std::endl;
      return -1;
    }

    setDefaultBehavior(robot);

    MotionGenerator motion_generator(0.4, params.q_goal);

    std::cout << "\nWARNING: The robot will move to the initial joint configuration." << std::endl;
    std::cout << "Make sure the workspace is free and the emergency stop is available." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    robot.control(motion_generator);

    std::cout << "Finished moving to initial joint configuration." << std::endl;

    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_initial(
        initial_state.O_T_EE.data());

    Eigen::Vector3d p_d = T_initial.block<3, 1>(0, 3);
    Eigen::Matrix3d R_d = T_initial.block<3, 3>(0, 0);

    Eigen::Vector3d tool_x_axis = R_d.col(0);
    Eigen::Vector3d tool_y_axis = R_d.col(1);
    Eigen::Vector3d tool_z_axis = R_d.col(2);

    std::cout << "Initial/desired position p_d [m]: " << p_d.transpose() << std::endl;
    std::cout << "Tool x-axis in base frame:       " << tool_x_axis.transpose() << std::endl;
    std::cout << "Tool y-axis in base frame:       " << tool_y_axis.transpose() << std::endl;
    std::cout << "Tool z-axis in base frame:       " << tool_z_axis.transpose() << std::endl;

    Eigen::Matrix3d Kp = params.Kp_diag.asDiagonal();
    Eigen::Matrix3d Dp = params.Dp_diag.asDiagonal();
    Eigen::Matrix3d KR = params.KR_diag.asDiagonal();
    Eigen::Matrix3d DR = params.DR_diag.asDiagonal();

    std::vector<LogData> log_data;
    log_data.reserve(static_cast<std::size_t>(params.experiment_duration * 1500.0));

    double time = 0.0;

    Eigen::Vector3d final_p_EE = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_e_R = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_m = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 7, 1> final_tau;
    final_tau.setZero();

    double max_position_error_norm = 0.0;
    double max_rotation_error_norm = 0.0;

    std::cout << "Starting baseline Cartesian impedance controller." << std::endl;

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {

      time += period.toSec();

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());

      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

      Eigen::Matrix<double, 6, 1> xdot = J * dq;
      Eigen::Vector3d pdot = xdot.head<3>();
      Eigen::Vector3d omega = xdot.tail<3>();

      Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_EE(state.O_T_EE.data());

      Eigen::Vector3d p_EE = T_EE.block<3, 1>(0, 3);
      Eigen::Matrix3d R_EE = T_EE.block<3, 3>(0, 0);

      Eigen::Vector3d e_p = p_d - p_EE;
      Eigen::Vector3d e_R = orientationError(R_EE, R_d);

      // Baseline static Cartesian impedance.
      // Desired translational and rotational velocities are zero in holding mode.
      Eigen::Vector3d f = Kp * e_p - Dp * pdot;
      Eigen::Vector3d m = KR * e_R - DR * omega;

      Eigen::Matrix<double, 6, 1> wrench;
      wrench.head<3>() = f;
      wrench.tail<3>() = m;

      Eigen::Matrix<double, 7, 1> tau_task = J.transpose() * wrench;

      std::array<double, 7> coriolis_array = model.coriolis(state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

      Eigen::Matrix<double, 7, 1> tau_cmd = tau_task + coriolis;

      LogData row;
      row.time = time;
      row.p_EE = p_EE;
      row.p_d = p_d;
      row.e_p = e_p;
      row.e_R = e_R;
      row.pdot = pdot;
      row.omega = omega;
      row.f = f;
      row.m = m;
      row.tau_cmd = tau_cmd;

      log_data.push_back(row);

      final_p_EE = p_EE;
      final_e_p = e_p;
      final_e_R = e_R;
      final_f = f;
      final_m = m;
      final_tau = tau_cmd;

      max_position_error_norm = std::max(max_position_error_norm, e_p.norm());
      max_rotation_error_norm = std::max(max_rotation_error_norm, e_R.norm());

      std::array<double, 7> tau_array = eigenToArray(tau_cmd);

      if (time >= params.experiment_duration) {
        return franka::MotionFinished(franka::Torques(tau_array));
      }

      return franka::Torques(tau_array);
    });

    writeLogToCsv(log_data, params.csv_file_name);

    std::cout << "\nExperiment finished." << std::endl;
    std::cout << "Final desired position p_d [m]:  " << p_d.transpose() << std::endl;
    std::cout << "Final reached position p_EE [m]: " << final_p_EE.transpose() << std::endl;
    std::cout << "Final position error e_p [m]:    " << final_e_p.transpose() << std::endl;
    std::cout << "Final position error norm [m]:   " << final_e_p.norm() << std::endl;
    std::cout << "Max position error norm [m]:     " << max_position_error_norm << std::endl;
    std::cout << "Final rotation error e_R [rad]:  " << final_e_R.transpose() << std::endl;
    std::cout << "Final rotation error norm [rad]: " << final_e_R.norm() << std::endl;
    std::cout << "Max rotation error norm [rad]:   " << max_rotation_error_norm << std::endl;
    std::cout << "Final force command f [N]:       " << final_f.transpose() << std::endl;
    std::cout << "Final moment command m [Nm]:     " << final_m.transpose() << std::endl;
    std::cout << "Final torque command tau [Nm]:   " << final_tau.transpose() << std::endl;
    std::cout << "CSV log written to: " << params.csv_file_name << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    std::cerr << "Baseline controller has no force/moment saturation, torque-rate limiting, or friction compensation." << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
