#pragma once

#include "controller_types.h"

inline LogData makeLogRow(
    double time,
    const Vec3& p_EE,
    const Vec3& p_d,
    const Vec3& p_end,
    const Vec3& e_p,
    const Vec3& e_R,
    const Vec3& pdot,
    const Vec3& pdot_d,
    const Vec3& omega,
    const Vec3& f,
    const Vec3& m,
    const Vec7& tau_cmd) {
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
  row.tau_cmd = tau_cmd;
  return row;
}

inline void writeLogToCsv(
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
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  for (const auto& row : log_data) {
    log_file << std::fixed << std::setprecision(6)
             << row.time << ","
             << row.p_EE(0) << "," << row.p_EE(1) << "," << row.p_EE(2) << ","
             << row.p_d(0) << "," << row.p_d(1) << "," << row.p_d(2) << ","
             << row.p_end(0) << "," << row.p_end(1) << "," << row.p_end(2) << ","
             << row.e_p(0) << "," << row.e_p(1) << "," << row.e_p(2) << ","
             << row.e_R(0) << "," << row.e_R(1) << "," << row.e_R(2) << ","
             << row.pdot(0) << "," << row.pdot(1) << "," << row.pdot(2) << ","
             << row.pdot_d(0) << "," << row.pdot_d(1) << "," << row.pdot_d(2) << ","
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
