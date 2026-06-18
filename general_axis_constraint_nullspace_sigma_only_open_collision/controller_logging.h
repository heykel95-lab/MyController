#pragma once

#include "controller_types.h"

inline LogData makeLogRow(
    double time,
    int phase,
    const Vec3& p_EE,
    const Vec3& p_d,
    const Vec3& p_end,
    const Vec3& tool_contact_point,
    const Vec3& first_contact_tcp,
    const Vec3& first_contact_point,
    const Vec3& edge_target,
    const Vec3& tool_contact_offset_ee,
    const Vec3& e_p,
    const Vec3& e_R,
    const Vec3& pdot,
    const Vec3& pdot_d,
    const Vec3& omega,
    const Vec3& f,
    const Vec3& m,
    const Vec3& external_force,
    const Vec3& external_moment,
    const Vec3& contact_force_bias,
    const Vec3& contact_moment_bias,
    double post_contact_push,
    const Vec7& tau_cmd) {
  LogData row;
  row.time = time;
  row.phase = phase;
  row.p_EE = p_EE;
  row.p_d = p_d;
  row.p_end = p_end;
  row.tool_contact_point = tool_contact_point;
  row.first_contact_tcp = first_contact_tcp;
  row.first_contact_point = first_contact_point;
  row.edge_target = edge_target;
  row.tool_contact_offset_ee = tool_contact_offset_ee;
  row.e_p = e_p;
  row.e_R = e_R;
  row.pdot = pdot;
  row.pdot_d = pdot_d;
  row.omega = omega;
  row.f = f;
  row.m = m;
  row.external_force = external_force;
  row.external_moment = external_moment;
  row.contact_force_bias = contact_force_bias;
  row.contact_moment_bias = contact_moment_bias;
  row.post_contact_push = post_contact_push;
  row.tau_cmd = tau_cmd;
  return row;
}

inline void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name) {
  std::ofstream log_file(csv_file_name);

  log_file << "time,phase,"
           << "p_EE_x,p_EE_y,p_EE_z,"
           << "p_d_x,p_d_y,p_d_z,"
           << "p_end_x,p_end_y,p_end_z,"
           << "tool_contact_x,tool_contact_y,tool_contact_z,"
           << "first_contact_tcp_x,first_contact_tcp_y,first_contact_tcp_z,"
           << "first_contact_x,first_contact_y,first_contact_z,"
           << "edge_target_x,edge_target_y,edge_target_z,"
           << "tool_contact_offset_ee_x,tool_contact_offset_ee_y,tool_contact_offset_ee_z,"
           << "e_p_x,e_p_y,e_p_z,"
           << "e_R_x,e_R_y,e_R_z,"
           << "pdot_x,pdot_y,pdot_z,"
           << "pdot_d_x,pdot_d_y,pdot_d_z,"
           << "omega_x,omega_y,omega_z,"
           << "f_x,f_y,f_z,"
           << "m_x,m_y,m_z,"
           << "external_force_x,external_force_y,external_force_z,"
           << "external_moment_x,external_moment_y,external_moment_z,"
           << "contact_force_bias_x,contact_force_bias_y,contact_force_bias_z,"
           << "contact_moment_bias_x,contact_moment_bias_y,contact_moment_bias_z,"
           << "force_after_contact_x,force_after_contact_y,force_after_contact_z,"
           << "moment_after_contact_x,moment_after_contact_y,moment_after_contact_z,"
           << "post_contact_push,"
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  for (const auto& row : log_data) {
    const Vec3 force_after_contact = row.external_force - row.contact_force_bias;
    const Vec3 moment_after_contact = row.external_moment - row.contact_moment_bias;
    log_file << std::fixed << std::setprecision(6)
             << row.time << "," << row.phase << ","
             << row.p_EE(0) << "," << row.p_EE(1) << "," << row.p_EE(2) << ","
             << row.p_d(0) << "," << row.p_d(1) << "," << row.p_d(2) << ","
             << row.p_end(0) << "," << row.p_end(1) << "," << row.p_end(2) << ","
             << row.tool_contact_point(0) << "," << row.tool_contact_point(1) << ","
             << row.tool_contact_point(2) << ","
             << row.first_contact_tcp(0) << "," << row.first_contact_tcp(1) << ","
             << row.first_contact_tcp(2) << ","
             << row.first_contact_point(0) << "," << row.first_contact_point(1) << ","
             << row.first_contact_point(2) << ","
             << row.edge_target(0) << "," << row.edge_target(1) << ","
             << row.edge_target(2) << ","
             << row.tool_contact_offset_ee(0) << "," << row.tool_contact_offset_ee(1) << ","
             << row.tool_contact_offset_ee(2) << ","
             << row.e_p(0) << "," << row.e_p(1) << "," << row.e_p(2) << ","
             << row.e_R(0) << "," << row.e_R(1) << "," << row.e_R(2) << ","
             << row.pdot(0) << "," << row.pdot(1) << "," << row.pdot(2) << ","
             << row.pdot_d(0) << "," << row.pdot_d(1) << "," << row.pdot_d(2) << ","
             << row.omega(0) << "," << row.omega(1) << "," << row.omega(2) << ","
             << row.f(0) << "," << row.f(1) << "," << row.f(2) << ","
             << row.m(0) << "," << row.m(1) << "," << row.m(2) << ","
             << row.external_force(0) << "," << row.external_force(1) << ","
             << row.external_force(2) << ","
             << row.external_moment(0) << "," << row.external_moment(1) << ","
             << row.external_moment(2) << ","
             << row.contact_force_bias(0) << "," << row.contact_force_bias(1) << ","
             << row.contact_force_bias(2) << ","
             << row.contact_moment_bias(0) << "," << row.contact_moment_bias(1) << ","
             << row.contact_moment_bias(2) << ","
             << force_after_contact(0) << "," << force_after_contact(1) << ","
             << force_after_contact(2) << ","
             << moment_after_contact(0) << "," << moment_after_contact(1) << ","
             << moment_after_contact(2) << ","
             << row.post_contact_push << ","
             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
}
