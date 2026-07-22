#include "controller_logging.h"

namespace {

// Writes "x,y,z," for one vector, so the row below reads as a list of columns
// instead of 60 hand-numbered stream inserts.
void writeVec3(std::ofstream& out, const Vec3& v) {
  out << v(0) << "," << v(1) << "," << v(2) << ",";
}

}  // namespace

void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name) {
  std::ofstream log_file(csv_file_name);

  log_file << "time,phase,"
           << "p_EE_x,p_EE_y,p_EE_z,"
           << "p_d_x,p_d_y,p_d_z,"
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
           << "push,"
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  log_file << std::fixed << std::setprecision(6);
  for (const auto& row : log_data) {
    log_file << row.time << "," << row.phase << ",";
    writeVec3(log_file, row.p_EE);
    writeVec3(log_file, row.p_d);
    writeVec3(log_file, row.tool_contact_point);
    writeVec3(log_file, row.first_contact_tcp);
    writeVec3(log_file, row.first_contact_point);
    writeVec3(log_file, row.edge_target);
    writeVec3(log_file, row.tool_contact_offset_ee);
    writeVec3(log_file, row.e_p);
    writeVec3(log_file, row.e_R);
    writeVec3(log_file, row.pdot);
    writeVec3(log_file, row.pdot_d);
    writeVec3(log_file, row.omega);
    writeVec3(log_file, row.f);
    writeVec3(log_file, row.m);
    writeVec3(log_file, row.external_force);
    writeVec3(log_file, row.external_moment);
    writeVec3(log_file, row.contact_force_bias);
    writeVec3(log_file, row.contact_moment_bias);
    writeVec3(log_file, Vec3(row.external_force - row.contact_force_bias));
    writeVec3(log_file, Vec3(row.external_moment - row.contact_moment_bias));
    log_file << row.push << ","
             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
}
