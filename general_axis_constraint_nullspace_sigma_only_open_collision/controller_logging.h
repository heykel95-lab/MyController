#pragma once

#include "controller_types.h"

LogData makeLogRow(
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
    const Vec7& tau_cmd);

void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name);
