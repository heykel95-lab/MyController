#pragma once

#include "controller_types.h"

void printVec3(const char* label, const Vec3& v);

void printVec3Mm(const char* label, const Vec3& v);

void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point);

void printVec3Deg(const char* label, const Vec3& v);

void printGainVec(const char* label, const Vec3& v);

void printMat3Rows(const char* label, const Mat3& m);

void printMat6Grid(const char* label, const Mat6x6& M);

void printSpatialGainEigenvalues(const char* label, const Mat6x6& M);

void printSpatialGain6(const char* label, const Mat6x6& M);

void printVec7(const char* label, const Vec7& v);

void printVec7Deg(const char* label, const Vec7& v);

std::string formatSuggestedGains(const Vec3& suggested,
                                 const Vec3& configured,
                                 double gain_threshold,
                                 const char* value_format,
                                 const bool fit_valid[3]);

void printMat3(const char* label, const Mat3& m);

void printMat4x4(const char* label, const Mat4x4& m);

void printJointStartEndTable(const Vec7& q_start, const Vec7& q_final);

void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final);

void printParameters(const Parameters& params);

void printOrientDebug(double phase_time,
                      double axis_error_deg,
                      double rot_error_deg);

void printSearchDebug(double phase_time,
                      double distance_mm,
                      double force_n,
                      double force_limit_n,
                      bool touch_saved,
                      bool show_to_plane = false,
                      double to_plane_mm = 0.0);

void printAlignDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double edge_mm,
                     bool pole_valid,
                     const Vec3& pole_nearest_edge_mm,
                     double pole_dist_mm);

void printImpedanceDebug(double phase_time,
                         double force_n,
                         double pos_error_mm,
                         double rot_error_deg);

void printFinalSummary(
    const Vec3& final_p_d,
    const Vec3& final_p_EE,
    const Vec3& final_e_p,
    const Vec3& final_e_R,
    const Vec3& final_instant_pole_to_edge,
    const Vec3& final_instant_axis_dir,
    const Vec3& last_best_axis_from_edge,
    const Vec3& last_best_axis_dir,
    double last_best_axis_pitch,
    double final_instant_screw_pitch,
    double final_instant_edge_axis_distance,
    double final_instant_axis_time,
    bool final_instant_pole_valid,
    const std::string& csv_file_name);
