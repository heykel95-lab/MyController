#pragma once

#include "controller_types.h"

const char* phaseName(ControlPhase phase);

const char* nullspaceModeName(NullspaceMode mode);

// ---- value formatting ----

void printVec3Mm(const char* label, const Vec3& v);

void printVec3Deg(const char* label, const Vec3& v);

void printGainVec(const char* label, const Vec3& v);

void printVec7Deg(const char* label, const Vec7& v);

// Prints a 6x6 spatial gain as ONE labeled grid. Rows are the wrench it
// produces (fx..fz force, mx..mz moment); columns are the displacement it acts
// on (tx..tz translation, rx..rz rotation). The dividers mark the four
// quadrants, so the off-diagonal force<->rotation coupling can be read
// directly: a spring that is decoupled at the pole has zeros there, the same
// spring moved out to the TCP has them filled.
void printSpatialGain6(const char* label, const Mat6x6& M);

// The six eigenvalues (ascending) plus a PSD verdict. A valid passive spring
// must be positive semi-definite; negative off-diagonal *entries* are fine,
// only the eigenvalues matter. The adjoint congruence preserves this from the
// decoupled pole gains, so this confirms the coupled K_TCP/D_TCP are still a
// valid spring numerically.
void printSpatialGainEigenvalues(const char* label, const Mat6x6& M);

void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final);

void printParameters(const Parameters& params);

// At the moment contact is detected, shows which tool edge was used and what it
// cost in base-frame mm, so the auto-selected side and the tilt-induced
// depth/lateral split can be checked against the physical setup.
void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point);

// ---- One short debug line per phase, all in the same "t=... | name=value"
//      style so live output is easy to scan whatever phase is running. ----

void printApproachOrientDebug(double phase_time,
                              double axis_error_deg,
                              double rot_error_deg);

void printApproachDescendDebug(double phase_time,
                               double distance_mm,
                               double height_mm,
                               double target_height_mm,
                               double force_n);

void printSetUpDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double edge_mm);

void printGrindDebug(double phase_time,
                     double sweep_mm,
                     double track_error_mm,
                     double press_n);

void printHoldDebug(double phase_time,
                    double force_n,
                    double pos_error_mm,
                    double rot_error_deg);

void printFinalSummary(const Vec3& final_p_d,
                       const Vec3& final_p_EE,
                       const Vec3& final_e_p,
                       const Vec3& final_e_R,
                       const std::string& csv_file_name);
