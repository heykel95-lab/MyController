#pragma once

#include "controller_types.h"

// Everything the one-shot set-up report needs, captured by the control loop at
// the moment the set-up phase ends.
struct SetUpReport {
  // How the phase ended.
  bool stopped_on_moment = false;  // false = hit the duration limit
  double phase_time = 0.0;
  double force_delta_norm = 0.0;
  double moment_delta_norm = 0.0;

  // Pose and contact wrench at the end of the phase.
  Vec3 p_EE = Vec3::Zero();
  Mat3 R_EE = Mat3::Identity();
  Vec3 tool_contact_point = Vec3::Zero();
  Vec3 external_force = Vec3::Zero();
  Vec3 contact_moment_at_edge = Vec3::Zero();

  // Reference captured when the phase started (first contact).
  Vec3 first_contact_tcp = Vec3::Zero();
  Vec3 first_contact_point = Vec3::Zero();
  Mat3 R_contact_start = Mat3::Identity();
  Vec3 contact_force_bias = Vec3::Zero();

  // The diagonal set-up spring that was active, already in base coordinates.
  Mat3 Kp = Mat3::Zero();
  Mat3 Dp = Mat3::Zero();
  Mat3 KR = Mat3::Zero();
  Mat3 DR = Mat3::Zero();
};

// Prints the one-shot report for a finished set-up phase: what the contact
// wrench and motion were in the surface frame, the finite screw axis the whole
// tipping motion turned out to follow, and the coupled K_TCP/D_TCP that axis
// implies compared against the manually placed pole.
//
// When the finite axis is valid and the coupled stiffness is not being applied
// this run, the resulting matrices are queued into parameter_updates so the
// next run can command them.
void reportSetUpResult(const Parameters& params,
                       const Mat3& R_alignment_target,
                       const SetUpReport& report,
                       std::vector<std::pair<std::string, std::string>>* parameter_updates);
