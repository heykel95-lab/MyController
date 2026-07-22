#include "controller_printing.h"

const char* phaseName(ControlPhase phase) {
  switch (phase) {
    case ControlPhase::kApproachOrient:
      return "approach_orient";
    case ControlPhase::kApproachDescend:
      return "approach_descend";
    case ControlPhase::kSetUp:
      return "set_up";
    case ControlPhase::kGrind:
      return "grind";
    case ControlPhase::kHold:
      return "hold";
    case ControlPhase::kManualGuide:
      return "manual_guide";
  }
  return "unknown";
}

const char* nullspaceModeName(NullspaceMode mode) {
  switch (mode) {
    case NullspaceMode::kOff:
      return "off";
    case NullspaceMode::kPostureOnly:
      return "tau_nullspace_only";
    case NullspaceMode::kSigmaOnly:
      return "tau_sigma_only";
    case NullspaceMode::kPostureAndSigma:
      return "tau_nullspace_plus_tau_sigma";
  }
  return "unknown";
}

// ====================================================================
// Value formatting
// ====================================================================

void printVec3Mm(const char* label, const Vec3& v) {
  printf("%s = [%.1f, %.1f, %.1f] mm\n",
         label, 1000.0 * v(0), 1000.0 * v(1), 1000.0 * v(2));
}

void printVec3Deg(const char* label, const Vec3& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.2f, %.2f, %.2f] deg\n",
         label, rad_to_deg * v(0), rad_to_deg * v(1), rad_to_deg * v(2));
}

void printGainVec(const char* label, const Vec3& v) {
  printf("%s = [%.4g, %.4g, %.4g]\n", label, v(0), v(1), v(2));
}

void printVec7Deg(const char* label, const Vec7& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg\n",
         label,
         rad_to_deg * v(0), rad_to_deg * v(1), rad_to_deg * v(2),
         rad_to_deg * v(3), rad_to_deg * v(4), rad_to_deg * v(5),
         rad_to_deg * v(6));
}

void printSpatialGain6(const char* label, const Mat6x6& M) {
  static const char* kRowNames[6] = {"fx", "fy", "fz", "mx", "my", "mz"};
  printf("%s:\n", label);
  printf("           tx        ty        tz    |    rx        ry        rz\n");
  for (int i = 0; i < 6; ++i) {
    printf("  %s [%8.4g %8.4g %8.4g | %8.4g %8.4g %8.4g ]\n",
           kRowNames[i],
           M(i, 0), M(i, 1), M(i, 2), M(i, 3), M(i, 4), M(i, 5));
    if (i == 2) {
      printf("     ---------------------------+---------------------------\n");
    }
  }
}

void printSpatialGainEigenvalues(const char* label, const Mat6x6& M) {
  const Mat6x6 symmetric = 0.5 * (M + M.transpose());
  Eigen::SelfAdjointEigenSolver<Mat6x6> solver(symmetric);
  const Eigen::Matrix<double, 6, 1> eig = solver.eigenvalues();
  const double tol = 1e-6 * std::max(1.0, eig.cwiseAbs().maxCoeff());
  const bool psd = eig.minCoeff() >= -tol;
  printf("%s eigenvalues = [%.4g, %.4g, %.4g, %.4g, %.4g, %.4g] -> %s (min=%.4g)\n",
         label, eig(0), eig(1), eig(2), eig(3), eig(4), eig(5),
         psd ? "PSD ok (>=0)" : "NOT PSD (<0!)", eig(0));
}

void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("\nJoint motion [deg]:\n");
  printf("joint    start   final   delta\n");
  printf("------------------------------\n");
  for (int i = 0; i < 7; ++i) {
    printf("q%-2d   %7.1f %7.1f %7.1f\n",
           i + 1,
           rad_to_deg * q_start(i),
           rad_to_deg * q_final(i),
           rad_to_deg * (q_final(i) - q_start(i)));
  }
}

void printParameters(const Parameters& params) {
  printf("\n=== Setup ===\n");
  printf("run: %s | approach_orient: %s | nullspace: %s\n",
         params.use_phase_sequence ? "phase sequence" : "hold",
         params.use_approach_orient ? "on" : "off",
         nullspaceModeName(params.nullspace_mode));
  printf("hold: Kp=%.1f N/m | damping=%s",
         params.hold_Kp,
         params.hold_auto_damping ? "auto-critical" : "manual");
  if (params.hold_auto_damping) {
    printf(" (factor %.2f)\n", params.hold_auto_damping_factor);
  } else {
    printf(" (Dp=%.1f Ns/m)\n", params.hold_Dp);
  }
  printf("manual_guidance_start: %s | manual_damping=%.2f\n",
         params.use_manual_guidance_start ? "on" : "off",
         params.manual_guidance_damping);
  printf("descend: clearance=%.1f mm | speed=%.3f m/s | max_distance=%.0f mm\n",
         1000.0 * params.descend_surface_clearance,
         params.descend_speed,
         1000.0 * params.descend_max_distance);
  printf("set_up: duration=%.1f s | moment_threshold=%.1f Nm | max_push=%.0f mm\n",
         params.setup_duration,
         params.setup_moment_threshold,
         1000.0 * params.setup_max_push);
  printf("grind: %s | axis=tangent%d | amplitude=%.0f mm | frequency=%.2f Hz\n",
         params.grind_sweep_enabled ? "sweep" : "free-slide hold",
         params.grind_axis == 2 ? 2 : 1,
         1000.0 * params.grind_amplitude_m,
         params.grind_frequency_hz);
  printf("coupled stiffness: apply=%s | eval=%s | saved=%s | pole=%s\n",
         params.use_coupled_stiffness ? "on" : "off",
         params.eval_coupled_stiffness ? "on" : "off",
         params.coupled_gains_saved ? "yes" : "no",
         params.coupled_use_block_diagonal
             ? "block-diagonal (no coupling)"
             : (params.coupled_pole_manual ? "manual" : "saved matrices"));
  printf("gates: pause_before_set_up=%s | pause_before_grind=%s | debug_period=%.2f s\n",
         params.pause_before_set_up ? "on" : "off",
         params.pause_before_grind ? "on" : "off",
         params.debug_period);
  printf("alignment-target normal=[%+.3f, %+.3f, %+.3f] | tilt a(x)=%.1f deg, b(y)=%.1f deg%s\n",
         params.alignment_target_normal(0),
         params.alignment_target_normal(1),
         params.alignment_target_normal(2),
         params.alignment_target_tilt_angle_deg,
         params.alignment_target_tilt_angle_y_deg,
         params.derive_tilt_angles_from_plane_normal
             ? " (derived from plane normal)"
             : (params.use_alignment_target_tilt_angle ? " (from angles)"
                                                       : " (manual normal)"));
}

void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point) {
  printVec3Mm("offset_ee", offset_ee);
  printVec3Mm("p_EE_at_contact", p_EE_at_contact);
  printVec3Mm("contact_point", contact_point);
  printVec3Mm("edge_offset_base", contact_point - p_EE_at_contact);
}

// ====================================================================
// Per-phase debug lines
// ====================================================================

void printApproachOrientDebug(double phase_time,
                              double axis_error_deg,
                              double rot_error_deg) {
  printf("orient:     t=%5.1f s | axis_err=%5.1f deg | rot_err=%5.1f deg\n",
         phase_time, axis_error_deg, rot_error_deg);
}

void printApproachDescendDebug(double phase_time,
                               double distance_mm,
                               double height_mm,
                               double target_height_mm,
                               double force_n) {
  printf("descend:    t=%5.1f s | distance=%6.1f mm | height=%+6.1f mm (target %.1f) | force=%5.1f N\n",
         phase_time, distance_mm, height_mm, target_height_mm, force_n);
}

void printSetUpDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double edge_mm) {
  printf("set_up:     t=%5.1f s | tip=%5.1f deg | F=%5.1f N | M=%5.1f Nm (limit %.1f) | edge=%5.1f mm\n",
         phase_time, tip_deg, force_n, moment_nm, moment_limit_nm, edge_mm);
}

void printGrindDebug(double phase_time,
                     double sweep_mm,
                     double track_error_mm,
                     double press_n) {
  printf("grind:      t=%5.1f s | sweep=%+6.1f mm | track_err=%+5.1f mm | press=%5.1f N\n",
         phase_time, sweep_mm, track_error_mm, press_n);
}

void printHoldDebug(double phase_time,
                    double force_n,
                    double pos_error_mm,
                    double rot_error_deg) {
  printf("hold:       t=%5.1f s | force=%5.1f N | pos_err=%5.1f mm | rot_err=%5.1f deg\n",
         phase_time, force_n, pos_error_mm, rot_error_deg);
}

void printFinalSummary(const Vec3& final_p_d,
                       const Vec3& final_p_EE,
                       const Vec3& final_e_p,
                       const Vec3& final_e_R,
                       const std::string& csv_file_name) {
  printf("\n=== Final result ===\n");
  printVec3Mm("p_d", final_p_d);
  printVec3Mm("p_EE", final_p_EE);
  printVec3Mm("e_p", final_e_p);
  printf("position_error = %.2f mm\n", 1000.0 * final_e_p.norm());
  printVec3Deg("e_R", final_e_R);
  printf("rotation_error = %.2f deg\n", (180.0 / M_PI) * final_e_R.norm());
  printf("csv: %s\n", csv_file_name.c_str());
}
