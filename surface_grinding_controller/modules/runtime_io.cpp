// ====================================================================
// Runtime input and output
// ====================================================================
// Everything the operator sees or types: CSV logging, the setup and per-phase
// debug printing, the startup menu, gripper actions and hand-guidance. No
// control law lives here.
#include "controller.h"

// ====================================================================
// CSV logging
// ====================================================================


namespace {

// Writes "x,y,z," for one vector, so the row below reads as a list of columns
// instead of 60 hand-numbered stream inserts.
void writeVec3(std::ofstream& out, const Vec3& v) {
  out << v(0) << "," << v(1) << "," << v(2) << ",";
}

void writeVec7(std::ofstream& out, const Vec7& v) {
  for (int i = 0; i < 7; ++i) {
    out << v(i) << ",";
  }
}

void writeVec3Scaled(std::ofstream& out, const Vec3& v, double scale) {
  out << scale * v(0) << "," << scale * v(1) << ","
      << scale * v(2) << ",";
}

void writeVec7Scaled(std::ofstream& out, const Vec7& v, double scale) {
  for (int i = 0; i < 7; ++i) {
    out << scale * v(i) << ",";
  }
}

const char* sigmaDebugEventName(SigmaDebugEvent event) {
  switch (event) {
    case SigmaDebugEvent::kSample:
      return "sample";
    case SigmaDebugEvent::kHoldStart:
      return "hold_start";
    case SigmaDebugEvent::kManualGuideStart:
      return "manual_guide_start";
    case SigmaDebugEvent::kRecapture:
      return "recapture";
    case SigmaDebugEvent::kStop:
      return "stop";
    case SigmaDebugEvent::kException:
      return "exception";
  }
  return "unknown";
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
           << "alignment_error_t1_deg,alignment_error_t2_deg,alignment_error_normal_deg,"
           << "alignment_angle_deg,"
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
           << "nullspace_mode,"
           << "sigma_samples_valid,sigma_push_active,"
           << "sigma_current,sigma_plus,sigma_minus,sigma_difference,"
           << "sigma_direction,sigma_alpha,sigma_k_sigma,sigma_deadband,"
           << "tau_sigma_norm,nullspace_speed,sigma_speed_toward_better,"
           << "sigma_direction_valid,"
           << "sigma_n_best_1,sigma_n_best_2,sigma_n_best_3,sigma_n_best_4,"
           << "sigma_n_best_5,sigma_n_best_6,sigma_n_best_7,"
           << "nullspace_dq_1,nullspace_dq_2,nullspace_dq_3,nullspace_dq_4,"
           << "nullspace_dq_5,nullspace_dq_6,nullspace_dq_7,"
           << "tau_sigma_1,tau_sigma_2,tau_sigma_3,tau_sigma_4,"
           << "tau_sigma_5,tau_sigma_6,tau_sigma_7,"
           << "sigma_dominant_joint,sigma_dominant_fraction,"
           << "nullspace_velocity_dominant_joint,"
           << "nullspace_velocity_dominant_fraction,sigma_Jn_norm,"
           << "tau_nullspace_norm,"
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  log_file << std::fixed << std::setprecision(9);
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
    writeVec3(log_file, (180.0 / M_PI) * row.alignment_error_surface);
    log_file << (180.0 / M_PI) * row.alignment_angle << ",";
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
             << row.nullspace_mode << ","
             << static_cast<int>(row.sigma.samples_valid) << ","
             << static_cast<int>(row.sigma.push_active) << ","
             << row.sigma.sigma_current << ","
             << row.sigma.sigma_plus << ","
             << row.sigma.sigma_minus << ","
             << row.sigma.sigma_difference << ","
             << row.sigma.direction_sign << ","
             << row.sigma.alpha << ","
             << row.sigma.k_sigma << ","
             << row.sigma.deadband << ","
             << row.sigma.tau_sigma_norm << ","
             << row.sigma.nullspace_speed << ","
             << row.sigma.speed_toward_better << ","
             << static_cast<int>(row.sigma.direction_valid) << ",";
    writeVec7(log_file, row.sigma.best_direction);
    writeVec7(log_file, row.sigma.nullspace_velocity);
    writeVec7(log_file, row.sigma.tau_sigma);
    log_file << row.sigma.dominant_direction_joint << ","
             << row.sigma.dominant_direction_fraction << ","
             << row.sigma.dominant_velocity_joint << ","
             << row.sigma.dominant_velocity_fraction << ","
             << row.sigma.jacobian_null_residual << ","
             << row.tau_nullspace_norm << ","
             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
}

bool writeSigmaDebugToCsv(
    const std::vector<SigmaDebugRow>& debug_data,
    const std::string& csv_file_name) {
  std::ofstream out(csv_file_name);
  if (!out) {
    fprintf(stderr, "Could not open sigma debug CSV: %s\n",
            csv_file_name.c_str());
    return false;
  }

  out << "run_time_s,segment_id,phase_time_s,event,"
      << "sigma_min,sigma_rate_per_s,sigma_plus,sigma_minus,"
      << "probe_difference,probe_confidence,gradient_abs_per_rad,"
      << "direction_valid,push_active,k_sigma_Nm,alpha_rad,"
      << "tau_sigma_norm_Nm,sigma_power_W,"
      << "nullspace_speed_rad_s,speed_toward_better_rad_s,"
      << "peak_nullspace_speed_rad_s,"
      << "peak_abs_speed_toward_better_rad_s,"
      << "min_speed_toward_better_rad_s,"
      << "max_speed_toward_better_rad_s,"
      << "raw_joint_speed_rad_s,nullspace_speed_fraction,"
      << "raw_velocity_dominant_joint,raw_velocity_dominant_share_pct,"
      << "sigma_dominant_joint,sigma_dominant_share_pct,"
      << "position_error_mm,rotation_error_deg,"
      << "peak_position_error_mm,peak_rotation_error_deg,"
      << "cartesian_speed_mm_s,angular_speed_deg_s,"
      << "command_force_norm_N,command_moment_norm_Nm,"
      << "tau_task_norm_Nm,tau_nullspace_norm_Nm,tau_cmd_norm_Nm,"
      << "external_force_delta_norm_N,"
      << "external_moment_delta_norm_Nm,"
      << "external_joint_torque_delta_norm_Nm,"
      << "external_joint_torque_along_nbest_Nm,"
      << "peak_external_force_delta_norm_N,"
      << "peak_external_moment_delta_norm_Nm,"
      << "peak_external_joint_torque_delta_norm_Nm,"
      << "external_joint_torque_baseline_valid,"
      << "joint_contact,cartesian_contact,Jn_norm,"
      << "e_p_x_mm,e_p_y_mm,e_p_z_mm,"
      << "e_R_x_deg,e_R_y_deg,e_R_z_deg,"
      << "q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,q7_deg,"
      << "dq1_rad_s,dq2_rad_s,dq3_rad_s,dq4_rad_s,"
      << "dq5_rad_s,dq6_rad_s,dq7_rad_s,"
      << "n_best_1,n_best_2,n_best_3,n_best_4,"
      << "n_best_5,n_best_6,n_best_7,"
      << "dqN_1_rad_s,dqN_2_rad_s,dqN_3_rad_s,dqN_4_rad_s,"
      << "dqN_5_rad_s,dqN_6_rad_s,dqN_7_rad_s,"
      << "tau_sigma_1_Nm,tau_sigma_2_Nm,tau_sigma_3_Nm,"
      << "tau_sigma_4_Nm,tau_sigma_5_Nm,tau_sigma_6_Nm,"
      << "tau_sigma_7_Nm,"
      << "tau_ext_delta_1_Nm,tau_ext_delta_2_Nm,"
      << "tau_ext_delta_3_Nm,tau_ext_delta_4_Nm,"
      << "tau_ext_delta_5_Nm,tau_ext_delta_6_Nm,"
      << "tau_ext_delta_7_Nm\n";

  out << std::fixed << std::setprecision(9);
  constexpr double kRadToDeg = 180.0 / M_PI;

  int previous_sample_segment = -1;
  double previous_sample_time = 0.0;
  double previous_sigma = 0.0;
  bool previous_sample_valid = false;

  for (const auto& row : debug_data) {
    const bool is_sample = row.event == SigmaDebugEvent::kSample;
    const bool rate_valid =
        is_sample && previous_sample_valid &&
        row.segment_id == previous_sample_segment &&
        row.phase_time > previous_sample_time;
    const double sigma_rate =
        rate_valid
            ? (row.sigma.sigma_current - previous_sigma) /
                  (row.phase_time - previous_sample_time)
            : 0.0;

    const double probe_difference = std::abs(row.sigma.sigma_difference);
    const double probe_confidence =
        row.sigma.deadband > 0.0
            ? probe_difference / row.sigma.deadband
            : 0.0;
    const double gradient_abs =
        row.sigma.alpha > 0.0
            ? probe_difference / (2.0 * row.sigma.alpha)
            : 0.0;
    const double raw_joint_speed = row.dq.norm();
    const double nullspace_fraction =
        raw_joint_speed > 1e-12
            ? row.sigma.nullspace_speed / raw_joint_speed
            : 0.0;

    int raw_dominant_joint = 0;
    double raw_dominant_share = 0.0;
    if (raw_joint_speed > 1e-12) {
      Eigen::Index raw_dominant_index = 0;
      row.dq.cwiseAbs().maxCoeff(&raw_dominant_index);
      raw_dominant_joint = static_cast<int>(raw_dominant_index) + 1;
      const double component = row.dq(raw_dominant_index);
      raw_dominant_share =
          100.0 * component * component / row.dq.squaredNorm();
    }

    const double external_torque_along_nbest =
        row.sigma.direction_valid
            ? row.sigma.best_direction.dot(
                  row.external_joint_torque_delta)
            : 0.0;

    out << row.run_time << ","
        << row.segment_id << ","
        << row.phase_time << ","
        << sigmaDebugEventName(row.event) << ","
        << row.sigma.sigma_current << ",";
    if (rate_valid) {
      out << sigma_rate;
    }
    out << ","
        << row.sigma.sigma_plus << ","
        << row.sigma.sigma_minus << ","
        << probe_difference << ","
        << probe_confidence << ","
        << gradient_abs << ","
        << static_cast<int>(row.sigma.direction_valid) << ","
        << static_cast<int>(row.sigma.push_active) << ","
        << row.sigma.k_sigma << ","
        << row.sigma.alpha << ","
        << row.sigma.tau_sigma_norm << ","
        << row.sigma.tau_sigma.dot(row.dq) << ","
        << row.sigma.nullspace_speed << ","
        << row.sigma.speed_toward_better << ","
        << row.peak_nullspace_speed << ","
        << row.peak_abs_speed_toward_better << ","
        << row.min_speed_toward_better << ","
        << row.max_speed_toward_better << ","
        << raw_joint_speed << ","
        << nullspace_fraction << ","
        << raw_dominant_joint << ","
        << raw_dominant_share << ","
        << row.sigma.dominant_direction_joint << ","
        << 100.0 * row.sigma.dominant_direction_fraction << ","
        << 1000.0 * row.e_p.norm() << ","
        << kRadToDeg * row.e_R.norm() << ","
        << 1000.0 * row.peak_position_error << ","
        << kRadToDeg * row.peak_rotation_error << ","
        << 1000.0 * row.pdot.norm() << ","
        << kRadToDeg * row.omega.norm() << ","
        << row.command_force.norm() << ","
        << row.command_moment.norm() << ","
        << row.tau_task_norm << ","
        << row.tau_nullspace_norm << ","
        << row.tau_cmd_norm << ","
        << row.external_force_delta.norm() << ","
        << row.external_moment_delta.norm() << ","
        << row.external_joint_torque_delta.norm() << ","
        << external_torque_along_nbest << ","
        << row.peak_external_force_delta << ","
        << row.peak_external_moment_delta << ","
        << row.peak_external_joint_torque_delta << ","
        << static_cast<int>(
               row.external_joint_torque_baseline_valid) << ","
        << static_cast<int>(row.joint_contact) << ","
        << static_cast<int>(row.cartesian_contact) << ","
        << row.sigma.jacobian_null_residual << ",";
    writeVec3Scaled(out, row.e_p, 1000.0);
    writeVec3Scaled(out, row.e_R, kRadToDeg);
    writeVec7Scaled(out, row.q, kRadToDeg);
    writeVec7(out, row.dq);
    writeVec7(out, row.sigma.best_direction);
    writeVec7(out, row.sigma.nullspace_velocity);
    writeVec7(out, row.sigma.tau_sigma);
    for (int i = 0; i < 7; ++i) {
      out << row.external_joint_torque_delta(i);
      if (i < 6) {
        out << ",";
      }
    }
    out << "\n";

    if (is_sample) {
      previous_sample_segment = row.segment_id;
      previous_sample_time = row.phase_time;
      previous_sigma = row.sigma.sigma_current;
      previous_sample_valid = true;
    } else if (row.event == SigmaDebugEvent::kRecapture ||
               row.event == SigmaDebugEvent::kHoldStart) {
      previous_sample_valid = false;
    }
  }

  out.flush();
  if (!out) {
    fprintf(stderr, "Could not finish writing sigma debug CSV: %s\n",
            csv_file_name.c_str());
    return false;
  }
  return true;
}

// ====================================================================
// Terminal printing
// ====================================================================

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
    case NullspaceMode::kDampingOnly:
      return "nullspace_damping_only";
    case NullspaceMode::kSigmaOnly:
      return "sigma_optimization_only";
    case NullspaceMode::kDampingAndSigma:
      return "nullspace_damping_plus_sigma";
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
         params.use_phase_sequence ? "phase sequence"
         : params.hold_with_setup_gains ? "hold (set-up impedance)"
                                        : "hold",
         params.use_approach_orient ? "on" : "off",
         nullspaceModeName(params.nullspace_mode));
  const bool nullspace_damping_active =
      params.nullspace_mode == NullspaceMode::kDampingOnly ||
      params.nullspace_mode == NullspaceMode::kDampingAndSigma;
  const bool nullspace_sigma_active =
      params.nullspace_mode == NullspaceMode::kSigmaOnly ||
      params.nullspace_mode == NullspaceMode::kDampingAndSigma;
  printf("nullspace parameters: d_null=%.3f%s | k_sigma=%.3f Nm%s | "
         "alpha=%.4f rad\n",
         params.nullspace_mode == NullspaceMode::kDampingOnly
             ? params.nullspace_damping_mode1
             : params.nullspace_damping,
         nullspace_damping_active ? "" : " (inactive)",
         params.nullspace_k_sigma,
         nullspace_sigma_active ? "" : " (inactive)",
         params.nullspace_alpha);
  printf("hold [xyz]: Kp=[%.1f, %.1f, %.1f] N/m | "
         "KR=[%.1f, %.1f, %.1f] Nm/rad | damping=%s",
         params.hold_Kp_diag(0), params.hold_Kp_diag(1), params.hold_Kp_diag(2),
         params.hold_KR_diag(0), params.hold_KR_diag(1), params.hold_KR_diag(2),
         params.hold_auto_damping ? "auto translation+rotation" : "manual");
  if (params.hold_auto_damping) {
    if (params.hold_auto_match_manual_damping) {
      printf(" (factor fitted online toward Dp=[%.1f, %.1f, %.1f] Ns/m)\n",
             params.hold_Dp_diag(0), params.hold_Dp_diag(1),
             params.hold_Dp_diag(2));
    } else {
      printf(" (factor %.2f)\n", params.hold_auto_damping_factor);
    }
  } else {
    printf(" (Dp=[%.1f, %.1f, %.1f] Ns/m, DR=[%.1f, %.1f, %.1f] Nms/rad)\n",
           params.hold_Dp_diag(0), params.hold_Dp_diag(1),
           params.hold_Dp_diag(2), params.hold_DR_diag(0),
           params.hold_DR_diag(1), params.hold_DR_diag(2));
  }
  printf("manual_guidance_start: %s | manual_damping=%.2f\n",
         params.use_manual_guidance_start ? "on" : "off",
         params.manual_guidance_damping);
  printf("descend: clearance=%.1f mm | speed=%.3f m/s | max_distance=%.0f mm\n",
         1000.0 * params.descend_surface_clearance,
         params.descend_speed,
         1000.0 * params.descend_max_distance);
  const double nominal_setup_distance =
      std::abs(params.setup_push_end + params.descend_surface_clearance);
  const double nominal_setup_ramp_time =
      (std::abs(params.setup_push_speed) > 1e-12)
          ? nominal_setup_distance / std::abs(params.setup_push_speed)
          : 0.0;
  printf("alignment push: start=captured | end=%+.0f mm | speed=%.3f m/s | "
         "ramp_time~=%.1f s | timeout=%.1f s | moment_threshold=%.1f Nm\n",
         1000.0 * params.setup_push_end,
         std::abs(params.setup_push_speed),
         nominal_setup_ramp_time,
         params.setup_timeout,
         params.setup_moment_threshold);
  if (params.setup_timeout > 0.0 &&
      nominal_setup_ramp_time > params.setup_timeout) {
    printf("  note: alignment timeout occurs before the configured push end "
           "(unless the captured start is closer).\n");
  }
  printf("grind: %s | axis=tangent%d | amplitude=%.0f mm | frequency=%.2f Hz\n",
         params.grind_sweep_enabled ? "sweep" : "free-slide hold",
         params.grind_axis == 2 ? 2 : 1,
         1000.0 * params.grind_amplitude_m,
         params.grind_frequency_hz);
  printf("set-up translation: frame=%s | Kp=[%.1f, %.1f, %.1f] N/m\n",
         params.setup_translation_surface_frame ? "surface [t1,t2,n]" : "base [x,y,z]",
         params.setup_translation_surface_frame ? params.setup_Kp_surface_diag(0)
                                                : params.setup_Kp_diag(0),
         params.setup_translation_surface_frame ? params.setup_Kp_surface_diag(1)
                                                : params.setup_Kp_diag(1),
         params.setup_translation_surface_frame ? params.setup_Kp_surface_diag(2)
                                                : params.setup_Kp_diag(2));
  printf("coupled stiffness: apply=%s | pole=%s\n",
         params.use_coupled_stiffness ? "on" : "off",
         params.coupled_use_block_diagonal
             ? "block-diagonal (no coupling)"
             : (params.coupled_pole_manual ? "commanded" : "invalid"));
  if (params.coupled_pole_manual && params.coupled_use_direct_rc_surface) {
    printf("  r_c=p_TCP-p_c [t1,t2,n] = [%+.1f, %+.1f, %+.1f] mm\n",
           1000.0 * params.coupled_rc_surface(0),
           1000.0 * params.coupled_rc_surface(1),
           1000.0 * params.coupled_rc_surface(2));
  }
  printf("gates: pause_before_set_up=%s | pause_auto_damping=%s | "
         "pause_before_grind=%s | debug_period=%.2f s\n",
         params.pause_before_set_up ? "on" : "off",
         params.pause_hold_auto_damping ? "on" : "off",
         params.pause_before_grind ? "on" : "off",
         params.debug_period);
  printf("surface-plane normal=[%+.3f, %+.3f, %+.3f] | tilt a(x)=%.1f deg, b(y)=%.1f deg (from angles)\n",
         params.alignment_target_normal(0),
         params.alignment_target_normal(1),
         params.alignment_target_normal(2),
         params.alignment_target_tilt_angle_deg,
         params.alignment_target_tilt_angle_y_deg);
  printf("tool-axis command offset: tangent1=%+.1f deg | tangent2=%+.1f deg\n",
         params.tool_target_offset_tangent1_deg,
         params.tool_target_offset_tangent2_deg);
  printf("tool face: %.0f x %.0f mm | center offset EE=[%+.1f, %+.1f, %+.1f] mm\n",
         2000.0 * params.tool_contact_half_width_ee.norm(),
         2000.0 * params.tool_contact_half_length_ee.norm(),
         1000.0 * params.tool_contact_face_center_ee(0),
         1000.0 * params.tool_contact_face_center_ee(1),
         1000.0 * params.tool_contact_face_center_ee(2));
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

void printSigmaDebug(double phase_time,
                     const SigmaDiagnostics& sigma,
                     double sigma_rate,
                     bool sigma_rate_valid) {
  if (!sigma.samples_valid) {
    printf("sigma:      t=%5.1f s | samples unavailable | sigma_min=%.6f | "
           "alpha=%.4f rad\n",
           phase_time, sigma.sigma_current, sigma.alpha);
  } else {
    const double probe_difference = std::abs(sigma.sigma_difference);
    const double gradient =
        sigma.alpha > 0.0 ? probe_difference / (2.0 * sigma.alpha) : 0.0;
    char rate_text[32];
    char confidence_text[32];
    if (sigma_rate_valid) {
      snprintf(rate_text, sizeof(rate_text), "%+.2e/s", sigma_rate);
    } else {
      snprintf(rate_text, sizeof(rate_text), "n/a");
    }
    if (sigma.deadband > 0.0) {
      snprintf(confidence_text, sizeof(confidence_text), "%.1fx",
               probe_difference / sigma.deadband);
    } else {
      snprintf(confidence_text, sizeof(confidence_text), "n/a");
    }

    const char* torque_state =
        !sigma.direction_valid ? "deadband"
                               : sigma.push_active ? "push" : "zero-torque";
    printf("sigma:      t=%5.1f s | a=%.3f k=%.3f | min=%.6f d/dt=%s | "
           "probe=%.2e C=%s |grad|=%.2e/rad | %s tau=%.4f Nm | "
           "vN=%.4f vBest=%+.4f rad/s\n",
           phase_time,
           sigma.alpha,
           sigma.k_sigma,
           sigma.sigma_current,
           rate_text,
           probe_difference,
           confidence_text,
           gradient,
           torque_state,
           sigma.tau_sigma_norm,
           sigma.nullspace_speed,
           sigma.speed_toward_better);
  }

  if (sigma.direction_valid) {
    printf("sigma-joint: nBest=[%+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f] | "
           "dominant=q%d (share %.1f%%) | ||J*nBest||=%.2e\n",
           sigma.best_direction(0),
           sigma.best_direction(1),
           sigma.best_direction(2),
           sigma.best_direction(3),
           sigma.best_direction(4),
           sigma.best_direction(5),
           sigma.best_direction(6),
           sigma.dominant_direction_joint,
           100.0 * sigma.dominant_direction_fraction,
           sigma.jacobian_null_residual);
  } else if (sigma.samples_valid) {
    printf("sigma-joint: nBest not selected (probe is inside the deadband)\n");
  } else {
    printf("sigma-joint: nBest unavailable (sigma probe is invalid)\n");
  }

  printf("sigma-joint: dqN  =[%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad/s",
         sigma.nullspace_velocity(0),
         sigma.nullspace_velocity(1),
         sigma.nullspace_velocity(2),
         sigma.nullspace_velocity(3),
         sigma.nullspace_velocity(4),
         sigma.nullspace_velocity(5),
         sigma.nullspace_velocity(6));
  if (sigma.dominant_velocity_joint > 0) {
    printf(" | moving=q%d (share %.1f%%)",
           sigma.dominant_velocity_joint,
           100.0 * sigma.dominant_velocity_fraction);
  } else {
    printf(" | moving=none");
  }
  printf("\n");

  printf("sigma-joint: tauS =[%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] Nm\n",
         sigma.tau_sigma(0),
         sigma.tau_sigma(1),
         sigma.tau_sigma(2),
         sigma.tau_sigma(3),
         sigma.tau_sigma(4),
         sigma.tau_sigma(5),
         sigma.tau_sigma(6));
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

// ====================================================================
// Startup, gripper, and manual guidance
// ====================================================================

namespace {

std::string readChoice() {
  std::string line;
  if (!std::getline(std::cin, line)) {
    return "";
  }
  line.erase(line.begin(),
             std::find_if(line.begin(), line.end(), [](unsigned char c) {
               return !std::isspace(c);
             }));
  line.erase(std::find_if(line.rbegin(), line.rend(), [](unsigned char c) {
               return !std::isspace(c);
             }).base(),
             line.end());
  std::transform(line.begin(), line.end(), line.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return line;
}

bool matches(const std::string& choice, std::initializer_list<const char*> words) {
  for (const char* word : words) {
    if (choice == word) {
      return true;
    }
  }
  return false;
}

NullspaceMode askHoldNullspaceMode(NullspaceMode configured,
                                   bool* stop_selected = nullptr) {
  if (stop_selected != nullptr) {
    *stop_selected = false;
  }
  const NullspaceMode fallback = configured;
  printf("\nSelect hold nullspace mode:\n");
  printf("  0 = no nullspace torque\n");
  printf("  1 = nullspace damping only (no return to q_start)\n");
  printf("  2 = sigma optimization only (push toward larger sigma_min)\n");
  printf("  3 = sigma optimization + nullspace damping (damped comeback)\n");
  if (stop_selected != nullptr) {
    printf("  e = stop\n");
  }
  printf("Choice [0/1/2/3, Enter = %s]: ",
         fallback == NullspaceMode::kOff ? "0"
             : fallback == NullspaceMode::kSigmaOnly ? "2"
             : fallback == NullspaceMode::kDampingAndSigma ? "3" : "1");

  const std::string choice = readChoice();
  if (matches(choice, {"0", "off", "none"})) {
    return NullspaceMode::kOff;
  }
  if (matches(choice, {"1", "tau_nullspace", "nullspace", "damping",
                       "posture"})) {
    return NullspaceMode::kDampingOnly;
  }
  if (matches(choice, {"2", "tau_sigma", "sigma"})) {
    return NullspaceMode::kSigmaOnly;
  }
  if (matches(choice, {"3", "both", "combined", "sigma+damping",
                       "damping+sigma"})) {
    return NullspaceMode::kDampingAndSigma;
  }
  if (stop_selected != nullptr && matches(choice, {"e", "stop", "exit"})) {
    *stop_selected = true;
  }
  return fallback;
}

bool selectHoldNullspaceMode(Parameters& params,
                             bool* stop_selected = nullptr) {
  const NullspaceMode selected =
      askHoldNullspaceMode(params.nullspace_mode, stop_selected);
  if (stop_selected != nullptr && *stop_selected) {
    return false;
  }
  params.nullspace_mode = selected;
  params.use_nullspace_optimization =
      params.nullspace_mode != NullspaceMode::kOff;
  return true;
}

}  // namespace

bool openGripper(const Parameters& params, Gripper& gripper) {
  try {
    const franka::GripperState before = gripper.readOnce();
    printf("Opening gripper to %.1f mm...\n", 1000.0 * params.gripper_open_width);
    const bool opened = gripper.move(params.gripper_open_width, params.gripper_open_speed);
    const franka::GripperState after = gripper.readOnce();
    const double width_tolerance = 0.002;
    const bool calibration_supports_target =
        after.max_width + width_tolerance >= params.gripper_open_width;
    const bool target_reached =
        after.width + width_tolerance >= params.gripper_open_width;
    const bool verified =
        opened && calibration_supports_target && target_reached;

    printf("Gripper width: %.1f -> %.1f mm (reported max %.1f mm).\n",
           1000.0 * before.width,
           1000.0 * after.width,
           1000.0 * after.max_width);
    if (verified) {
      printf("Gripper opened and width verified.\n");
    } else {
      fprintf(stderr,
              "Gripper did not reach the requested open width. Support/remove "
              "the tool and select r to recalibrate the hand.\n");
    }
    return verified;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper open failed: %s\n", e.what());
    return false;
  }
}

bool graspTool(const Parameters& params, Gripper& gripper) {
  try {
    const franka::GripperState before = gripper.readOnce();
    printf("Grasping tool: width %.1f mm, force %.1f N...\n",
           1000.0 * params.gripper_grasp_width, params.gripper_grasp_force);
    // epsilon_inner/outer set how far the final width may fall short of /
    // exceed the target and still count as a successful grasp.
    const bool grasped = gripper.grasp(
        params.gripper_grasp_width, params.gripper_grasp_speed,
        params.gripper_grasp_force, params.gripper_grasp_epsilon_inner,
        params.gripper_grasp_epsilon_outer);
    const franka::GripperState after = gripper.readOnce();
    const bool width_in_band =
        after.width >=
            params.gripper_grasp_width - params.gripper_grasp_epsilon_inner &&
        after.width <=
            params.gripper_grasp_width + params.gripper_grasp_epsilon_outer;
    const bool verified = grasped && after.is_grasped && width_in_band;

    printf("Gripper width: %.1f -> %.1f mm | grasped flag: %s.\n",
           1000.0 * before.width,
           1000.0 * after.width,
           after.is_grasped ? "yes" : "no");
    if (verified) {
      printf("Gripper closed on the tool and grasp verified.\n");
    } else {
      fprintf(stderr,
              "Tool grasp was not verified. Support the tool, check its "
              "placement, and select r if the fingers did not travel.\n");
    }
    return verified;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper grasp failed: %s\n", e.what());
    return false;
  }
}

bool saveGuidedPoseAsQInit(const Vec7& q) {
  const std::string path = "params/Q_Init.txt";
  std::ifstream input(path);
  if (!input) {
    fprintf(stderr, "Could not open %s to save the pose.\n", path.c_str());
    return false;
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  input.close();

  // Rewrite the seven saved joints and point q_init_case at them. Every other
  // line, including the other postures and the comments, is left as it is.
  int replaced = 0;
  bool case_set = false;
  for (std::string& text : lines) {
    const std::string trimmed = trim(text);
    if (trimmed.rfind("q_init_case", 0) == 0) {
      text = "q_init_case = saved_qinit";
      case_set = true;
      continue;
    }
    for (int i = 0; i < 7; ++i) {
      const std::string key = "q_init_saved_" + std::to_string(i + 1);
      if (trimmed.rfind(key + " ", 0) == 0 || trimmed.rfind(key + "=", 0) == 0) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%s = %.6f", key.c_str(), q(i));
        text = buffer;
        ++replaced;
        break;
      }
    }
  }
  if (replaced != 7 || !case_set) {
    fprintf(stderr,
            "%s is missing q_init_case or a q_init_saved_* line "
            "(found %d of 7). Nothing was written.\n",
            path.c_str(), replaced);
    return false;
  }

  std::ofstream output(path);
  if (!output) {
    fprintf(stderr, "Could not write %s.\n", path.c_str());
    return false;
  }
  for (const std::string& text : lines) {
    output << text << "\n";
  }
  printf("Saved this pose as q_init_case = saved_qinit in %s.\n", path.c_str());
  printVec7Deg("saved q1..q7 [deg]", q);
  return true;
}

bool recalibrateGripper(const Parameters& params,
                        Gripper& gripper,
                        bool confirmation_already_received) {
  try {
    const franka::GripperState before = gripper.readOnce();
    printf("\n=== Franka Hand width recalibration ===\n");
    printf("Current width: %.1f mm | reported max: %.1f mm | grasped: %s\n",
           1000.0 * before.width,
           1000.0 * before.max_width,
           before.is_grasped ? "yes" : "no");
    printf("This opens the fingers completely. A held tool WILL FALL.\n");
    printf("Support the tool by hand or remove it before continuing.\n");

    if (!confirmation_already_received) {
      printf("Type  recal  and press Enter to continue, anything else to abort: ");
      fflush(stdout);
      if (readChoice() != "recal") {
        printf("Recalibration aborted. Nothing was moved.\n");
        return false;
      }
    } else {
      printf("Explicit recal command received; starting.\n");
    }

    const bool homed = gripper.homing();
    const franka::GripperState after = gripper.readOnce();
    const double width_tolerance = 0.002;
    const bool calibration_supports_target =
        after.max_width + width_tolerance >= params.gripper_open_width;
    const bool opened =
        after.width + width_tolerance >= params.gripper_open_width;
    const bool verified = homed && calibration_supports_target && opened;

    printf("After recalibration: width %.1f mm | reported max %.1f mm.\n",
           1000.0 * after.width,
           1000.0 * after.max_width);
    if (verified) {
      printf("Recalibrated. The hand is open; place the tool and select c.\n");
    } else {
      fprintf(stderr,
              "Recalibration was not verified. Do not start the robot "
              "experiment; inspect the Franka Hand state.\n");
    }
    return verified;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper recalibration failed: %s\n", e.what());
    return false;
  }
}

namespace {

// Fetch (grasp = true) or return (grasp = false) the tool at the stored pickup
// posture. Always travels through q_init so the swept path is the same every
// time, and leaves the arm at the pickup posture.
bool runToolPickup(const Parameters& params,
                   Robot& robot,
                   const Model& model,
                   bool grasp,
                   const std::function<void()>& ensure_q_init) {
  if (!params.use_tool_pickup) {
    printf("Tool pickup is not configured. Guide the arm to the tool (g),\n"
           "press e, paste the printed pose into q_pickup_* in\n"
           "params/Gripper_Action.txt, and set use_tool_pickup = 1.\n");
    return false;
  }

  Gripper gripper(params.robot_ip);
  const franka::GripperState state = gripper.readOnce();
  if (grasp && state.is_grasped) {
    printf("The hand is already holding something (width %.1f mm).\n"
           "Press b to put it back, or o to release it, before fetching.\n",
           1000.0 * state.width);
    return false;
  }
  if (!grasp && !state.is_grasped) {
    printf("The hand does not report a grasp; going to the pickup posture "
           "and opening anyway.\n");
  }

  // Stand-off posture: the same pose retreated along -Z_EE, so the last leg
  // in is a descent along the tool axis instead of a sideways sweep.
  Array7 q_above{};
  const RobotState state_now = robot.readOnce();
  if (!solveStandoffPosture(model, state_now, params.q_pickup,
                            params.pickup_standoff, q_above)) {
    printf("Could not solve a stand-off posture %.0f mm back along -Z_EE.\n"
           "Reduce pickup_standoff or re-measure q_pickup_*.\n",
           1000.0 * params.pickup_standoff);
    return false;
  }
  int bad_joint = 0;
  if (!withinJointLimits(q_above, bad_joint)) {
    printf("The stand-off posture puts joint %d outside its limit. "
           "Reduce pickup_standoff.\n", bad_joint);
    return false;
  }

  printf("\nStand-off %.0f mm back along -Z_EE from the pickup pose:\n",
         1000.0 * params.pickup_standoff);
  printf("  q [deg] = [");
  for (int i = 0; i < 7; ++i) {
    printf("%s%.1f", i ? ", " : "", 180.0 / M_PI * q_above[i]);
  }
  printf("]\n");
  printf("Path: q_init -> stand-off -> down onto the tool -> %s -> lift.\n",
         grasp ? "grasp" : "release");
  printf("Press Enter to run it, anything else to abort: ");
  fflush(stdout);
  if (!readChoice().empty()) {
    printf("Aborted. Nothing was moved.\n");
    return false;
  }

  // Open before travelling: the fingers must clear the tool on the way in.
  // On release the hand is still carrying, so it opens at the holder instead.
  if (grasp && !openGripper(params, gripper)) {
    printf("Not moving: the hand did not reach the open width.\n");
    return false;
  }

  ensure_q_init();
  printf("Moving above the tool...\n");
  MotionGenerator to_above(0.4, q_above);
  robot.control(to_above);

  printf("Descending onto the tool...\n");
  MotionGenerator descend(std::max(0.05, params.pickup_descend_speed_factor),
                          params.q_pickup);
  robot.control(descend);

  const bool ok = grasp ? graspTool(params, gripper)
                        : openGripper(params, gripper);

  // Lift straight back to the stand-off either way: leaving the arm down in
  // the holder would drag the tool sideways on the next motion.
  printf("Lifting clear of the holder...\n");
  MotionGenerator lift(std::max(0.05, params.pickup_descend_speed_factor),
                       q_above);
  robot.control(lift);

  if (ok) {
    printf(grasp ? "Tool fetched. The arm waits above the holder.\n"
                 : "Tool released. The arm waits above the holder.\n");
  }
  return ok;
}

}  // namespace

bool askStartupRunMode(Parameters& params, Robot& robot,
                       const Model& model) {
  bool q_init_reached = false;

  const auto move_to_q_init = [&]() {
    printf("Moving to q_init...\n");
    MotionGenerator motion_generator(0.4, params.q_init);
    robot.control(motion_generator);
    printf("q_init reached.\n");
    q_init_reached = true;
  };

  const auto ensure_q_init = [&]() {
    if (!q_init_reached) {
      move_to_q_init();
    }
  };

  // One choice selects both the start-pose source and the run mode. q, o and c
  // inspect or set up first and return to this menu.
  while (true) {
    printf("\n=== Startup mode ===\n");
    printf("  s = go to q_init, then run the phase sequence\n");
    printf("  h = go to q_init, then hold that pose\n");
    printf("  t = test the set-up impedance: hold with the set-up K and D\n");
    printf("  g = guiding mode: go to q_init, then hand-guide to your pose;\n");
    printf("      use s+Enter for sequence or h+Enter for hold\n");
    printf("  q = go to q_init, inspect the posture, then choose again\n");
    printf("  o = open the Franka hand now (release/load tool), then choose again\n");
    printf("  c = close/grasp the tool now, then choose again\n");
    printf("  r = recalibrate the hand width (opens fully; support the tool)\n");
    printf("  f = fetch the tool: via q_init, down onto it, grasp, lift\n");
    printf("  b = put the tool back: same path, releases at the holder\n");
    printf("  e = stop and quit\n");
    printf("While a run is going: e+Enter stops, m+Enter comes back here.\n");
    printf("Choice [s/h/t/g/q/o/c/r/f/b/e]: ");

    const std::string choice = readChoice();
    if (matches(choice, {"s", "sequence"})) {
      ensure_q_init();
      params.use_manual_guidance_start = false;
      params.use_phase_sequence = true;
      params.hold_with_setup_gains = false;
      break;
    }
    if (matches(choice, {"h", "hold"})) {
      ensure_q_init();
      params.use_manual_guidance_start = false;
      params.use_phase_sequence = false;
      params.hold_with_setup_gains = false;
      break;
    }
    if (matches(choice, {"t", "test", "setup"})) {
      ensure_q_init();
      params.use_manual_guidance_start = false;
      params.use_phase_sequence = false;
      params.hold_with_setup_gains = true;
      break;
    }
    if (matches(choice, {"q", "qinit"})) {
      move_to_q_init();
      continue;
    }
    if (matches(choice, {"o", "open"})) {
      try {
        Gripper gripper(params.robot_ip);
        openGripper(params, gripper);
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper connection failed: %s\n", e.what());
      }
      // Explicit manual open: don't let the automatic q_init action close it.
      params.startup_gripper_manual = true;
      continue;
    }
    if (matches(choice, {"c", "close", "grasp"})) {
      try {
        Gripper gripper(params.robot_ip);
        graspTool(params, gripper);
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper connection failed: %s\n", e.what());
      }
      params.startup_gripper_manual = true;
      continue;
    }
    if (matches(choice, {"r", "recal", "recalibrate"})) {
      try {
        Gripper gripper(params.robot_ip);
        // The synchronous startup menu always presents the full warning before
        // accepting the exact confirmation word.
        recalibrateGripper(params, gripper, false);
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper connection failed: %s\n", e.what());
      }
      // Recalibration leaves the fingers open. Do not let the automatic
      // startup action close them before the operator explicitly selects c.
      params.startup_gripper_manual = true;
      continue;
    }
    if (matches(choice, {"f", "fetch", "tool"}) ||
        matches(choice, {"b", "back", "putback"})) {
      const bool grasp = matches(choice, {"f", "fetch", "tool"});
      try {
        if (runToolPickup(params, robot, model, grasp, ensure_q_init)) {
          // The hand was set deliberately, and the arm is no longer at q_init.
          params.startup_gripper_manual = true;
        }
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Tool pickup failed: %s\n", e.what());
      }
      q_init_reached = false;
      continue;
    }
    if (matches(choice, {"e", "stop", "quit", "exit"})) {
      printf("Stopping without starting a run.\n");
      return false;
    }
    if (matches(choice, {"g", "guide", "guiding"})) {
      ensure_q_init();
      params.use_manual_guidance_start = true;
      printf("Selected: guiding mode (hand-place the start pose after q_init).\n");
      break;
    }
    if (choice.empty()) {
      printf("Choose s, h, t, g, q, o, c, r, f, b, or e explicitly.\n");
    } else {
      printf("Unknown startup choice '%s'; choose s, h, t, g, q, o, c, r, f, "
             "b, or e.\n", choice.c_str());
    }
  }

  // In guiding mode the run mode is chosen at the END of guiding, so asking
  // here would be redundant.
  if (params.use_manual_guidance_start) {
    printf("Run mode (s/h) will be chosen at the end of guiding.\n");
    return true;
  }

  if (params.use_phase_sequence) {
    printf("Selected: phase sequence. Nullspace mode from parameters: %s.\n",
           nullspaceModeName(params.nullspace_mode));
    return true;
  }

  // The t mode exists to reproduce set-up conditions, and a sequence run takes
  // its nullspace mode from the parameter file without asking. Asking here
  // would let the two drift apart, so it is not asked.
  if (params.hold_with_setup_gains) {
    printf("Selected: hold with the SET-UP gains. Nullspace mode from "
           "parameters: %s.\n", nullspaceModeName(params.nullspace_mode));
    if (params.use_coupled_stiffness) {
      printf("Coupled set-up stiffness is active in this hold.\n");
    }
    return true;
  }

  // Plain hold is where the nullspace terms are studied, so ask for the mode
  // rather than taking it silently. Enter keeps the parameter-file value.
  (void)selectHoldNullspaceMode(params);
  printf("Selected: hold mode with %s.\n",
         nullspaceModeName(params.nullspace_mode));
  return true;
}

bool performStartupGripperAction(const Parameters& params) {
  if (!params.open_gripper_before_run || params.use_manual_guidance_start ||
      params.startup_gripper_manual) {
    return true;
  }

  bool gripper_ok = false;
  try {
    Gripper gripper(params.robot_ip);
    if (params.gripper_grasp_on_tool) {
      // Keep an existing grasp: re-grasping with no gap left reports failure,
      // and re-homing would open the hand and drop the tool.
      const franka::GripperState state = gripper.readOnce();
      const bool already_holding =
          state.is_grasped &&
          state.width >= params.gripper_grasp_width - params.gripper_grasp_epsilon_inner &&
          state.width <= params.gripper_grasp_width + params.gripper_grasp_epsilon_outer;
      if (already_holding) {
        printf("Gripper already holding the tool (width %.1f mm); keeping grasp.\n",
               1000.0 * state.width);
        gripper_ok = true;
      } else {
        gripper_ok = graspTool(params, gripper);
      }
    } else {
      gripper_ok = openGripper(params, gripper);
    }
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper action failed: %s\n", e.what());
    gripper_ok = false;
  }

  if (!gripper_ok && params.require_gripper_open) {
    fprintf(stderr, "Stopping because require_gripper_open = 1.\n");
    return false;
  }
  return true;
}

bool runManualGuidanceStart(Parameters& params,
                            Robot& robot,
                            const Model& model,
                            std::atomic<bool>& stop_requested,
                            std::atomic<char>& guidance_menu_key,
                            std::atomic<bool>& guided_hold_selector_pending) {
  struct PendingFlagReset {
    std::atomic<bool>& flag;
    ~PendingFlagReset() {
      flag.store(false);
    }
  };

  // This is also initialized before the keyboard thread starts in main, which
  // closes the small interval before this function begins executing.
  guided_hold_selector_pending.store(true);
  PendingFlagReset pending_flag_reset{guided_hold_selector_pending};

  Gripper gripper(params.robot_ip);
  Vec7 stop_q = Vec7::Zero();
  const auto print_stop_pose = [&stop_q]() {
    printf("\n=== Manual guidance stop pose ===\n");
    printf("q1..q7 [rad] (paste into a q_init_* case):\n");
    for (int i = 0; i < 7; ++i) {
      printf("  q_init_%d = %.6f\n", i + 1, stop_q(i));
    }
    printVec7Deg("q1..q7 [deg]", stop_q);
  };

  while (true) {
    printf("\nphase: manual_guidance_start\n");
    printf("Move the robot by hand. Then:\n");
    printf("  o+Enter = open the gripper\n");
    printf("  c+Enter = close/grasp the tool\n");
    printf("  recal+Enter = recalibrate the hand width (opens fully)\n");
    printf("  m+Enter = back to the startup menu\n");
    printf("  w+Enter = save this pose as q_init (writes params/Q_Init.txt)\n");
    printf("  s+Enter = start phase sequence from this pose\n");
    printf("  h+Enter = start hold from this pose\n");
    printf("  e+Enter = stop (prints this pose as a q_init_* case)\n");

    // Gravity compensation with a little joint damping, so the arm can be moved
    // by hand. Ends as soon as the keyboard thread reports a menu key.
    robot.control([&](const RobotState& state, Duration /*period*/) -> Torques {
      Map<const Vec7> dq(state.dq.data());
      Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());
      const Array7 tau_array =
          vec7ToArray(Vec7(coriolis - params.manual_guidance_damping * dq));
      if (stop_requested.load()) {
        stop_q = Map<const Vec7>(state.q.data());
        return MotionFinished(Torques(tau_array));
      }
      if (guidance_menu_key.load() != 0) {
        return MotionFinished(Torques(tau_array));
      }
      return Torques(tau_array);
    });

    if (stop_requested.load()) {
      print_stop_pose();
      return false;
    }

    const char key = guidance_menu_key.exchange(0);
    if (key == 'o') {
      openGripper(params, gripper);
    } else if (key == 'c') {
      graspTool(params, gripper);
    } else if (key == 'r') {
      recalibrateGripper(params, gripper, true);
    } else if (key == 'w') {
      const RobotState saved_state = robot.readOnce();
      (void)saveGuidedPoseAsQInit(Map<const Vec7>(saved_state.q.data()));
    } else if (key == 's') {
      params.use_phase_sequence = true;
      printf("Selected: phase sequence from the guided pose.\n");
      return true;
    } else if (key == 'h') {
      params.use_phase_sequence = false;
      bool stop_selected = false;
      if (!selectHoldNullspaceMode(params, &stop_selected)) {
        stop_requested.store(true);
        const RobotState stop_state = robot.readOnce();
        stop_q = Map<const Vec7>(stop_state.q.data());
        print_stop_pose();
        return false;
      }
      printf("Selected: hold at the guided pose with %s.\n",
             nullspaceModeName(params.nullspace_mode));
      return true;
    }
  }
}
