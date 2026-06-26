#include "controller_printing.h"

#include "controller_helpers.h"  // normalizedOrFallback, pointDistanceToAxis

void printVec3(const char* label, const Vec3& v) {
  printf("%s = [%.6f, %.6f, %.6f]\n", label, v(0), v(1), v(2));
}

void printVec3Mm(const char* label, const Vec3& v) {
  printf("%s = [%.1f, %.1f, %.1f] mm\n",
         label,
         1000.0 * v(0),
         1000.0 * v(1),
         1000.0 * v(2));
}

// At the moment contact is detected, shows exactly which tool edge was used
// and what it cost in base-frame mm, so the auto-selected side (+/- offset_ee)
// and the tilt-induced depth/lateral split (edge_offset_base) can be checked
// directly against the physical setup instead of trusted blindly.
void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point) {
  printVec3Mm("offset_ee", offset_ee);
  printVec3Mm("p_EE_at_contact", p_EE_at_contact);
  printVec3Mm("contact_point", contact_point);
  printVec3Mm("edge_offset_base", contact_point - p_EE_at_contact);
}

void printVec3Deg(const char* label, const Vec3& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.2f, %.2f, %.2f] deg\n",
         label,
         rad_to_deg * v(0),
         rad_to_deg * v(1),
         rad_to_deg * v(2));
}

void printGainVec(const char* label, const Vec3& v) {
  printf("%s = [%.4g, %.4g, %.4g]\n", label, v(0), v(1), v(2));
}

void printMat3Rows(const char* label, const Mat3& m) {
  printf("%s:\n", label);
  for (int i = 0; i < 3; ++i) {
    printf("  [%.4g, %.4g, %.4g]\n", m(i, 0), m(i, 1), m(i, 2));
  }
}

// Prints a 6x6 spatial gain as ONE labeled grid instead of four loose 3x3
// blocks. Rows are the wrench it produces (fx..fz force, mx..mz moment);
// columns are the displacement it acts on (tx..tz translation, rx..rz
// rotation). The vertical bar and horizontal rule mark the four quadrants:
//   top-left  = translation -> force      top-right = rotation    -> force
//   bot-left  = translation -> moment     bot-right = rotation    -> moment
// The off-diagonal quadrants are the force<->rotation coupling, so a spring
// that is decoupled at the pole (zeros there) versus coupled at the TCP
// (filled there) can be read off directly.
// Plain 6x6 grid with the quadrant dividers but no row/column meaning, for
// matrices that are transforms rather than gains (e.g. the offset adjoint
// Ad = [[I, skew(r_c)], [0, I]]). The top-right block is skew(r_c); the two
// diagonal blocks are I; the bottom-left block is zero.
void printMat6Grid(const char* label, const Mat6x6& M) {
  printf("%s:\n", label);
  for (int i = 0; i < 6; ++i) {
    printf("  [%8.4g %8.4g %8.4g | %8.4g %8.4g %8.4g ]\n",
           M(i, 0), M(i, 1), M(i, 2), M(i, 3), M(i, 4), M(i, 5));
    if (i == 2) {
      printf("  --------------------------+--------------------------\n");
    }
  }
}

// Prints the six eigenvalues of a symmetric 6x6 gain (ascending) plus a PSD
// verdict. A valid passive spring/damper must be positive semi-definite, i.e.
// every eigenvalue >= 0; negative off-diagonal *entries* are fine, only the
// eigenvalues matter. The adjoint congruence preserves these from the
// decoupled pole gains, so this is a numerical confirmation that the coupled
// K_TCP/D_TCP are still valid. Uses the symmetric (self-adjoint) solver.
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

void printVec7(const char* label, const Vec7& v) {
  printf("%s = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
         label,
         v(0),
         v(1),
         v(2),
         v(3),
         v(4),
         v(5),
         v(6));
}

void printVec7Deg(const char* label, const Vec7& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg\n",
         label,
         rad_to_deg * v(0),
         rad_to_deg * v(1),
         rad_to_deg * v(2),
         rad_to_deg * v(3),
         rad_to_deg * v(4),
         rad_to_deg * v(5),
         rad_to_deg * v(6));
}

// Formats a per-axis gain suggestion, replacing an axis with "n/a" when
// either: the configured gain there is below gain_threshold (a passive axis,
// where the measured wrench comes from real contact mechanics, not from the
// spring law the suggestion is fit against), or fit_valid says that axis's
// fit was ill-conditioned (position/velocity too collinear over the sampled
// window to separate K from D).
std::string formatSuggestedGains(const Vec3& suggested,
                                 const Vec3& configured,
                                 double gain_threshold,
                                 const char* value_format,
                                 const bool fit_valid[3]) {
  char value[3][32];
  for (int i = 0; i < 3; ++i) {
    if (std::abs(configured(i)) < gain_threshold || !fit_valid[i]) {
      snprintf(value[i], sizeof(value[i]), "n/a");
    } else {
      snprintf(value[i], sizeof(value[i]), value_format, suggested(i));
    }
  }
  char line[128];
  snprintf(line, sizeof(line), "[%s, %s, %s]", value[0], value[1], value[2]);
  return std::string(line);
}

void printMat3(const char* label, const Mat3& m) {
  printf("%s = [\n", label);
  printf("  %.6f, %.6f, %.6f;\n", m(0, 0), m(0, 1), m(0, 2));
  printf("  %.6f, %.6f, %.6f;\n", m(1, 0), m(1, 1), m(1, 2));
  printf("  %.6f, %.6f, %.6f\n", m(2, 0), m(2, 1), m(2, 2));
  printf("]\n");
}

void printMat4x4(const char* label, const Mat4x4& m) {
  printf("%s = [\n", label);
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(0, 0), m(0, 1), m(0, 2), m(0, 3));
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(1, 0), m(1, 1), m(1, 2), m(1, 3));
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(2, 0), m(2, 1), m(2, 2), m(2, 3));
  printf("  %.6f, %.6f, %.6f, %.6f\n", m(3, 0), m(3, 1), m(3, 2), m(3, 3));
  printf("]\n");
}

void printJointStartEndTable(const Vec7& q_start, const Vec7& q_final) {
  printf("\nq_start_q_final_delta = [\n");
  printf("  %% joint, q_start_rad, q_final_rad, delta_rad\n");
  for (int i = 0; i < 7; ++i) {
    printf("  %d, %.6f, %.6f, %.6f%s\n",
           i + 1,
           q_start(i),
           q_final(i),
           q_final(i) - q_start(i),
           (i == 6) ? "" : ";");
  }
  printf("]\n");
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
  printf("phase_sequence: %s | orientation_phase: %s | contact_search: %s | orientation_test: %s\n",
         params.use_phase_sequence ? "on" : "off",
         params.use_orientation_phase ? "on" : "off",
         params.use_contact_search ? "on" : "off",
         params.orientation_test_only ? "on" : "off");
  printf("manual_guidance_start: %s | manual_damping=%.2f\n",
         params.use_manual_guidance_start ? "on" : "off",
         params.manual_guidance_damping);
  printf("force/moment limits: search=%.1f N | align=%.1f N | post_moment=%.1f Nm\n",
         params.contact_force_threshold,
         params.alignment_contact_force_threshold,
         params.post_contact_moment_threshold);
  printf("post_align_duration=%.1f s | method2_eval: %s | method2_apply: %s (saved=%s) | debug_period=%.2f s\n",
         params.post_contact_align_duration,
         params.post_contact_eval_method2_tcp_wrench ? "on" : "off",
         params.post_contact_apply_method2_tcp_wrench ? "on" : "off",
         params.method2_tcp_wrench_saved ? "yes" : "no",
         params.debug_period);
}

// One short, fixed-format debug line per phase, all sharing the same
// "t=... | name=value" style so the live output is easy to scan no matter
// which phase the controller is in.

void printOrientDebug(double phase_time,
                      double axis_error_deg,
                      double rot_error_deg) {
  printf("orient:     t=%5.1f s | axis_err=%5.1f deg | rot_err=%5.1f deg\n",
         phase_time, axis_error_deg, rot_error_deg);
}

void printSearchDebug(double phase_time,
                      double distance_mm,
                      double force_n,
                      double force_limit_n,
                      bool touch_saved) {
  printf("search:     t=%5.1f s | distance=%6.1f mm | force=%5.1f N (limit %.1f) | touch=%s\n",
         phase_time, distance_mm, force_n, force_limit_n, touch_saved ? "yes" : "no");
}

void printAlignDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double edge_mm,
                     bool pole_valid,
                     const Vec3& pole_nearest_edge_mm,
                     double pole_dist_mm) {
  char pole_text[64];
  if (pole_valid) {
    snprintf(pole_text, sizeof(pole_text), "pole=[%+.1f,%+.1f,%+.1f] D=%.1f mm",
             pole_nearest_edge_mm(0), pole_nearest_edge_mm(1), pole_nearest_edge_mm(2),
             pole_dist_mm);
  } else {
    snprintf(pole_text, sizeof(pole_text), "pole=slow");
  }
  printf("align:      t=%5.1f s | tip=%5.1f deg | F=%5.1f N | M=%5.1f Nm (limit %.1f) | edge=%5.1f mm | %s\n",
         phase_time, tip_deg, force_n, moment_nm, moment_limit_nm, edge_mm, pole_text);
}

void printImpedanceDebug(double phase_time,
                         double force_n,
                         double pos_error_mm,
                         double rot_error_deg) {
  printf("impedance:  t=%5.1f s | force=%5.1f N | pos_err=%5.1f mm | rot_err=%5.1f deg\n",
         phase_time, force_n, pos_error_mm, rot_error_deg);
}

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
    const std::string& csv_file_name) {
  printf("\n=== Final result ===\n");
  printVec3Mm("p_d", final_p_d);
  printVec3Mm("p_EE", final_p_EE);
  printVec3Mm("e_p", final_e_p);
  printf("position_error = %.2f mm\n", 1000.0 * final_e_p.norm());
  printVec3Deg("e_R", final_e_R);
  printf("rotation_error = %.2f deg\n", (180.0 / M_PI) * final_e_R.norm());
  if (final_instant_pole_valid) {
    printf("best_valid_axis: t=%.3f s | edge=%.2f mm | axis_point_from_edge=[%+.1f, %+.1f, %+.1f] mm | pitch=%.2f mm/rad | dir=[%+.3f, %+.3f, %+.3f]\n",
           final_instant_axis_time,
           1000.0 * final_instant_edge_axis_distance,
           1000.0 * final_instant_pole_to_edge(0),
           1000.0 * final_instant_pole_to_edge(1),
           1000.0 * final_instant_pole_to_edge(2),
           1000.0 * final_instant_screw_pitch,
           final_instant_axis_dir(0),
           final_instant_axis_dir(1),
           final_instant_axis_dir(2));
    const Vec3 last_best_axis_dir_unit =
        normalizedOrFallback(last_best_axis_dir, Vec3(1.0, 0.0, 0.0));
    const Vec3 axis_point_error = final_instant_pole_to_edge - last_best_axis_from_edge;
    const double axis_dir_dot =
        std::abs(std::max(
            -1.0,
            std::min(1.0, final_instant_axis_dir.dot(last_best_axis_dir_unit))));
    const double axis_dir_error_deg = (180.0 / M_PI) * std::acos(axis_dir_dot);
    const double desired_axis_edge_distance =
        pointDistanceToAxis(Vec3::Zero(), last_best_axis_from_edge, last_best_axis_dir_unit);
    printf("axis_error_vs_last_best: point=[%+.1f, %+.1f, %+.1f] mm | point_norm=%.1f mm | dir_error=%.1f deg | edge_error=%+.1f mm | pitch_error=%+.1f mm/rad\n",
           1000.0 * axis_point_error(0),
           1000.0 * axis_point_error(1),
           1000.0 * axis_point_error(2),
           1000.0 * axis_point_error.norm(),
           axis_dir_error_deg,
           1000.0 * (final_instant_edge_axis_distance - desired_axis_edge_distance),
           1000.0 * (final_instant_screw_pitch - last_best_axis_pitch));
  } else {
    printf("instant_axis: slow_rotation\n");
  }
  printf("csv: %s\n", csv_file_name.c_str());
}
