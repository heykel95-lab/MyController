#include "controller_report.h"

#include "controller_helpers.h"
#include "controller_printing.h"

namespace {

// Prints "num/den" or "n/a" when the denominator is too small to divide by.
void formatRatio(char* out, size_t n, double num, double den, double den_floor) {
  if (std::abs(den) < den_floor) {
    snprintf(out, n, "%9s", "n/a");
  } else {
    snprintf(out, n, "%9.1f", num / den);
  }
}

// The contact wrench and the motion it produced, both resolved into the
// alignment-target frame [tangent1, tangent2, normal]. Force is paired with TCP
// displacement (where the translational spring acts); the contact moment is
// taken at the pressed edge and paired with the tip angle. This gives real
// per-axis force-vs-displacement and moment-vs-angle pairs measured from the
// run instead of from a target ratio.
void printSurfaceFrameBreakdown(const Mat3& R_alignment_target,
                                const SetUpReport& r) {
  const Vec3 force_surf =
      R_alignment_target.transpose() * (r.external_force - r.contact_force_bias);
  const Vec3 moment_surf = R_alignment_target.transpose() * r.contact_moment_at_edge;
  const Vec3 tcp_disp_surf =
      R_alignment_target.transpose() * (r.p_EE - r.first_contact_tcp);
  const Vec3 edge_disp_surf =
      R_alignment_target.transpose() * (r.tool_contact_point - r.first_contact_point);
  const Vec3 tip_surf =
      R_alignment_target.transpose() * orientationError(r.R_EE, r.R_contact_start);

  char kp[3][16];
  char kr[3][16];
  for (int i = 0; i < 3; ++i) {
    formatRatio(kp[i], sizeof(kp[i]), force_surf(i), tcp_disp_surf(i), 5e-5);
    formatRatio(kr[i], sizeof(kr[i]), moment_surf(i), tip_surf(i), 5e-4);
  }

  printf("\n=== Set-up surface-frame breakdown ===\n");
  printf("frame: alignment-target [tangent1, tangent2, normal]\n");
  printf("force        [N]      = [%+9.2f, %+9.2f, %+9.2f]\n",
         force_surf(0), force_surf(1), force_surf(2));
  printf("tcp_disp     [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         1000.0 * tcp_disp_surf(0), 1000.0 * tcp_disp_surf(1), 1000.0 * tcp_disp_surf(2));
  printf("edge_disp    [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         1000.0 * edge_disp_surf(0), 1000.0 * edge_disp_surf(1), 1000.0 * edge_disp_surf(2));
  printf("Kp_eff=F/tcp [N/m]    = [%s, %s, %s]\n", kp[0], kp[1], kp[2]);
  printf("moment@edge  [Nm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         moment_surf(0), moment_surf(1), moment_surf(2));
  printf("tip_angle    [deg]    = [%+9.2f, %+9.2f, %+9.2f]\n",
         (180.0 / M_PI) * tip_surf(0), (180.0 / M_PI) * tip_surf(1),
         (180.0 / M_PI) * tip_surf(2));
  printf("KR_eff=M/ang [Nm/rad] = [%s, %s, %s]\n", kr[0], kr[1], kr[2]);
}

}  // namespace

void reportSetUpResult(const Parameters& params,
                       const Mat3& R_alignment_target,
                       const SetUpReport& r,
                       std::vector<std::pair<std::string, std::string>>* parameter_updates) {
  const double actual_tip_deg =
      (180.0 / M_PI) * orientationError(r.R_EE, r.R_contact_start).norm();
  const Vec3 edge_from_contact_mm = 1000.0 * (r.tool_contact_point - r.first_contact_point);
  const Vec3 tcp_from_contact_mm = 1000.0 * (r.p_EE - r.first_contact_point);

  printf("\n=== Set-up result ===\n");
  printf("stop: %s | t=%.1f s | tip=%.1f deg | F=%.1f N | M=%.1f Nm\n",
         r.stopped_on_moment ? "moment" : "time",
         r.phase_time, actual_tip_deg, r.force_delta_norm, r.moment_delta_norm);
  printf("edge_from_contact = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
         edge_from_contact_mm(0), edge_from_contact_mm(1), edge_from_contact_mm(2),
         edge_from_contact_mm.norm());
  printf("tcp_from_contact  = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
         tcp_from_contact_mm(0), tcp_from_contact_mm(1), tcp_from_contact_mm(2),
         tcp_from_contact_mm.norm());

  printSurfaceFrameBreakdown(R_alignment_target, r);

  // One exact axis for the whole tipping motion, computed only from the start
  // and end edge pose -- unlike a per-cycle instantaneous pole it is not an
  // average of noisy samples and does not depend on which cycle looked cleanest.
  const FiniteScrewAxis finite_axis = computeFiniteScrewAxis(
      r.first_contact_point, r.R_contact_start, r.tool_contact_point, r.R_EE);
  if (finite_axis.valid) {
    printf("\nfinite_axis: angle=%.1f deg | axis_from_edge=[%+.1f, %+.1f, %+.1f] mm | dir=[%+.3f, %+.3f, %+.3f] | pitch=%+.1f mm/rad\n",
           (180.0 / M_PI) * finite_axis.angle,
           1000.0 * finite_axis.axis_point_from_start(0),
           1000.0 * finite_axis.axis_point_from_start(1),
           1000.0 * finite_axis.axis_point_from_start(2),
           finite_axis.axis_dir(0), finite_axis.axis_dir(1), finite_axis.axis_dir(2),
           1000.0 * finite_axis.pitch);
  } else {
    printf("\nfinite_axis: angle=%.1f deg, too small for a well-defined axis\n",
           (180.0 / M_PI) * finite_axis.angle);
  }

  if (!params.print_coupled_diagnostics) {
    return;
  }

  // ================================================================
  // Coupled (pole-based) stiffness
  // ================================================================
  // The same diagonal set-up spring is expressed at the tool tip for two poles
  // via K_TCP = Ad(r_c)^T K_pole Ad(r_c), r_c = TCP - pole:
  //   (A) the measured finite screw axis -> a diagnostic of the motion.
  //   (B) the manually placed pole       -> the spring actually commanded in
  //       manual-pole mode.
  // Each result is ONE 6x6 (rows fx..mz = wrench, cols tx..rz = displacement);
  // the off-diagonal quadrants are the coupling.
  const Mat6x6 K_pole = blockDiagonal(r.Kp, r.KR);
  const Mat6x6 D_pole = blockDiagonal(r.Dp, r.DR);

  // Pole (A): the point on the measured axis nearest the contact edge. Without
  // a valid axis, fall back to the edge itself (r_c = TCP - edge).
  const Vec3 pole_measured =
      finite_axis.valid
          ? nearestPointOnAxis(r.tool_contact_point,
                               r.first_contact_point + finite_axis.axis_point_from_start,
                               finite_axis.axis_dir)
          : r.tool_contact_point;
  const Vec3 r_c_measured = r.p_EE - pole_measured;
  const Mat6x6 K_tcp_measured = adjointTransformedGain(K_pole, r_c_measured);
  const Mat6x6 D_tcp_measured = adjointTransformedGain(D_pole, r_c_measured);

  // Pole (B): the commanded manual pole, referenced either to the frozen
  // first-contact pose or to the live edge, matching what the control loop does.
  const Vec3 edge_ref = params.coupled_pole_freeze_at_contact ? r.first_contact_point
                                                              : r.tool_contact_point;
  const Vec3 tcp_ref = params.coupled_pole_freeze_at_contact ? r.first_contact_tcp
                                                             : r.p_EE;
  const Vec3 pole_manual = edge_ref + params.coupled_pole_from_edge;
  const Vec3 r_c_manual = tcp_ref - pole_manual;
  const Mat6x6 K_tcp_manual = adjointTransformedGain(K_pole, r_c_manual);
  const Mat6x6 D_tcp_manual = adjointTransformedGain(D_pole, r_c_manual);

  if (finite_axis.valid && parameter_updates != nullptr &&
      !params.use_coupled_stiffness) {
    parameter_updates->emplace_back("coupled_gains_saved", "1");
    appendMat6ParameterUpdates(*parameter_updates, "coupled_K_tcp", K_tcp_measured);
    appendMat6ParameterUpdates(*parameter_updates, "coupled_D_tcp", D_tcp_measured);
    printf("(coupled_K_tcp/coupled_D_tcp queued for the next run)\n");
  }

  printf("\n=== Coupled stiffness from the measured pole ===\n");
  printf("source spring K_pole/D_pole (diagonal, from parameters):\n");
  printGainVec("  Kp [N/m]    ", params.setup_Kp_diag);
  printGainVec("  KR [Nm/rad] ", params.setup_KR_diag);
  printGainVec("  Dp [Ns/m]   ", params.setup_Dp_diag);
  printGainVec("  DR [Nms/rad]", params.setup_DR_diag);

  const bool manual_pole_commanded = params.use_coupled_stiffness &&
                                     !params.coupled_use_block_diagonal &&
                                     params.coupled_pole_manual;
  printf("\npoles compared (lever r_c = TCP - pole):\n");
  if (finite_axis.valid) {
    printf("  (A) measured axis : angle=%.1f deg | pitch=%+.1f mm/rad\n",
           (180.0 / M_PI) * finite_axis.angle, 1000.0 * finite_axis.pitch);
  } else {
    printf("  (A) measured axis : not valid (rotation too small) -> pole at the edge\n");
  }
  printVec3Mm("      pole_from_edge", pole_measured - r.tool_contact_point);
  printVec3Mm("      r_c           ", r_c_measured);
  printf("  (B) manual pole   : %s, %s reference\n",
         manual_pole_commanded ? "COMMANDED this run" : "comparison only",
         params.coupled_pole_freeze_at_contact ? "contact" : "live");
  printVec3Mm("      pole_from_edge", pole_manual - r.tool_contact_point);
  printVec3Mm("      r_c           ", r_c_manual);
  printVec3Mm("      r_c(B)-r_c(A) ", r_c_manual - r_c_measured);

  printf("\nK_TCP = Ad^T K_pole Ad  [rows fx..mz | cols tx..rz]:\n");
  printSpatialGain6("  (A) measured axis", K_tcp_measured);
  printSpatialGainEigenvalues("  (A) K_TCP", K_tcp_measured);
  printSpatialGain6("  (B) manual pole", K_tcp_manual);
  printSpatialGainEigenvalues("  (B) K_TCP", K_tcp_manual);

  printf("\nD_TCP = Ad^T D_pole Ad:\n");
  printSpatialGain6("  (A) measured axis", D_tcp_measured);
  printSpatialGainEigenvalues("  (A) D_TCP", D_tcp_measured);
  printSpatialGain6("  (B) manual pole", D_tcp_manual);
  printSpatialGainEigenvalues("  (B) D_TCP", D_tcp_manual);
}

void CoupledEvalStats::addSample(const Parameters& params,
                                 const Vec6& dx,
                                 const Vec6& dv,
                                 const Vec3& commanded_force,
                                 const Vec3& commanded_moment) {
  const Vec6 coupled = params.coupled_K_tcp * dx + params.coupled_D_tcp * dv;
  const Vec3 coupled_force = coupled.head<3>();
  const Vec3 coupled_moment = coupled.tail<3>();

  const Vec3 force_from_translation =
      params.coupled_K_tcp.block<3, 3>(0, 0) * dx.head<3>() +
      params.coupled_D_tcp.block<3, 3>(0, 0) * dv.head<3>();
  const Vec3 force_from_rotation =
      params.coupled_K_tcp.block<3, 3>(0, 3) * dx.tail<3>() +
      params.coupled_D_tcp.block<3, 3>(0, 3) * dv.tail<3>();
  const Vec3 moment_from_translation =
      params.coupled_K_tcp.block<3, 3>(3, 0) * dx.head<3>() +
      params.coupled_D_tcp.block<3, 3>(3, 0) * dv.head<3>();
  const Vec3 moment_from_rotation =
      params.coupled_K_tcp.block<3, 3>(3, 3) * dx.tail<3>() +
      params.coupled_D_tcp.block<3, 3>(3, 3) * dv.tail<3>();

  const double force_error = (coupled_force - commanded_force).norm();
  const double moment_error = (coupled_moment - commanded_moment).norm();

  ++samples;
  commanded_force_sum += commanded_force.norm();
  commanded_moment_sum += commanded_moment.norm();
  coupled_force_sum += coupled_force.norm();
  coupled_moment_sum += coupled_moment.norm();
  force_error_sq_sum += force_error * force_error;
  moment_error_sq_sum += moment_error * moment_error;
  force_error_max = std::max(force_error_max, force_error);
  moment_error_max = std::max(moment_error_max, moment_error);
  force_from_translation_sum += force_from_translation.norm();
  force_from_rotation_sum += force_from_rotation.norm();
  moment_from_translation_sum += moment_from_translation.norm();
  moment_from_rotation_sum += moment_from_rotation.norm();
}

void printCoupledEvalSummary(const CoupledEvalStats& stats) {
  if (stats.samples == 0) {
    return;
  }
  const double n = static_cast<double>(stats.samples);

  printf("\n=== Coupled wrench comparison ===\n");
  printf("samples=%zu | commanded=decoupled wrench | coupled=6x6 TCP wrench (not commanded)\n",
         stats.samples);
  printf("force:  commanded_avg=%5.1f N | coupled_avg=%5.1f N | rms_diff=%5.1f N | max_diff=%5.1f N\n",
         stats.commanded_force_sum / n,
         stats.coupled_force_sum / n,
         std::sqrt(stats.force_error_sq_sum / n),
         stats.force_error_max);
  printf("moment: commanded_avg=%5.1f Nm | coupled_avg=%5.1f Nm | rms_diff=%5.1f Nm | max_diff=%5.1f Nm\n",
         stats.commanded_moment_sum / n,
         stats.coupled_moment_sum / n,
         std::sqrt(stats.moment_error_sq_sum / n),
         stats.moment_error_max);
  printf("coupled split avg: F(from translation)=%5.1f N | F(from rotation)=%5.1f N | M(from translation)=%5.1f Nm | M(from rotation)=%5.1f Nm\n",
         stats.force_from_translation_sum / n,
         stats.force_from_rotation_sum / n,
         stats.moment_from_translation_sum / n,
         stats.moment_from_rotation_sum / n);
  printf("read: small diffs mean applying the coupled spring behaves like the decoupled baseline;\n");
  printf("      large diffs mean it will drive a visibly different tipping motion.\n");
}
