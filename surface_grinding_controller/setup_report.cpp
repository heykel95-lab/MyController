#include "controller.h"

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
                       const SetUpReport& r) {
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

  // Alignment quality against the calibrated surface. tip (above) says how far
  // the tool turned; this says how flat it ended up. They are different
  // questions and only this one answers "did the alignment work".
  const double align_before_deg =
      (180.0 / M_PI) *
      toolSurfaceMisalignmentAngle(params, r.R_contact_start, R_alignment_target);
  const double align_after_deg =
      (180.0 / M_PI) *
      toolSurfaceMisalignmentAngle(params, r.R_EE, R_alignment_target);
  printf("alignment: before=%.2f deg | after=%.2f deg | gain=%+.2f deg\n",
         align_before_deg, align_after_deg, align_before_deg - align_after_deg);
  const Vec3 align_before_surface_deg =
      (180.0 / M_PI) * R_alignment_target.transpose() *
      toolSurfaceAlignmentErrorInBase(
          params, r.R_contact_start, R_alignment_target);
  const Vec3 align_after_surface_deg =
      (180.0 / M_PI) * R_alignment_target.transpose() *
      toolSurfaceAlignmentErrorInBase(params, r.R_EE, R_alignment_target);
  printf("alignment components [t1,t2,n] deg: before=[%+.2f,%+.2f,%+.2f] | "
         "after=[%+.2f,%+.2f,%+.2f]\n",
         align_before_surface_deg(0), align_before_surface_deg(1),
         align_before_surface_deg(2), align_after_surface_deg(0),
         align_after_surface_deg(1), align_after_surface_deg(2));

  printSurfaceFrameBreakdown(R_alignment_target, r);

  if (!params.print_coupled_diagnostics) {
    return;
  }

  // Report only the deliberately commanded centre of compliance. No motion-
  // inferred axis or gain is used to construct the controller.
  const Mat6x6 K_pole = blockDiagonal(r.Kp, r.KR);
  const Mat6x6 D_pole = blockDiagonal(r.Dp, r.DR);
  const Vec3 edge_ref = params.coupled_pole_freeze_at_contact ? r.first_contact_point
                                                              : r.tool_contact_point;
  const Vec3 tcp_ref = params.coupled_pole_freeze_at_contact ? r.first_contact_tcp
                                                             : r.p_EE;
  Vec3 r_c;
  if (params.coupled_use_direct_rc_surface) {
    r_c = R_alignment_target * params.coupled_rc_surface;
  } else {
    r_c = tcp_ref - (edge_ref + params.coupled_pole_from_edge);
  }
  const Vec3 pole = tcp_ref - r_c;
  const Mat6x6 K_tcp = adjointTransformedGain(K_pole, r_c);
  const Mat6x6 D_tcp = adjointTransformedGain(D_pole, r_c);

  printf("\n=== Commanded centre of compliance ===\n");
  printf("lever convention: r_c = p_TCP - p_c\n");
  printf("parameterization: %s\n",
         params.coupled_use_direct_rc_surface
             ? "direct [tangent1,tangent2,normal]"
             : "legacy base-frame pole-from-edge");
  printVec3Mm("  r_c base", r_c);
  printVec3Mm("  r_c [t1,t2,n]", R_alignment_target.transpose() * r_c);
  printVec3Mm("  pole_from_edge", pole - edge_ref);

  printf("\nK_TCP = Ad^T K_pole Ad  [rows fx..mz | cols tx..rz]:\n");
  printSpatialGain6("  commanded", K_tcp);
  printSpatialGainEigenvalues("  K_TCP", K_tcp);

  printf("\nD_TCP = Ad^T D_pole Ad:\n");
  printSpatialGain6("  commanded", D_tcp);
  printSpatialGainEigenvalues("  D_TCP", D_tcp);
}
