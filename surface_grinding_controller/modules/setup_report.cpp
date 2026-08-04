// ====================================================================
// Set-up report
// ====================================================================
// Printed once when the set-up phase ends: what the press achieved, resolved
// into the alignment-target frame, plus the alignment quality against the
// calibrated surface.
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

// Contact wrench and the motion it produced, in the alignment-target frame.
// Pairs force with TCP displacement and edge moment with tip angle, so the
// per-axis ratios come from the run rather than from a target.
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

  printSection("set-up surface-frame breakdown");
  printf("  %-16s   alignment-target [tangent1, tangent2, normal]\n",
         "frame");
  printf("  force        [N]      = [%+9.2f, %+9.2f, %+9.2f]\n",
         force_surf(0), force_surf(1), force_surf(2));
  printf("  tcp_disp     [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         1000.0 * tcp_disp_surf(0), 1000.0 * tcp_disp_surf(1), 1000.0 * tcp_disp_surf(2));
  printf("  edge_disp    [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         1000.0 * edge_disp_surf(0), 1000.0 * edge_disp_surf(1), 1000.0 * edge_disp_surf(2));
  printf("  Kp_eff=F/tcp [N/m]    = [%s, %s, %s]\n", kp[0], kp[1], kp[2]);
  printf("  moment@edge  [Nm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         moment_surf(0), moment_surf(1), moment_surf(2));
  printf("  tip_angle    [deg]    = [%+9.2f, %+9.2f, %+9.2f]\n",
         (180.0 / M_PI) * tip_surf(0), (180.0 / M_PI) * tip_surf(1),
         (180.0 / M_PI) * tip_surf(2));
  printf("  KR_eff=M/ang [Nm/rad] = [%s, %s, %s]\n", kr[0], kr[1], kr[2]);
}

}  // namespace

void reportSetUpResult(const Parameters& params,
                       const Mat3& R_alignment_target,
                       const SetUpReport& r) {
  const double actual_tip_deg =
      (180.0 / M_PI) * orientationError(r.R_EE, r.R_contact_start).norm();
  const Vec3 edge_from_contact_mm = 1000.0 * (r.tool_contact_point - r.first_contact_point);
  const Vec3 tcp_from_contact_mm = 1000.0 * (r.p_EE - r.first_contact_point);

  printBanner("SET-UP RESULT");
  printf("  stop: %s | t=%.1f s | tip=%.1f deg | F=%.1f N | M=%.1f Nm\n",
         r.stopped_on_moment ? "moment" : "time",
         r.phase_time, actual_tip_deg, r.force_delta_norm, r.moment_delta_norm);
  printf("  edge_from_contact = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
         edge_from_contact_mm(0), edge_from_contact_mm(1), edge_from_contact_mm(2),
         edge_from_contact_mm.norm());
  printf("  tcp_from_contact  = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
         tcp_from_contact_mm(0), tcp_from_contact_mm(1), tcp_from_contact_mm(2),
         tcp_from_contact_mm.norm());

  // Alignment quality against the calibrated surface. tip says how far the
  // tool turned; this says how flat it ended up -- the question that matters.
  const double align_before_deg =
      (180.0 / M_PI) *
      toolSurfaceMisalignmentAngle(params, r.R_contact_start, R_alignment_target);
  const double align_after_deg =
      (180.0 / M_PI) *
      toolSurfaceMisalignmentAngle(params, r.R_EE, R_alignment_target);
  printf("  alignment: before=%.2f deg | after=%.2f deg | gain=%+.2f deg\n",
         align_before_deg, align_after_deg, align_before_deg - align_after_deg);
  const Vec3 align_before_surface_deg =
      (180.0 / M_PI) * R_alignment_target.transpose() *
      toolSurfaceAlignmentErrorInBase(
          params, r.R_contact_start, R_alignment_target);
  const Vec3 align_after_surface_deg =
      (180.0 / M_PI) * R_alignment_target.transpose() *
      toolSurfaceAlignmentErrorInBase(params, r.R_EE, R_alignment_target);
  printf("  alignment components [t1,t2,n] deg: before=[%+.2f,%+.2f,%+.2f] | "
         "after=[%+.2f,%+.2f,%+.2f]\n",
         align_before_surface_deg(0), align_before_surface_deg(1),
         align_before_surface_deg(2), align_after_surface_deg(0),
         align_after_surface_deg(1), align_after_surface_deg(2));

  printSurfaceFrameBreakdown(R_alignment_target, r);

  // A decoupled set-up does not command a centre of compliance. Do not print
  // the inactive legacy pole parameters as though they affected the run.
  if (!params.print_coupled_diagnostics || !params.use_coupled_stiffness) {
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
  if (params.coupled_use_pole_ee) {
    // The pole rode with the tool, so the lever is resolved at the orientation
    // the press ended in, which is the one the spring last commanded.
    r_c = -(r.R_EE * params.coupled_pole_ee);
  } else if (params.coupled_use_direct_rc_surface) {
    r_c = R_alignment_target * params.coupled_rc_surface;
  } else {
    r_c = tcp_ref - (edge_ref + params.coupled_pole_from_edge);
  }
  const Vec3 pole = tcp_ref - r_c;
  const Mat6x6 K_tcp = adjointTransformedGain(K_pole, r_c);
  const Mat6x6 D_tcp = adjointTransformedGain(D_pole, r_c);

  // r_c = p_TCP - p_c, the lever the 6x6 spring is built about. The frame it
  // was commanded in is the one worth reading; the other is the same vector.
  const Vec3 r_c_surface = R_alignment_target.transpose() * r_c;
  // The same lever on the tool. A pole commanded in the surface frame does not
  // sit still in the tool: as the tilt changes, so do these numbers, which is
  // what makes them the ones to report -- they say where on the tool the pivot
  // was, in the frame the tool geometry is measured in.
  const Mat3& R_pole_ref =
      (params.coupled_use_pole_ee || !params.coupled_pole_freeze_at_contact)
          ? r.R_EE
          : r.R_contact_start;
  const Vec3 r_c_ee = R_pole_ref.transpose() * r_c;
  // p_c - p_EE = -r_c, and the grinding face centre is at
  // tool_contact_face_center_ee from p_EE, so this is the pivot measured from
  // the face the tool grinds with.
  const Vec3 pole_from_face_ee = -r_c_ee - params.tool_contact_face_center_ee;
  printSection("centre of compliance");
  printf("  %-16s   r_c = p_TCP - p_c\n", "definition");
  if (params.coupled_use_pole_ee) {
    printVec3Mm("p_c [EE]", params.coupled_pole_ee);
    printVec3Mm("r_c [t1,t2,n]", r_c_surface);
  } else if (params.coupled_use_direct_rc_surface) {
    printVec3Mm("r_c [t1,t2,n]", r_c_surface);
  } else {
    printVec3Mm("r_c [x,y,z]", r_c);
    printVec3Mm("pole_from_edge", pole - edge_ref);
  }
  printVec3Mm("r_c [EE]", r_c_ee);
  printVec3Mm("p_c from face [EE]", pole_from_face_ee);
  printf("  %-16s   EE rows resolved at %s\n", "",
         (params.coupled_use_pole_ee || !params.coupled_pole_freeze_at_contact)
             ? "the end of set up"
             : "first contact");

  // What the TCP actually feels after the shift. Translation is unchanged by
  // the congruence; rotation picks up skew(r_c)^T Kp skew(r_c), so those
  // entries are larger than the commanded KR.
  const Vec3 k_trans = K_tcp.block<3, 3>(0, 0).diagonal();
  const Vec3 k_rot = K_tcp.block<3, 3>(3, 3).diagonal();
  const Vec3 d_trans = D_tcp.block<3, 3>(0, 0).diagonal();
  const Vec3 d_rot = D_tcp.block<3, 3>(3, 3).diagonal();
  printSection("what the TCP feels");
  printRow("K_TCP translation", k_trans, "N/m", "");
  printRow("K_TCP rotation", k_rot, "Nm/rad", "");
  printRow("commanded KR was", Vec3(r.KR.diagonal()), "Nm/rad",
           "the rest is the lever");
  printRow("D_TCP translation", d_trans, "Ns/m", "");
  printRow("D_TCP rotation", d_rot, "Nms/rad", "");

  // The lever turns force into moment: the largest off-diagonal entry says
  // how strongly translation and rotation are coupled.
  const double k_coupling = K_tcp.block<3, 3>(3, 0).cwiseAbs().maxCoeff();
  const double d_coupling = D_tcp.block<3, 3>(3, 0).cwiseAbs().maxCoeff();
  printf("  %-16s   max |moment per unit translation| = %.1f Nm/m (K), "
         "%.1f Nms/m (D)\n", "coupling", k_coupling, d_coupling);

  const Eigen::SelfAdjointEigenSolver<Mat6x6> eig_k(K_tcp);
  const Eigen::SelfAdjointEigenSolver<Mat6x6> eig_d(D_tcp);
  printf("  %-16s   K_TCP %.2f .. %.0f | D_TCP %.2f .. %.0f%s\n",
         "eigenvalues",
         eig_k.eigenvalues().minCoeff(), eig_k.eigenvalues().maxCoeff(),
         eig_d.eigenvalues().minCoeff(), eig_d.eigenvalues().maxCoeff(),
         (eig_k.eigenvalues().minCoeff() > 0.0 &&
          eig_d.eigenvalues().minCoeff() >= -1e-8)
             ? "  (both positive: valid springs)"
             : "  *** NOT POSITIVE DEFINITE ***");

  // The full spring, off-diagonals included: the upper-right and lower-left
  // quadrants are the lever coupling that the diagonals above cannot show.
  printf("\n");
  printSpatialGain6("  K_TCP [N/m, Nm/rad | rows fx..mz, cols tx..rz]", K_tcp);
  printf("\n");
  printSpatialGain6("  D_TCP [Ns/m, Nms/rad]", D_tcp);
}
