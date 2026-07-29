#include "controller.h"

// Offline preflight for the experiment overlays. It reads parameters and
// constructs the exact gain matrices used by the controller, but never opens a
// robot connection or commands motion.

int main(int argc, char** argv) {
  const std::string params_dir = (argc > 1) ? argv[1] : "params";
  const std::string prefix =
      (!params_dir.empty() && params_dir.back() == '/')
          ? params_dir
          : params_dir + "/";

  try {
    const Parameters params = readParameters({
        prefix + "common.txt",
        prefix + "safety.txt",
        prefix + "sequence.txt",
        prefix + "hold.txt",
        prefix + "guidance.txt",
    });
    const Mat3 R_surface = makeAlignmentTargetFrame(params);
    const Mat3 Kp =
        params.setup_translation_surface_frame
            ? makeSpatialGainMatrix(params.setup_Kp_surface_diag, R_surface)
            : params.setup_Kp_diag.asDiagonal();
    const Mat3 Dp =
        params.setup_translation_surface_frame
            ? makeSpatialGainMatrix(params.setup_Dp_surface_diag, R_surface)
            : params.setup_Dp_diag.asDiagonal();
    const Mat3 KR = makeSpatialGainMatrix(params.setup_KR_diag, R_surface);
    const Mat3 DR = makeSpatialGainMatrix(params.setup_DR_diag, R_surface);

    const Vec3 kp_diag =
        params.setup_translation_surface_frame
            ? params.setup_Kp_surface_diag
            : params.setup_Kp_diag;
    if (!kp_diag.allFinite() || !params.setup_KR_diag.allFinite() ||
        kp_diag.minCoeff() <= 0.0 || params.setup_KR_diag.minCoeff() <= 0.0) {
      fprintf(stderr, "ERROR: all stiffness entries must be finite and positive.\n");
      return 2;
    }

    printf("experiment preflight (no robot connection)\n");
    printf("q_init_case = %s\n", params.q_init_case.c_str());
    printf("surface t1 = [%+.6f, %+.6f, %+.6f]\n",
           R_surface(0, 0), R_surface(1, 0), R_surface(2, 0));
    printf("surface t2 = [%+.6f, %+.6f, %+.6f]\n",
           R_surface(0, 1), R_surface(1, 1), R_surface(2, 1));
    printf("surface normal = [%+.6f, %+.6f, %+.6f]\n",
           R_surface(0, 2), R_surface(1, 2), R_surface(2, 2));
    if ((R_surface.transpose() * R_surface - Mat3::Identity()).norm() > 1e-9 ||
        std::abs(R_surface.determinant() - 1.0) > 1e-9) {
      fprintf(stderr, "ERROR: surface frame is not right-handed and orthonormal.\n");
      return 2;
    }
    printf("tool offsets [t1,t2] = [%+.1f, %+.1f] deg\n",
           params.tool_target_offset_tangent1_deg,
           params.tool_target_offset_tangent2_deg);
    printf("Kp frame = %s\n",
           params.setup_translation_surface_frame ? "surface [t1,t2,n]"
                                                  : "base [x,y,z]");
    printf("Kp diagonal in selected frame = [%.1f, %.1f, %.1f] N/m\n",
           kp_diag(0), kp_diag(1), kp_diag(2));
    printf("KR surface diagonal = [%.1f, %.1f, %.1f] Nm/rad\n",
           params.setup_KR_diag(0),
           params.setup_KR_diag(1),
           params.setup_KR_diag(2));

    if (params.use_coupled_stiffness &&
        !params.coupled_use_block_diagonal) {
      if (!params.coupled_pole_manual ||
          !params.coupled_use_direct_rc_surface) {
        fprintf(stderr,
                "ERROR: new coupled experiments require a deliberately "
                "commanded direct surface-frame r_c.\n");
        return 2;
      }
      const Vec3 r_c = R_surface * params.coupled_rc_surface;
      const Mat6x6 K_tcp =
          adjointTransformedGain(blockDiagonal(Kp, KR), r_c);
      const Mat6x6 D_tcp =
          adjointTransformedGain(blockDiagonal(Dp, DR), r_c);
      Eigen::SelfAdjointEigenSolver<Mat6x6> eig_K(K_tcp);
      Eigen::SelfAdjointEigenSolver<Mat6x6> eig_D(D_tcp);
      if (eig_K.info() != Eigen::Success ||
          eig_D.info() != Eigen::Success ||
          !K_tcp.allFinite() || !D_tcp.allFinite() ||
          (K_tcp - K_tcp.transpose()).norm() > 1e-8 ||
          (D_tcp - D_tcp.transpose()).norm() > 1e-8 ||
          eig_K.eigenvalues().minCoeff() <= 0.0 ||
          eig_D.eigenvalues().minCoeff() < -1e-8) {
        fprintf(stderr, "ERROR: transformed K/D failed the finite/symmetry/eigenvalue check.\n");
        return 2;
      }
      // With e_R=0, the adopted convention must produce
      // m = -r_c x f. Checking the whole lower-left block catches a lever-sign
      // inversion independently of the positive-definiteness test above.
      const double coupling_sign_residual =
          (K_tcp.block<3, 3>(3, 0) + skewMatrix(r_c) * Kp).norm();
      if (coupling_sign_residual > 1e-8) {
        fprintf(stderr,
                "ERROR: point-shift coupling has the wrong r_c sign "
                "(residual %.3e).\n",
                coupling_sign_residual);
        return 2;
      }
      printf("r_c=p_TCP-p_c [t1,t2,n] = [%+.1f, %+.1f, %+.1f] mm\n",
             1000.0 * params.coupled_rc_surface(0),
             1000.0 * params.coupled_rc_surface(1),
             1000.0 * params.coupled_rc_surface(2));
      printf("K_TCP eigenvalue range = [%.6f, %.6f]\n",
             eig_K.eigenvalues().minCoeff(),
             eig_K.eigenvalues().maxCoeff());
      printf("D_TCP eigenvalue range = [%.6f, %.6f]\n",
             eig_D.eigenvalues().minCoeff(),
             eig_D.eigenvalues().maxCoeff());
      printf("coupling sign check m=-r_c x f: PASS\n");
    } else {
      printf("coupled set-up = %s\n",
             params.use_coupled_stiffness ? "block diagonal" : "off");
    }

    printf("preflight: PASS\n");
    return 0;
  } catch (const std::exception& e) {
    fprintf(stderr, "preflight exception: %s\n", e.what());
    return 2;
  }
}
