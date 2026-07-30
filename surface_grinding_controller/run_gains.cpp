// ====================================================================
// Run gains
// ====================================================================
// Builds every stiffness and damping matrix a run commands, once, from the
// parameters alone. Surface-frame triples are rotated into the base frame
// here so the control loop only ever sees base-frame matrices.
#include "controller.h"

// ====================================================================
// Gain table
// ====================================================================
// Every stiffness and damping matrix a run commands, built once from the
// parameters. Surface-frame triples are rotated into the base frame here, so
// the control loop only ever sees base-frame matrices.
RunGains buildRunGains(const Parameters& params) {
  RunGains gains;
  gains.R_alignment_target = makeAlignmentTargetFrame(params);
  auto taskGain = [&](const Vec3& diagonal) -> Mat3 {
    return makeSpatialGainMatrix(diagonal, gains.R_alignment_target);
  };

  // Approach (orient + descend) share one impedance.
  gains.Kp_approach = taskGain(params.approach_Kp_diag);
  gains.Dp_approach = taskGain(params.approach_Dp_diag);
  gains.KR_approach = taskGain(params.approach_KR_diag);
  gains.DR_approach = taskGain(params.approach_DR_diag);
  // Set up + grind share one impedance.
  gains.setup_Kp_active_diag =
      params.setup_translation_surface_frame
          ? params.setup_Kp_surface_diag
          : params.setup_Kp_diag;
  gains.setup_Dp_active_diag =
      params.setup_translation_surface_frame
          ? params.setup_Dp_surface_diag
          : params.setup_Dp_diag;
  gains.Kp_setup =
      params.setup_translation_surface_frame
          ? taskGain(params.setup_Kp_surface_diag)
          : params.setup_Kp_diag.asDiagonal();
  gains.Dp_setup =
      params.setup_translation_surface_frame
          ? taskGain(params.setup_Dp_surface_diag)
          : params.setup_Dp_diag.asDiagonal();
  gains.KR_setup = taskGain(params.setup_KR_diag);
  gains.DR_setup = taskGain(params.setup_DR_diag);
  // Hold: isotropic in the base frame.
  gains.Kp_hold = params.hold_Kp_diag.asDiagonal();
  gains.Dp_hold = params.hold_Dp_diag.asDiagonal();
  gains.KR_hold = params.hold_KR_diag.asDiagonal();
  gains.DR_hold = params.hold_DR_diag.asDiagonal();
  // Stiff lock used only while paused at a gate.
  gains.Kp_pause = params.pause_hold_Kp_diag.asDiagonal();
  gains.Dp_pause = params.pause_hold_Dp_diag.asDiagonal();
  gains.KR_pause = taskGain(params.pause_hold_KR_diag);
  gains.DR_pause = taskGain(params.pause_hold_DR_diag);

  return gains;
}
