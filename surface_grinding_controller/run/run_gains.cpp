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

// ====================================================================
// Auto damping
// ====================================================================

DampingCache manualDampingCache(const RunGains& gains) {
  DampingCache damping;
  damping.Dp_approach = gains.Dp_approach;
  damping.DR_approach = gains.DR_approach;
  damping.Dp_setup = gains.Dp_setup;
  damping.DR_setup = gains.DR_setup;
  damping.Dp_hold = gains.Dp_hold;
  damping.DR_hold = gains.DR_hold;
  damping.Dp_pause = gains.Dp_pause;
  damping.DR_pause = gains.DR_pause;
  return damping;
}

void updateAutoDamping(const Parameters& params,
                       const RunGains& gains,
                       const Model& model,
                       const RobotState& state,
                       const Mat6x7& J,
                       ControlPhase phase,
                       bool after_contact,
                       bool pause_hold_active,
                       DampingCache& damping) {
  // The menu's t mode holds with the set-up impedance, so its damping comes
  // from the set-up branch below rather than the hold one.
  const bool hold_as_setup =
      phase == ControlPhase::kHold && params.hold_with_setup_gains;
  const bool use_setup_branch = after_contact || hold_as_setup;
  // ---------------------------------------------------------------
  // Auto-damping: compute once per phase group and cache.
  // ---------------------------------------------------------------
  const bool in_approach = (phase == ControlPhase::kApproachOrient ||
                            phase == ControlPhase::kApproachDescend);
  if (!in_approach) {
    damping.approach_computed = false;
  }
  if (!use_setup_branch) {
    damping.setup_computed = false;
  }
  if (phase != ControlPhase::kHold || hold_as_setup) {
    damping.hold_computed = false;
  }
  if (!pause_hold_active) {
    damping.pause_computed = false;
  }

  const bool need_damping_update =
      (pause_hold_active && params.pause_hold_auto_damping &&
       !damping.pause_computed) ||
      (in_approach && params.approach_auto_damping && !damping.approach_computed) ||
      (use_setup_branch && params.setup_auto_damping &&
       !damping.setup_computed) ||
      (phase == ControlPhase::kHold && !hold_as_setup &&
       params.hold_auto_damping && !damping.hold_computed);
  if (need_damping_update) {
    std::array<double, 49> mass_array = model.mass(state);
    Map<const Mat7x7> joint_mass(mass_array.data());

    // Manual damping can be either fallback-only or a per-axis floor.
    auto dampingFloor = [&](const Vec3& manual) {
      return params.auto_damping_min_from_manual ? manual : Vec3::Zero();
    };
    // Print auto/manual damping for tuning.
    auto reportDamping = [&](const char* label, const char* unit,
                             const Vec3& computed, const Vec3& manual) {
      if (!params.print_auto_damping) {
        return;
      }
      printf("%-18s auto=[%7.2f, %7.2f, %7.2f]  manual=[%7.2f, %7.2f, %7.2f] %s%s\n",
             label,
             computed(0), computed(1), computed(2),
             manual(0), manual(1), manual(2), unit,
             params.auto_damping_min_from_manual ? " (manual = floor)" : "");
    };

    if (pause_hold_active) {
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      const CartesianInertiaEstimate inertia_task =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_alignment_target);
      if (inertia_base.valid && inertia_task.valid) {
        const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
            inertia_base.translational,
            params.pause_hold_Kp_diag,
            1.0, Vec3::Zero(), params.auto_damping_max);
        const Vec3 unit_critical_DR = criticalDampingFromStiffness(
            inertia_task.rotational,
            params.pause_hold_KR_diag,
            1.0, Vec3::Zero(), params.auto_damping_max);

        const Vec3& target_Dp = params.pause_hold_Dp_diag;
        const Vec3& target_DR = params.pause_hold_DR_diag;
        const auto fittedFactor = [](const Vec3& unit_critical,
                                     const Vec3& target) {
          const double denominator = unit_critical.squaredNorm();
          return (denominator > 1e-12)
                     ? unit_critical.dot(target) / denominator
                     : 1.0;
        };
        const double Dp_factor = fittedFactor(unit_critical_Dp, target_Dp);
        const double DR_factor = fittedFactor(unit_critical_DR, target_DR);

        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_base.translational,
            params.pause_hold_Kp_diag,
            Dp_factor, dampingFloor(target_Dp), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_task.rotational,
            params.pause_hold_KR_diag,
            DR_factor, dampingFloor(target_DR), params.auto_damping_max);
        damping.Dp_pause = Dp_diag.asDiagonal();
        damping.DR_pause =
            makeSpatialGainMatrix(DR_diag, gains.R_alignment_target);
        printf("pause auto damping: fitted Dp factor=%.3f, DR factor=%.3f\n",
               Dp_factor, DR_factor);
        reportDamping("pause Dp [xyz]", "Ns/m", Dp_diag, target_Dp);
        reportDamping("pause DR [t1t2n]", "Nms/rad", DR_diag, target_DR);
      } else {
        damping.Dp_pause = gains.Dp_pause;
        damping.DR_pause = gains.DR_pause;
        printf("pause damping: inertia estimate unavailable, using manual "
               "Dp=[%.1f, %.1f, %.1f] Ns/m and DR=[%.1f, %.1f, %.1f] "
               "Nms/rad\n",
               params.pause_hold_Dp_diag(0), params.pause_hold_Dp_diag(1),
               params.pause_hold_Dp_diag(2), params.pause_hold_DR_diag(0),
               params.pause_hold_DR_diag(1), params.pause_hold_DR_diag(2));
      }
      damping.pause_computed = true;
    } else if (in_approach) {
      const CartesianInertiaEstimate inertia =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_alignment_target);
      if (inertia.valid) {
        const double zeta = params.approach_auto_damping_factor;
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia.translational, params.approach_Kp_diag, zeta,
            dampingFloor(params.approach_Dp_diag), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia.rotational, params.approach_KR_diag, zeta,
            dampingFloor(params.approach_DR_diag), params.auto_damping_max);
        damping.Dp_approach = makeSpatialGainMatrix(Dp_diag, gains.R_alignment_target);
        damping.DR_approach = makeSpatialGainMatrix(DR_diag, gains.R_alignment_target);
        reportDamping("approach Dp", "Ns/m", Dp_diag, params.approach_Dp_diag);
        reportDamping("approach DR", "Nms/rad", DR_diag, params.approach_DR_diag);
        damping.approach_computed = true;
      }
    } else if (use_setup_branch) {
      // Translation uses its selected parameter frame; rotation uses the
      // surface frame.
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      const CartesianInertiaEstimate inertia_task =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_alignment_target);
      const CartesianInertiaEstimate& inertia_translation =
          params.setup_translation_surface_frame ? inertia_task : inertia_base;
      if (inertia_translation.valid && inertia_task.valid) {
        const double zeta = params.setup_auto_damping_factor;
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_translation.translational, gains.setup_Kp_active_diag, zeta,
            dampingFloor(gains.setup_Dp_active_diag), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_task.rotational, params.setup_KR_diag, zeta,
            dampingFloor(params.setup_DR_diag), params.auto_damping_max);
        damping.Dp_setup =
            params.setup_translation_surface_frame
                ? makeSpatialGainMatrix(Dp_diag, gains.R_alignment_target)
                : Dp_diag.asDiagonal();
        damping.DR_setup = makeSpatialGainMatrix(DR_diag, gains.R_alignment_target);
        reportDamping(
            params.setup_translation_surface_frame
                ? "set_up Dp [t1t2n]"
                : "set_up Dp [xyz]",
            "Ns/m", Dp_diag, gains.setup_Dp_active_diag);
        reportDamping("set_up DR [t1t2n]", "Nms/rad", DR_diag, params.setup_DR_diag);
        damping.setup_computed = true;
      }
    } else {
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      if (inertia_base.valid) {
        const Vec3& manual_hold_Dp = params.hold_Dp_diag;
        const Vec3& manual_hold_DR = params.hold_DR_diag;
        double hold_factor = params.hold_auto_damping_factor;
        if (params.hold_auto_match_manual_damping) {
          const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
              inertia_base.translational,
              params.hold_Kp_diag,
              1.0, Vec3::Zero(), params.auto_damping_max);
          const double denominator = unit_critical_Dp.squaredNorm();
          if (denominator > 1e-12) {
            // Least-squares scalar that makes factor*unit_critical_Dp as
            // close as possible to the manual hold_Dp axes.
            hold_factor =
                unit_critical_Dp.dot(manual_hold_Dp) / denominator;
          }
          printf("hold auto damping: fitted factor=%.3f toward "
                 "Dp=[%.1f, %.1f, %.1f] Ns/m\n",
                 hold_factor, manual_hold_Dp(0), manual_hold_Dp(1),
                 manual_hold_Dp(2));
        }
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_base.translational,
            params.hold_Kp_diag,
            hold_factor,
            dampingFloor(manual_hold_Dp),
            params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_base.rotational,
            params.hold_KR_diag,
            hold_factor,
            dampingFloor(manual_hold_DR),
            params.auto_damping_max);
        damping.Dp_hold = Dp_diag.asDiagonal();
        damping.DR_hold = DR_diag.asDiagonal();
        reportDamping("hold Dp", "Ns/m", Dp_diag, manual_hold_Dp);
        reportDamping("hold DR", "Nms/rad", DR_diag, manual_hold_DR);
      } else {
        damping.Dp_hold = gains.Dp_hold;
        damping.DR_hold = gains.DR_hold;
        printf("hold damping: inertia estimate unavailable, using manual "
               "hold_Dp=[%.1f, %.1f, %.1f] Ns/m and "
               "hold_DR=[%.1f, %.1f, %.1f] Nms/rad\n",
               params.hold_Dp_diag(0), params.hold_Dp_diag(1),
               params.hold_Dp_diag(2), params.hold_DR_diag(0),
               params.hold_DR_diag(1), params.hold_DR_diag(2));
      }
      damping.hold_computed = true;
    }
  }
}
