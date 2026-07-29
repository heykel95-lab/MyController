// Self-alignment / grinding controller for the Franka arm.
//
// Runs the approach -> set up -> grind sequence, or a startup-selected hold.
#include "controller.h"

int main() {
  try {
    // ================================================================
    // 1. Parameters, robot connection, start pose
    // ================================================================
    // Later parameter files override earlier ones on duplicate keys.
    Parameters params = readParameters({
        "params/common.txt",
        "params/safety.txt",
        "params/sequence.txt",
        "params/hold.txt",
        "params/guidance.txt",
    });
    Robot robot(params.robot_ip);

    printf("\nPress Enter to recover and configure the robot.\n");
    std::string enter_line;
    std::getline(std::cin, enter_line);

    try {
      robot.automaticErrorRecovery();
      printf("Robot recovered or already ready.\n");
    } catch (const franka::Exception& e) {
      fprintf(stderr, "Automatic error recovery failed: %s\n", e.what());
      fprintf(stderr, "Please recover/unlock the robot manually in Franka Desk.\n");
      return -1;
    }

    configureCollisionBehavior(robot, params);
    askStartupRunMode(params, robot);
    printParameters(params);

    // Saved coupled matrices are needed only when they are the selected source.
    const bool needs_saved_coupled_gains =
        params.use_phase_sequence &&
        params.use_coupled_stiffness && !params.coupled_use_block_diagonal &&
        !params.coupled_pole_manual;
    if (needs_saved_coupled_gains && !params.coupled_gains_saved) {
      fprintf(stderr, "Coupled stiffness needs saved K_TCP/D_TCP, but none was found.\n");
      fprintf(stderr, "Run once with use_coupled_stiffness = 0 so the set-up report can save them.\n");
      return -1;
    }

    if (!performStartupGripperAction(params)) {
      return -1;
    }

    Model model = robot.loadModel();

    std::atomic<bool> stop_requested(false);
    std::atomic<bool> proceed_requested(false);
    std::atomic<bool> guide_requested(false);
    std::atomic<char> guidance_menu_key(0);
    std::atomic<bool> guided_hold_selector_pending(
        params.use_manual_guidance_start);
    std::atomic<bool> gate_continue(false);
    startKeyboardStopThread(params, stop_requested, proceed_requested,
                            guide_requested, guidance_menu_key,
                            guided_hold_selector_pending, gate_continue);

    if (params.use_manual_guidance_start &&
        !runManualGuidanceStart(params, robot, model, stop_requested,
                                guidance_menu_key,
                                guided_hold_selector_pending)) {
      return 0;
    }

    // ================================================================
    // 2. Task frames, gains and run state
    // ================================================================
    RobotState initial_state = robot.readOnce();
    Map<const Mat4x4> T_initial(initial_state.O_T_EE.data());
    Vec3 p_start = T_initial.block<3, 1>(0, 3);
    Mat3 R_d = T_initial.block<3, 3>(0, 0);
    Vec7 q_start = Map<const Vec7>(initial_state.q.data());

    // Baseline external wrench: contact detection and the set-up report both
    // measure changes against this, not against absolute readings.
    Map<const Vec6> initial_external_wrench(initial_state.O_F_ext_hat_K.data());
    Vec3 contact_force_bias = initial_external_wrench.head<3>();
    Vec3 contact_moment_bias = initial_external_wrench.tail<3>();

    // Surface frame columns: [tangent1, tangent2, normal].
    const Mat3 R_alignment_target = makeAlignmentTargetFrame(params);
    // Mutable: recomputed from the re-captured pose when the sequence is
    // restarted through manual re-guidance.
    Mat3 R_d_alignment_target =
        makeToolOrientationForAlignmentTarget(params, R_alignment_target, R_d);
    Vec3 surface_point_runtime =
        params.use_start_as_surface_point ? p_start : params.surface_point;

    const Vec3 descend_direction = -R_alignment_target.col(2);

    // Four corners of the rectangular grinding face. Selecting every corner
    // tied for the greatest projection along the descend direction gives the
    // center of a leading face (4-way tie), edge (2-way tie), or corner.
    const Vec3& face_center = params.tool_contact_face_center_ee;
    const Vec3& half_width = params.tool_contact_half_width_ee;
    const Vec3& half_length = params.tool_contact_half_length_ee;
    const std::array<Vec3, 4> tool_face_corners_ee = {{
        face_center + half_width + half_length,
        face_center + half_width - half_length,
        face_center - half_width + half_length,
        face_center - half_width - half_length,
    }};
    const auto selectLeadingToolContact =
        [&](const Mat3& R_EE, double& leading_projection_out) -> Vec3 {
      std::array<double, 4> projection;
      double leading_projection =
          (R_EE * tool_face_corners_ee[0]).dot(descend_direction);
      projection[0] = leading_projection;
      for (std::size_t i = 1; i < tool_face_corners_ee.size(); ++i) {
        projection[i] = (R_EE * tool_face_corners_ee[i]).dot(descend_direction);
        leading_projection = std::max(leading_projection, projection[i]);
      }
      leading_projection_out = leading_projection;

      // Exact single-axis tilts produce equal projections at both endpoints of
      // the leading edge. Average tied points so the controlled point is the
      // edge center rather than an arbitrary corner.
      Vec3 selected = Vec3::Zero();
      int selected_count = 0;
      for (std::size_t i = 0; i < tool_face_corners_ee.size(); ++i) {
        if (leading_projection - projection[i] <=
            params.tool_contact_feature_tie_tolerance) {
          selected += tool_face_corners_ee[i];
          ++selected_count;
        }
      }
      return selected / static_cast<double>(selected_count);
    };

    // ---- gain sets, one per phase group ----
    auto taskGain = [&](const Vec3& diagonal) -> Mat3 {
      return makeSpatialGainMatrix(diagonal, R_alignment_target);
    };
    // Approach (orient + descend) share one impedance.
    const Mat3 Kp_approach = taskGain(params.approach_Kp_diag);
    const Mat3 Dp_approach = taskGain(params.approach_Dp_diag);
    const Mat3 KR_approach = taskGain(params.approach_KR_diag);
    const Mat3 DR_approach = taskGain(params.approach_DR_diag);
    // Set up + grind share one impedance.
    const Mat3 Kp_setup = params.setup_Kp_diag.asDiagonal();
    const Mat3 Dp_setup = params.setup_Dp_diag.asDiagonal();
    const Mat3 KR_setup = taskGain(params.setup_KR_diag);
    const Mat3 DR_setup = taskGain(params.setup_DR_diag);
    // Hold: isotropic in the base frame.
    const Mat3 Kp_hold = Mat3::Identity() * params.hold_Kp;
    const Mat3 Dp_hold = Mat3::Identity() * params.hold_Dp;
    const Mat3 KR_hold = Mat3::Identity() * params.hold_KR;
    const Mat3 DR_hold = Mat3::Identity() * params.hold_DR;
    // Stiff position lock used only while paused at a gate.
    const Mat3 Kp_pause = Mat3::Identity() * params.pause_hold_Kp;
    const Mat3 Dp_pause = Mat3::Identity() * params.pause_hold_Dp;

    // ---- phase machine ----
    const ControlPhase initial_phase =
        !params.use_phase_sequence  ? ControlPhase::kHold
        : params.use_approach_orient ? ControlPhase::kApproachOrient
                                     : ControlPhase::kApproachDescend;
    const bool sigma_hold_diagnostics_enabled =
        initial_phase == ControlPhase::kHold &&
        (params.nullspace_mode == NullspaceMode::kSigmaOnly ||
         params.nullspace_mode == NullspaceMode::kDampingAndSigma);
    ControlPhase phase = initial_phase;
    double phase_start_time = 0.0;
    double next_debug_time = 0.0;
    double next_sigma_debug_time = 0.0;
    double last_sigma_debug_time = 0.0;
    double last_sigma_debug_value = 0.0;
    bool last_sigma_debug_valid = false;
    bool descend_failed = false;

    // Contact reference, captured when the descend step reaches the clearance
    // height and held for the rest of the run.
    Vec3 first_contact_tcp = p_start;
    Vec3 first_contact_point = p_start;
    Mat3 R_contact_start = R_d_alignment_target;
    Vec3 active_tool_contact_offset_ee = params.tool_contact_face_center_ee;

    // Gate state. Each gate arms once, then blocks until a bare Enter.
    bool gate_set_up_armed = false;
    bool gate_set_up_passed = false;
    Vec3 gate_set_up_hold_pd = Vec3::Zero();
    double gate_paused_time = 0.0;  // frozen descend clock while gated
    bool gate_grind_armed = false;
    bool gate_grind_passed = false;

    // The preload frozen when the set-up phase ends; grind presses with it.
    double setup_push_start = -params.descend_surface_clearance;
    double grind_push = 0.0;

    // Auto-damping is cached per phase group.
    bool approach_damp_computed = false;
    Mat3 Dp_approach_cached = Dp_approach;
    Mat3 DR_approach_cached = DR_approach;
    bool setup_damp_computed = false;
    Mat3 Dp_setup_cached = Dp_setup;
    Mat3 DR_setup_cached = DR_setup;
    bool hold_damp_computed = false;
    Mat3 Dp_hold_cached = Dp_hold;
    Mat3 DR_hold_cached = DR_hold;
    bool pause_damp_computed = false;
    Mat3 Dp_pause_cached = Dp_pause;
    Mat3 DR_pause_cached = DR_approach;

    std::vector<std::pair<std::string, std::string>> pending_parameter_updates;
    pending_parameter_updates.reserve(96);

    printf("\n=== Start pose ===\n");
    printVec7Deg("q_start", q_start);
    printVec3Mm("p_start", p_start);

    // Bounded log buffer, pre-sized so the realtime callback never allocates.
    const int log_every_n_cycles = std::max(1, params.log_every_n_cycles);
    const std::size_t max_log_rows = static_cast<std::size_t>(std::max(0, params.max_log_rows));
    std::vector<LogData> log_data(max_log_rows);
    std::size_t control_cycle_count = 0;
    std::size_t log_write_index = 0;
    std::size_t log_rows_written = 0;
    bool log_buffer_wrapped = false;

    // Independent compact logger for sigma-enabled hold tuning. It is sampled
    // much more slowly than the full CSV and retains explicit guide/recapture
    // events even when the full 1 kHz log wraps.
    const double sigma_debug_log_period =
        std::max(0.001, params.sigma_debug_log_period);
    const std::size_t max_sigma_debug_rows =
        static_cast<std::size_t>(
            std::max(0, params.max_sigma_debug_rows));
    std::vector<SigmaDebugRow> sigma_debug_data(max_sigma_debug_rows);
    std::size_t sigma_debug_write_index = 0;
    std::size_t sigma_debug_rows_written = 0;
    bool sigma_debug_buffer_wrapped = false;
    int sigma_debug_segment_id = 0;
    double next_sigma_debug_file_time = 0.0;

    double peak_sigma_nullspace_speed = 0.0;
    double peak_sigma_abs_speed_toward_better = 0.0;
    double min_sigma_speed_toward_better = 0.0;
    double max_sigma_speed_toward_better = 0.0;
    double peak_sigma_position_error = 0.0;
    double peak_sigma_rotation_error = 0.0;
    double peak_sigma_external_force_delta = 0.0;
    double peak_sigma_external_moment_delta = 0.0;
    double peak_sigma_external_joint_torque_delta = 0.0;

    Vec7 sigma_debug_external_joint_torque_bias = Vec7::Zero();
    bool sigma_debug_external_joint_torque_bias_valid = false;

    auto appendSigmaDebugRow = [&](const SigmaDebugRow& row) {
      if (max_sigma_debug_rows == 0) {
        return;
      }
      sigma_debug_data[sigma_debug_write_index] = row;
      sigma_debug_write_index =
          (sigma_debug_write_index + 1) % max_sigma_debug_rows;
      if (sigma_debug_rows_written < max_sigma_debug_rows) {
        ++sigma_debug_rows_written;
      } else {
        sigma_debug_buffer_wrapped = true;
      }
    };

    auto resetSigmaDebugPeaks = [&]() {
      peak_sigma_nullspace_speed = 0.0;
      peak_sigma_abs_speed_toward_better = 0.0;
      min_sigma_speed_toward_better = 0.0;
      max_sigma_speed_toward_better = 0.0;
      peak_sigma_position_error = 0.0;
      peak_sigma_rotation_error = 0.0;
      peak_sigma_external_force_delta = 0.0;
      peak_sigma_external_moment_delta = 0.0;
      peak_sigma_external_joint_torque_delta = 0.0;
    };

    bool sigma_debug_file_written = false;
    auto persistSigmaDebugBuffer = [&]() -> bool {
      if (sigma_debug_file_written ||
          params.sigma_debug_csv_file_name.empty() ||
          sigma_debug_rows_written == 0) {
        return sigma_debug_file_written;
      }

      try {
        std::vector<SigmaDebugRow> ordered_sigma_debug_data;
        ordered_sigma_debug_data.reserve(sigma_debug_rows_written);
        if (sigma_debug_buffer_wrapped) {
          ordered_sigma_debug_data.insert(
              ordered_sigma_debug_data.end(),
              sigma_debug_data.begin() +
                  static_cast<std::ptrdiff_t>(
                      sigma_debug_write_index),
              sigma_debug_data.end());
          ordered_sigma_debug_data.insert(
              ordered_sigma_debug_data.end(),
              sigma_debug_data.begin(),
              sigma_debug_data.begin() +
                  static_cast<std::ptrdiff_t>(
                      sigma_debug_write_index));
          printf("Sigma debug buffer wrapped: kept latest %zu rows at %.1f Hz.\n",
                 sigma_debug_rows_written,
                 1.0 / sigma_debug_log_period);
        } else {
          ordered_sigma_debug_data.insert(
              ordered_sigma_debug_data.end(),
              sigma_debug_data.begin(),
              sigma_debug_data.begin() +
                  static_cast<std::ptrdiff_t>(
                      sigma_debug_rows_written));
        }

        sigma_debug_file_written =
            writeSigmaDebugToCsv(
                ordered_sigma_debug_data,
                params.sigma_debug_csv_file_name);
        if (sigma_debug_file_written) {
          printf("sigma debug csv: %s (%zu rows)\n",
                 params.sigma_debug_csv_file_name.c_str(),
                 ordered_sigma_debug_data.size());
        }
      } catch (const std::exception& e) {
        fprintf(stderr, "Could not persist sigma debug buffer: %s\n",
                e.what());
      }
      return sigma_debug_file_written;
    };

    if (sigma_hold_diagnostics_enabled) {
      SigmaDebugRow hold_start_row;
      hold_start_row.segment_id = sigma_debug_segment_id;
      hold_start_row.event = SigmaDebugEvent::kHoldStart;
      hold_start_row.q = q_start;
      appendSigmaDebugRow(hold_start_row);
    }

    double time = 0.0;
    Vec3 final_p_EE = Vec3::Zero();
    Vec3 final_p_d = Vec3::Zero();
    Vec3 final_e_p = Vec3::Zero();
    Vec3 final_e_R = Vec3::Zero();
    Vec7 final_q = q_start;

    printf("\n=== Run ===\n");
    printf("phase: %s\n", phaseName(phase));

    // ================================================================
    // 3. Control loop: libfranka calls this back at ~1 kHz with the current
    //    state and expects the 7 commanded joint torques in return.
    // ================================================================
    try {
      robot.control([&](const RobotState& state, Duration period) -> Torques {
      time += period.toSec();

      Map<const Vec7> dq(state.dq.data());
      Map<const Vec7> q_current(state.q.data());
      Map<const Vec7> external_joint_torque(
          state.tau_ext_hat_filtered.data());
      const bool joint_contact =
          std::any_of(state.joint_contact.begin(), state.joint_contact.end(),
                      [](double value) { return value > 0.0; });
      const bool cartesian_contact =
          std::any_of(state.cartesian_contact.begin(),
                      state.cartesian_contact.end(),
                      [](double value) { return value > 0.0; });

      // kEndEffector Jacobian includes any configured F_T_EE tool offset.
      std::array<double, 42> jacobian_array = model.zeroJacobian(Frame::kEndEffector, state);
      Map<const Mat6x7> J(jacobian_array.data());
      const Vec6 xdot = J * dq;
      const Vec3 pdot = xdot.head<3>();
      const Vec3 omega = xdot.tail<3>();

      Map<const Mat4x4> T_EE(state.O_T_EE.data());
      const Vec3 p_EE = T_EE.block<3, 1>(0, 3);
      const Mat3 R_EE = T_EE.block<3, 3>(0, 0);

      Map<const Vec6> external_wrench(state.O_F_ext_hat_K.data());
      const Vec3 external_force = external_wrench.head<3>();
      const Vec3 external_moment = external_wrench.tail<3>();

      // ---------------------------------------------------------------
      // Manual re-guidance from hold.
      // ---------------------------------------------------------------
      if (phase == ControlPhase::kHold && guide_requested.load()) {
        guide_requested.store(false);
        proceed_requested.store(false);

        if (sigma_hold_diagnostics_enabled) {
          SigmaDebugRow guide_row;
          guide_row.run_time = time;
          guide_row.phase_time = time - phase_start_time;
          guide_row.segment_id = sigma_debug_segment_id;
          guide_row.event = SigmaDebugEvent::kManualGuideStart;
          guide_row.q = q_current;
          guide_row.dq = dq;
          guide_row.e_p = p_start - p_EE;
          guide_row.e_R = orientationError(R_EE, R_d);
          guide_row.pdot = pdot;
          guide_row.omega = omega;
          guide_row.external_force_delta =
              external_force - contact_force_bias;
          guide_row.external_moment_delta =
              external_moment - contact_moment_bias;
          guide_row.external_joint_torque_baseline_valid =
              sigma_debug_external_joint_torque_bias_valid;
          if (sigma_debug_external_joint_torque_bias_valid) {
            guide_row.external_joint_torque_delta =
                external_joint_torque -
                sigma_debug_external_joint_torque_bias;
          }
          guide_row.joint_contact = joint_contact;
          guide_row.cartesian_contact = cartesian_contact;
          appendSigmaDebugRow(guide_row);
        }

        phase = ControlPhase::kManualGuide;
        phase_start_time = time;
        printf("\nphase: manual_guide (move the tool by hand; p+Enter recaptures and resumes hold, e+Enter stops)\n");
      }
      if (phase == ControlPhase::kManualGuide) {
        Array7 coriolis_array = model.coriolis(state);
        Map<const Vec7> coriolis(coriolis_array.data());
        const Array7 tau_array =
            vec7ToArray(Vec7(coriolis - params.manual_guidance_damping * dq));
        if (stop_requested.load()) {
          if (sigma_hold_diagnostics_enabled) {
            SigmaDebugRow stop_row;
            stop_row.run_time = time;
            stop_row.phase_time = time - phase_start_time;
            stop_row.segment_id = sigma_debug_segment_id;
            stop_row.event = SigmaDebugEvent::kStop;
            stop_row.q = q_current;
            stop_row.dq = dq;
            stop_row.pdot = pdot;
            stop_row.omega = omega;
            stop_row.external_force_delta =
                external_force - contact_force_bias;
            stop_row.external_moment_delta =
                external_moment - contact_moment_bias;
            stop_row.external_joint_torque_baseline_valid =
                sigma_debug_external_joint_torque_bias_valid;
            if (sigma_debug_external_joint_torque_bias_valid) {
              stop_row.external_joint_torque_delta =
                  external_joint_torque -
                  sigma_debug_external_joint_torque_bias;
            }
            stop_row.joint_contact = joint_contact;
            stop_row.cartesian_contact = cartesian_contact;
            appendSigmaDebugRow(stop_row);
          }
          printf("\nStop requested with e + Enter. Finishing control loop...\n");
          return MotionFinished(Torques(tau_array));
        }
        if (proceed_requested.load()) {
          proceed_requested.store(false);
          // Re-capture the hand-moved pose as the new sequence start.
          p_start = p_EE;
          R_d = R_EE;
          q_start = q_current;
          R_d_alignment_target =
              makeToolOrientationForAlignmentTarget(params, R_alignment_target, R_d);
          surface_point_runtime =
              params.use_start_as_surface_point ? p_start : params.surface_point;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          first_contact_tcp = p_start;
          first_contact_point = p_start;
          R_contact_start = R_d_alignment_target;
          active_tool_contact_offset_ee = params.tool_contact_face_center_ee;

          phase = initial_phase;
          phase_start_time = time;
          ++sigma_debug_segment_id;
          next_sigma_debug_file_time = time;
          sigma_debug_external_joint_torque_bias = external_joint_torque;
          sigma_debug_external_joint_torque_bias_valid = true;
          resetSigmaDebugPeaks();
          next_debug_time = time;
          next_sigma_debug_time = time;
          last_sigma_debug_valid = false;
          gate_set_up_armed = false;
          gate_set_up_passed = false;
          gate_paused_time = 0.0;
          gate_grind_armed = false;
          gate_grind_passed = false;
          setup_push_start = -params.descend_surface_clearance;
          grind_push = 0.0;
          hold_damp_computed = false;
          Dp_hold_cached = Dp_hold;
          DR_hold_cached = DR_hold;
          pause_damp_computed = false;
          Dp_pause_cached = Dp_pause;
          DR_pause_cached = DR_approach;

          if (sigma_hold_diagnostics_enabled) {
            SigmaDebugRow recapture_row;
            recapture_row.run_time = time;
            recapture_row.phase_time = 0.0;
            recapture_row.segment_id = sigma_debug_segment_id;
            recapture_row.event = SigmaDebugEvent::kRecapture;
            recapture_row.q = q_current;
            recapture_row.dq = dq;
            recapture_row.pdot = pdot;
            recapture_row.omega = omega;
            recapture_row.external_joint_torque_baseline_valid = true;
            recapture_row.joint_contact = joint_contact;
            recapture_row.cartesian_contact = cartesian_contact;
            appendSigmaDebugRow(recapture_row);
          }

          printf("\n=== Resuming hold from re-guided pose ===\n");
          printVec7Deg("q_start", q_start);
          printVec3Mm("p_start", p_start);
          printf("phase: %s\n", phaseName(phase));
        }
        return Torques(tau_array);
      }

      // Use the pre-transition phase so the selected edge locks on contact.
      const bool edge_locked =
          (phase == ControlPhase::kSetUp || phase == ControlPhase::kGrind);

      // ---------------------------------------------------------------
      // Active tool contact point.
      // ---------------------------------------------------------------
      Vec3 tool_contact_offset_ee = Vec3::Zero();
      double leading_contact_projection = 0.0;
      if (params.use_tool_contact_point_control) {
        if (edge_locked) {
          tool_contact_offset_ee = active_tool_contact_offset_ee;
          leading_contact_projection =
              (R_EE * tool_contact_offset_ee).dot(descend_direction);
        } else if (params.auto_select_tool_contact_edge) {
          tool_contact_offset_ee =
              selectLeadingToolContact(R_EE, leading_contact_projection);
        } else {
          tool_contact_offset_ee = params.tool_contact_face_center_ee;
          leading_contact_projection =
              (R_EE * tool_contact_offset_ee).dot(descend_direction);
        }
      }
      const Vec3 tool_contact_point = p_EE + R_EE * tool_contact_offset_ee;

      DesiredMotion desired{p_start, Vec3::Zero()};
      Vec3 edge_target_log = first_contact_point;
      double push_log = 0.0;
      // Enables the stiff position lock only while a gate blocks.
      bool pause_hold_active = false;

      switch (phase) {
        // -------------------------------------------------------------
        // Phase 1a: orient the tool.
        // -------------------------------------------------------------
        case ControlPhase::kApproachOrient: {
          const Vec3 tool_axis_current = currentToolAxisInBase(params, R_EE).normalized();
          const Vec3 tool_axis_target =
              desiredToolAxisInBase(params, R_alignment_target).normalized();
          const double tool_axis_error = std::acos(
              std::max(-1.0, std::min(1.0, tool_axis_current.dot(tool_axis_target))));
          const double phase_time = time - phase_start_time;

          if (params.debug_period > 0.0 && time >= next_debug_time) {
            const Vec3 e_R_target = applyRotationalAxisMask(
                params, orientationError(R_EE, R_d_alignment_target), R_alignment_target);
            printApproachOrientDebug(phase_time,
                                     (180.0 / M_PI) * tool_axis_error,
                                     (180.0 / M_PI) * e_R_target.norm());
            next_debug_time = time + params.debug_period;
          }

          if (phase_time >= params.approach_orient_min_time &&
              tool_axis_error <= params.approach_orient_error_threshold) {
            phase = ControlPhase::kApproachDescend;
            phase_start_time = time;
            next_debug_time = time;
            contact_force_bias = external_force;
            contact_moment_bias = external_moment;
            printf("\nOrientation reached: axis_err=%.1f deg\n",
                   (180.0 / M_PI) * tool_axis_error);
            printf("phase: %s\n", phaseName(phase));
          }
          break;  // desired stays at p_start: rotate without moving the TCP
        }

        // -------------------------------------------------------------
        // Phase 1b: descend to the configured surface clearance.
        // -------------------------------------------------------------
        case ControlPhase::kApproachDescend: {
          const double phase_time = time - phase_start_time;
          const double distance =
              std::min(params.descend_speed * (phase_time - gate_paused_time),
                       params.descend_max_distance);
          desired.p_d = p_start + distance * descend_direction;
          desired.pdot_d = params.descend_speed * descend_direction;

          const Vec3 surface_normal = (-descend_direction).normalized();
          const double height_above_surface =
              surface_normal.dot(p_EE - surface_point_runtime) -
              leading_contact_projection;
          const double controlled_point_height =
              surface_normal.dot(tool_contact_point - surface_point_runtime);
          const Vec3 projected_surface_point =
              tool_contact_point - controlled_point_height * surface_normal;
          const bool clearance_reached =
              height_above_surface <= params.descend_surface_clearance;
          const double force_along_descend =
              (external_force - contact_force_bias).dot(descend_direction);

          // Gate: freeze the tool at the clearance height until a bare Enter.
          bool gate_blocking = false;
          if (params.pause_before_set_up && clearance_reached && !gate_set_up_passed) {
            if (!gate_set_up_armed) {
              gate_set_up_armed = true;
              // Hold the actual pose reached at the gate.
              gate_set_up_hold_pd = p_EE;
              gate_continue.store(false);
              printf("\n[GATE] Reached %.0f mm above the plane. Press Enter to start "
                     "the set-up press (e+Enter stops).\n",
                     1000.0 * params.descend_surface_clearance);
            }
            if (gate_continue.load()) {
              gate_set_up_passed = true;
              gate_continue.store(false);
              printf("[GATE] Continuing to the set-up press.\n");
            } else {
              gate_blocking = true;
              pause_hold_active = true;
              desired.p_d = gate_set_up_hold_pd;
              desired.pdot_d.setZero();
              // Freeze the descend clock so the commanded distance does not
              // creep to the max-distance failure while waiting at the gate.
              gate_paused_time += period.toSec();
            }
          }

          if (params.debug_period > 0.0 && time >= next_debug_time && !gate_blocking) {
            printApproachDescendDebug(phase_time,
                                      1000.0 * distance,
                                      1000.0 * height_above_surface,
                                      1000.0 * params.descend_surface_clearance,
                                      force_along_descend);
            next_debug_time = time + params.debug_period;
          }

          if (clearance_reached && !gate_blocking) {
            active_tool_contact_offset_ee = tool_contact_offset_ee;
            first_contact_tcp = p_EE;
            first_contact_point = projected_surface_point;
            // Signed plane coordinate: negative above the surface, positive
            // into it. Capture the actual controlled-point height so setup
            // begins continuously even if descend overshot the clearance.
            setup_push_start = -controlled_point_height;
            R_contact_start = R_EE;
            contact_force_bias = external_force;
            contact_moment_bias = external_moment;
            phase = ControlPhase::kSetUp;
            phase_start_time = time;
            next_debug_time = time;
            printf("\nClearance reached: distance=%.1f mm | height=%.1f mm | target=%.1f mm | force=%.1f N (not used for the switch)\n",
                   1000.0 * distance,
                   1000.0 * height_above_surface,
                   1000.0 * params.descend_surface_clearance,
                   force_along_descend);
            printContactEdgeDebug(active_tool_contact_offset_ee, first_contact_tcp,
                                  first_contact_point);
            printf("phase: %s\n", phaseName(phase));
          } else if (distance >= params.descend_max_distance) {
            descend_failed = true;
            stop_requested.store(true);
            desired.p_d = p_EE;
            desired.pdot_d.setZero();
          }
          break;
        }

        // -------------------------------------------------------------
        // Phase 2: press the contact edge while holding the contact orientation
        // as a soft rotational target.
        // -------------------------------------------------------------
        case ControlPhase::kSetUp: {
          const double phase_time = time - phase_start_time;
          double setup_push_velocity = 0.0;
          const double push =
              setUpPush(params, phase_time, setup_push_start,
                        setup_push_velocity);
          const Vec3 edge_target = first_contact_point + push * descend_direction;
          desired.p_d = edge_target - R_contact_start * tool_contact_offset_ee;
          desired.pdot_d = setup_push_velocity * descend_direction;
          edge_target_log = edge_target;
          push_log = push;

          const double force_delta_norm = (external_force - contact_force_bias).norm();
          const double moment_delta_norm = (external_moment - contact_moment_bias).norm();
          // Contact moment moved from the TCP out to the pressed edge:
          //   M_C = m - r_C x f,  r_C = p_edge - p_EE.
          const Vec3 edge_from_tcp = tool_contact_point - p_EE;
          const Vec3 contact_moment_at_edge =
              (external_moment - contact_moment_bias) -
              edge_from_tcp.cross(external_force - contact_force_bias);

          const bool waiting_at_gate = gate_grind_armed && !gate_grind_passed;
          if (params.debug_period > 0.0 && time >= next_debug_time && !waiting_at_gate) {
            // tip = passive rotation away from the contact orientation.
            printSetUpDebug(phase_time,
                            (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm(),
                            force_delta_norm,
                            moment_delta_norm,
                            params.setup_moment_threshold,
                            1000.0 * (tool_contact_point - first_contact_point).norm());
            next_debug_time = time + params.debug_period;
          }

          // End on contact moment jump or time limit.
          const bool stopped_on_moment =
              phase_time >= params.setup_min_time &&
              moment_delta_norm >= params.setup_moment_threshold;
          if (!stopped_on_moment && phase_time < params.setup_timeout) {
            break;
          }

          if (params.pause_before_grind && !gate_grind_passed) {
            if (!gate_grind_armed) {
              gate_grind_armed = true;
              gate_continue.store(false);
              printf("\n[GATE] Set up finished (holding the pressed pose). Press "
                     "Enter to start the grind (e+Enter stops).\n");
            }
            if (!gate_continue.load()) {
              break;  // keep pressing while waiting
            }
            gate_grind_passed = true;
            gate_continue.store(false);
            printf("[GATE] Continuing to grind.\n");
          }

          SetUpReport report;
          report.stopped_on_moment = stopped_on_moment;
          report.phase_time = phase_time;
          report.force_delta_norm = force_delta_norm;
          report.moment_delta_norm = moment_delta_norm;
          report.p_EE = p_EE;
          report.R_EE = R_EE;
          report.tool_contact_point = tool_contact_point;
          report.external_force = external_force;
          report.contact_moment_at_edge = contact_moment_at_edge;
          report.first_contact_tcp = first_contact_tcp;
          report.first_contact_point = first_contact_point;
          report.R_contact_start = R_contact_start;
          report.contact_force_bias = contact_force_bias;
          report.Kp = Kp_setup;
          report.Dp = params.setup_auto_damping ? Dp_setup_cached : Dp_setup;
          report.KR = KR_setup;
          report.DR = params.setup_auto_damping ? DR_setup_cached : DR_setup;
          reportSetUpResult(params, R_alignment_target, report, &pending_parameter_updates);

          // Grind keeps the final set-up preload.
          grind_push = push;
          phase = ControlPhase::kGrind;
          phase_start_time = time;
          next_debug_time = time;
          printf("phase: %s\n", phaseName(phase));
          break;
        }

        // -------------------------------------------------------------
        // Phase 3: keep the frozen preload, with optional tangential sweep.
        // -------------------------------------------------------------
        case ControlPhase::kGrind: {
          const Vec3 n = descend_direction;  // unit, into the surface
          Vec3 edge_target;

          if (params.grind_sweep_enabled) {
            const Vec3 grind_tangent = (params.grind_axis == 2)
                                           ? Vec3(R_alignment_target.col(1))
                                           : Vec3(R_alignment_target.col(0));
            double sweep_s = 0.0;
            double sweep_s_dot = 0.0;
            grindSweep(time - phase_start_time, params.grind_amplitude_m,
                       grindStrokeDuration(params), sweep_s, sweep_s_dot);
            edge_target = first_contact_point + grind_push * n + sweep_s * grind_tangent;
            desired.pdot_d = sweep_s_dot * grind_tangent;
          } else {
            const double edge_penetration = n.dot(tool_contact_point - first_contact_point);
            edge_target = tool_contact_point + (grind_push - edge_penetration) * n;
          }

          desired.p_d = edge_target - R_contact_start * tool_contact_offset_ee;
          edge_target_log = edge_target;
          push_log = grind_push;
          break;
        }

        case ControlPhase::kHold:
        case ControlPhase::kManualGuide:
          break;  // hold the captured start position
      }

      // Below this point, use the post-transition phase.
      const bool after_contact =
          (phase == ControlPhase::kSetUp || phase == ControlPhase::kGrind);

      // ---------------------------------------------------------------
      // Cartesian errors
      // ---------------------------------------------------------------
      const Vec3 e_p = desired.p_d - p_EE;
      const Mat3& R_d_used = after_contact ? R_contact_start
                           : (phase == ControlPhase::kHold) ? R_d
                                                            : R_d_alignment_target;
      const Vec3 e_R =
          applyRotationalAxisMask(params, orientationError(R_EE, R_d_used), R_alignment_target);

      if (phase == ControlPhase::kHold && params.print_hold_debug &&
          params.debug_period > 0.0 && time >= next_debug_time) {
        printHoldDebug(time,
                       (external_force - contact_force_bias).norm(),
                       1000.0 * e_p.norm(),
                       (180.0 / M_PI) * e_R.norm());
        next_debug_time = time + params.debug_period;
      }

      // ---------------------------------------------------------------
      // Auto-damping: compute once per phase group and cache.
      // ---------------------------------------------------------------
      const bool in_approach = (phase == ControlPhase::kApproachOrient ||
                                phase == ControlPhase::kApproachDescend);
      if (!in_approach) {
        approach_damp_computed = false;
      }
      if (!after_contact) {
        setup_damp_computed = false;
      }
      if (phase != ControlPhase::kHold) {
        hold_damp_computed = false;
      }
      if (!pause_hold_active) {
        pause_damp_computed = false;
      }

      const bool need_damping_update =
          (pause_hold_active && params.pause_hold_auto_damping &&
           !pause_damp_computed) ||
          (in_approach && params.approach_auto_damping && !approach_damp_computed) ||
          (after_contact && params.setup_auto_damping && !setup_damp_computed) ||
          (phase == ControlPhase::kHold && params.hold_auto_damping && !hold_damp_computed);
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
              computeCartesianInertiaEstimate(joint_mass, J, R_alignment_target);
          if (inertia_base.valid && inertia_task.valid) {
            const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
                inertia_base.translational,
                Vec3::Constant(params.pause_hold_Kp),
                1.0, Vec3::Zero(), params.auto_damping_max);
            const Vec3 unit_critical_DR = criticalDampingFromStiffness(
                inertia_task.rotational,
                params.approach_KR_diag,
                1.0, Vec3::Zero(), params.auto_damping_max);

            const Mat3 approach_DR_task =
                R_alignment_target.transpose() * DR_approach_cached *
                R_alignment_target;
            const Vec3 target_Dp = Vec3::Constant(params.pause_hold_Dp);
            const Vec3 target_DR = approach_DR_task.diagonal();
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
                Vec3::Constant(params.pause_hold_Kp),
                Dp_factor, dampingFloor(target_Dp), params.auto_damping_max);
            const Vec3 DR_diag = criticalDampingFromStiffness(
                inertia_task.rotational,
                params.approach_KR_diag,
                DR_factor, dampingFloor(target_DR), params.auto_damping_max);
            Dp_pause_cached = Dp_diag.asDiagonal();
            DR_pause_cached =
                makeSpatialGainMatrix(DR_diag, R_alignment_target);
            printf("pause auto damping: fitted Dp factor=%.3f, DR factor=%.3f\n",
                   Dp_factor, DR_factor);
            reportDamping("pause Dp [xyz]", "Ns/m", Dp_diag, target_Dp);
            reportDamping("pause DR [t1t2n]", "Nms/rad", DR_diag, target_DR);
          } else {
            Dp_pause_cached = Dp_pause;
            DR_pause_cached = params.approach_auto_damping
                                  ? DR_approach_cached
                                  : DR_approach;
            printf("pause damping: inertia estimate unavailable, using "
                   "Dp=%.1f Ns/m and inherited approach DR\n",
                   params.pause_hold_Dp);
          }
          pause_damp_computed = true;
        } else if (in_approach) {
          const CartesianInertiaEstimate inertia =
              computeCartesianInertiaEstimate(joint_mass, J, R_alignment_target);
          if (inertia.valid) {
            const double zeta = params.approach_auto_damping_factor;
            const Vec3 Dp_diag = criticalDampingFromStiffness(
                inertia.translational, params.approach_Kp_diag, zeta,
                dampingFloor(params.approach_Dp_diag), params.auto_damping_max);
            const Vec3 DR_diag = criticalDampingFromStiffness(
                inertia.rotational, params.approach_KR_diag, zeta,
                dampingFloor(params.approach_DR_diag), params.auto_damping_max);
            Dp_approach_cached = makeSpatialGainMatrix(Dp_diag, R_alignment_target);
            DR_approach_cached = makeSpatialGainMatrix(DR_diag, R_alignment_target);
            reportDamping("approach Dp", "Ns/m", Dp_diag, params.approach_Dp_diag);
            reportDamping("approach DR", "Nms/rad", DR_diag, params.approach_DR_diag);
            approach_damp_computed = true;
          }
        } else if (after_contact) {
          // Position is base frame; rotation is surface frame.
          const CartesianInertiaEstimate inertia_base =
              computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
          const CartesianInertiaEstimate inertia_task =
              computeCartesianInertiaEstimate(joint_mass, J, R_alignment_target);
          if (inertia_base.valid && inertia_task.valid) {
            const double zeta = params.setup_auto_damping_factor;
            const Vec3 Dp_diag = criticalDampingFromStiffness(
                inertia_base.translational, params.setup_Kp_diag, zeta,
                dampingFloor(params.setup_Dp_diag), params.auto_damping_max);
            const Vec3 DR_diag = criticalDampingFromStiffness(
                inertia_task.rotational, params.setup_KR_diag, zeta,
                dampingFloor(params.setup_DR_diag), params.auto_damping_max);
            Dp_setup_cached = Dp_diag.asDiagonal();
            DR_setup_cached = makeSpatialGainMatrix(DR_diag, R_alignment_target);
            reportDamping("set_up Dp [xyz]", "Ns/m", Dp_diag, params.setup_Dp_diag);
            reportDamping("set_up DR [t1t2n]", "Nms/rad", DR_diag, params.setup_DR_diag);
            setup_damp_computed = true;
          }
        } else {
          const CartesianInertiaEstimate inertia_base =
              computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
          if (inertia_base.valid) {
            const Vec3 manual_hold_Dp = Vec3::Constant(params.hold_Dp);
            const Vec3 manual_hold_DR = Vec3::Constant(params.hold_DR);
            double hold_factor = params.hold_auto_damping_factor;
            if (params.hold_auto_match_manual_damping) {
              const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
                  inertia_base.translational,
                  Vec3::Constant(params.hold_Kp),
                  1.0, Vec3::Zero(), params.auto_damping_max);
              const double denominator = unit_critical_Dp.squaredNorm();
              if (denominator > 1e-12) {
                // Least-squares scalar that makes factor*unit_critical_Dp as
                // close as possible to [hold_Dp, hold_Dp, hold_Dp].
                hold_factor =
                    params.hold_Dp * unit_critical_Dp.sum() / denominator;
              }
              printf("hold auto damping: fitted factor=%.3f toward Dp=%.1f Ns/m\n",
                     hold_factor, params.hold_Dp);
            }
            const Vec3 Dp_diag = criticalDampingFromStiffness(
                inertia_base.translational,
                Vec3::Constant(params.hold_Kp),
                hold_factor,
                dampingFloor(manual_hold_Dp),
                params.auto_damping_max);
            const Vec3 DR_diag = criticalDampingFromStiffness(
                inertia_base.rotational,
                Vec3::Constant(params.hold_KR),
                hold_factor,
                dampingFloor(manual_hold_DR),
                params.auto_damping_max);
            Dp_hold_cached = Dp_diag.asDiagonal();
            DR_hold_cached = DR_diag.asDiagonal();
            reportDamping("hold Dp", "Ns/m", Dp_diag, manual_hold_Dp);
            reportDamping("hold DR", "Nms/rad", DR_diag, manual_hold_DR);
          } else {
            Dp_hold_cached = Dp_hold;
            DR_hold_cached = DR_hold;
            printf("hold damping: inertia estimate unavailable, using "
                   "hold_Dp=%.1f Ns/m and hold_DR=%.1f Nms/rad\n",
                   params.hold_Dp, params.hold_DR);
          }
          hold_damp_computed = true;
        }
      }

      const Mat3& Dp_approach_eff =
          params.approach_auto_damping ? Dp_approach_cached : Dp_approach;
      const Mat3& DR_approach_eff =
          params.approach_auto_damping ? DR_approach_cached : DR_approach;
      const Mat3& Dp_setup_eff = params.setup_auto_damping ? Dp_setup_cached : Dp_setup;
      const Mat3& DR_setup_eff = params.setup_auto_damping ? DR_setup_cached : DR_setup;
      const Mat3& Dp_hold_eff = params.hold_auto_damping ? Dp_hold_cached : Dp_hold;
      const Mat3& DR_hold_eff = params.hold_auto_damping ? DR_hold_cached : DR_hold;
      const Mat3& Dp_pause_eff =
          params.pause_hold_auto_damping ? Dp_pause_cached : Dp_pause;
      const Mat3& DR_pause_eff =
          params.pause_hold_auto_damping ? DR_pause_cached : DR_approach_eff;

      // ---------------------------------------------------------------
      // Select gains for this phase; gate hold overrides position gains.
      // ---------------------------------------------------------------
      const Mat3* Kp_phase = &Kp_hold;
      const Mat3* Dp_phase = &Dp_hold_eff;
      const Mat3* KR_phase = &KR_hold;
      const Mat3* DR_phase = &DR_hold_eff;
      switch (phase) {
        case ControlPhase::kApproachOrient:
        case ControlPhase::kApproachDescend:
          Kp_phase = &Kp_approach;
          Dp_phase = &Dp_approach_eff;
          KR_phase = &KR_approach;
          DR_phase = &DR_approach_eff;
          break;
        case ControlPhase::kSetUp:
        case ControlPhase::kGrind:
          Kp_phase = &Kp_setup;
          Dp_phase = &Dp_setup_eff;
          KR_phase = &KR_setup;
          DR_phase = &DR_setup_eff;
          break;
        case ControlPhase::kHold:
        case ControlPhase::kManualGuide:
          break;
      }
      const Mat3& Kp_used = pause_hold_active ? Kp_pause : *Kp_phase;
      const Mat3& Dp_used = pause_hold_active ? Dp_pause_eff : *Dp_phase;
      const Mat3& KR_used = *KR_phase;
      const Mat3& DR_used = pause_hold_active ? DR_pause_eff : *DR_phase;

      // ---------------------------------------------------------------
      // Control law: decoupled 3x3 springs or coupled 6x6 spring.
      // ---------------------------------------------------------------
      Vec6 dx;
      dx.head<3>() = e_p;
      dx.tail<3>() = e_R;
      Vec6 dv;
      dv.head<3>() = desired.pdot_d - pdot;
      dv.tail<3>() = -omega;

      Vec6 wrench;
      if (phase == ControlPhase::kSetUp && params.use_coupled_stiffness) {
        Mat6x6 K_tcp;
        Mat6x6 D_tcp;
        if (params.coupled_use_block_diagonal) {
          K_tcp = blockDiagonal(Kp_used, KR_used);
          D_tcp = blockDiagonal(Dp_used, DR_used);
        } else if (params.coupled_pole_manual) {
          // Frozen = first-contact reference; live = current edge reference.
          const Vec3 edge_ref = params.coupled_pole_freeze_at_contact ? first_contact_point
                                                                     : tool_contact_point;
          const Vec3 tcp_ref = params.coupled_pole_freeze_at_contact ? first_contact_tcp
                                                                    : p_EE;
          const Vec3 r_c = tcp_ref - (edge_ref + params.coupled_pole_from_edge);
          K_tcp = adjointTransformedGain(blockDiagonal(Kp_used, KR_used), r_c);
          D_tcp = adjointTransformedGain(blockDiagonal(Dp_used, DR_used), r_c);
        } else {
          K_tcp = params.coupled_K_tcp;
          D_tcp = params.coupled_D_tcp;
        }
        wrench = K_tcp * dx + D_tcp * dv;
      } else {
        wrench.head<3>() = Kp_used * e_p + Dp_used * dv.head<3>();
        wrench.tail<3>() = KR_used * e_R - DR_used * omega;
      }
      const Vec3 f = wrench.head<3>();
      const Vec3 m = wrench.tail<3>();

      if (phase == ControlPhase::kGrind && params.grind_sweep_enabled &&
          params.print_grind_debug && params.debug_period > 0.0 &&
          time >= next_debug_time) {
        // sweep = target offset, track_err = position error, press = force.
        const Vec3 grind_tangent = (params.grind_axis == 2)
                                       ? Vec3(R_alignment_target.col(1))
                                       : Vec3(R_alignment_target.col(0));
        double sweep_s = 0.0;
        double sweep_s_dot = 0.0;
        grindSweep(time - phase_start_time, params.grind_amplitude_m,
                   grindStrokeDuration(params), sweep_s, sweep_s_dot);
        printGrindDebug(time - phase_start_time,
                        1000.0 * sweep_s,
                        1000.0 * e_p.dot(grind_tangent),
                        f.dot(descend_direction));
        next_debug_time = time + params.debug_period;
      }

      // nullspace_mode controls whether this is active for the whole sequence.
      SigmaDiagnostics sigma_diagnostics;
      const Vec7 tau_nullspace =
          computeNullspaceTorque(
              params, model, state, J, dq, sigma_diagnostics);

      if (phase == ControlPhase::kHold &&
          sigma_hold_diagnostics_enabled &&
          params.print_sigma_debug &&
          params.debug_period > 0.0 &&
          time >= next_sigma_debug_time) {
        double sigma_rate = 0.0;
        const bool sigma_rate_valid =
            last_sigma_debug_valid && time > last_sigma_debug_time;
        if (sigma_rate_valid) {
          sigma_rate =
              (sigma_diagnostics.sigma_current - last_sigma_debug_value) /
              (time - last_sigma_debug_time);
        }
        printSigmaDebug(
            time - phase_start_time,
            sigma_diagnostics,
            sigma_rate,
            sigma_rate_valid);
        last_sigma_debug_time = time;
        last_sigma_debug_value = sigma_diagnostics.sigma_current;
        last_sigma_debug_valid = true;
        next_sigma_debug_time = time + params.debug_period;
      }

      Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());
      const Vec7 tau_task = J.transpose() * wrench;
      const Vec7 tau_cmd = tau_task + tau_nullspace + coriolis;

      Vec7 sigma_debug_external_joint_torque_delta = Vec7::Zero();
      if (phase == ControlPhase::kHold &&
          sigma_hold_diagnostics_enabled &&
          !sigma_debug_external_joint_torque_bias_valid) {
        sigma_debug_external_joint_torque_bias = external_joint_torque;
        sigma_debug_external_joint_torque_bias_valid = true;
      }
      if (sigma_debug_external_joint_torque_bias_valid) {
        sigma_debug_external_joint_torque_delta =
            external_joint_torque -
            sigma_debug_external_joint_torque_bias;
      }

      if (phase == ControlPhase::kHold &&
          sigma_hold_diagnostics_enabled) {
        const Vec3 external_force_delta =
            external_force - contact_force_bias;
        const Vec3 external_moment_delta =
            external_moment - contact_moment_bias;

        peak_sigma_nullspace_speed =
            std::max(peak_sigma_nullspace_speed,
                     sigma_diagnostics.nullspace_speed);
        peak_sigma_abs_speed_toward_better =
            std::max(peak_sigma_abs_speed_toward_better,
                     std::abs(
                         sigma_diagnostics.speed_toward_better));
        min_sigma_speed_toward_better =
            std::min(min_sigma_speed_toward_better,
                     sigma_diagnostics.speed_toward_better);
        max_sigma_speed_toward_better =
            std::max(max_sigma_speed_toward_better,
                     sigma_diagnostics.speed_toward_better);
        peak_sigma_position_error =
            std::max(peak_sigma_position_error, e_p.norm());
        peak_sigma_rotation_error =
            std::max(peak_sigma_rotation_error, e_R.norm());
        peak_sigma_external_force_delta =
            std::max(peak_sigma_external_force_delta,
                     external_force_delta.norm());
        peak_sigma_external_moment_delta =
            std::max(peak_sigma_external_moment_delta,
                     external_moment_delta.norm());
        peak_sigma_external_joint_torque_delta =
            std::max(
                peak_sigma_external_joint_torque_delta,
                sigma_debug_external_joint_torque_delta.norm());

        if (time >= next_sigma_debug_file_time) {
          SigmaDebugRow debug_row;
          debug_row.run_time = time;
          debug_row.phase_time = time - phase_start_time;
          debug_row.segment_id = sigma_debug_segment_id;
          debug_row.event = SigmaDebugEvent::kSample;
          debug_row.q = q_current;
          debug_row.dq = dq;
          debug_row.e_p = e_p;
          debug_row.e_R = e_R;
          debug_row.pdot = pdot;
          debug_row.omega = omega;
          debug_row.command_force = f;
          debug_row.command_moment = m;
          debug_row.external_force_delta = external_force_delta;
          debug_row.external_moment_delta = external_moment_delta;
          debug_row.external_joint_torque_delta =
              sigma_debug_external_joint_torque_delta;
          debug_row.external_joint_torque_baseline_valid =
              sigma_debug_external_joint_torque_bias_valid;
          debug_row.joint_contact = joint_contact;
          debug_row.cartesian_contact = cartesian_contact;
          debug_row.sigma = sigma_diagnostics;
          debug_row.tau_task_norm = tau_task.norm();
          debug_row.tau_nullspace_norm = tau_nullspace.norm();
          debug_row.tau_cmd_norm = tau_cmd.norm();
          debug_row.peak_nullspace_speed =
              peak_sigma_nullspace_speed;
          debug_row.peak_abs_speed_toward_better =
              peak_sigma_abs_speed_toward_better;
          debug_row.min_speed_toward_better =
              min_sigma_speed_toward_better;
          debug_row.max_speed_toward_better =
              max_sigma_speed_toward_better;
          debug_row.peak_position_error =
              peak_sigma_position_error;
          debug_row.peak_rotation_error =
              peak_sigma_rotation_error;
          debug_row.peak_external_force_delta =
              peak_sigma_external_force_delta;
          debug_row.peak_external_moment_delta =
              peak_sigma_external_moment_delta;
          debug_row.peak_external_joint_torque_delta =
              peak_sigma_external_joint_torque_delta;
          appendSigmaDebugRow(debug_row);
          resetSigmaDebugPeaks();
          next_sigma_debug_file_time +=
              sigma_debug_log_period;
        }
      }

      // ---------------------------------------------------------------
      // Logging into the pre-sized ring buffer (no allocation here).
      // ---------------------------------------------------------------
      ++control_cycle_count;
      if (max_log_rows > 0 &&
          (control_cycle_count % static_cast<std::size_t>(log_every_n_cycles)) == 0) {
        LogData& row = log_data[log_write_index];
        row.time = time;
        row.phase = static_cast<int>(phase);
        row.nullspace_mode = static_cast<int>(params.nullspace_mode);
        row.p_EE = p_EE;
        row.p_d = desired.p_d;
        row.tool_contact_point = tool_contact_point;
        row.first_contact_tcp = first_contact_tcp;
        row.first_contact_point = first_contact_point;
        row.edge_target = edge_target_log;
        row.tool_contact_offset_ee = tool_contact_offset_ee;
        row.e_p = e_p;
        row.e_R = e_R;
        row.alignment_error_surface =
            R_alignment_target.transpose() *
            toolSurfaceAlignmentErrorInBase(params, R_EE, R_alignment_target);
        row.alignment_angle = row.alignment_error_surface.norm();
        row.pdot = pdot;
        row.pdot_d = desired.pdot_d;
        row.omega = omega;
        row.f = f;
        row.m = m;
        row.external_force = external_force;
        row.external_moment = external_moment;
        row.contact_force_bias = contact_force_bias;
        row.contact_moment_bias = contact_moment_bias;
        row.push = push_log;
        row.sigma = sigma_diagnostics;
        row.tau_nullspace_norm = tau_nullspace.norm();
        row.tau_cmd = tau_cmd;

        log_write_index = (log_write_index + 1) % max_log_rows;
        if (log_rows_written < max_log_rows) {
          ++log_rows_written;
        } else {
          log_buffer_wrapped = true;
        }
      }

      final_p_EE = p_EE;
      final_p_d = desired.p_d;
      final_e_p = e_p;
      final_e_R = e_R;
      final_q = q_current;

      const Array7 tau_array = vec7ToArray(tau_cmd);
      if ((params.experiment_duration > 0.0 && time >= params.experiment_duration) ||
          stop_requested.load()) {
        if (phase == ControlPhase::kHold &&
            sigma_hold_diagnostics_enabled) {
          SigmaDebugRow stop_row;
          stop_row.run_time = time;
          stop_row.phase_time = time - phase_start_time;
          stop_row.segment_id = sigma_debug_segment_id;
          stop_row.event = SigmaDebugEvent::kStop;
          stop_row.q = q_current;
          stop_row.dq = dq;
          stop_row.e_p = e_p;
          stop_row.e_R = e_R;
          stop_row.pdot = pdot;
          stop_row.omega = omega;
          stop_row.command_force = f;
          stop_row.command_moment = m;
          stop_row.external_force_delta =
              external_force - contact_force_bias;
          stop_row.external_moment_delta =
              external_moment - contact_moment_bias;
          stop_row.external_joint_torque_delta =
              sigma_debug_external_joint_torque_delta;
          stop_row.external_joint_torque_baseline_valid =
              sigma_debug_external_joint_torque_bias_valid;
          stop_row.joint_contact = joint_contact;
          stop_row.cartesian_contact = cartesian_contact;
          stop_row.sigma = sigma_diagnostics;
          stop_row.tau_task_norm = tau_task.norm();
          stop_row.tau_nullspace_norm = tau_nullspace.norm();
          stop_row.tau_cmd_norm = tau_cmd.norm();
          stop_row.peak_nullspace_speed =
              std::max(peak_sigma_nullspace_speed,
                       sigma_diagnostics.nullspace_speed);
          stop_row.peak_abs_speed_toward_better =
              std::max(peak_sigma_abs_speed_toward_better,
                       std::abs(
                           sigma_diagnostics.speed_toward_better));
          stop_row.min_speed_toward_better =
              std::min(min_sigma_speed_toward_better,
                       sigma_diagnostics.speed_toward_better);
          stop_row.max_speed_toward_better =
              std::max(max_sigma_speed_toward_better,
                       sigma_diagnostics.speed_toward_better);
          stop_row.peak_position_error =
              std::max(peak_sigma_position_error, e_p.norm());
          stop_row.peak_rotation_error =
              std::max(peak_sigma_rotation_error, e_R.norm());
          stop_row.peak_external_force_delta =
              std::max(
                  peak_sigma_external_force_delta,
                  stop_row.external_force_delta.norm());
          stop_row.peak_external_moment_delta =
              std::max(
                  peak_sigma_external_moment_delta,
                  stop_row.external_moment_delta.norm());
          stop_row.peak_external_joint_torque_delta =
              std::max(
                  peak_sigma_external_joint_torque_delta,
                  sigma_debug_external_joint_torque_delta.norm());
          appendSigmaDebugRow(stop_row);
        }
        if (stop_requested.load()) {
          printf("\nStop requested with e + Enter. Finishing control loop...\n");
        }
        return MotionFinished(Torques(tau_array));
      }
      return Torques(tau_array);
      });
    } catch (...) {
      // Preserve the diagnostic trace even when a collision/reflex or another
      // control exception ends the experiment.
      if (sigma_hold_diagnostics_enabled) {
        SigmaDebugRow exception_row;
        exception_row.run_time = time;
        exception_row.phase_time = time - phase_start_time;
        exception_row.segment_id = sigma_debug_segment_id;
        exception_row.event = SigmaDebugEvent::kException;
        exception_row.q = final_q;
        appendSigmaDebugRow(exception_row);
      }
      (void)persistSigmaDebugBuffer();
      throw;
    }

    // Make the compact diagnostic available before writing the large log.
    (void)persistSigmaDebugBuffer();

    // ================================================================
    // 4. Post-run: persist queued parameters, write the CSV, print results
    // ================================================================
    if (!pending_parameter_updates.empty()) {
      // The auto-written keys all live in params/sequence.txt, and
      // updateParameterValues only rewrites keys already present there.
      updateParameterValues("params/sequence.txt", pending_parameter_updates);
      printf("params/sequence.txt updated after control loop (%zu queued values).\n",
             pending_parameter_updates.size());
    }

    if (descend_failed) {
      printf("\nDescend stopped: maximum distance reached before the clearance height.\n");
    }
    printJointStartEndTableDeg(q_start, final_q);

    // Unwrap the ring buffer into chronological order before writing.
    std::vector<LogData> ordered_log_data;
    ordered_log_data.reserve(log_rows_written);
    if (log_buffer_wrapped) {
      ordered_log_data.insert(ordered_log_data.end(),
                              log_data.begin() + static_cast<std::ptrdiff_t>(log_write_index),
                              log_data.end());
      ordered_log_data.insert(ordered_log_data.end(),
                              log_data.begin(),
                              log_data.begin() + static_cast<std::ptrdiff_t>(log_write_index));
      printf("Log buffer wrapped: kept latest %zu rows, sampled every %d control cycles.\n",
             log_rows_written, log_every_n_cycles);
    } else {
      ordered_log_data.insert(ordered_log_data.end(),
                              log_data.begin(),
                              log_data.begin() + static_cast<std::ptrdiff_t>(log_rows_written));
    }

    writeLogToCsv(ordered_log_data, params.csv_file_name);
    printFinalSummary(final_p_d, final_p_EE, final_e_p, final_e_R, params.csv_file_name);

  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    fprintf(stderr, "If the robot is still in an error/reflex state, recover it manually in Franka Desk.\n");
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }

  return 0;
}
