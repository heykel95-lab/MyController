// ====================================================================
// Run control loop
// ====================================================================
// One run: captures the start pose, builds the run state, then drives the
// 1 kHz torque callback through the phase machine until it stops. Returns
// what the report and the CSV need; commands nothing after it returns.
#include "controller.h"

// ====================================================================
// 2 + 3. One run: task frames, run state, and the 1 kHz control loop
// ====================================================================
RunResult runControlLoop(Parameters& params,
                         Robot& robot,
                         const Model& model,
                         RunGains gains,  // by value: retuning rebuilds it
                         KeyboardSignals& signals) {
  RunResult result;
  // Named aliases so the loop below reads the same as the gain table.
  const Mat3& R_alignment_target = gains.R_alignment_target;
  const Mat3& Kp_approach = gains.Kp_approach;
  const Mat3& Dp_approach = gains.Dp_approach;
  const Mat3& KR_approach = gains.KR_approach;
  const Mat3& DR_approach = gains.DR_approach;
  const Mat3& Kp_setup = gains.Kp_setup;
  const Mat3& Dp_setup = gains.Dp_setup;
  const Mat3& KR_setup = gains.KR_setup;
  const Mat3& DR_setup = gains.DR_setup;
  const Mat3& Kp_hold = gains.Kp_hold;
  const Mat3& Dp_hold = gains.Dp_hold;
  const Mat3& KR_hold = gains.KR_hold;
  const Mat3& DR_hold = gains.DR_hold;
  const Mat3& Kp_pause = gains.Kp_pause;
  const Mat3& Dp_pause = gains.Dp_pause;
  const Mat3& KR_pause = gains.KR_pause;
  const Mat3& DR_pause = gains.DR_pause;

  // ================================================================
  // 2. Task frames, gains and run state
  // ================================================================
  RobotState initial_state = robot.readOnce();
  Map<const Mat4x4> T_initial(initial_state.O_T_EE.data());
  Vec3 p_start = T_initial.block<3, 1>(0, 3);
  Mat3 R_d = T_initial.block<3, 3>(0, 0);
  Vec7 q_start = Map<const Vec7>(initial_state.q.data());

  // Where the hold was established. t returns to it rather than holding the
  // pose the sequence stopped at, so every tuning cycle presses from the same
  // place: pressed poses differ by the depth reached, and a set-up tried from
  // one is not the set-up tried from the next.
  Vec3 hold_return_p = p_start;
  Mat3 hold_return_R = R_d;
  bool hold_returning = false;

  // Baseline external wrench: contact detection and the set-up report both
  // measure changes against this, not against absolute readings.
  Map<const Vec6> initial_external_wrench(initial_state.O_F_ext_hat_K.data());
  Vec3 contact_force_bias = initial_external_wrench.head<3>();
  Vec3 contact_moment_bias = initial_external_wrench.tail<3>();

  // Mutable: recomputed from the re-captured pose when the sequence is
  // restarted through manual re-guidance.
  Mat3 R_d_alignment_target =
      makeToolOrientationForAlignmentTarget(params, R_alignment_target, R_d);
  Vec3 surface_point_runtime =
      params.use_start_as_surface_point ? p_start : params.surface_point;

  const Vec3 descend_direction = -R_alignment_target.col(2);

  // Four corners of the grinding face. Taking every corner tied for the
  // greatest projection gives a leading face, edge or corner center.
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

    // A single-axis tilt ties both ends of the leading edge. Average the
    // tied points so the controlled point is the edge center.
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

  // ---- phase machine ----
  const ControlPhase sequence_first_phase =
      params.use_approach_orient ? ControlPhase::kApproachOrient
                                 : ControlPhase::kApproachDescend;
  const ControlPhase initial_phase =
      params.use_phase_sequence ? sequence_first_phase : ControlPhase::kHold;
  Vec3 disturbance_force_direction_base = Vec3::Zero();
  if (params.disturbance_auto_enabled) {
    std::string error;
    if (!validateAutomaticDisturbance(params, error)) {
      throw std::runtime_error("automatic disturbance: " + error);
    }
    if (initial_phase != ControlPhase::kHold ||
        params.hold_with_setup_gains) {
      throw std::runtime_error(
          "automatic disturbance requires the plain h hold mode");
    }
    const std::array<double, 42> initial_jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, initial_state);
    Map<const Mat6x7> initial_jacobian(initial_jacobian_array.data());
    disturbance_force_direction_base = automaticDisturbanceDirection(
        params, model, initial_state, initial_jacobian);
    if (disturbance_force_direction_base.norm() <= 1e-9) {
      throw std::runtime_error(
          "automatic disturbance point cannot excite the redundant axis");
    }
  }
  const bool sigma_hold_diagnostics_enabled =
      initial_phase == ControlPhase::kHold &&
      (params.disturbance_auto_enabled || params.disturbance_cues_enabled ||
       params.nullspace_mode == NullspaceMode::kSigmaOnly ||
       params.nullspace_mode == NullspaceMode::kDampingAndSigma);
  ControlPhase phase = initial_phase;
  // Where p returns to after hand guiding. It follows the s and t switches
  // below, so re-guiding restarts the mode running now, not the one the run
  // was started in.
  ControlPhase restart_phase = initial_phase;
  // A key typed before this run began belongs to the menu or to guiding.
  signals.run_mode_request.store(0);
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

  // Orientation the orient step actually commands. The full target is a step
  // the moment the run starts, and a large one -- a commanded spin about the
  // tool axis can be most of a half turn -- trips the power limit against the
  // approach KR. Slewed from the start pose at a bounded rate instead.
  Mat3 R_orient_start = R_d;
  Mat3 R_orient_command = R_d_alignment_target;

  // Gate state. Each gate arms once, then blocks until a bare Enter.
  bool gate_set_up_armed = false;
  bool gate_set_up_passed = false;
  Vec3 gate_set_up_hold_pd = Vec3::Zero();
  double gate_paused_time = 0.0;  // frozen descend clock while gated
  bool gate_grind_armed = false;
  bool gate_grind_passed = false;
  double gate_grind_paused_time = 0.0;  // frozen push ramp while gated
  bool setup_reported = false;          // the result prints once, at phase end
  bool disturb_push_cued = false;       // scripted disturbance cues, once each
  bool disturb_hold_cued = false;
  bool disturb_release_cued = false;

  // The preload frozen when the set-up phase ends; grind presses with it.
  double setup_push_start = -params.descend_surface_clearance;
  double grind_push = 0.0;

  // Auto-damping is cached per phase group.
  DampingCache damping = manualDampingCache(gains);

  printSection("start pose");
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

  // Compact logger for sigma hold tuning: sampled far slower than the full
  // CSV and keeps guide/recapture events even when the 1 kHz log wraps.
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

  printBanner("RUN");
  // The t mode is about the set-up spring; the nullspace belongs to the plain
  // hold, where it is what is being studied. The set-up block waits until the
  // auto damping has been fitted, so it can print the values in use.
  bool setup_law_printed = false;
  // kManualGuide is the sentinel: it never opens a block of its own, so it
  // also means "nothing printed yet", and every phase entry opens one.
  ControlPhase intro_printed_for = ControlPhase::kManualGuide;
  bool gate_block_printed = false;
  if (phase == ControlPhase::kHold && !params.hold_with_setup_gains) {
    printNullspaceLaw(params);
    printAutomaticDisturbance(params, disturbance_force_direction_base);
  }

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
    // Take the pose reached as the start of whatever runs next.
    // ---------------------------------------------------------------
    // Shared by the p recapture after hand guiding and by the s and t
    // switches below: all three continue from where the arm is, so the
    // commanded pose does not step at the moment of the switch. Only the
    // phase the caller sets afterwards differs.
    const auto restartFromPoseReached = [&](bool reanchor_surface_point) {
      p_start = p_EE;
      R_d = R_EE;
      q_start = q_current;
      R_d_alignment_target =
          makeToolOrientationForAlignmentTarget(params, R_alignment_target, R_d);
      // The slew restarts from the pose reached, not the original one.
      R_orient_start = R_d;
      R_orient_command = R_d;
      // Re-anchoring the plane belongs to the hand-placed start alone. An s
      // pressed after a sequence would otherwise take the pressed pose for
      // the surface, and each cycle would descend a clearance deeper.
      if (reanchor_surface_point && params.use_start_as_surface_point) {
        surface_point_runtime = p_start;
      }
      contact_force_bias = external_force;
      contact_moment_bias = external_moment;
      first_contact_tcp = p_start;
      first_contact_point = p_start;
      R_contact_start = R_d_alignment_target;
      active_tool_contact_offset_ee = params.tool_contact_face_center_ee;

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
      gate_grind_paused_time = 0.0;
      setup_reported = false;
      setup_push_start = -params.descend_surface_clearance;
      grind_push = 0.0;
      damping.hold_computed = false;
      damping.Dp_hold = Dp_hold;
      damping.DR_hold = DR_hold;
      damping.pause_computed = false;
      damping.Dp_pause = Dp_pause;
      damping.DR_pause = DR_pause;

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
    };

    // ---------------------------------------------------------------
    // Switching the run mode: s runs the sequence, t holds with the set-up
    // impedance.
    // ---------------------------------------------------------------
    // The t hold is where the set-up spring is tuned, so the sequence has to
    // be reachable from it without ending the run: going out through the menu
    // re-reads the parameter files and drops every kp/kr/r just typed. t takes
    // the pose the sequence reached and walks back to where the hold started,
    // so the next set-up is pressed from the same place as the last.
    const char run_mode_request = signals.run_mode_request.exchange(0);
    if (run_mode_request == 's') {
      if (phase == ControlPhase::kManualGuide) {
        printf("Ignored: p+Enter re-captures the pose first.\n");
      } else if (phase != ControlPhase::kHold) {
        printf("Ignored: the sequence is already running; t+Enter holds.\n");
      } else if (hold_returning) {
        // Starting from halfway back would press from a pose no other cycle
        // uses, which is the one thing this loop is meant to avoid.
        printf("Ignored: still returning to the pose the hold started from.\n");
      } else if (params.use_coupled_stiffness &&
                 !params.coupled_use_block_diagonal &&
                 !params.coupled_pole_manual) {
        // The rule the menu applies before a sequence run holds here too: the
        // coupled spring needs a pole it was given, not one inferred.
        printf("Ignored: a sequence with the coupled stiffness needs "
               "block-diagonal mode or a commanded pole.\n");
      } else {
        params.use_phase_sequence = true;
        params.hold_with_setup_gains = false;
        restartFromPoseReached(false);
        restart_phase = sequence_first_phase;
        phase = sequence_first_phase;
        intro_printed_for = ControlPhase::kManualGuide;
        setup_law_printed = false;
        printSection("s: sequence from the pose held");
        printVec3Mm("p_start", p_start);
        printf("  %-16s   the one commanded now\n", "impedance");
        printf("  %-16s   t1 %.2f deg | t2 %.2f deg\n", "tilt",
               params.tool_target_offset_tangent1_deg,
               params.tool_target_offset_tangent2_deg);
      }
    } else if (run_mode_request == 't') {
      if (phase == ControlPhase::kManualGuide) {
        printf("Ignored: p+Enter re-captures the pose first.\n");
      } else if (phase == ControlPhase::kHold && params.hold_with_setup_gains) {
        printf(hold_returning
                   ? "Ignored: already on the way back to the hold pose.\n"
                   : "Ignored: already holding with the set-up impedance.\n");
      } else {
        params.use_phase_sequence = false;
        params.hold_with_setup_gains = true;
        restartFromPoseReached(false);
        restart_phase = ControlPhase::kHold;
        phase = ControlPhase::kHold;
        hold_returning = true;
        // Both blocks may already be the ones in force -- t is pressed from
        // the set-up press as often as from the grind -- so ask for them
        // again explicitly. What they say has changed: the hold commands the
        // gains now, and the impedance block gains its keys.
        intro_printed_for = ControlPhase::kManualGuide;
        setup_law_printed = false;
        printSection("t: set-up impedance hold, returning to its pose");
        printVec3Mm("p_start", hold_return_p);
        printf("  %-16s   %.3f m/s, turning at %.1f deg/s\n", "returning at",
               params.descend_speed, params.approach_orient_max_rate_deg);
      }
    }

    // ---------------------------------------------------------------
    // Manual re-guidance from hold.
    // ---------------------------------------------------------------
    // Accepted from every phase, like e and m: g drops the commanded torque to
    // gravity compensation, so leaving a pressed phase just releases the
    // preload rather than moving the arm.
    if (phase != ControlPhase::kManualGuide && signals.guide_requested.load()) {
      signals.guide_requested.store(false);
      signals.proceed_requested.store(false);

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
      // Guiding returns to the phase it interrupted, so the block that phase
      // printed has to be opened again when p re-captures.
      intro_printed_for = ControlPhase::kManualGuide;
      printPhaseHeader(ControlPhase::kManualGuide);
      printf("  %-16s   move the tool by hand\n", "motion");
      // p restarts the mode running now: the sequence's first phase after an
      // s, hold after a t or a hold run. Name the one that will happen.
      printf("  %-16s   p re-capture and restart %s | e stop\n", "keys",
             phaseName(restart_phase));
    }
    if (phase == ControlPhase::kManualGuide) {
      Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());
      const Array7 tau_array =
          vec7ToArray(Vec7(coriolis - params.manual_guidance_damping * dq));
      if (signals.stop_requested.load()) {
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
      if (signals.proceed_requested.load()) {
        signals.proceed_requested.store(false);
        // Re-capture the hand-moved pose as the new sequence start. It is
        // also where t returns to from now on: the hand placed it there.
        restartFromPoseReached(true);
        phase = restart_phase;
        hold_return_p = p_start;
        hold_return_R = R_d;
        hold_returning = false;

        printSection("resuming from the re-guided pose");
        printVec7Deg("q_start", q_start);
        printVec3Mm("p_start", p_start);
      }
      return Torques(tau_array);
    }

    // ---------------------------------------------------------------
    // Scripted hold disturbance.
    // ---------------------------------------------------------------
    // Printed on a clock the run owns, so the push lands at the same time in
    // every repetition. Automatic runs also use these markers as phase edges.
    if ((params.disturbance_cues_enabled ||
         params.disturbance_auto_enabled) &&
        phase == ControlPhase::kHold) {
      const auto cue = [&](const char* text, SigmaDebugEvent event) {
        printf("\n>>> %s  (t = %.1f s)\n", text, time);
        fflush(stdout);
        if (sigma_hold_diagnostics_enabled) {
          SigmaDebugRow row;
          row.run_time = time;
          row.phase_time = time - phase_start_time;
          row.segment_id = sigma_debug_segment_id;
          row.event = event;
          row.q = q_current;
          row.dq = dq;
          row.pdot = pdot;
          row.omega = omega;
          row.joint_contact = joint_contact;
          row.cartesian_contact = cartesian_contact;
          appendSigmaDebugRow(row);
        }
      };
      const double hold_time = time - phase_start_time;
      if (!disturb_push_cued && hold_time >= params.disturbance_push_time) {
        disturb_push_cued = true;
        cue(params.disturbance_auto_enabled
                ? "AUTOMATIC PUSH START"
                : "PUSH THE ARM NOW",
            params.disturbance_auto_enabled
                ? SigmaDebugEvent::kDisturbAutoPush
                : SigmaDebugEvent::kDisturbCuePush);
      }
      // Separating "stop moving" from "let go" marks the transition. Without
      // it the driven stretch and the statically held one run together, and
      // the moment the hand stopped driving is only inferable.
      if (!disturb_hold_cued && hold_time >= params.disturbance_hold_time) {
        disturb_hold_cued = true;
        cue(params.disturbance_auto_enabled
                ? "AUTOMATIC PUSH AT FULL FORCE"
                : "STOP MOVING - hold it still",
            params.disturbance_auto_enabled
                ? SigmaDebugEvent::kDisturbAutoHold
                : SigmaDebugEvent::kDisturbCueHold);
      }
      if (!disturb_release_cued &&
          hold_time >= params.disturbance_release_time) {
        disturb_release_cued = true;
        cue(params.disturbance_auto_enabled
                ? "AUTOMATIC RELEASE START"
                : "RELEASE - do not touch until the run ends",
            params.disturbance_auto_enabled
                ? SigmaDebugEvent::kDisturbAutoRelease
                : SigmaDebugEvent::kDisturbCueRelease);
      }
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
        // Split into the surface frame: the normal component is the spin about
        // the tool axis, the part axis_err cannot see. Kept separate because
        // the mounted system settles near 1.5 deg of axis error, so a combined
        // norm would never clear a 2 deg gate however well the spin converged.
        const Vec3 e_R_orient =
            R_alignment_target.transpose() *
            applyRotationalAxisMask(
                params, orientationError(R_EE, R_d_alignment_target),
                R_alignment_target);
        const double spin_error = std::abs(e_R_orient(2));

        // Rotate the commanded frame toward the target at no more than
        // approach_orient_max_rate. The spring then sees a small standing
        // error instead of the whole rotation at once.
        {
          const Eigen::AngleAxisd to_target(R_d_alignment_target *
                                            R_orient_start.transpose());
          const double reachable =
              (M_PI / 180.0) * params.approach_orient_max_rate_deg * phase_time;
          const double commanded =
              std::min(std::abs(to_target.angle()), std::max(0.0, reachable));
          R_orient_command =
              Mat3(Eigen::AngleAxisd(commanded, to_target.axis())) *
              R_orient_start;
        }

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            intro_printed_for == phase) {
          printApproachOrientDebug(phase_time,
                                   (180.0 / M_PI) * tool_axis_error,
                                   (180.0 / M_PI) * spin_error);
          next_debug_time = time + params.debug_period;
        }

        // Each commanded degree of freedom clears the same gate on its own.
        const bool orientation_reached =
            tool_axis_error <= params.approach_orient_error_threshold &&
            (!params.command_tool_twist ||
             spin_error <= params.approach_orient_spin_error_threshold);
        // Settled short of the gate is still the end of the phase: the axis
        // error stops improving at a value the commanded tilt decides, and
        // waiting past that only spends time. Said plainly, because it changes
        // what the trial handed over with.
        const bool orient_timed_out =
            params.approach_orient_timeout > 0.0 &&
            phase_time >= params.approach_orient_timeout;
        if (phase_time >= params.approach_orient_min_time &&
            (orientation_reached || orient_timed_out)) {
          phase = ControlPhase::kApproachDescend;
          phase_start_time = time;
          next_debug_time = time;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          if (orientation_reached) {
            printf("\nOrientation reached: axis_err=%.1f deg | spin_err=%.1f deg\n",
                   (180.0 / M_PI) * tool_axis_error,
                   (180.0 / M_PI) * spin_error);
          } else {
            printf("\nOrientation settled short of the %.1f deg gate after "
                   "%.1f s: axis_err=%.1f deg | spin_err=%.1f deg\n",
                   (180.0 / M_PI) * params.approach_orient_error_threshold,
                   params.approach_orient_timeout,
                   (180.0 / M_PI) * tool_axis_error,
                   (180.0 / M_PI) * spin_error);
          }
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
        // Once armed it stays armed. The gate holds the pose the tool reached,
        // which is within one descend step of the clearance height, so the
        // height it is compared against sits on the threshold and crosses it
        // on measurement noise. Re-testing it each cycle then released the
        // gate for single cycles, and on those the descend clock advanced and
        // commanded the tool further down, so the longer the gate waited the
        // harder it pushed when it let go.
        bool gate_blocking = false;
        if (params.pause_before_set_up && !gate_set_up_passed &&
            (gate_set_up_armed || clearance_reached)) {
          if (!gate_set_up_armed) {
            gate_set_up_armed = true;
            // Hold the actual pose reached at the gate.
            gate_set_up_hold_pd = p_EE;
            signals.gate_continue.store(false);
            printf("\n[GATE] Reached %.0f mm above the plane. Press Enter to start "
                   "the set-up press (e+Enter stops).\n",
                   1000.0 * params.descend_surface_clearance);
          }
          if (signals.gate_continue.load()) {
            gate_set_up_passed = true;
            signals.gate_continue.store(false);
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

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            !gate_blocking && intro_printed_for == phase) {
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
          // Signed plane coordinate [m], negative above the surface. Taken
          // live so set up continues smoothly if descend overshot.
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
          } else if (distance >= params.descend_max_distance) {
          descend_failed = true;
          signals.stop_requested.store(true);
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
        // Frozen clock, like descend's: waiting at the grind gate must not go
        // on ramping the commanded depth, or the preload grows while you read.
        const double push =
            setUpPush(params, phase_time - gate_grind_paused_time,
                      setup_push_start, setup_push_velocity);
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
        if (params.debug_period > 0.0 && time >= next_debug_time &&
            !waiting_at_gate && intro_printed_for == phase) {
          // tip = passive rotation away from the contact orientation.
          printSetUpDebug(phase_time,
                          (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm(),
                          force_delta_norm,
                          moment_delta_norm,
                          params.setup_moment_threshold,
                          1000.0 * (tool_contact_point - first_contact_point).norm());
          next_debug_time = time + params.debug_period;
        }

        // End on contact moment jump or time limit. Once the grind gate has
        // armed the phase keeps reaching it, whatever the criterion reads now:
        // a press ended on the moment threshold sits at that threshold, so the
        // test crosses back on measurement noise, and a cycle that returned
        // here early would skip the gate, leave its clock unfrozen and ramp the
        // commanded depth while the operator reads the report.
        const bool stopped_on_moment =
            phase_time >= params.setup_min_time &&
            moment_delta_norm >= params.setup_moment_threshold;
        if (!gate_grind_armed && !stopped_on_moment &&
            phase_time < params.setup_timeout) {
          break;
        }

        // Reported the moment set up ends, before the gate rather than beyond
        // it. The result belongs to the phase that just finished: the operator
        // reads it to decide at the gate, its phase time is the press and not
        // the wait, and stopping at the gate no longer discards it.
        if (!setup_reported) {
          setup_reported = true;
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
          report.Dp = params.setup_auto_damping ? damping.Dp_setup : Dp_setup;
          report.KR = KR_setup;
          report.DR = params.setup_auto_damping ? damping.DR_setup : DR_setup;
          reportSetUpResult(params, R_alignment_target, report);
        }

        if (params.pause_before_grind && !gate_grind_passed) {
          if (!gate_grind_armed) {
            gate_grind_armed = true;
            signals.gate_continue.store(false);
            printf("\n[GATE] Set up finished (holding the pressed pose). Press "
                   "Enter to start the grind (e+Enter stops).\n");
          }
          if (!signals.gate_continue.load()) {
            gate_grind_paused_time += period.toSec();
            break;  // keep pressing, at the preload reached, while waiting
          }
          gate_grind_passed = true;
          signals.gate_continue.store(false);
          printf("[GATE] Continuing to grind.\n");
        }

        // Grind keeps the final set-up preload.
        grind_push = push;
        phase = ControlPhase::kGrind;
        phase_start_time = time;
        next_debug_time = time;
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

      // -------------------------------------------------------------
      // Hold: the captured start position, or the way back to it.
      // -------------------------------------------------------------
      case ControlPhase::kHold: {
        if (!hold_returning) {
          break;
        }
        // t is pressed from a pressed pose, so the commanded pose walks back
        // to where the hold started instead of jumping: at descend_speed,
        // the speed it travelled in on, and the orient rate for the tilt the
        // set-up press put in.
        const Vec3 to_home = hold_return_p - p_start;
        const double distance = to_home.norm();
        const double step = params.descend_speed * period.toSec();
        const bool position_home = distance <= step;
        if (position_home) {
          p_start = hold_return_p;
        } else {
          p_start += (step / distance) * to_home;
          desired.pdot_d = (params.descend_speed / distance) * to_home;
        }

        const Eigen::AngleAxisd to_home_R(hold_return_R * R_d.transpose());
        const double step_R =
            (M_PI / 180.0) * params.approach_orient_max_rate_deg * period.toSec();
        const bool orientation_home = std::abs(to_home_R.angle()) <= step_R;
        if (orientation_home) {
          R_d = hold_return_R;
        } else {
          R_d = Mat3(Eigen::AngleAxisd(std::copysign(step_R, to_home_R.angle()),
                                       to_home_R.axis())) *
                R_d;
        }

        desired.p_d = p_start;
        if (position_home && orientation_home) {
          hold_returning = false;
          desired.pdot_d.setZero();
          printf("Back at the pose the hold started from.\n");
        }
        break;
      }

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
    const Mat3& R_d_used =
        after_contact ? R_contact_start
        : (phase == ControlPhase::kHold) ? R_d
        : (phase == ControlPhase::kApproachOrient) ? R_orient_command
                                                   : R_d_alignment_target;
    const Vec3 e_R =
        applyRotationalAxisMask(params, orientationError(R_EE, R_d_used), R_alignment_target);

    if (phase == ControlPhase::kHold && params.print_hold_debug &&
        params.debug_period > 0.0 && time >= next_debug_time &&
        intro_printed_for == phase) {
      printHoldDebug(time,
                     (external_force - contact_force_bias).norm(),
                     1000.0 * e_p.norm(),
                     (180.0 / M_PI) * e_R.norm());
      next_debug_time = time + params.debug_period;
    }

    updateAutoDamping(params, gains, model, state, J, phase, after_contact,
                      pause_hold_active, damping);

    const Mat3& Dp_approach_eff =
        params.approach_auto_damping ? damping.Dp_approach : Dp_approach;
    const Mat3& DR_approach_eff =
        params.approach_auto_damping ? damping.DR_approach : DR_approach;
    const Mat3& Dp_setup_eff = params.setup_auto_damping ? damping.Dp_setup : Dp_setup;
    const Mat3& DR_setup_eff = params.setup_auto_damping ? damping.DR_setup : DR_setup;
    const Mat3& Dp_hold_eff = params.hold_auto_damping ? damping.Dp_hold : Dp_hold;
    const Mat3& DR_hold_eff = params.hold_auto_damping ? damping.DR_hold : DR_hold;
    const Mat3& Dp_pause_eff =
        params.pause_hold_auto_damping ? damping.Dp_pause : Dp_pause;
    const Mat3& DR_pause_eff =
        params.pause_hold_auto_damping ? damping.DR_pause : DR_pause;

    // ---------------------------------------------------------------
    // Select gains for this phase; gate hold overrides position gains.
    // ---------------------------------------------------------------
    // Hold uses the hold gains, unless the menu's t mode asked for the set-up
    // impedance instead.
    const Mat3* Kp_phase = params.hold_with_setup_gains ? &Kp_setup : &Kp_hold;
    const Mat3* Dp_phase =
        params.hold_with_setup_gains ? &Dp_setup_eff : &Dp_hold_eff;
    const Mat3* KR_phase = params.hold_with_setup_gains ? &KR_setup : &KR_hold;
    const Mat3* DR_phase =
        params.hold_with_setup_gains ? &DR_setup_eff : &DR_hold_eff;
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
    const Mat3& KR_used = pause_hold_active ? KR_pause : *KR_phase;
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

    ContactReference contact;
    contact.tcp = p_EE;
    contact.edge = tool_contact_point;
    contact.tcp_at_contact = first_contact_tcp;
    contact.edge_at_contact = first_contact_point;
    contact.R_EE = R_EE;
    const Vec6 wrench =
        computeSpringWrench(params, phase, Kp_used, Dp_used, KR_used, DR_used,
                            R_alignment_target, dx, dv, contact);
    const Vec3 f = wrench.head<3>();
    const Vec3 m = wrench.tail<3>();

    if (phase == ControlPhase::kGrind && params.grind_sweep_enabled &&
        params.print_grind_debug && params.debug_period > 0.0 &&
        time >= next_debug_time && intro_printed_for == phase) {
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

    // Live tuning typed as "d <value>" or "k <value>". Applied only while
    // holding: changing a gain inside a sequence run would leave the archived
    // params_effective describing something the robot never commanded.
    const double damping_request =
        signals.nullspace_damping_request.exchange(
            std::numeric_limits<double>::quiet_NaN());
    const double k_sigma_request =
        signals.nullspace_k_sigma_request.exchange(
            std::numeric_limits<double>::quiet_NaN());
    const double alpha_deg_request =
        signals.nullspace_alpha_deg_request.exchange(
            std::numeric_limits<double>::quiet_NaN());
    const int mode_request = signals.nullspace_mode_request.exchange(-1);
    if (std::isfinite(damping_request) || std::isfinite(k_sigma_request) ||
        std::isfinite(alpha_deg_request) || mode_request >= 0) {
      if (phase == ControlPhase::kHold && !params.hold_with_setup_gains) {
        if (std::isfinite(damping_request)) {
          params.nullspace_damping = damping_request;
          printf("nullspace damping -> %.3f Nms/rad\n", damping_request);
        }
        if (std::isfinite(k_sigma_request)) {
          params.nullspace_k_sigma = k_sigma_request;
          printf("nullspace k_sigma -> %.3f Nm\n", k_sigma_request);
        }
        if (std::isfinite(alpha_deg_request)) {
          // Typed in degrees, used in radians.
          params.nullspace_alpha = alpha_deg_request * M_PI / 180.0;
          printf("nullspace alpha -> %.3f deg = %.6f rad\n",
                 alpha_deg_request, params.nullspace_alpha);
        }
        if (mode_request >= 0) {
          params.nullspace_mode = static_cast<NullspaceMode>(mode_request);
          params.use_nullspace_optimization =
              params.nullspace_mode != NullspaceMode::kOff;
        }
        printNullspaceLaw(params);
      } else if (phase == ControlPhase::kHold) {
        printf("Ignored: the t hold does not tune the nullspace.\n");
      } else {
        printf("Ignored: nullspace tuning is only accepted while holding.\n");
      }
    }

    // The tilt the next sequence will command. It changes nothing while the
    // hold holds -- the hold keeps the pose it captured -- so the new value is
    // printed when it is accepted, and again by the s block that acts on it.
    for (int i = 0; i < 2; ++i) {
      const double tilt_deg = signals.setup_tilt_deg_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(tilt_deg)) {
        continue;
      }
      if (phase != ControlPhase::kHold || !params.hold_with_setup_gains) {
        printf("Ignored: the commanded tilt is only settable in the t hold.\n");
        continue;
      }
      if (i == 0) {
        params.tool_target_offset_tangent1_deg = tilt_deg;
      } else {
        params.tool_target_offset_tangent2_deg = tilt_deg;
      }
      printf("commanded tilt for s -> t1 %.2f deg | t2 %.2f deg\n",
             params.tool_target_offset_tangent1_deg,
             params.tool_target_offset_tangent2_deg);
    }

    // Set-up impedance, retuned while the t mode holds. The gain matrices are
    // built once per run, so they are rebuilt here, and the cached damping is
    // dropped so the next cycle refits it to the new stiffness.
    bool setup_impedance_changed = false;
    for (int i = 0; i < 3; ++i) {
      const double kp = signals.setup_kp_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double kr = signals.setup_kr_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double pole_mm = signals.setup_pole_mm_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double rc_mm = signals.setup_rc_mm_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(kp) && !std::isfinite(kr) && !std::isfinite(pole_mm) &&
          !std::isfinite(rc_mm)) {
        continue;
      }
      if (phase != ControlPhase::kHold || !params.hold_with_setup_gains) {
        printf("Ignored: the set-up impedance is only tunable in the t hold.\n");
        continue;
      }
      if (std::isfinite(kp) && kp > 0.0) {
        if (params.setup_translation_surface_frame) {
          params.setup_Kp_surface_diag(i) = kp;
        } else {
          params.setup_Kp_diag(i) = kp;
        }
        setup_impedance_changed = true;
      }
      if (std::isfinite(kr) && kr > 0.0) {
        params.setup_KR_diag(i) = kr;
        setup_impedance_changed = true;
      }
      // The compliance centre, in either of the two frames the block prints.
      // A request is read as a change to one component of the lever the
      // spring is commanding now, and written back in whichever convention
      // the run stores. Typing in the frame the run does not store is exact
      // when typed and no longer exact once the tool turns, which is the
      // difference between the conventions and not a rounding of it.
      if (std::isfinite(pole_mm) || std::isfinite(rc_mm)) {
        if (!params.use_coupled_stiffness) {
          printf("Ignored: the compliance centre belongs to the coupled "
                 "spring, and the decoupled spring is in use.\n");
        } else {
          const Vec3& tcp_ref = params.coupled_pole_freeze_at_contact
                                    ? contact.tcp_at_contact
                                    : contact.tcp;
          const Vec3& edge_ref = params.coupled_pole_freeze_at_contact
                                     ? contact.edge_at_contact
                                     : contact.edge;
          // The lever the spring is commanding right now, in the base frame.
          Vec3 r_c_base;
          if (params.coupled_use_pole_ee) {
            r_c_base = -(R_EE * params.coupled_pole_ee);
          } else if (params.coupled_use_direct_rc_surface) {
            r_c_base = R_alignment_target * params.coupled_rc_surface;
          } else {
            r_c_base = tcp_ref - (edge_ref + params.coupled_pole_from_edge);
          }

          // One component of it, in the frame the key names.
          if (std::isfinite(pole_mm)) {
            Vec3 pole_ee = -(R_EE.transpose() * r_c_base);
            pole_ee(i) = 0.001 * pole_mm;
            r_c_base = -(R_EE * pole_ee);
          }
          if (std::isfinite(rc_mm)) {
            Vec3 rc_surface = R_alignment_target.transpose() * r_c_base;
            rc_surface(i) = 0.001 * rc_mm;
            r_c_base = R_alignment_target * rc_surface;
          }

          // Back into the convention this run stores.
          if (params.coupled_use_pole_ee) {
            params.coupled_pole_ee = -(R_EE.transpose() * r_c_base);
          } else if (params.coupled_use_direct_rc_surface) {
            params.coupled_rc_surface =
                R_alignment_target.transpose() * r_c_base;
          } else {
            params.coupled_pole_from_edge = tcp_ref - edge_ref - r_c_base;
          }
          setup_impedance_changed = true;
        }
      }
    }
    if (setup_impedance_changed) {
      gains = buildRunGains(params);
      damping = manualDampingCache(gains);
      setup_law_printed = false;  // reprinted with the new gains
    }
    // The gate's stiff lock gets the same treatment as a phase: one block on
    // its first cycle.
    if (pause_hold_active && !gate_block_printed) {
      printGateHold(params, damping);
      gate_block_printed = true;
    }
    if (!pause_hold_active) {
      gate_block_printed = false;
    }

    // One intro per phase, on its first cycle. It sits after
    // updateAutoDamping, so the damping it reports is already fitted: no
    // waiting and no timing logic are involved.
    if (intro_printed_for != phase) {
      printPhaseHeader(phase);
      printPhaseIntro(params, damping, phase);
      intro_printed_for = phase;
    }

    const bool wants_setup_block =
        phase == ControlPhase::kSetUp ||
        (phase == ControlPhase::kHold && params.hold_with_setup_gains);
    if (wants_setup_block && !setup_law_printed) {
      printSetUpImpedanceLaw(params, damping,
                             phase == ControlPhase::kHold,
                             R_alignment_target, R_EE);
      setup_law_printed = true;
    }
    if (!wants_setup_block) {
      setup_law_printed = false;  // print it again on the next entry
    }
    // nullspace_mode controls whether this is active for the whole sequence.
    SigmaDiagnostics sigma_diagnostics;
    const Vec7 tau_nullspace =
        computeNullspaceTorque(
            params, model, state, J, dq, sigma_diagnostics);

    AutomaticDisturbance disturbance;
    if (phase == ControlPhase::kHold && params.disturbance_auto_enabled) {
      disturbance = computeAutomaticDisturbance(
          params, model, state, disturbance_force_direction_base,
          time - phase_start_time);
    }

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
    const Vec7 tau_cmd =
        tau_task + tau_nullspace + disturbance.tau + coriolis;

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
        debug_row.nullspace_damping = params.nullspace_damping;
        debug_row.tau_cmd_norm = tau_cmd.norm();
        debug_row.disturbance = disturbance;
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
      row.disturbance = disturbance;
      row.sigma = sigma_diagnostics;
      row.tau_nullspace_norm = tau_nullspace.norm();
      row.nullspace_damping = params.nullspace_damping;
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
        signals.stop_requested.load()) {
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
        stop_row.nullspace_damping = params.nullspace_damping;
        stop_row.tau_cmd_norm = tau_cmd.norm();
        stop_row.disturbance = disturbance;
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
      if (signals.stop_requested.load()) {
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
  result.log = std::move(ordered_log_data);

  // Hand the run's outcome back for the report and the CSV.
  result.descend_failed = descend_failed;
  result.q_start = q_start;
  result.final_q = final_q;
  result.final_p_d = final_p_d;
  result.final_p_EE = final_p_EE;
  result.final_e_p = final_e_p;
  result.final_e_R = final_e_R;
  return result;
}
