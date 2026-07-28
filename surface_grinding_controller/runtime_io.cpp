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
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  log_file << std::fixed << std::setprecision(6);
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
             << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6)
             << "\n";
  }
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
    case NullspaceMode::kPostureOnly:
      return "tau_nullspace_only";
    case NullspaceMode::kSigmaOnly:
      return "tau_sigma_only";
    case NullspaceMode::kPostureAndSigma:
      return "tau_nullspace_plus_tau_sigma";
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
         params.use_phase_sequence ? "phase sequence" : "hold",
         params.use_approach_orient ? "on" : "off",
         nullspaceModeName(params.nullspace_mode));
  printf("hold: Kp=%.1f N/m | KR=%.1f Nm/rad | damping=%s",
         params.hold_Kp, params.hold_KR,
         params.hold_auto_damping ? "auto translation+rotation" : "manual");
  if (params.hold_auto_damping) {
    if (params.hold_auto_match_manual_damping) {
      printf(" (factor fitted online toward Dp=%.1f Ns/m)\n", params.hold_Dp);
    } else {
      printf(" (factor %.2f)\n", params.hold_auto_damping_factor);
    }
  } else {
    printf(" (Dp=%.1f Ns/m, DR=%.1f Nms/rad)\n",
           params.hold_Dp, params.hold_DR);
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
  printf("coupled stiffness: apply=%s | saved=%s | pole=%s\n",
         params.use_coupled_stiffness ? "on" : "off",
         params.coupled_gains_saved ? "yes" : "no",
         params.coupled_use_block_diagonal
             ? "block-diagonal (no coupling)"
             : (params.coupled_pole_manual ? "manual" : "saved matrices"));
  printf("gates: pause_before_set_up=%s | pause_auto_damping=%s | "
         "pause_before_grind=%s | debug_period=%.2f s\n",
         params.pause_before_set_up ? "on" : "off",
         params.pause_hold_auto_damping ? "on" : "off",
         params.pause_before_grind ? "on" : "off",
         params.debug_period);
  printf("alignment-target normal=[%+.3f, %+.3f, %+.3f] | tilt a(x)=%.1f deg, b(y)=%.1f deg (from angles)\n",
         params.alignment_target_normal(0),
         params.alignment_target_normal(1),
         params.alignment_target_normal(2),
         params.alignment_target_tilt_angle_deg,
         params.alignment_target_tilt_angle_y_deg);
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

NullspaceMode askHoldNullspaceMode(NullspaceMode configured) {
  // The combined mode is meant for the sequence run; hold defaults to the
  // posture spring unless the file already asked for something simpler.
  const NullspaceMode fallback =
      (configured == NullspaceMode::kPostureAndSigma) ? NullspaceMode::kPostureOnly
                                                      : configured;
  printf("\nSelect hold nullspace mode:\n");
  printf("  0 = no nullspace torque\n");
  printf("  1 = tau_nullspace only (posture spring + damping)\n");
  printf("  2 = tau_sigma only (singular-value direction)\n");
  printf("Choice [0/1/2, Enter = %s]: ",
         fallback == NullspaceMode::kOff ? "0"
             : fallback == NullspaceMode::kSigmaOnly ? "2" : "1");

  const std::string choice = readChoice();
  if (matches(choice, {"0", "off", "none"})) {
    return NullspaceMode::kOff;
  }
  if (matches(choice, {"1", "tau_nullspace", "nullspace", "posture"})) {
    return NullspaceMode::kPostureOnly;
  }
  if (matches(choice, {"2", "tau_sigma", "sigma"})) {
    return NullspaceMode::kSigmaOnly;
  }
  return fallback;
}

}  // namespace

bool openGripper(const Parameters& params, Gripper& gripper) {
  try {
    printf("Opening gripper to %.1f mm...\n", 1000.0 * params.gripper_open_width);
    const bool opened = gripper.move(params.gripper_open_width, params.gripper_open_speed);
    printf(opened ? "Gripper opened.\n" : "Gripper open command returned false.\n");
    return opened;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper open failed: %s\n", e.what());
    return false;
  }
}

bool graspTool(const Parameters& params, Gripper& gripper) {
  try {
    printf("Grasping tool: width %.1f mm, force %.1f N...\n",
           1000.0 * params.gripper_grasp_width, params.gripper_grasp_force);
    // epsilon_inner/outer set how far the final width may fall short of /
    // exceed the target and still count as a successful grasp.
    const bool grasped = gripper.grasp(
        params.gripper_grasp_width, params.gripper_grasp_speed,
        params.gripper_grasp_force, params.gripper_grasp_epsilon_inner,
        params.gripper_grasp_epsilon_outer);
    printf(grasped ? "Gripper closed on the tool.\n"
                   : "Gripper grasp returned false (tool not held within tolerance).\n");
    return grasped;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper grasp failed: %s\n", e.what());
    return false;
  }
}

void askStartupRunMode(Parameters& params, Robot& robot) {
  const bool default_sequence = params.use_phase_sequence;
  bool q_init_reached = false;

  const auto move_to_q_init = [&]() {
    printf("Moving to q_init...\n");
    MotionGenerator motion_generator(0.4, params.q_init);
    robot.control(motion_generator);
    printf("q_init reached.\n");
    q_init_reached = true;
  };

  // Menu 1: where the start pose comes from. Loops so the gripper action runs
  // and q_init can be reached before the user continues.
  while (true) {
    printf("\n=== Startup choice ===\n");
    printf("  q = go to q_init (from params/common.txt), then choose again\n");
    printf("  g = guiding mode: go to q_init, then hand-guide to your pose,\n");
    printf("      p+Enter to lock it as the start (e+Enter prints it as q_init)\n");
    printf("  o = open the Franka hand now (release/load tool), then choose again\n");
    printf("  c = close/grasp the tool now, then choose again\n");
    printf("  r = continue to run mode%s\n",
           q_init_reached ? "" : " (available after q_init is reached)");
    printf("Choice [q/g/o/c/r, Enter = q]: ");

    const std::string choice = readChoice();
    if (choice.empty() || matches(choice, {"q", "qinit"})) {
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
    if (matches(choice, {"r", "run", "continue"})) {
      if (!q_init_reached) {
        printf("Reach q_init with q before continuing.\n");
        continue;
      }
      params.use_manual_guidance_start = false;
      break;
    }
    if (matches(choice, {"g", "guide", "guiding"})) {
      if (!q_init_reached) {
        move_to_q_init();
      }
      params.use_manual_guidance_start = true;
      printf("Selected: guiding mode (hand-place the start pose after q_init).\n");
      break;
    }
    printf("Unknown startup choice '%s'; choose q, g, o, c, or r.\n",
           choice.c_str());
  }

  // In guiding mode the run mode is chosen at the END of guiding, so asking
  // here would be redundant.
  if (params.use_manual_guidance_start) {
    printf("Run mode (s/h) will be chosen at the end of guiding.\n");
    return;
  }

  // Menu 2: run mode.
  printf("\n=== Run mode ===\n");
  printf("  s = phase sequence (approach / set up / grind)\n");
  printf("  h = hold at the start pose\n");
  printf("Choice [s/h, Enter = %s]: ", default_sequence ? "s" : "h");

  const std::string choice = readChoice();
  if (matches(choice, {"s", "sequence"})) {
    params.use_phase_sequence = true;
  } else if (matches(choice, {"h", "hold"})) {
    params.use_phase_sequence = false;
  } else {
    params.use_phase_sequence = default_sequence;
    if (!choice.empty()) {
      printf("Unknown run-mode choice '%s'; using default %s.\n",
             choice.c_str(), default_sequence ? "sequence" : "hold");
    }
  }

  if (params.use_phase_sequence) {
    printf("Selected: phase sequence. Nullspace mode from parameters: %s.\n",
           nullspaceModeName(params.nullspace_mode));
    return;
  }

  params.nullspace_mode = askHoldNullspaceMode(params.nullspace_mode);
  params.use_nullspace_optimization = (params.nullspace_mode != NullspaceMode::kOff);
  printf("Selected: hold mode with %s.\n", nullspaceModeName(params.nullspace_mode));
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
      // If the fingers are already closed on the tool from a previous run, keep
      // the grasp. Re-grasping with no gap left to close into reports failure,
      // and re-homing would open the hand and drop the tool.
      const franka::GripperState state = gripper.readOnce();
      const bool already_holding =
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
                            std::atomic<char>& guidance_menu_key) {
  Gripper gripper(params.robot_ip);
  Vec7 stop_q = Vec7::Zero();

  while (true) {
    printf("\nphase: manual_guidance_start\n");
    printf("Move the robot by hand. Then:\n");
    printf("  o+Enter = open the gripper\n");
    printf("  c+Enter = close/grasp the tool\n");
    printf("  s+Enter = start phase sequence from this pose\n");
    printf("  h+Enter = start hold from this pose\n");
    printf("  e+Enter = stop (prints this pose as a q_init_* case)\n");
    guidance_menu_key.store(0);

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
      printf("\n=== Manual guidance stop pose ===\n");
      printf("q1..q7 [rad] (paste into a q_init_* case):\n");
      for (int i = 0; i < 7; ++i) {
        printf("  q_init_%d = %.6f\n", i + 1, stop_q(i));
      }
      printVec7Deg("q1..q7 [deg]", stop_q);
      return false;
    }

    const char key = guidance_menu_key.exchange(0);
    if (key == 'o') {
      openGripper(params, gripper);
    } else if (key == 'c') {
      graspTool(params, gripper);
    } else if (key == 's') {
      params.use_phase_sequence = true;
      printf("Selected: phase sequence from the guided pose.\n");
      return true;
    } else if (key == 'h') {
      params.use_phase_sequence = false;
      printf("Selected: hold at the guided pose.\n");
      return true;
    }
  }
}
