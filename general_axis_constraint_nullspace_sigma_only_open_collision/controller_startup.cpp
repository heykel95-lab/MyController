#include "controller_startup.h"

#include "controller_helpers.h"
#include "controller_printing.h"

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

void askStartupRunMode(Parameters& params) {
  const bool default_sequence = params.use_phase_sequence;

  // Menu 1: where the start pose comes from. Loops so the gripper action runs
  // and returns here instead of leaving the menu.
  while (true) {
    printf("\n=== Startup choice ===\n");
    printf("Start pose:\n");
    printf("  q = go to q_init (from params/common.txt)\n");
    printf("  g = guiding mode: go to q_init, then hand-guide to your pose,\n");
    printf("      p+Enter to lock it as the start (e+Enter prints it as q_init)\n");
    printf("  o = open the Franka hand now (release/load tool), then choose again\n");
    printf("Choice [q/g/o, Enter = q]: ");

    const std::string choice = readChoice();
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
    if (matches(choice, {"g", "guide", "guiding"})) {
      params.use_manual_guidance_start = true;
      printf("Selected: guiding mode (hand-place the start pose after q_init).\n");
    } else {
      params.use_manual_guidance_start = false;
      if (!choice.empty() && !matches(choice, {"q", "qinit"})) {
        printf("Unknown start-pose choice '%s'; using q_init.\n", choice.c_str());
      }
    }
    break;
  }

  // In guiding mode the run mode is chosen at the END of guiding, so asking
  // here would be redundant.
  if (params.use_manual_guidance_start) {
    printf("Run mode (s/h) will be chosen at the end of guiding.\n");
    return;
  }

  // Menu 2: run mode, with the same on-demand gripper action.
  while (true) {
    printf("\n=== Run mode ===\n");
    printf("  s = phase sequence (approach / set up / grind)\n");
    printf("  h = hold at the start pose\n");
    printf("  c = close/grasp the tool now, then choose again\n");
    printf("Choice [s/h/c, Enter = %s]: ", default_sequence ? "s" : "h");

    const std::string choice = readChoice();
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
    break;
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
