// ====================================================================
// Self-alignment / grinding controller for the Franka arm
// ====================================================================
// Connects the robot, then loops: startup menu -> one run -> report. A run
// is either the approach / set up / grind sequence or a hold, chosen in the
// menu. The run itself lives in run_loop.cpp.
#include "controller.h"

int main() {
  try {
    // ================================================================
    // 1. Parameters, robot connection, start pose
    // ================================================================
    // One topic per file; see parameterFiles() for the list and load order.
    const Parameters base_params = readParameters(parameterFiles());
    Robot robot(base_params.robot_ip);

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

    configureCollisionBehavior(robot, base_params);

    Model model = robot.loadModel();

    // Owned by main so one keyboard thread serves every session; it parks
    // itself while the startup menu owns stdin.
    KeyboardSignals signals;
    startKeyboardStopThread(base_params, signals);

    // One session per menu visit. m+Enter during a run comes back here; the
    // second run writes ..._log_s2.csv, so no earlier log is overwritten.
    for (int session = 1;; ++session) {
      Parameters params = base_params;
      if (session > 1) {
        params.csv_file_name = sessionFileName(params.csv_file_name, session);
        params.sigma_debug_csv_file_name =
            sessionFileName(params.sigma_debug_csv_file_name, session);
      }
      signals.stop_requested.store(false);
      signals.proceed_requested.store(false);
      signals.guide_requested.store(false);
      signals.guidance_menu_key.store(0);
      signals.gate_continue.store(false);

      if (!askStartupRunMode(params, robot, model)) {
        break;
      }
      signals.guided_hold_selector_pending.store(
          params.use_manual_guidance_start);
      // The menu is done with stdin: let the keyboard thread read again.
      signals.menu_requested.store(false);

      printParameters(params);

      if (params.use_phase_sequence && params.use_coupled_stiffness &&
          !params.coupled_use_block_diagonal && !params.coupled_pole_manual) {
        fprintf(stderr,
                "Coupled stiffness requires either block-diagonal mode or a "
                "deliberately commanded pole.\n");
        return -1;
      }

      if (!performStartupGripperAction(params)) {
        return -1;
      }

      if (params.use_manual_guidance_start &&
          !runManualGuidanceStart(params, robot, model,
                                  signals.stop_requested,
                                  signals.guidance_menu_key,
                                  signals.guided_hold_selector_pending)) {
        // Guiding was ended by m+Enter rather than e+Enter: ask again.
        if (signals.menu_requested.load()) {
          printf("\n=== returning to the startup menu ===\n");
          continue;
        }
        return 0;
      }

      const RunGains gains = buildRunGains(params);
      RunResult result =
          runControlLoop(params, robot, model, gains, signals);
      writeRunLogs(params, result);


      if (!signals.menu_requested.load()) {
        break;
      }
      printf("\n=== returning to the startup menu ===\n");
    }


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

