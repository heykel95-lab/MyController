// ====================================================================
// Franka Hand homing
// ====================================================================
// Homes the hand, then optionally re-grasps the tool. Separate from the
// controller because homing drives the fingers to both limits: THE TOOL WILL
// FALL unless it is held or supported. The program asks before moving.
//
// Needed after a power cycle or gripper fault, where the hand loses its width
// calibration and move()/grasp() then report success without travelling.
#include "controller.h"

int main() {
  try {
    const Parameters params = readParameters(parameterFiles());

    printf("Franka Hand homing.\n");
    printf("Robot: %s\n\n", params.robot_ip.c_str());

    Gripper gripper(params.robot_ip);
    const franka::GripperState before = gripper.readOnce();
    printf("current width: %.1f mm (max %.1f mm)\n",
           1000.0 * before.width, 1000.0 * before.max_width);
    printf("grasped flag : %s\n\n", before.is_grasped ? "yes" : "no");

    printf("Homing opens the hand fully. IF THE TOOL IS IN THE GRIPPER IT WILL\n"
           "FALL. Support it by hand, or take it out first.\n\n");
    printf("Type  home  and press Enter to proceed, anything else to abort: ");
    fflush(stdout);

    std::string answer;
    if (!std::getline(std::cin, answer) || answer != "home") {
      printf("\nAborted. Nothing was moved.\n");
      return 0;
    }

    printf("\nHoming...\n");
    if (!gripper.homing()) {
      fprintf(stderr, "homing() returned false.\n");
      return 1;
    }
    const franka::GripperState after = gripper.readOnce();
    printf("Homed. width now %.1f mm, max %.1f mm\n",
           1000.0 * after.width, 1000.0 * after.max_width);

    printf("\nRe-grasp the tool now at %.1f mm / %.1f N?\n",
           1000.0 * params.gripper_grasp_width, params.gripper_grasp_force);
    printf("Put the tool between the fingers first. Type  grasp  to close, "
           "anything else to leave the hand open: ");
    fflush(stdout);

    if (!std::getline(std::cin, answer) || answer != "grasp") {
      printf("\nLeft open. Use the controller's c option when ready.\n");
      return 0;
    }

    const bool ok = gripper.grasp(params.gripper_grasp_width,
                                  params.gripper_grasp_speed,
                                  params.gripper_grasp_force,
                                  params.gripper_grasp_epsilon_inner,
                                  params.gripper_grasp_epsilon_outer);
    const franka::GripperState held = gripper.readOnce();
    printf("\ngrasp() %s | width %.1f mm | grasped flag %s\n",
           ok ? "succeeded" : "returned false",
           1000.0 * held.width, held.is_grasped ? "yes" : "no");
    if (!ok) {
      printf("If the width looks right the tool is probably held anyway; the\n"
             "flag only reports whether it landed inside the epsilon band.\n");
    }
    return ok ? 0 : 1;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
