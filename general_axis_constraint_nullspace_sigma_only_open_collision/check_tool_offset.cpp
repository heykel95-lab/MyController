#include "controller_helpers.h"
#include "controller_parameters.h"

void printTransform(const char* name, const std::array<double, 16>& transform) {
  printf("%s:\n", name);
  for (int row = 0; row < 4; ++row) {
    printf("  ");
    for (int col = 0; col < 4; ++col) {
      printf("% .6f ", transform[col * 4 + row]);
    }
    printf("\n");
  }
}

void printTranslation(const char* name, const std::array<double, 16>& transform) {
  printf("%s translation [m]: x=% .6f  y=% .6f  z=% .6f\n",
         name,
         transform[12],
         transform[13],
         transform[14]);
}

int main() {
  try {
    const Parameters params = readParameters("parameters.txt");

    printf("Read-only tool offset check. No robot motion is commanded.\n");
    printf("Connecting to robot: %s\n", params.robot_ip.c_str());

    Robot robot(params.robot_ip);
    const RobotState state = robot.readOnce();

    printf("\nF_T_EE is the transform from flange F to end-effector/tool TCP EE.\n");
    printf("If this translation is all zero, no physical tool TCP offset is configured.\n\n");

    printTransform("F_T_EE", state.F_T_EE);
    printTranslation("F_T_EE", state.F_T_EE);

    printf("\nEE_T_K is the transform from end-effector EE to stiffness frame K.\n");
    printTransform("EE_T_K", state.EE_T_K);
    printTranslation("EE_T_K", state.EE_T_K);

    return 0;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
