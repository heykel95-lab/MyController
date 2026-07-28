#include "controller.h"

// Read-only measurement of the physical surface tilt. Commands no motion.
//
// The controller measures alignment against the CONFIGURED normal, built in
// config.cpp from alignment_target_tilt_angle_deg (a, about base x) and
// alignment_target_tilt_angle_y_deg (b, about base y). Those two numbers are
// deliberately left slightly off the real plane, which is the point of the
// experiment -- but it means align_after at equilibrium is a residual against
// an assumption, not against the surface the tool actually sat on. Interpreting
// the B-series sweep needs the real normal as a separate, measured constant.
//
// Procedure:
//   1. Start the controller, choose g (guiding mode), and hand-place the tool
//      face FLAT on the plane. Take your time -- rock it until it seats on the
//      whole face, not a corner or an edge.
//   2. Leave it seated and stop the controller.
//   3. Run this. It reads the pose once and prints the tilt angles the surface
//      would need for the tool axis to be its normal.
//
// The number it prints is only as good as how flat you seated the face. Repeat
// it a few times and take the spread as the uncertainty.

namespace {

// Inverse of the construction in config.cpp:
//   n = R_y(b) * R_x(a) * [0,0,1] = [sin b cos a, -sin a, cos b cos a]
void tiltAnglesFromNormal(const Vec3& n, double& a_deg, double& b_deg) {
  const double a = std::asin(std::max(-1.0, std::min(1.0, -n(1))));
  const double b = std::atan2(n(0), n(2));
  a_deg = (180.0 / M_PI) * a;
  b_deg = (180.0 / M_PI) * b;
}

}  // namespace

int main() {
  try {
    const Parameters params =
        readParameters({"params/common.txt", "params/sequence.txt"});

    printf("Read-only plane measurement. No robot motion is commanded.\n");
    printf("Seat the tool face FLAT on the plane before running this.\n");
    printf("Connecting to robot: %s\n\n", params.robot_ip.c_str());

    Robot robot(params.robot_ip);
    const RobotState state = robot.readOnce();

    Map<const Mat4x4> T_EE(state.O_T_EE.data());
    const Mat3 R_EE = T_EE.block<3, 3>(0, 0);
    const Vec3 p_EE = T_EE.block<3, 1>(0, 3);

    Vec3 tool_axis_ee = params.tool_axis_ee;
    if (tool_axis_ee.norm() < 1e-9) {
      tool_axis_ee = Vec3(0.0, 0.0, 1.0);
    }
    tool_axis_ee.normalize();

    // Point the measured normal out of the surface, the same convention the
    // configured normal uses, so the two are directly comparable.
    Vec3 n_measured = R_EE * tool_axis_ee;
    if (n_measured(2) < 0.0) {
      n_measured = -n_measured;
    }
    n_measured.normalize();

    double a_meas = 0.0, b_meas = 0.0;
    tiltAnglesFromNormal(n_measured, a_meas, b_meas);

    Vec3 n_config = params.alignment_target_normal;
    if (n_config(2) < 0.0) {
      n_config = -n_config;
    }
    n_config.normalize();

    const double dot =
        std::max(-1.0, std::min(1.0, n_measured.dot(n_config)));
    const double mismatch_deg = (180.0 / M_PI) * std::acos(dot);

    printf("TCP position [mm]      = [%+8.1f, %+8.1f, %+8.1f]\n",
           1000.0 * p_EE(0), 1000.0 * p_EE(1), 1000.0 * p_EE(2));
    printf("tool_axis_ee           = [%+7.3f, %+7.3f, %+7.3f]\n\n",
           tool_axis_ee(0), tool_axis_ee(1), tool_axis_ee(2));

    printf("measured plane normal  = [%+7.4f, %+7.4f, %+7.4f]\n",
           n_measured(0), n_measured(1), n_measured(2));
    printf("configured normal      = [%+7.4f, %+7.4f, %+7.4f]\n\n",
           n_config(0), n_config(1), n_config(2));

    printf("MEASURED tilt: a(x) = %+6.2f deg | b(y) = %+6.2f deg\n",
           a_meas, b_meas);
    printf("CONFIGURED   : a(x) = %+6.2f deg | b(y) = %+6.2f deg\n",
           params.alignment_target_tilt_angle_deg,
           params.alignment_target_tilt_angle_y_deg);
    printf("difference   : a(x) = %+6.2f deg | b(y) = %+6.2f deg\n\n",
           a_meas - params.alignment_target_tilt_angle_deg,
           b_meas - params.alignment_target_tilt_angle_y_deg);

    printf("total mismatch between measured and configured normal: %.2f deg\n",
           mismatch_deg);
    printf("\nThis is the number align_after is measured against. A perfectly\n"
           "seated tool reports align_after ~= %.2f deg, not zero.\n",
           mismatch_deg);
    printf("\nDo NOT paste these into params/ -- the mismatch is the experiment.\n");

    return 0;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
