// ====================================================================
// Automatic nullspace-disturbance inspector
// ====================================================================
// Reads one robot state and evaluates the configured link-point force. It
// never enters control or commands motion; use it before a Case F trial.
#include "controller.h"

int main(int argc, char** argv) {
  const std::string params_dir = (argc > 1) ? argv[1] : "params";
  try {
    std::vector<std::string> files = parameterFiles(params_dir);
    if (argc > 2) {
      files.push_back(argv[2]);
    }
    const Parameters params = readParameters(files);
    std::string error;
    if (!validateAutomaticDisturbance(params, error)) {
      fprintf(stderr, "ERROR: automatic disturbance: %s.\n", error.c_str());
      return 2;
    }
    if (!params.disturbance_auto_enabled) {
      fprintf(stderr, "ERROR: disturbance_auto_enabled is 0.\n");
      return 2;
    }

    Robot robot(params.robot_ip);
    Model model = robot.loadModel();
    const RobotState state = robot.readOnce();
    const std::array<double, 42> ee_jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, state);
    Map<const Mat6x7> J_ee(ee_jacobian_array.data());
    const Vec3 direction =
        automaticDisturbanceDirection(params, model, state, J_ee);
    if (direction.norm() <= 1e-9) {
      fprintf(stderr, "ERROR: selected point cannot excite the redundant axis.\n");
      return 2;
    }

    const AutomaticDisturbance command = computeAutomaticDisturbance(
        params, model, state, direction, params.disturbance_hold_time);
    Eigen::JacobiSVD<Mat6x7> svd(
        J_ee, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Vec7 n = svd.matrixV().col(6).normalized();

    printf("automatic disturbance inspection (read only)\n");
    printf("link = %d\n", params.disturbance_link);
    printf("point base = [%+.6f, %+.6f, %+.6f] m\n",
           command.point_base(0), command.point_base(1),
           command.point_base(2));
    printf("force direction base = [%+.6f, %+.6f, %+.6f]\n",
           direction(0), direction(1), direction(2));
    printf("applied force = [%+.3f, %+.3f, %+.3f] N\n",
           command.force_base(0), command.force_base(1),
           command.force_base(2));
    printf("joint torque = [%+.3f, %+.3f, %+.3f, %+.3f, %+.3f, %+.3f, %+.3f] Nm\n",
           command.tau(0), command.tau(1), command.tau(2), command.tau(3),
           command.tau(4), command.tau(5), command.tau(6));
    printf("torque norm = %.3f Nm (scale %.3f)\n",
           command.tau.norm(), command.torque_scale);
    printf("torque along redundant axis = %.3f Nm\n",
           std::abs(n.dot(command.tau)));
    return 0;
  } catch (const std::exception& e) {
    fprintf(stderr, "inspection exception: %s\n", e.what());
    return 2;
  }
}
