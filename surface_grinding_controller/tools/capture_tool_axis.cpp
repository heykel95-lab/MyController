#include "controller.h"

#include <fstream>

// Read-only capture of the physical grinding-face normal in the EE frame.
//
// Place the complete tool face flat on a validated physical plane. If n is the
// calibrated upward plane normal and tool_axis_target_sign=-1, the physical
// tool axis in base coordinates is -n. For the measured EE orientation R_EE,
// the corresponding constant tool-frame vector is
//
//   a_EE = R_EE^T (sign * n).
//
// Four samples are captured. T1--T3 estimate the vector and T4 validates it.

namespace {

bool labelAlreadyExists(const std::string& path, const std::string& label) {
  std::ifstream input(path);
  std::string line;
  const std::string prefix = label + ",";
  while (std::getline(input, line)) {
    if (line.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    fprintf(stderr,
            "usage: %s grinding_tool horizontal|tilted T1|T2|T3|T4\n",
            argv[0]);
    return 2;
  }

  const std::string tool_profile(argv[1]);
  if (tool_profile != "grinding_tool") {
    fprintf(stderr, "Tool profile must be 'grinding_tool'.\n");
    return 2;
  }
  const std::string plane_profile(argv[2]);
  if (plane_profile != "horizontal" && plane_profile != "tilted") {
    fprintf(stderr, "Plane profile must be 'horizontal' or 'tilted'.\n");
    return 2;
  }
  const std::string label(argv[3]);
  if (label != "T1" && label != "T2" &&
      label != "T3" && label != "T4") {
    fprintf(stderr, "Sample label must be T1, T2, T3, or T4.\n");
    return 2;
  }

  const std::string plane_overlay =
      "../experiments/calibration/planes/" + plane_profile +
      "/plane_overlay.txt";
  const std::string output_path =
      "../experiments/calibration/tools/" + tool_profile +
      "/tool_axis_samples.csv";
  if (labelAlreadyExists(output_path, label)) {
    fprintf(stderr,
            "Sample %s already exists in %s. Remove that row explicitly "
            "before re-recording it.\n",
            label.c_str(), output_path.c_str());
    return 2;
  }

  try {
    const Parameters params =
        readParameters({"params/common.txt", plane_overlay});

    printf("Read-only tool-axis capture. No robot motion is commanded.\n");
    printf("The COMPLETE tool face must be flat on the validated %s plane.\n",
           plane_profile.c_str());
    printf("Tool profile: %s | sample: %s\n",
           tool_profile.c_str(), label.c_str());
    printf("Connecting to robot: %s\n", params.robot_ip.c_str());

    Robot robot(params.robot_ip);
    const RobotState state = robot.readOnce();
    Map<const Mat4x4> T_EE(state.O_T_EE.data());
    const Mat3 R_EE = T_EE.block<3, 3>(0, 0);
    const double sign = params.tool_axis_target_sign >= 0.0 ? 1.0 : -1.0;
    const Vec3 target_axis_base =
        sign * params.alignment_target_normal.normalized();
    const Vec3 sample_axis_ee =
        (R_EE.transpose() * target_axis_base).normalized();
    const Vec3 nominal_axis_ee =
        params.tool_axis_ee.normalized();
    const double dot =
        std::max(-1.0, std::min(1.0, sample_axis_ee.dot(nominal_axis_ee)));
    const double nominal_offset_deg = 180.0 / M_PI * std::acos(dot);

    bool write_header = true;
    {
      std::ifstream existing(output_path);
      write_header =
          !existing.good() ||
          existing.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream output(output_path, std::ios::app);
    if (!output) {
      fprintf(stderr, "Cannot open %s for writing.\n", output_path.c_str());
      return 2;
    }
    if (write_header) {
      output << "label,plane_profile,axis_ee_x,axis_ee_y,axis_ee_z\n";
    }
    output << std::fixed << std::setprecision(12)
           << label << "," << plane_profile << ","
           << sample_axis_ee(0) << ","
           << sample_axis_ee(1) << ","
           << sample_axis_ee(2) << "\n";

    printf("surface normal base = [%+.9f, %+.9f, %+.9f]\n",
           params.alignment_target_normal(0),
           params.alignment_target_normal(1),
           params.alignment_target_normal(2));
    printf("sample tool axis EE = [%+.9f, %+.9f, %+.9f]\n",
           sample_axis_ee(0), sample_axis_ee(1), sample_axis_ee(2));
    printf("offset from currently configured tool axis = %.4f deg\n",
           nominal_offset_deg);
    printf("appended to %s\n", output_path.c_str());
    return 0;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
