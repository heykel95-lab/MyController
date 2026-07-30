// ====================================================================
// Plane point capture
// ====================================================================
// Read-only, commands no motion. Touch the same +X_EE,+Y_EE tool-face corner
// to the workpiece for every point; the measured EE pose is converted to that
// corner's base position and appended to the profile's plane_points.csv.
#include "controller.h"

#include <fstream>

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
  if (argc != 3) {
    fprintf(stderr, "usage: %s tilted|horizontal P1|P2|P3|P4\n", argv[0]);
    return 2;
  }

  const std::string profile(argv[1]);
  if (profile != "tilted" && profile != "horizontal") {
    fprintf(stderr, "Plane profile must be 'tilted' or 'horizontal'.\n");
    return 2;
  }
  const std::string label(argv[2]);
  if (label.empty() || label.find(',') != std::string::npos) {
    fprintf(stderr, "Point label must be non-empty and contain no comma.\n");
    return 2;
  }

  const std::string output_path =
      "../experiments/calibration/planes/" + profile + "/plane_points.csv";
  if (labelAlreadyExists(output_path, label)) {
    fprintf(stderr,
            "Point %s already exists in %s. Remove that row explicitly before "
            "re-recording it.\n",
            label.c_str(), output_path.c_str());
    return 2;
  }

  try {
    const Parameters params =
        readParameters(parameterFiles());

    printf("Read-only plane-point capture. No robot motion is commanded.\n");
    printf("Touch the SAME +X_EE,+Y_EE tool-face corner to the plane.\n");
    printf("Plane profile: %s\n", profile.c_str());
    printf("Capturing %s from robot %s\n", label.c_str(), params.robot_ip.c_str());

    Robot robot(params.robot_ip);
    const RobotState state = robot.readOnce();
    Map<const Mat4x4> T_EE(state.O_T_EE.data());
    const Mat3 R_EE = T_EE.block<3, 3>(0, 0);
    const Vec3 p_EE = T_EE.block<3, 1>(0, 3);

    const Vec3 probe_offset_ee =
        params.tool_contact_face_center_ee +
        params.tool_contact_half_width_ee +
        params.tool_contact_half_length_ee;
    const Vec3 probe_point = p_EE + R_EE * probe_offset_ee;

    bool write_header = true;
    {
      std::ifstream existing(output_path);
      write_header = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream output(output_path, std::ios::app);
    if (!output) {
      fprintf(stderr, "Cannot open %s for writing.\n", output_path.c_str());
      return 2;
    }
    if (write_header) {
      output << "label,x_m,y_m,z_m\n";
    }
    output << std::fixed << std::setprecision(9)
           << label << ","
           << probe_point(0) << ","
           << probe_point(1) << ","
           << probe_point(2) << "\n";

    printf("EE position [m]    = [%+.6f, %+.6f, %+.6f]\n",
           p_EE(0), p_EE(1), p_EE(2));
    printf("probe offset EE [m]= [%+.6f, %+.6f, %+.6f]\n",
           probe_offset_ee(0), probe_offset_ee(1), probe_offset_ee(2));
    printf("%s base point [m] = [%+.9f, %+.9f, %+.9f]\n",
           label.c_str(), probe_point(0), probe_point(1), probe_point(2));
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
