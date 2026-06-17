#pragma once

#include "controller_types.h"

inline void printVec3(const char* label, const Vec3& v) {
  printf("%s = [%.6f, %.6f, %.6f]\n", label, v(0), v(1), v(2));
}

inline void printVec3Mm(const char* label, const Vec3& v) {
  printf("%s = [%.1f, %.1f, %.1f] mm\n",
         label,
         1000.0 * v(0),
         1000.0 * v(1),
         1000.0 * v(2));
}

inline void printVec3Deg(const char* label, const Vec3& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.2f, %.2f, %.2f] deg\n",
         label,
         rad_to_deg * v(0),
         rad_to_deg * v(1),
         rad_to_deg * v(2));
}

inline void printVec7(const char* label, const Vec7& v) {
  printf("%s = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
         label,
         v(0),
         v(1),
         v(2),
         v(3),
         v(4),
         v(5),
         v(6));
}

inline void printVec7Deg(const char* label, const Vec7& v) {
  const double rad_to_deg = 180.0 / M_PI;
  printf("%s = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg\n",
         label,
         rad_to_deg * v(0),
         rad_to_deg * v(1),
         rad_to_deg * v(2),
         rad_to_deg * v(3),
         rad_to_deg * v(4),
         rad_to_deg * v(5),
         rad_to_deg * v(6));
}

inline void printMat3(const char* label, const Mat3& m) {
  printf("%s = [\n", label);
  printf("  %.6f, %.6f, %.6f;\n", m(0, 0), m(0, 1), m(0, 2));
  printf("  %.6f, %.6f, %.6f;\n", m(1, 0), m(1, 1), m(1, 2));
  printf("  %.6f, %.6f, %.6f\n", m(2, 0), m(2, 1), m(2, 2));
  printf("]\n");
}

inline void printMat4x4(const char* label, const Mat4x4& m) {
  printf("%s = [\n", label);
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(0, 0), m(0, 1), m(0, 2), m(0, 3));
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(1, 0), m(1, 1), m(1, 2), m(1, 3));
  printf("  %.6f, %.6f, %.6f, %.6f;\n", m(2, 0), m(2, 1), m(2, 2), m(2, 3));
  printf("  %.6f, %.6f, %.6f, %.6f\n", m(3, 0), m(3, 1), m(3, 2), m(3, 3));
  printf("]\n");
}

inline void printJointStartEndTable(const Vec7& q_start, const Vec7& q_final) {
  printf("\nq_start_q_final_delta = [\n");
  printf("  %% joint, q_start_rad, q_final_rad, delta_rad\n");
  for (int i = 0; i < 7; ++i) {
    printf("  %d, %.6f, %.6f, %.6f%s\n",
           i + 1,
           q_start(i),
           q_final(i),
           q_final(i) - q_start(i),
           (i == 6) ? "" : ";");
  }
  printf("]\n");
}

inline void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final) {
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

inline void printParameters(const Parameters& params) {
  printf("\n=== Controller setup ===\n");
  if (params.experiment_duration <= 0.0) {
    printf("time: indefinite, stop with e + Enter\n");
  } else {
    printf("time: %.1f s, or stop with e + Enter\n", params.experiment_duration);
  }
  printf("q_init: %s | contact: %s | orientation_test: %s\n",
         params.q_init_case.c_str(),
         params.use_contact_search ? "on" : "off",
         params.orientation_test_only ? "on" : "off");
  printf("gripper_open: %s | width=%.1f mm | require=%s\n",
         params.open_gripper_before_run ? "on" : "off",
         1000.0 * params.gripper_open_width,
         params.require_gripper_open ? "yes" : "no");
  printf("logging: every %d cycles | max_rows=%d\n",
         params.log_every_n_cycles,
         params.max_log_rows);
  printf("search_direction: %s\n",
         params.contact_search_use_surface_normal ? "-surface_normal" : "manual");
  printf("phases: %s | virtual_center: %s | vcr_offset: %.1f mm\n",
         params.use_phase_sequence ? "on" : "off",
         params.use_virtual_center_after_contact ? "on" : "off",
         1000.0 * params.vcr_offset);
  printf("gain_suggestion: %s | omega_ref=%.2f rad/s | angle_ref=%.2f rad\n",
         params.suggest_gains_from_desired_axis ? "on" : "off",
         params.suggested_gain_omega_ref,
         params.suggested_gain_angle_ref);
  printVec3Mm("desired_axis_from_edge", params.desired_axis_from_edge);
  printVec3("desired_axis_dir", normalizedOrFallback(params.desired_axis_dir, Vec3(1.0, 0.0, 0.0)));
  printf("desired_axis_pitch = %.1f mm/rad\n", 1000.0 * params.desired_axis_pitch);
  printVec3("surface_n", params.surface_normal);
  if (params.use_surface_tilt_angle) {
    printf("surface_tilt_angle: %.1f deg\n", params.surface_tilt_angle_deg);
  }
  printVec3("surface_t1", params.surface_tangent1);
  printVec3("tool_axis_ee", params.tool_axis_ee);
  printf("tool_axis_target_sign = %.0f\n",
         (params.tool_axis_target_sign >= 0.0) ? 1.0 : -1.0);
  printf("edge_point_control: %s | auto_edge: %s\n",
         params.use_tool_contact_point_control ? "on" : "off",
         params.auto_select_tool_contact_edge ? "on" : "off");
  printVec3Mm("tool_contact_point_ee", params.tool_contact_point_ee);
  printf("rot_axes [normal, tangent1, tangent2] = [%d, %d, %d]\n",
         params.constrain_rotation_about_surface_normal ? 1 : 0,
         params.constrain_rotation_about_surface_tangent1 ? 1 : 0,
         params.constrain_rotation_about_surface_tangent2 ? 1 : 0);
  printf("contact thresholds: search %.1f N, align %.1f N, post %.1f Nm\n",
         params.contact_force_threshold,
         params.alignment_contact_force_threshold,
         params.post_contact_moment_threshold);
  printf("orient_tol: %.1f deg | post_align: %.1f s\n",
         (180.0 / M_PI) * params.orient_phase_error_threshold,
         params.post_contact_align_duration);
  printf("post_push: %.1f mm + %.1f mm/s, max %.1f mm\n",
         1000.0 * params.post_contact_normal_push,
         1000.0 * params.post_contact_push_speed,
         1000.0 * params.post_contact_max_push);
  printVec3("post_Kp", params.post_contact_Kp_diag);
  printVec3("post_Dp", params.post_contact_Dp_diag);
  printVec3("post_KR [normal, tangent1, tangent2]", params.post_contact_KR_diag);
  printVec3("post_DR [normal, tangent1, tangent2]", params.post_contact_DR_diag);
  printf("contact_surface_after_align: %s\n",
         params.use_search_direction_surface_after_alignment ? "on" : "off");
  printVec3("Kp", params.Kp_diag);
  printVec3("Dp", params.Dp_diag);
  printVec3("KR", params.KR_diag);
  printVec3("DR", params.DR_diag);
  printf("nullspace: k_start=%.2f damping=%.2f k_sigma=%.2f tau_max=%.2f Nm\n",
         params.nullspace_k_start,
         params.nullspace_damping,
         params.nullspace_k_sigma,
         params.nullspace_tau_max);
}

inline void printFinalSummary(
    const Vec3& final_p_d,
    const Vec3& final_p_EE,
    const Vec3& final_e_p,
    const Vec3& final_e_R,
    const Vec3& final_instant_pole_to_edge,
    const Vec3& final_instant_axis_dir,
    const Vec3& desired_axis_from_edge,
    const Vec3& desired_axis_dir,
    double desired_axis_pitch,
    double final_instant_screw_pitch,
    double final_instant_edge_axis_distance,
    double final_instant_axis_time,
    bool final_instant_pole_valid,
    const std::string& csv_file_name) {
  printf("\n=== Final result ===\n");
  printVec3Mm("p_d", final_p_d);
  printVec3Mm("p_EE", final_p_EE);
  printVec3Mm("e_p", final_e_p);
  printf("position_error = %.2f mm\n", 1000.0 * final_e_p.norm());
  printVec3Deg("e_R", final_e_R);
  printf("rotation_error = %.2f deg\n", (180.0 / M_PI) * final_e_R.norm());
  if (final_instant_pole_valid) {
    printf("best_valid_axis: t=%.3f s | edge=%.2f mm | axis_point_from_edge=[%+.1f, %+.1f, %+.1f] mm | pitch=%.2f mm/rad | dir=[%+.3f, %+.3f, %+.3f]\n",
           final_instant_axis_time,
           1000.0 * final_instant_edge_axis_distance,
           1000.0 * final_instant_pole_to_edge(0),
           1000.0 * final_instant_pole_to_edge(1),
           1000.0 * final_instant_pole_to_edge(2),
           1000.0 * final_instant_screw_pitch,
           final_instant_axis_dir(0),
           final_instant_axis_dir(1),
           final_instant_axis_dir(2));
    const Vec3 desired_axis_dir_unit =
        normalizedOrFallback(desired_axis_dir, Vec3(1.0, 0.0, 0.0));
    const Vec3 axis_point_error = final_instant_pole_to_edge - desired_axis_from_edge;
    const double axis_dir_dot =
        std::abs(std::max(
            -1.0,
            std::min(1.0, final_instant_axis_dir.dot(desired_axis_dir_unit))));
    const double axis_dir_error_deg = (180.0 / M_PI) * std::acos(axis_dir_dot);
    const double desired_axis_edge_distance =
        pointDistanceToAxis(Vec3::Zero(), desired_axis_from_edge, desired_axis_dir_unit);
    printf("axis_error_vs_desired: point=[%+.1f, %+.1f, %+.1f] mm | point_norm=%.1f mm | dir_error=%.1f deg | edge_error=%+.1f mm | pitch_error=%+.1f mm/rad\n",
           1000.0 * axis_point_error(0),
           1000.0 * axis_point_error(1),
           1000.0 * axis_point_error(2),
           1000.0 * axis_point_error.norm(),
           axis_dir_error_deg,
           1000.0 * (final_instant_edge_axis_distance - desired_axis_edge_distance),
           1000.0 * (final_instant_screw_pitch - desired_axis_pitch));
  } else {
    printf("instant_axis: slow_rotation\n");
  }
  printf("csv: %s\n", csv_file_name.c_str());
}
