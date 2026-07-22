#pragma once

#include "controller_types.h"

// Opens the hand to gripper_open_width. Reports failures but never throws.
bool openGripper(const Parameters& params, Gripper& gripper);

// Closes the fingers onto the held tool at gripper_grasp_width /
// gripper_grasp_force. Reports failures but never throws.
bool graspTool(const Parameters& params, Gripper& gripper);

// The two startup menus: where the start pose comes from (q_init or manual
// hand guidance) and which run mode to use (phase sequence or hold). Both
// menus also offer the gripper actions, so the tool can be loaded or released
// without leaving the menu. Writes the choices back into params.
void askStartupRunMode(Parameters& params);

// The gripper action performed once after q_init, skipped when the user has
// already worked the gripper by hand from a menu. Returns false if the action
// failed and require_gripper_open is set, meaning the run should not start.
bool performStartupGripperAction(const Parameters& params);

// Compliant hand-guidance phase with its own keyboard menu (o/c/s/h/e). The
// robot stays in gravity compensation so the arm can be moved by hand; the
// chosen run mode is written back into params.
//
// Returns false if the user stopped with e + Enter, in which case the guided
// joint configuration has been printed as a paste-ready q_init case and the
// run should end.
bool runManualGuidanceStart(Parameters& params,
                            Robot& robot,
                            const Model& model,
                            std::atomic<bool>& stop_requested,
                            std::atomic<char>& guidance_menu_key);
