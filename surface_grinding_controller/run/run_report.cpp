// ====================================================================
// Run report
// ====================================================================
// What is printed and written after a run ends: the joint table, the CSV,
// and the closing summary. Also names the log files of repeat sessions.
#include "controller.h"

// Log file name for a repeat session. Pressing m returns to the menu and
// starts a second run in the same program start; that run writes
// surface_grinding_controller_log_s2.csv, the third _s3, and so on, so an
// earlier run's log is never overwritten. Session 1 keeps the plain name.
std::string sessionFileName(const std::string& name, int session) {
  const std::size_t dot = name.find_last_of('.');
  const std::string suffix = "_s" + std::to_string(session);
  return (dot == std::string::npos) ? name + suffix
                                    : name.substr(0, dot) + suffix +
                                          name.substr(dot);
}

// ====================================================================
// 4. Post-run report
// ====================================================================
void writeRunLogs(const Parameters& params, const RunResult& result) {
  if (result.descend_failed) {
    printf("\nDescend stopped: maximum distance reached before the clearance height.\n");
  }
  printJointStartEndTableDeg(result.q_start, result.final_q);
  writeLogToCsv(result.log, params.csv_file_name);
  printFinalSummary(result.final_p_d, result.final_p_EE, result.final_e_p,
                    result.final_e_R, params.csv_file_name);
}
