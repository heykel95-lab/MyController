#pragma once

#include "controller_types.h"

// Writes the sampled control-loop rows to CSV. Rows are filled in place by the
// control loop (see LogData in controller_types.h), so there is no per-cycle
// allocation or copy on the realtime path.
void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name);
