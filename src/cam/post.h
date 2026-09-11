#pragma once

#include "cam/job.h"

#include <string>
#include <vector>

namespace cam {

struct PostResult {
  std::string text;
  std::size_t line_count = 0;
  std::size_t rapid_moves = 0;
  std::size_t cutting_moves = 0;
  double estimated_minutes = 0.0;
  std::vector<std::string> warnings;
  std::string error;

  bool ok() const { return error.empty(); }
};

// Turns a planned job into G-code for the machine each operation names. Motion
// is linear throughout: arc output changes the size of the file rather than
// what it cuts, and is worth adding only once the rest is trusted.
//
// Travel that the simulation found removes nothing is emitted at the machine's
// cutting-feed ceiling rather than as a rapid, because it still happens at
// depth. Only a planned retract actually lifts to the clearance plane.
PostResult post_job(const JobPlan& plan,
                    const Job& job,
                    const ToolLibrary& tools,
                    const MachineLibrary& machines);

// Posts one already-planned operation as a complete, standalone program. The
// operation keeps the job's origin, work offset, machine profile, and tool
// conventions, but no other operation is emitted.
PostResult post_operation(const JobPlan& plan,
                          const Job& job,
                          const ToolLibrary& tools,
                          const MachineLibrary& machines,
                          std::size_t operation_index);

// Posts a chosen set of already-planned operations as one standalone program,
// retaining their original job order and machine/tool conventions.
PostResult post_operations(const JobPlan& plan,
                           const Job& job,
                           const ToolLibrary& tools,
                           const MachineLibrary& machines,
                           const std::vector<std::size_t>& operation_indices);

}  // namespace cam
