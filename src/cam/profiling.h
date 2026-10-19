#pragma once
#include "cam/job.h"
namespace cam {
// Retained CPU payloads, with shared heightmap pages/arrays counted once.
// RSS/virtual address figures are reported separately on Linux.
void release_planning_scratch();
void report_plan_memory(const JobPlan&);
}  // namespace cam
