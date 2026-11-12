#pragma once
#include "cam/job.h"
namespace freecad_adaptive { struct Region; }
namespace cam::detail {
void append_adaptive_paths(wave::Toolpath& path, const freecad_adaptive::Region& region);
void generate_adaptive_roughing(LevelPlan& level, const Operation& operation,
    double radius, double stepover, const std::function<bool(double)>& report, double simulation_pitch = .02);
void add_adaptive_helices(LevelPlan& level, const CuttingFeeds& feeds);
}
