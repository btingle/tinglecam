#pragma once

#include <array>
#include <map>

#include "cam/surface_finishing.h"

namespace cam::detail {
using SurfaceRun = SurfaceContour;
std::map<std::size_t, std::vector<double>> surface_lanes(
    const CadModel&, const Operation&, PlanningMetrics&, const ProgressCallback&);
void build_surface_adjacency(const CadModel&, const Operation&,
                             SurfaceContourPreview&, const ProgressCallback&);
void link_surface_runs(const CadModel&, const Operation&, const Tool&,
                       const Machine&, const CuttingFeeds&, double rapid_plane,
                       std::vector<SurfaceRun>&, OperationPlan&,
                       const ProgressCallback&,
                       std::vector<SurfaceRun>* joined = nullptr,
                       bool fixed_order = false);
}  // namespace cam::detail
