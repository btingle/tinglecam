#pragma once
#include "cam/job.h"
namespace cam {
struct HoleDetection {
  std::vector<HoleSystem> systems;
  std::vector<std::string> warnings;
  std::string signature, error;
};
std::string hole_detection_signature(const JobGeometry&, const HoleParameters&);
HoleDetection identify_holes(const JobGeometry&, const HoleParameters&,
                             const ProgressCallback& = {});
void apply_hole_detection(HoleParameters&, const HoleDetection&);
HoleStrategy effective_hole_strategy(const HoleParameters&, const HoleLevel&);
// A linked pilot operation inherits selected milling systems, but retains its
// own tool, feed overrides and drilling settings.
Operation resolve_hole_source(const Job&, std::size_t operation_index);
OperationPlan plan_hole_operation(const Operation&, const Tool&, const Machine&,
    const CuttingFeeds&, double rapid_plane, const PriorOperations&,
    const ProgressCallback& = {}, double from = 0, double to = 1,
    const JobGeometry* = nullptr, const ToolLibrary* = nullptr,
    const std::string& material_id = {});
}
