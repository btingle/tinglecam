#pragma once
#include "cam/job.h"
namespace cam {
// Executes exactly the XYZ phases consumed by the post and playback, assigns
// feeds from measured engagement and, for roughing, links layers at depth
// where the simulated stock shows the way clear.
void simulate_operation_xyz(OperationPlan&, const JobGeometry&, const Job&,
                            const Operation&, const Tool&, const Machine&,
                            const CuttingFeeds&, const PriorOperations&,
                            const ProgressCallback& = {}, const ToolLibrary* = nullptr);
wave::MillingCutter milling_cutter(const Tool&);
wave::Vec3d motion_point(const MotionPhase&, double fraction);
void advance_stock_motion(wave::Heightmap&, const LevelPlan&,
                          const wave::MillingCutter&, double from, double to);
wave::Heightmap stock_at_motion(const LevelPlan&, const wave::MillingCutter&,
                                double distance);
}  // namespace cam
