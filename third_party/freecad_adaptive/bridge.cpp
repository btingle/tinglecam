#include "bridge.h"
#include "Adaptive.hpp"
namespace freecad_adaptive {
std::vector<Region> generate(const Paths& stock, const Paths& boundary,
    const Paths& cleared, double diameter, double stepover, bool finish,
    const std::function<bool()>& cancelled) {
  AdaptivePath::Adaptive2d engine;
  engine.toolDiameter=diameter;
  engine.stepOverFactor=stepover/diameter;
  engine.forceInsideOut=true; // Air is supplied explicitly, within TingleCAM's limits.
  engine.finishingProfile=finish;
  engine.opType=AdaptivePath::otClearingInside;
  auto output=engine.Execute(stock,boundary,cleared,
      [&](AdaptivePath::TPaths) {return cancelled && cancelled();});
  std::vector<Region> result;
  for(auto& region:output)
    result.push_back({region.StartPoint,region.HelixCenterPoint,
      std::move(region.AdaptivePaths),region.StartPointNotFound,
      region.LeadPathFailed || region.UnexpectedRotateIterations ||
      region.TooManyFailedEngagements || region.UnclearedAreaRemains ||
      region.FailedToSetUpFinishingPass || region.FinishingLeadInFailed});
  return result;
}
}
