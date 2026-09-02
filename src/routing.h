#pragma once

#include "geometry.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace wave {

// The shortest route the tool can take, at depth, from one point to another
// without leaving the space it is allowed to travel through.
//
// A boundary walk is not this. Following the edge of an obstruction gets around
// it, but the way around and the way there are different problems: a 5 mm rib
// standing between two points 40 mm apart costs a walk of the whole part
// outline, which loses to lifting every time even though a 45 mm route exists.
// The shortest path in a polygon with holes turns only at the corners of those
// holes, so the exact answer is a graph over those corners, and this builds it.
//
// Construction is the expensive half -- the corners are collected once and the
// pairs of them that can see each other are found once -- so a caller with
// several routes to plan through one region should build one of these and query
// it, rather than building one per query.
class Router {
 public:
  // `region` is a cutter-centre region: everywhere the tool's centre may go,
  // which is the cleared material eroded by nothing, because the erosion is
  // already in how that region was built. `slack`, in integer units, is how far
  // a route may sit outside it. Routes run between points that lie exactly on
  // the region's own boundary -- the ends of open fronts do -- and two
  // separately rounded descriptions of one wall do not agree to the unit, so
  // without an allowance every such route reads as leaving the region.
  Router(const Paths64& region, double slack);

  // False when the region is empty or too degenerate to route through, in which
  // case `route` always returns nothing.
  bool usable() const { return !region_.empty(); }

  // How many points the graph may turn at. Diagnostic; a caller does not need
  // it.
  std::size_t corner_count() const { return corners_.size(); }

  // The shortest route from `from` to `to` that stays inside the region, both
  // endpoints included. Nothing when either end is outside it, when no route
  // exists, or when the route that was found fails the same containment test
  // the rest of the planner applies -- the graph is an accelerator, and a route
  // it proposes is still checked before it is offered.
  std::optional<Path64> route(const Point64& from, const Point64& to) const;

 private:
  // Visibility within the graph: no proper crossing of the simplified outline,
  // and a point on the segment inside the padded region.
  bool visible(const Point64& a, const Point64& b) const;

  Paths64 region_;
  Paths64 padded_;
  // The padded region with the detail the slack does not vouch for taken out.
  // Corners, obstacle edges and containment all come from this one outline, so
  // a route that runs along it is legal without needing a tolerance to say so.
  Paths64 outline_;
  // The region padded by the whole allowance. Points are tested against this
  // one rather than against the outline they sit on: a probe on a boundary is
  // a coin toss, and a probe well inside one is not.
  Paths64 probe_;
  double slack_ = 0.0;
  std::vector<Point64> corners_;
  // Lower triangle stored whole for a plain index; corners_.size() is small.
  // 0 = blocked, 1 = visible, 2 = not tested yet. One router may answer more
  // than one query, so visibility discovered by an earlier search remains
  // available to the next one without eagerly constructing the dense graph.
  mutable std::vector<std::uint8_t> corner_visible_;
  // Every edge of `padded_`, flattened, so a visibility test is one pass.
  std::vector<Point64> edge_from_;
  std::vector<Point64> edge_to_;
};

}  // namespace wave
