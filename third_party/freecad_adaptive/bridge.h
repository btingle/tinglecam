#pragma once
#include <functional>
#include <utility>
#include <vector>
namespace freecad_adaptive {
using Point = std::pair<double,double>;
using Path = std::vector<Point>;
using Paths = std::vector<Path>;
struct Region {
  Point start, center;
  std::vector<std::pair<int,Path>> paths;
  bool failed = false;
  bool incomplete = false;
};
std::vector<Region> generate(const Paths& stock, const Paths& boundary,
    const Paths& cleared, double diameter, double stepover, bool finish,
    const std::function<bool()>& cancelled);
}
