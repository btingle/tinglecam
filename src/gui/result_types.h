#pragma once

#include "app_ui_api.h"

#include <cstdint>
#include <vector>

namespace tinglecam_gui {

struct Facet {
  std::uint32_t first = 0;
  std::uint32_t count = 0;
  float depth = 0.0f;
  ImU32 fill = 0;
  ImU32 edge = 0;
};

struct OrbitScene {
  ImVec2 top_left;
  ImVec2 size;
  ImDrawList* list = nullptr;
  bool hovered = false;
  Projection projection;
  Vec3 light;
  double spacing = 0.05;
  std::vector<Facet> facets;
  std::vector<ImVec2> pool;
};

}  // namespace tinglecam_gui
