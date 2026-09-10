#include "gui_types.h"

#include <algorithm>
#include <cmath>

namespace tinglecam_gui {

Basis orbit_basis(float yaw, float pitch) {
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  return {{-sy, cy, 0.0f},
          {-sp * cy, -sp * sy, cp},
          {cp * cy, cp * sy, sp}};
}

}  // namespace tinglecam_gui
