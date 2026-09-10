#pragma once

#include <imgui.h>

namespace tinglecam_gui {

// Small, application-independent view primitives shared by GUI consumers.
struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Projection {
  ImVec2 centre;
  Vec3 model_centre;
  Vec3 right;
  Vec3 up;
  Vec3 toward;
  float scale = 1.0f;

  ImVec2 point(const Vec3& value) const {
    const Vec3 r{value.x - model_centre.x, value.y - model_centre.y,
                 value.z - model_centre.z};
    return {centre.x + (r.x * right.x + r.y * right.y + r.z * right.z) * scale,
            centre.y - (r.x * up.x + r.y * up.y + r.z * up.z) * scale};
  }

  float depth(const Vec3& value) const {
    const Vec3 r{value.x - model_centre.x, value.y - model_centre.y,
                 value.z - model_centre.z};
    return r.x * toward.x + r.y * toward.y + r.z * toward.z;
  }
};

struct Basis {
  Vec3 right, up, toward;
};

struct CubeFace {
  Vec3 normal, across, down;
};

inline constexpr float kCubeBox = 124.0f;
inline constexpr float kCubeMargin = 12.0f;
inline constexpr float kCubeFit = 1.78f;

Basis orbit_basis(float yaw, float pitch);

}  // namespace tinglecam_gui
