#pragma once

#include <array>
#include <cstdlib>
#include <cmath>
#include <stdexcept>

namespace wave {
// Signed source axes for each output axis. Only proper cube rotations are valid.
// Composition uses world axes and never introduces trigonometric rounding.
struct ModelOrientation {
  std::array<int, 3> axes{1, 2, 3};
  bool operator==(const ModelOrientation&) const = default;

  bool valid() const {
    std::array<bool, 3> seen{};
    int determinant = 1;
    for (int a : axes) {
      if (a < -3 || a == 0 || a > 3 || seen[std::abs(a) - 1]) return false;
      seen[std::abs(a) - 1] = true;
      if (a < 0) determinant = -determinant;
    }
    for (int i = 0; i < 3; ++i)
      for (int j = i + 1; j < 3; ++j)
        if (std::abs(axes[i]) > std::abs(axes[j])) determinant = -determinant;
    return determinant == 1;
  }
  void validate() const {
    if (!valid()) throw std::invalid_argument("Invalid model orientation");
  }
  void quarter_turn(int axis, int direction) {
    validate();
    if (axis < 0 || axis > 2 || (direction != 1 && direction != -1))
      throw std::invalid_argument("Invalid quarter turn");
    const int a = (axis + 1) % 3, b = (axis + 2) % 3;
    const int old = axes[a];
    axes[a] = -direction * axes[b];
    axes[b] = direction * old;
  }
  ModelOrientation inverse() const {
    ModelOrientation result;
    for (int i = 0; i < 3; ++i)
      result.axes[std::abs(axes[i]) - 1] = axes[i] < 0 ? -(i + 1) : i + 1;
    return result;
  }
  // Apply this rotation after the supplied rotation.
  ModelOrientation composed(const ModelOrientation& before) const {
    ModelOrientation result;
    for (int i = 0; i < 3; ++i) {
      const int a = axes[i];
      result.axes[i] = (a < 0 ? -1 : 1) * before.axes[std::abs(a) - 1];
    }
    return result;
  }
  template<class Point> Point apply(Point p) const {
    const std::array values{p.x, p.y, p.z};
    const auto value = [&](int a) {
      return a < 0 ? -values[-a - 1] : values[a - 1];
    };
    return {value(axes[0]), value(axes[1]), value(axes[2])};
  }
};
}  // namespace wave

namespace wave {
struct ModelTransform {
  ModelOrientation orientation;
  std::array<double, 3> offset{};
  bool operator==(const ModelTransform&) const = default;
  void validate() const {
    orientation.validate();
    for (double value : offset)
      if (!std::isfinite(value)) throw std::invalid_argument("Model offsets must be finite");
  }
  template<class Point> Point apply(Point p) const {
    p = orientation.apply(p);
    p.x += offset[0]; p.y += offset[1]; p.z += offset[2];
    return p;
  }
  ModelTransform inverse() const {
    ModelTransform result{orientation.inverse(), {}};
    struct Point { double x, y, z; };
    const auto p = result.orientation.apply(Point{-offset[0], -offset[1], -offset[2]});
    result.offset = {p.x, p.y, p.z};
    return result;
  }
  ModelTransform composed(const ModelTransform& before) const {
    struct Point { double x, y, z; };
    const auto p = apply(Point{before.offset[0], before.offset[1], before.offset[2]});
    return {orientation.composed(before.orientation), {p.x, p.y, p.z}};
  }
  bool preserves_columns() const { return std::abs(orientation.axes[2]) == 3; }
};
}  // namespace wave
