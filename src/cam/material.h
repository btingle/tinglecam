#pragma once

#include <string>
#include <vector>

namespace cam {

struct Material {
  std::string id;
  std::string name;
};

std::vector<std::string> validate(const Material& material);

struct MaterialLibrary {
  std::vector<Material> materials;

  const Material* find(const std::string& id) const;
  Material* find(const std::string& id);
  bool upsert(Material material);
};

}  // namespace cam
