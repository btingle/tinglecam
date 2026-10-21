#pragma once

#include <functional>
#include <string>
#include <vector>

namespace cam {

// A readable identifier from a name, suffixed until `taken` refuses it.
std::string library_id(const std::string& name, const std::string& fallback,
                       const std::function<bool(const std::string&)>& taken);

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
  bool erase(const std::string& id);
  std::string unique_id_for(const std::string& name) const;
};

}  // namespace cam
