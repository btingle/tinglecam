#include "cam/material.h"

#include <algorithm>
#include <cctype>
#include <utility>

namespace cam {

std::string library_id(const std::string& name, const std::string& fallback,
                       const std::function<bool(const std::string&)>& taken) {
  std::string base;
  bool separated = true;
  for (const unsigned char character : name) {
    if (std::isalnum(character)) {
      base.push_back(static_cast<char>(std::tolower(character)));
      separated = false;
    } else if (!separated) {
      base.push_back('-');
      separated = true;
    }
  }
  while (!base.empty() && base.back() == '-') base.pop_back();
  if (base.empty()) base = fallback;
  if (!taken(base)) return base;
  for (int suffix = 2; suffix < 10000; ++suffix) {
    const std::string candidate = base + "-" + std::to_string(suffix);
    if (!taken(candidate)) return candidate;
  }
  return base;
}

std::vector<std::string> validate(const Material& material) {
  std::vector<std::string> problems;
  if (material.id.empty()) problems.push_back("The material needs an identifier");
  if (material.name.empty()) problems.push_back("The material needs a name");
  return problems;
}

const Material* MaterialLibrary::find(const std::string& id) const {
  for (const auto& material : materials)
    if (material.id == id) return &material;
  return nullptr;
}

Material* MaterialLibrary::find(const std::string& id) {
  for (auto& material : materials)
    if (material.id == id) return &material;
  return nullptr;
}

bool MaterialLibrary::upsert(Material material) {
  for (auto& existing : materials) {
    if (existing.id != material.id) continue;
    existing = std::move(material);
    return true;
  }
  materials.push_back(std::move(material));
  return false;
}

bool MaterialLibrary::erase(const std::string& id) {
  const auto it = std::remove_if(materials.begin(), materials.end(),
                                 [&](const Material& material) { return material.id == id; });
  if (it == materials.end()) return false;
  materials.erase(it, materials.end());
  return true;
}

std::string MaterialLibrary::unique_id_for(const std::string& name) const {
  return library_id(name, "material", [&](const std::string& id) { return find(id) != nullptr; });
}

}  // namespace cam
