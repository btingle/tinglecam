#include "cam/material.h"

#include <utility>

namespace cam {

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

}  // namespace cam
