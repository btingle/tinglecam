#include "cam/material.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace cam {
namespace {

template <typename Enum, std::size_t N>
const char* name_of(Enum value, const std::pair<Enum, const char*> (&table)[N]) {
  for (const auto& entry : table)
    if (entry.first == value) return entry.second;
  return "unknown";
}

template <typename Enum, std::size_t N>
std::optional<Enum> value_of(const std::string& text,
                             const std::pair<Enum, const char*> (&table)[N]) {
  for (const auto& entry : table)
    if (text == entry.second) return entry.first;
  return std::nullopt;
}

constexpr std::pair<MaterialGroup, const char*> kGroups[]{
    {MaterialGroup::Aluminium, "aluminium"}, {MaterialGroup::Steel, "steel"},
    {MaterialGroup::Stainless, "stainless"}, {MaterialGroup::Titanium, "titanium"},
    {MaterialGroup::Brass, "brass"},         {MaterialGroup::Plastic, "plastic"},
    {MaterialGroup::Wood, "wood"},           {MaterialGroup::Composite, "composite"},
    {MaterialGroup::Foam, "foam"},
};

constexpr std::pair<ChipClass, const char*> kChipClasses[]{
    {ChipClass::Gummy, "gummy"},
    {ChipClass::Short, "short"},
    {ChipClass::Abrasive, "abrasive"},
};

}  // namespace

const char* material_group_name(MaterialGroup value) { return name_of(value, kGroups); }
const char* chip_class_name(ChipClass value) { return name_of(value, kChipClasses); }

std::optional<MaterialGroup> material_group_from_name(const std::string& text) {
  return value_of(text, kGroups);
}
std::optional<ChipClass> chip_class_from_name(const std::string& text) {
  return value_of(text, kChipClasses);
}

double specific_energy(const Material& material, double chip_thickness) {
  if (material.kienzle_kc11 > 0.0 && chip_thickness > 0.0) {
    // kc = kc1.1 * h^(-mc), with h in millimetres. Clamped at a micron so a
    // vanishing chip cannot send the figure to infinity; below that the tool is
    // rubbing and the model has stopped describing cutting anyway.
    //
    // Kienzle coefficients are quoted as a force per unit chip section in
    // N/mm^2, and a thousand of those is one joule per cubic millimetre. The
    // division is what keeps this function's answer in the unit it promises
    // whichever branch produced it.
    const double thickness = std::max(chip_thickness, 0.001);
    return material.kienzle_kc11 * std::pow(thickness, -material.kienzle_mc) / 1000.0;
  }
  return material.specific_cutting_energy;
}

std::vector<std::string> validate(const Material& material) {
  std::vector<std::string> problems;
  const auto require = [&](bool condition, std::string message) {
    if (!condition) problems.push_back(std::move(message));
  };
  require(!material.id.empty(), "The material needs an identifier");
  require(!material.name.empty(), "The material needs a name");
  require(material.specific_cutting_energy > 0.0 || material.kienzle_kc11 > 0.0,
          "A specific cutting energy or a Kienzle coefficient is needed");
  require(material.vc_multiplier > 0.0, "Surface speed multiplier must be positive");
  require(material.fz_multiplier > 0.0, "Feed multiplier must be positive");
  require(material.kienzle_mc >= 0.0 && material.kienzle_mc < 1.0,
          "The Kienzle exponent must be within [0, 1)");
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
