#include "cam/tooling.h"

#include <algorithm>
#include <cctype>
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

constexpr std::pair<CoolantNeed, const char*> kCoolants[]{
    {CoolantNeed::Required, "required"},
    {CoolantNeed::Recommended, "recommended"},
    {CoolantNeed::Dry, "dry"},
    {CoolantNeed::AirBlast, "air-blast"},
};

}  // namespace

const char* coolant_need_name(CoolantNeed value) {
  return name_of(value, kCoolants);
}
std::optional<CoolantNeed> coolant_need_from_name(const std::string& text) {
  return value_of(text, kCoolants);
}

const CuttingData* Tool::data_for(const std::string& material_id) const {
  for (const auto& data : cutting_data)
    if (data.material_id == material_id) return &data;
  return nullptr;
}

std::vector<std::string> validate(const Tool& tool) {
  std::vector<std::string> problems;
  const auto require = [&](bool condition, std::string message) {
    if (!condition) problems.push_back(std::move(message));
  };
  require(!tool.id.empty(), "The tool needs an identifier");
  require(!tool.name.empty(), "The tool needs a name");
  require(std::isfinite(tool.diameter) && tool.diameter > 0.0,
          "Diameter must be finite and greater than zero");
  require(std::isfinite(tool.flute_length) && tool.flute_length > 0.0,
          "Flute length must be finite and greater than zero");
  std::vector<std::string> seen;
  for (const auto& data : tool.cutting_data) {
    const std::string where = " for " + data.material_id;
    require(!data.material_id.empty(), "Cutting data is missing its material");
    require(std::isfinite(data.spindle_rpm) && data.spindle_rpm > 0.0,
            "Spindle speed must be finite and positive" + where);
    require(data.max_ae_over_d > 0.0 && data.max_ae_over_d <= 1.0,
            "Radial engagement ceiling must be within (0, 1]" + where);
    require(std::isfinite(data.max_ap_over_d) && data.max_ap_over_d > 0.0,
            "Axial engagement ceiling must be finite and positive" + where);
    require(std::isfinite(data.plunge_feed) && data.plunge_feed > 0.0,
            "Plunge feed must be finite and positive" + where);
    if (std::find(seen.begin(), seen.end(), data.material_id) != seen.end())
      problems.push_back("Two cutting-data entries for " + data.material_id);
    seen.push_back(data.material_id);
  }
  return problems;
}

const Tool* ToolLibrary::find(const std::string& id) const {
  for (const auto& tool : tools)
    if (tool.id == id) return &tool;
  return nullptr;
}
Tool* ToolLibrary::find(const std::string& id) {
  for (auto& tool : tools)
    if (tool.id == id) return &tool;
  return nullptr;
}
bool ToolLibrary::upsert(Tool tool) {
  for (auto& existing : tools) {
    if (existing.id != tool.id) continue;
    existing = std::move(tool);
    return true;
  }
  tools.push_back(std::move(tool));
  return false;
}
bool ToolLibrary::erase(const std::string& id) {
  const auto it = std::remove_if(tools.begin(), tools.end(),
                                 [&](const Tool& tool) { return tool.id == id; });
  if (it == tools.end()) return false;
  tools.erase(it, tools.end());
  return true;
}
std::string ToolLibrary::unique_id_for(const std::string& name) const {
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
  if (base.empty()) base = "tool";
  if (!find(base)) return base;
  for (int suffix = 2; suffix < 10000; ++suffix) {
    const std::string candidate = base + "-" + std::to_string(suffix);
    if (!find(candidate)) return candidate;
  }
  return base;
}

}  // namespace cam
