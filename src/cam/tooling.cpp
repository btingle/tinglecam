#include "cam/tooling.h"
#include "cam/material.h"

#include <algorithm>
#include <cmath>
#include <numbers>
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

constexpr std::pair<ToolType, const char*> kTypes[]{
    {ToolType::FlatEndMill, "flat"}, {ToolType::BallEndMill, "ball"},
    {ToolType::Drill, "drill"}, {ToolType::VBit, "v-bit"},
};

}  // namespace

double drill_tip_height(const Tool& tool) {
  if (tool.point_angle_degrees >= 180.0) return 0.0;
  return (tool.diameter-tool.tip_flat_diameter)*.5 /
      std::tan(tool.point_angle_degrees*std::numbers::pi/360.0);
}

const char* tool_type_name(ToolType value) { return name_of(value, kTypes); }
std::optional<ToolType> tool_type_from_name(const std::string& text) {
  return value_of(text, kTypes);
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
  require(tool.flutes > 0, "Number of flutes must be positive");
  require(tool_type_from_name(tool_type_name(tool.type)).has_value(),
          "Unknown tool type");
  if (tool.type == ToolType::Drill) {
    require(std::isfinite(tool.point_angle_degrees) && tool.point_angle_degrees>0 &&
            tool.point_angle_degrees<=180, "Drill point angle must be in (0, 180] degrees");
    require(std::isfinite(tool.tip_flat_diameter) && tool.tip_flat_diameter>=0 &&
            tool.tip_flat_diameter<=tool.diameter, "Drill tip flat must fit inside its diameter");
    if(tool.point_angle_degrees>0 && tool.point_angle_degrees<=180)
      require(drill_tip_height(tool)<=tool.flute_length,"Drill point must fit within its flute length");
  }
  std::vector<std::string> seen;
  for (const auto& data : tool.cutting) {
    const std::string where = " for " + data.material_id;
    require(!data.material_id.empty(), "Cutting data is missing its material");
    require(std::isfinite(data.spindle_rpm) && data.spindle_rpm > 0.0,
            "Spindle speed must be positive" + where);
    require(std::isfinite(data.chip_load) && data.chip_load > 0.0,
            "Chip load must be positive" + where);
    require(std::isfinite(data.plunge_feed) && data.plunge_feed > 0.0,
            "Plunge feed must be positive" + where);
    require(std::find(seen.begin(), seen.end(), data.material_id) == seen.end(),
            "Two cutting data entries name " + data.material_id);
    seen.push_back(data.material_id);
  }
  return problems;
}

const CuttingData* Tool::cutting_for(const std::string& material_id) const {
  for (const auto& data : cutting)
    if (data.material_id == material_id) return &data;
  return nullptr;
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
  return library_id(name, "tool", [&](const std::string& id) { return find(id) != nullptr; });
}

}  // namespace cam
