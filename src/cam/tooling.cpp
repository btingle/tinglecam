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

constexpr std::pair<ToolType, const char*> kTypes[]{
    {ToolType::FlatEndMill, "flat"}, {ToolType::BallEndMill, "ball"},
    {ToolType::Drill, "drill"}, {ToolType::VBit, "v-bit"},
};

}  // namespace

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
