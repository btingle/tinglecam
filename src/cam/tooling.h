#pragma once

#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class ToolType { FlatEndMill, BallEndMill, Drill, VBit };

struct Tool {
  std::string id;
  std::string name;
  double diameter = 6.0;
  double flute_length = 20.0;
  int flutes = 2;
  ToolType type = ToolType::FlatEndMill;
};

const char* tool_type_name(ToolType value);
std::optional<ToolType> tool_type_from_name(const std::string& text);

std::vector<std::string> validate(const Tool& tool);

struct ToolLibrary {
  std::vector<Tool> tools;

  const Tool* find(const std::string& id) const;
  Tool* find(const std::string& id);
  bool upsert(Tool tool);
  bool erase(const std::string& id);
  std::string unique_id_for(const std::string& name) const;
};

}  // namespace cam
