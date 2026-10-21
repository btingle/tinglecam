#pragma once

#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class ToolType { FlatEndMill, BallEndMill, Drill, VBit };

// How this tool cuts one material. Feed follows from chip load, so changing the
// spindle speed keeps the chip the same.
struct CuttingData {
  std::string material_id;
  double spindle_rpm = 12000.0;
  // mm per tooth at full radial engagement.
  double chip_load = 0.02;
  double plunge_feed = 100.0;  // mm/min
};

struct Tool {
  std::string id;
  std::string name;
  double diameter = 6.0;
  double flute_length = 20.0;
  int flutes = 2;
  ToolType type = ToolType::FlatEndMill;
  std::vector<CuttingData> cutting;
  double point_angle_degrees = 118.0;
  double tip_flat_diameter = 0.0;

  const CuttingData* cutting_for(const std::string& material_id) const;
};

double drill_tip_height(const Tool&);

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
