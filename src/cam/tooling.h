#pragma once

#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class CoolantNeed {
  Required,
  Recommended,
  Dry,
  AirBlast,
};

struct CuttingData {
  std::string material_id;
  double spindle_rpm = 12000.0;
  double max_ae_over_d = 0.5;
  double max_ap_over_d = 1.0;
  double plunge_feed = 100.0;
  CoolantNeed coolant = CoolantNeed::Recommended;
};

struct Tool {
  std::string id;
  std::string name;
  std::string notes;
  double diameter = 6.0;
  double flute_length = 20.0;
  bool center_cutting = true;
  int tool_number = 1;
  int length_offset = 1;
  std::vector<CuttingData> cutting_data;

  const CuttingData* data_for(const std::string& material_id) const;
};

const char* coolant_need_name(CoolantNeed value);
std::optional<CoolantNeed> coolant_need_from_name(const std::string& text);

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
