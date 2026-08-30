#pragma once

#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class Controller {
  Smoothieware,
  Grbl,
  LinuxCnc,
  Mach3,
  Fanuc,
  Haas,
  Marlin,
};

enum class ToolChange {
  Manual,
  SemiAutomatic,
  Automatic,
};

enum class LengthOffsetMode {
  None,
  G43WithH,
  G43Dynamic,
};

struct MaterialFeedRange {
  std::string material_id;
  double full_engagement = 500.0;
  double no_engagement = 2000.0;
};

struct Machine {
  std::string id;
  std::string name;
  bool specifications_unverified = false;
  Controller controller = Controller::Grbl;

  double rapid_feed = 3000.0;
  double min_rpm = 0.0;
  double max_rpm = 0.0;
  double rpm_resolution = 1.0;
  double spindle_spinup_seconds = 2.0;
  std::vector<MaterialFeedRange> feed_ranges;

  ToolChange tool_change = ToolChange::Manual;
  double tool_change_x = 0.0;
  double tool_change_y = 0.0;
  LengthOffsetMode length_offset_mode = LengthOffsetMode::None;
  bool coolant_flood = false;
  bool coolant_mist = false;

  bool output_inches = false;
  int decimals = 3;
  bool line_numbers = false;
  int line_number_increment = 10;
  std::string program_header;
  std::string program_footer;
  std::string path_blending;

  const MaterialFeedRange* feeds_for(const std::string& material_id) const;
};

double commandable_rpm(const Machine& machine, double requested);
double rapid_feed(const Machine& machine);

const char* controller_name(Controller value);
const char* tool_change_name(ToolChange value);
const char* length_offset_mode_name(LengthOffsetMode value);
std::optional<Controller> controller_from_name(const std::string& text);
std::optional<ToolChange> tool_change_from_name(const std::string& text);
std::optional<LengthOffsetMode> length_offset_mode_from_name(const std::string& text);

std::vector<std::string> validate(const Machine& machine);

struct MachineLibrary {
  std::vector<Machine> machines;

  const Machine* find(const std::string& id) const;
  Machine* find(const std::string& id);
  bool upsert(Machine machine);
};

}  // namespace cam
