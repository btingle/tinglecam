#include "cam/machine.h"

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

constexpr std::pair<Controller, const char*> kControllers[]{
    {Controller::Smoothieware, "smoothieware"}, {Controller::Grbl, "grbl"},
    {Controller::LinuxCnc, "linuxcnc"},         {Controller::Mach3, "mach3"},
    {Controller::Fanuc, "fanuc"},               {Controller::Haas, "haas"},
    {Controller::Marlin, "marlin"},
};
constexpr std::pair<ToolChange, const char*> kToolChanges[]{
    {ToolChange::Manual, "manual"},
    {ToolChange::SemiAutomatic, "semi-automatic"},
    {ToolChange::Automatic, "automatic"},
};
constexpr std::pair<LengthOffsetMode, const char*> kOffsetModes[]{
    {LengthOffsetMode::None, "none"},
    {LengthOffsetMode::G43WithH, "g43-h"},
    {LengthOffsetMode::G43Dynamic, "g43-dynamic"},
};

}  // namespace

const MaterialFeedRange* Machine::feeds_for(const std::string& material_id) const {
  for (const auto& range : feed_ranges)
    if (range.material_id == material_id) return &range;
  return nullptr;
}

const char* controller_name(Controller value) { return name_of(value, kControllers); }
const char* tool_change_name(ToolChange value) { return name_of(value, kToolChanges); }
const char* length_offset_mode_name(LengthOffsetMode value) {
  return name_of(value, kOffsetModes);
}

std::optional<Controller> controller_from_name(const std::string& text) {
  return value_of(text, kControllers);
}
std::optional<ToolChange> tool_change_from_name(const std::string& text) {
  return value_of(text, kToolChanges);
}
std::optional<LengthOffsetMode> length_offset_mode_from_name(const std::string& text) {
  return value_of(text, kOffsetModes);
}

double commandable_rpm(const Machine& machine, double requested) {
  double rpm = requested;
  if (machine.max_rpm > 0.0) rpm = std::min(rpm, machine.max_rpm);
  if (machine.min_rpm > 0.0) rpm = std::max(rpm, machine.min_rpm);
  if (machine.rpm_resolution > 1.0)
    rpm = std::round(rpm / machine.rpm_resolution) * machine.rpm_resolution;
  return rpm;
}

double rapid_feed(const Machine& machine) { return machine.rapid_feed; }

std::vector<std::string> validate(const Machine& machine) {
  std::vector<std::string> problems;
  const auto require = [&](bool condition, std::string message) {
    if (!condition) problems.push_back(std::move(message));
  };
  require(!machine.id.empty(), "The machine needs an identifier");
  require(!machine.name.empty(), "The machine needs a name");
  require(machine.rapid_feed > 0.0, "Rapid feed must be positive");
  require(machine.max_rpm > machine.min_rpm, "Spindle range is empty");
  require(machine.min_rpm >= 0.0, "Minimum spindle speed cannot be negative");
  require(machine.decimals >= 1 && machine.decimals <= 6,
          "Output precision must be between 1 and 6 decimals");
  std::vector<std::string> seen_materials;
  for (const auto& range : machine.feed_ranges) {
    const std::string where = " for " + range.material_id;
    require(!range.material_id.empty(), "A feed range is missing its material");
    require(range.full_engagement > 0.0 &&
                std::isfinite(range.full_engagement),
            "Full-engagement feed must be positive" + where);
    require(range.no_engagement >= range.full_engagement &&
                std::isfinite(range.no_engagement),
            "No-engagement feed must not be slower than full engagement" + where);
    require(std::find(seen_materials.begin(), seen_materials.end(),
                      range.material_id) == seen_materials.end(),
            "Two feed ranges name " + range.material_id);
    seen_materials.push_back(range.material_id);
  }
  return problems;
}

const Machine* MachineLibrary::find(const std::string& id) const {
  for (const auto& machine : machines)
    if (machine.id == id) return &machine;
  return nullptr;
}
Machine* MachineLibrary::find(const std::string& id) {
  for (auto& machine : machines)
    if (machine.id == id) return &machine;
  return nullptr;
}
bool MachineLibrary::upsert(Machine machine) {
  for (auto& existing : machines) {
    if (existing.id != machine.id) continue;
    existing = std::move(machine);
    return true;
  }
  machines.push_back(std::move(machine));
  return false;
}

}  // namespace cam
