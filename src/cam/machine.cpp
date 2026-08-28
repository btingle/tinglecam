#include "cam/machine.h"

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

constexpr std::pair<Controller, const char*> kControllers[]{
    {Controller::Smoothieware, "smoothieware"}, {Controller::Grbl, "grbl"},
    {Controller::LinuxCnc, "linuxcnc"},         {Controller::Mach3, "mach3"},
    {Controller::Fanuc, "fanuc"},               {Controller::Haas, "haas"},
    {Controller::Marlin, "marlin"},
};

constexpr std::pair<SpindleType, const char*> kSpindleTypes[]{
    {SpindleType::Router, "router"},
    {SpindleType::VfdSpindle, "vfd"},
    {SpindleType::BeltStepped, "belt-stepped"},
};

constexpr std::pair<RigidityClass, const char*> kRigidities[]{
    {RigidityClass::GantryRouter, "gantry-router"},
    {RigidityClass::DesktopEnclosed, "desktop-enclosed"},
    {RigidityClass::BenchtopMill, "benchtop-mill"},
    {RigidityClass::KneeMill, "knee-mill"},
    {RigidityClass::MachiningCentre, "machining-centre"},
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

constexpr std::pair<ArcSupport, const char*> kArcSupports[]{
    {ArcSupport::None, "none"},
    {ArcSupport::CentreOffset, "centre-offset"},
    {ArcSupport::Radius, "radius"},
};

}  // namespace

const char* controller_name(Controller value) { return name_of(value, kControllers); }
const char* spindle_type_name(SpindleType value) { return name_of(value, kSpindleTypes); }
const char* rigidity_class_name(RigidityClass value) {
  return name_of(value, kRigidities);
}
const char* tool_change_name(ToolChange value) { return name_of(value, kToolChanges); }
const char* length_offset_mode_name(LengthOffsetMode value) {
  return name_of(value, kOffsetModes);
}
const char* arc_support_name(ArcSupport value) { return name_of(value, kArcSupports); }

std::optional<Controller> controller_from_name(const std::string& text) {
  return value_of(text, kControllers);
}
std::optional<SpindleType> spindle_type_from_name(const std::string& text) {
  return value_of(text, kSpindleTypes);
}
std::optional<RigidityClass> rigidity_class_from_name(const std::string& text) {
  return value_of(text, kRigidities);
}
std::optional<ToolChange> tool_change_from_name(const std::string& text) {
  return value_of(text, kToolChanges);
}
std::optional<LengthOffsetMode> length_offset_mode_from_name(const std::string& text) {
  return value_of(text, kOffsetModes);
}
std::optional<ArcSupport> arc_support_from_name(const std::string& text) {
  return value_of(text, kArcSupports);
}

double spindle_power(const Machine& machine, double rpm) {
  if (machine.power_curve.size() >= 2) {
    std::vector<PowerPoint> points = machine.power_curve;
    std::sort(points.begin(), points.end(),
              [](const PowerPoint& a, const PowerPoint& b) { return a.rpm < b.rpm; });
    if (rpm <= points.front().rpm) return points.front().kilowatts;
    if (rpm >= points.back().rpm) return points.back().kilowatts;
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (rpm > points[i].rpm) continue;
      const PowerPoint& low = points[i - 1];
      const PowerPoint& high = points[i];
      const double span = high.rpm - low.rpm;
      if (!(span > 0.0)) return high.kilowatts;
      const double t = (rpm - low.rpm) / span;
      return low.kilowatts + (high.kilowatts - low.kilowatts) * t;
    }
    return points.back().kilowatts;
  }
  if (machine.power_curve.size() == 1) {
    // A single point is a rating at a speed. Below the base speed the spindle
    // holds torque rather than power, so power falls away in proportion.
    const double rated = machine.power_curve.front().kilowatts;
    const double base = machine.base_speed_rpm > 0.0 ? machine.base_speed_rpm
                                                     : machine.power_curve.front().rpm;
    if (!(base > 0.0) || rpm >= base) return rated;
    return rated * (rpm / base);
  }
  return 0.0;
}

double spindle_torque(const Machine& machine, double rpm) {
  if (!(rpm > 0.0)) return 0.0;
  const double watts = spindle_power(machine, rpm) * 1000.0;
  const double radians_per_second = rpm * 2.0 * std::numbers::pi / 60.0;
  if (!(radians_per_second > 0.0)) return 0.0;
  return watts / radians_per_second;
}

double commandable_rpm(const Machine& machine, double requested) {
  double rpm = requested;
  if (machine.max_rpm > 0.0) rpm = std::min(rpm, machine.max_rpm);
  if (machine.min_rpm > 0.0) rpm = std::max(rpm, machine.min_rpm);
  if (machine.rpm_resolution > 1.0)
    rpm = std::round(rpm / machine.rpm_resolution) * machine.rpm_resolution;
  return rpm;
}

double rapid_feed(const Machine& machine) {
  double slowest = 0.0;
  for (const AxisLimits* axis : {&machine.x, &machine.y, &machine.z}) {
    if (!(axis->max_rapid_feed > 0.0)) continue;
    slowest = slowest > 0.0 ? std::min(slowest, axis->max_rapid_feed)
                            : axis->max_rapid_feed;
  }
  return slowest;
}

double limiting_acceleration(const Machine& machine) {
  double slowest = 0.0;
  for (const AxisLimits* axis : {&machine.x, &machine.y, &machine.z}) {
    if (!(axis->max_acceleration > 0.0)) continue;
    slowest = slowest > 0.0 ? std::min(slowest, axis->max_acceleration)
                            : axis->max_acceleration;
  }
  return slowest;
}

double default_cutting_force(RigidityClass rigidity) {
  switch (rigidity) {
    case RigidityClass::GantryRouter: return 90.0;
    case RigidityClass::DesktopEnclosed: return 160.0;
    case RigidityClass::BenchtopMill: return 400.0;
    case RigidityClass::KneeMill: return 1200.0;
    case RigidityClass::MachiningCentre: return 3500.0;
  }
  return 90.0;
}

double default_mrr(RigidityClass rigidity) {
  switch (rigidity) {
    case RigidityClass::GantryRouter: return 25.0;
    case RigidityClass::DesktopEnclosed: return 40.0;
    case RigidityClass::BenchtopMill: return 100.0;
    case RigidityClass::KneeMill: return 250.0;
    case RigidityClass::MachiningCentre: return 900.0;
  }
  return 25.0;
}

std::vector<std::string> validate(const Machine& machine) {
  std::vector<std::string> problems;
  const auto require = [&](bool condition, std::string message) {
    if (!condition) problems.push_back(std::move(message));
  };

  require(!machine.id.empty(), "The machine needs an identifier");
  require(!machine.name.empty(), "The machine needs a name");
  require(machine.x.travel() > 0.0, "X travel must be greater than zero");
  require(machine.y.travel() > 0.0, "Y travel must be greater than zero");
  require(machine.z.travel() > 0.0, "Z travel must be greater than zero");
  require(machine.max_cutting_feed > 0.0, "Maximum cutting feed must be positive");
  require(rapid_feed(machine) > 0.0, "At least one axis needs a rapid rate");
  require(limiting_acceleration(machine) > 0.0,
          "At least one axis needs an acceleration");
  require(machine.block_rate > 0.0, "Block rate must be positive");
  require(machine.max_rpm > machine.min_rpm, "Spindle range is empty");
  require(machine.min_rpm >= 0.0, "Minimum spindle speed cannot be negative");
  require(!machine.power_curve.empty(), "The spindle needs at least a power rating");
  require(machine.drivetrain_efficiency > 0.0 && machine.drivetrain_efficiency <= 1.0,
          "Drivetrain efficiency must be within (0, 1]");
  require(machine.decimals >= 1 && machine.decimals <= 6,
          "Output precision must be between 1 and 6 decimals");
  require(!machine.work_offsets.empty(), "At least one work offset is needed");
  require(machine.tool_change != ToolChange::Automatic || machine.tool_pockets > 0,
          "An automatic changer needs a pocket count");
  for (const auto& point : machine.power_curve)
    require(point.kilowatts > 0.0, "A power curve point has no power");
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
