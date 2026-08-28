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

enum class SpindleType {
  Router,
  VfdSpindle,
  BeltStepped,
};

enum class RigidityClass {
  GantryRouter,
  DesktopEnclosed,
  BenchtopMill,
  KneeMill,
  MachiningCentre,
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

enum class ArcSupport {
  None,
  CentreOffset,  // I/J
  Radius,        // R
};

struct AxisLimits {
  double minimum = 0.0;
  double maximum = 0.0;
  double max_rapid_feed = 0.0;    // mm/min
  double max_acceleration = 0.0;  // mm/s^2
  double backlash = 0.0;          // mm

  double travel() const { return maximum - minimum; }
};

struct PowerPoint {
  double rpm = 0.0;
  double kilowatts = 0.0;
};

struct PulleyStep {
  std::string name;
  double min_rpm = 0.0;
  double max_rpm = 0.0;
  double max_torque_nm = 0.0;
};

struct Machine {
  std::string id;
  std::string name;
  std::string notes;
  // Set on any profile whose numbers have not been confirmed against the real
  // machine. The planner still runs, but every screen that shows a limit says
  // where the limit came from.
  bool specifications_unverified = false;

  Controller controller = Controller::Grbl;

  // Motion
  AxisLimits x;
  AxisLimits y;
  AxisLimits z;
  double max_cutting_feed = 0.0;   // mm/min
  double block_rate = 0.0;         // blocks/second the controller can consume
  int lookahead_blocks = 0;
  double junction_deviation = 0.0; // mm
  double min_segment_length = 0.0; // mm
  double repeatability = 0.0;      // mm

  // Spindle
  double min_rpm = 0.0;
  double max_rpm = 0.0;
  std::vector<PowerPoint> power_curve;
  double base_speed_rpm = 0.0;
  double drivetrain_efficiency = 0.85;
  SpindleType spindle_type = SpindleType::VfdSpindle;
  std::vector<PulleyStep> pulley_steps;
  double spindle_spinup_seconds = 2.0;
  double rpm_resolution = 1.0;
  bool supports_reverse = false;

  // Structure
  double max_cutting_force = 0.0;  // newtons
  double max_mrr = 0.0;            // cm^3/min
  RigidityClass rigidity = RigidityClass::GantryRouter;

  // Tooling and coolant
  ToolChange tool_change = ToolChange::Manual;
  int tool_pockets = 0;
  double tool_change_x = 0.0;
  double tool_change_y = 0.0;
  double tool_change_z = 0.0;
  LengthOffsetMode length_offset_mode = LengthOffsetMode::None;
  bool coolant_flood = false;
  bool coolant_mist = false;
  bool coolant_air = false;
  bool probe_support = false;

  // Emission
  bool output_inches = false;
  ArcSupport arc_support = ArcSupport::CentreOffset;
  int decimals = 3;
  std::vector<std::string> work_offsets{"G54"};
  bool line_numbers = false;
  int line_number_increment = 10;
  std::string program_header;
  std::string program_footer;
  std::string path_blending;  // e.g. "G64 P0.01", empty to emit nothing
};

// Available spindle power at a speed, in kilowatts. Interpolates the curve when
// there is one; otherwise falls back to constant torque below the base speed
// and constant power above it, which is how a VFD spindle behaves and is close
// enough for a router.
double spindle_power(const Machine& machine, double rpm);

// Available torque at a speed, in newton metres, derived from the power.
double spindle_torque(const Machine& machine, double rpm);

// The rpm actually commandable, after clamping to the range and rounding to the
// controller's S-word granularity.
double commandable_rpm(const Machine& machine, double requested);

// The slowest of the per-axis rapid rates, which is what a diagonal rapid runs
// at on a machine that does not coordinate them.
double rapid_feed(const Machine& machine);

// The smallest per-axis acceleration, used for the reachable-feed limit since a
// move may be along any axis.
double limiting_acceleration(const Machine& machine);

const char* controller_name(Controller value);
const char* spindle_type_name(SpindleType value);
const char* rigidity_class_name(RigidityClass value);
const char* tool_change_name(ToolChange value);
const char* length_offset_mode_name(LengthOffsetMode value);
const char* arc_support_name(ArcSupport value);

std::optional<Controller> controller_from_name(const std::string& text);
std::optional<SpindleType> spindle_type_from_name(const std::string& text);
std::optional<RigidityClass> rigidity_class_from_name(const std::string& text);
std::optional<ToolChange> tool_change_from_name(const std::string& text);
std::optional<LengthOffsetMode> length_offset_mode_from_name(const std::string& text);
std::optional<ArcSupport> arc_support_from_name(const std::string& text);

std::vector<std::string> validate(const Machine& machine);

// Defaults for a rigidity class, used when a profile omits the structural
// figures. These are deliberately conservative: a plan that is too gentle wastes
// time, one that is too aggressive breaks something.
double default_cutting_force(RigidityClass rigidity);
double default_mrr(RigidityClass rigidity);

struct MachineLibrary {
  std::vector<Machine> machines;

  const Machine* find(const std::string& id) const;
  Machine* find(const std::string& id);
  bool upsert(Machine machine);
};

}  // namespace cam
