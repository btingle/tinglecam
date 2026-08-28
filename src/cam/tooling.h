#pragma once

#include <optional>
#include <string>
#include <vector>

// Every length in this namespace is millimetres, every angle degrees, every
// feed millimetres per minute and every rotation speed rpm. Inches exist only
// where a machine profile asks for them on the way out.
namespace cam {

enum class ToolType {
  FlatEndmill,
  BallEndmill,
  BullNose,
  Chamfer,
  VBit,
  Drill,
  FaceMill,
  ThreadMill,
  Engraver,
};

enum class Substrate {
  HighSpeedSteel,
  Cobalt,
  Carbide,
  MicrograinCarbide,
  Ceramic,
};

enum class Coating {
  None,
  TiN,
  TiCN,
  TiAlN,
  AlTiN,
  ZrN,
  DLC,
  Diamond,
};

enum class GeometryClass {
  General,
  HighHelixAluminium,
  RoughingChipbreaker,
  Finisher,
  HardMilling,
};

enum class HolderType {
  ERCollet,
  ShrinkFit,
  Weldon,
  Hydraulic,
};

enum class CuttingDirection {
  Climb,
  Conventional,
  Either,
};

enum class CoolantNeed {
  Required,
  Recommended,
  Dry,
  AirBlast,
};

// What a tool is prepared to do in one particular workpiece material. A tool
// carries one of these per material it has been characterised for; an
// operation in an uncharacterised material has no book values to scale and is
// refused rather than guessed at.
struct CuttingData {
  std::string material_id;
  double surface_speed = 0.0;          // Vc, m/min
  double feed_per_tooth = 0.0;         // fz at reference engagement, mm/tooth
  double min_chip_thickness = 0.0;     // below this the edge rubs, mm
  double max_ae_over_d = 0.5;          // radial engagement ceiling
  double max_ap_over_d = 1.0;          // axial engagement ceiling
  double max_ramp_angle = 3.0;         // degrees of descent while cutting
  bool helical_entry = true;
  double min_helix_diameter_ratio = 1.2;  // as a multiple of tool diameter
  double plunge_feed_factor = 0.3;     // vertical feed as a fraction of lateral
  CuttingDirection preferred_direction = CuttingDirection::Climb;
  CoolantNeed coolant = CoolantNeed::Recommended;
};

struct Tool {
  // Identity
  std::string id;
  std::string name;
  std::string vendor;
  std::string part_number;
  std::string notes;

  // Geometry
  ToolType type = ToolType::FlatEndmill;
  double diameter = 6.0;
  double corner_radius = 0.0;
  int flutes = 2;
  double flute_length = 20.0;
  double overall_length = 50.0;
  double shank_diameter = 6.0;
  double neck_diameter = 0.0;   // zero when the tool has no reduced neck
  double neck_length = 0.0;
  double helix_angle = 30.0;
  double included_angle = 0.0;  // V-bits, chamfer tools, engravers
  double tip_diameter = 0.0;
  bool center_cutting = true;
  bool variable_helix = false;

  // Substrate, coating and holder
  Substrate substrate = Substrate::Carbide;
  Coating coating = Coating::None;
  GeometryClass geometry_class = GeometryClass::General;
  double stickout = 20.0;
  HolderType holder = HolderType::ERCollet;
  double runout = 0.01;
  int tool_number = 1;
  int length_offset = 1;
  double cost = 0.0;
  double expected_life_minutes = 0.0;

  std::vector<CuttingData> cutting_data;

  const CuttingData* data_for(const std::string& material_id) const;
};

// Young's modulus in MPa for the deflection model. A carbide cutter is roughly
// three times as stiff as a steel one of the same section, which is most of
// why the same stickout is fine on one and not the other.
double youngs_modulus(Substrate substrate);

// The fraction of rated surface speed a coating supports. Uncoated carbide in
// aluminium is the reference.
double coating_speed_factor(Coating coating);

// The core diameter that actually resists bending. Flutes remove a great deal
// of the section, and treating an endmill as a solid cylinder of its nominal
// diameter understates deflection by a factor of two or more.
double effective_core_diameter(const Tool& tool);

// Static deflection at the tip under a lateral load, in mm. Cantilever beam
// over the stickout, using the core diameter through the flutes.
double tip_deflection(const Tool& tool, double lateral_force_newtons);

// The diameter actually cutting at a given axial depth. A ball nose engaged
// only at its tip presents far less than its nominal diameter, and running it
// at the nominal surface speed leaves the cutting edge crawling.
double effective_diameter(const Tool& tool, double axial_depth);

const char* tool_type_name(ToolType value);
const char* substrate_name(Substrate value);
const char* coating_name(Coating value);
const char* geometry_class_name(GeometryClass value);
const char* holder_type_name(HolderType value);
const char* cutting_direction_name(CuttingDirection value);
const char* coolant_need_name(CoolantNeed value);

std::optional<ToolType> tool_type_from_name(const std::string& text);
std::optional<Substrate> substrate_from_name(const std::string& text);
std::optional<Coating> coating_from_name(const std::string& text);
std::optional<GeometryClass> geometry_class_from_name(const std::string& text);
std::optional<HolderType> holder_type_from_name(const std::string& text);
std::optional<CuttingDirection> cutting_direction_from_name(const std::string& text);
std::optional<CoolantNeed> coolant_need_from_name(const std::string& text);

// Everything wrong with a tool, so a form can show all of it at once rather
// than one complaint per attempt to save.
std::vector<std::string> validate(const Tool& tool);

struct ToolLibrary {
  std::vector<Tool> tools;

  const Tool* find(const std::string& id) const;
  Tool* find(const std::string& id);
  // Replaces a tool of the same id, or appends. Returns true when it replaced.
  bool upsert(Tool tool);
  bool erase(const std::string& id);
  // A slug derived from the name that no existing tool is using.
  std::string unique_id_for(const std::string& name) const;
};

}  // namespace cam
