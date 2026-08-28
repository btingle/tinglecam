#include "cam/tooling.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>

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

constexpr std::pair<ToolType, const char*> kToolTypes[]{
    {ToolType::FlatEndmill, "flat"},   {ToolType::BallEndmill, "ball"},
    {ToolType::BullNose, "bull-nose"}, {ToolType::Chamfer, "chamfer"},
    {ToolType::VBit, "vee"},           {ToolType::Drill, "drill"},
    {ToolType::FaceMill, "face-mill"}, {ToolType::ThreadMill, "thread-mill"},
    {ToolType::Engraver, "engraver"},
};

constexpr std::pair<Substrate, const char*> kSubstrates[]{
    {Substrate::HighSpeedSteel, "hss"},
    {Substrate::Cobalt, "cobalt"},
    {Substrate::Carbide, "carbide"},
    {Substrate::MicrograinCarbide, "micrograin-carbide"},
    {Substrate::Ceramic, "ceramic"},
};

constexpr std::pair<Coating, const char*> kCoatings[]{
    {Coating::None, "none"},   {Coating::TiN, "tin"},   {Coating::TiCN, "ticn"},
    {Coating::TiAlN, "tialn"}, {Coating::AlTiN, "altin"}, {Coating::ZrN, "zrn"},
    {Coating::DLC, "dlc"},     {Coating::Diamond, "diamond"},
};

constexpr std::pair<GeometryClass, const char*> kGeometryClasses[]{
    {GeometryClass::General, "general"},
    {GeometryClass::HighHelixAluminium, "high-helix-aluminium"},
    {GeometryClass::RoughingChipbreaker, "roughing-chipbreaker"},
    {GeometryClass::Finisher, "finisher"},
    {GeometryClass::HardMilling, "hard-milling"},
};

constexpr std::pair<HolderType, const char*> kHolders[]{
    {HolderType::ERCollet, "er-collet"},
    {HolderType::ShrinkFit, "shrink-fit"},
    {HolderType::Weldon, "weldon"},
    {HolderType::Hydraulic, "hydraulic"},
};

constexpr std::pair<CuttingDirection, const char*> kDirections[]{
    {CuttingDirection::Climb, "climb"},
    {CuttingDirection::Conventional, "conventional"},
    {CuttingDirection::Either, "either"},
};

constexpr std::pair<CoolantNeed, const char*> kCoolantNeeds[]{
    {CoolantNeed::Required, "required"},
    {CoolantNeed::Recommended, "recommended"},
    {CoolantNeed::Dry, "dry"},
    {CoolantNeed::AirBlast, "air-blast"},
};

}  // namespace

const char* tool_type_name(ToolType value) { return name_of(value, kToolTypes); }
const char* substrate_name(Substrate value) { return name_of(value, kSubstrates); }
const char* coating_name(Coating value) { return name_of(value, kCoatings); }
const char* geometry_class_name(GeometryClass value) {
  return name_of(value, kGeometryClasses);
}
const char* holder_type_name(HolderType value) { return name_of(value, kHolders); }
const char* cutting_direction_name(CuttingDirection value) {
  return name_of(value, kDirections);
}
const char* coolant_need_name(CoolantNeed value) {
  return name_of(value, kCoolantNeeds);
}

std::optional<ToolType> tool_type_from_name(const std::string& text) {
  return value_of(text, kToolTypes);
}
std::optional<Substrate> substrate_from_name(const std::string& text) {
  return value_of(text, kSubstrates);
}
std::optional<Coating> coating_from_name(const std::string& text) {
  return value_of(text, kCoatings);
}
std::optional<GeometryClass> geometry_class_from_name(const std::string& text) {
  return value_of(text, kGeometryClasses);
}
std::optional<HolderType> holder_type_from_name(const std::string& text) {
  return value_of(text, kHolders);
}
std::optional<CuttingDirection> cutting_direction_from_name(const std::string& text) {
  return value_of(text, kDirections);
}
std::optional<CoolantNeed> coolant_need_from_name(const std::string& text) {
  return value_of(text, kCoolantNeeds);
}

const CuttingData* Tool::data_for(const std::string& material_id) const {
  for (const auto& entry : cutting_data)
    if (entry.material_id == material_id) return &entry;
  return nullptr;
}

double youngs_modulus(Substrate substrate) {
  switch (substrate) {
    case Substrate::HighSpeedSteel: return 210000.0;
    case Substrate::Cobalt: return 220000.0;
    case Substrate::Carbide: return 580000.0;
    case Substrate::MicrograinCarbide: return 620000.0;
    case Substrate::Ceramic: return 380000.0;
  }
  return 210000.0;
}

double coating_speed_factor(Coating coating) {
  switch (coating) {
    case Coating::None: return 1.0;
    case Coating::TiN: return 1.15;
    case Coating::TiCN: return 1.25;
    case Coating::TiAlN: return 1.45;
    case Coating::AlTiN: return 1.5;
    case Coating::ZrN: return 1.2;
    case Coating::DLC: return 1.3;
    case Coating::Diamond: return 1.8;
  }
  return 1.0;
}

double effective_core_diameter(const Tool& tool) {
  // Fluting removes section, and the more flutes the less core is left. These
  // are the usual rules of thumb rather than a measurement: a two-flute cutter
  // keeps around 60% of its diameter as core, a four-flute around 70%, and the
  // difference matters because the fourth power of it drives deflection.
  double fraction = 0.65;
  switch (tool.flutes) {
    case 1: fraction = 0.55; break;
    case 2: fraction = 0.60; break;
    case 3: fraction = 0.66; break;
    case 4: fraction = 0.70; break;
    default: fraction = 0.75; break;
  }
  const double core = tool.diameter * fraction;
  // A reduced neck is narrower than any of that and governs on its own.
  if (tool.neck_diameter > 0.0 && tool.neck_diameter < core) return tool.neck_diameter;
  return core;
}

double tip_deflection(const Tool& tool, double lateral_force_newtons) {
  if (!(lateral_force_newtons > 0.0)) return 0.0;
  const double length = std::max(tool.stickout, 0.001);
  const double diameter = std::max(effective_core_diameter(tool), 0.001);
  // Second moment of area of a circular section, mm^4.
  const double inertia = std::numbers::pi * std::pow(diameter, 4.0) / 64.0;
  const double modulus = youngs_modulus(tool.substrate);
  return lateral_force_newtons * std::pow(length, 3.0) / (3.0 * modulus * inertia);
}

double effective_diameter(const Tool& tool, double axial_depth) {
  const double depth = std::max(axial_depth, 0.0);
  const double radius = tool.diameter * 0.5;
  switch (tool.type) {
    case ToolType::BallEndmill: {
      if (depth >= radius) return tool.diameter;
      // Chord of the ball at the depth of cut.
      const double reach = radius - depth;
      return 2.0 * std::sqrt(std::max(radius * radius - reach * reach, 0.0));
    }
    case ToolType::BullNose: {
      const double corner = std::clamp(tool.corner_radius, 0.0, radius);
      if (corner <= 0.0 || depth >= corner) return tool.diameter;
      const double reach = corner - depth;
      const double flat = tool.diameter - 2.0 * corner;
      return flat + 2.0 * std::sqrt(std::max(corner * corner - reach * reach, 0.0));
    }
    case ToolType::VBit:
    case ToolType::Engraver: {
      if (!(tool.included_angle > 0.0)) return tool.diameter;
      const double half = tool.included_angle * 0.5 * std::numbers::pi / 180.0;
      const double at_depth = tool.tip_diameter + 2.0 * depth * std::tan(half);
      return std::clamp(at_depth, 0.0, tool.diameter);
    }
    default:
      return tool.diameter;
  }
}

std::vector<std::string> validate(const Tool& tool) {
  std::vector<std::string> problems;
  const auto require = [&](bool condition, std::string message) {
    if (!condition) problems.push_back(std::move(message));
  };

  require(!tool.id.empty(), "The tool needs an identifier");
  require(!tool.name.empty(), "The tool needs a name");
  require(tool.diameter > 0.0, "Diameter must be greater than zero");
  require(tool.flutes >= 1, "A cutter needs at least one flute");
  require(tool.flute_length > 0.0, "Flute length must be greater than zero");
  require(tool.stickout > 0.0, "Stickout must be greater than zero");
  require(tool.shank_diameter > 0.0, "Shank diameter must be greater than zero");
  require(tool.corner_radius >= 0.0, "Corner radius cannot be negative");
  require(tool.corner_radius <= tool.diameter * 0.5 + 1e-9,
          "Corner radius cannot exceed half the diameter");
  require(tool.overall_length <= 0.0 || tool.overall_length >= tool.flute_length,
          "Overall length is shorter than the flute length");
  require(tool.stickout <= 0.0 || tool.overall_length <= 0.0 ||
              tool.stickout <= tool.overall_length,
          "Stickout is longer than the tool");
  require(tool.neck_length <= 0.0 || tool.neck_diameter > 0.0,
          "A neck length needs a neck diameter");

  if (tool.type == ToolType::BallEndmill)
    require(std::abs(tool.corner_radius - tool.diameter * 0.5) < 1e-6,
            "A ball nose has a corner radius of half its diameter");
  if (tool.type == ToolType::VBit || tool.type == ToolType::Engraver ||
      tool.type == ToolType::Chamfer)
    require(tool.included_angle > 0.0 && tool.included_angle < 180.0,
            "A pointed tool needs an included angle between 0 and 180 degrees");

  for (const auto& data : tool.cutting_data) {
    const std::string where = " for " + data.material_id;
    require(!data.material_id.empty(), "Cutting data is missing its material");
    require(data.surface_speed > 0.0, "Surface speed must be positive" + where);
    require(data.feed_per_tooth > 0.0, "Feed per tooth must be positive" + where);
    require(data.min_chip_thickness >= 0.0,
            "Minimum chip thickness cannot be negative" + where);
    require(data.min_chip_thickness < data.feed_per_tooth,
            "Minimum chip thickness is not below the feed per tooth" + where);
    require(data.max_ae_over_d > 0.0 && data.max_ae_over_d <= 1.0,
            "Radial engagement ceiling must be within (0, 1]" + where);
    require(data.max_ap_over_d > 0.0, "Axial engagement ceiling must be positive" + where);
    require(data.plunge_feed_factor > 0.0 && data.plunge_feed_factor <= 1.0,
            "Plunge feed factor must be within (0, 1]" + where);
  }

  std::vector<std::string> seen;
  for (const auto& data : tool.cutting_data) {
    if (std::find(seen.begin(), seen.end(), data.material_id) != seen.end())
      problems.push_back("Two cutting-data entries for " + data.material_id);
    seen.push_back(data.material_id);
  }
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
