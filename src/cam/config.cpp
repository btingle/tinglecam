#include "cam/config.h"
#include "cam/hole_config.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cam {
namespace {

using nlohmann::json;

std::optional<OperationMode> operation_mode_from_name(const std::string& name) {
  if (name == "roughing") return OperationMode::Roughing;
  if (name == "contour_finishing") return OperationMode::ContourFinishing;
  if (name == "hole") return OperationMode::Hole;
  if (name == "pocket") return OperationMode::Pocket;
  if (name == "plane_clearing") return OperationMode::PlaneClearing;
  if (name == "surface_finishing") return OperationMode::SurfaceFinishing;
  return std::nullopt;
}

std::optional<ContourDirectionPolicy> contour_direction_from_name(
    const std::string& name) {
  if (name == "either") return ContourDirectionPolicy::Either;
  if (name == "climb") return ContourDirectionPolicy::Climb;
  if (name == "conventional") return ContourDirectionPolicy::Conventional;
  return std::nullopt;
}

std::size_t operation_default_index(OperationMode mode) {
  switch (mode) {
    case OperationMode::Roughing: return 0;
    case OperationMode::ContourFinishing: return 1;
    case OperationMode::Pocket: return 2;
    case OperationMode::PlaneClearing: return 3;
    case OperationMode::SurfaceFinishing: return 4;
    case OperationMode::Hole: return 5;
  }
  return 0;
}

constexpr std::array<OperationMode, 6> kOperationModes{
    OperationMode::Roughing, OperationMode::ContourFinishing,
    OperationMode::Pocket, OperationMode::PlaneClearing, OperationMode::SurfaceFinishing, OperationMode::Hole};

template <typename T>
void read(const json& node, const char* key, T& target) {
  if (!node.contains(key) || node.at(key).is_null()) return;
  target = node.at(key).get<T>();
}

// Names fields a reader does not know, so an old or mistyped file says what
// it lost instead of loading quietly without it.
void report_unknown(const json& node, std::initializer_list<const char*> known,
                    const std::string& where, std::vector<std::string>* warnings) {
  if (!warnings || !node.is_object()) return;
  for (const auto& item : node.items()) {
    const bool found = std::any_of(known.begin(), known.end(), [&](const char* key) {
      return item.key() == key;
    });
    if (!found) warnings->push_back(where + ": unknown field \"" + item.key() + "\" ignored");
  }
}

template <typename Enum>
void read_enum(const json& node, const char* key, Enum& target,
               std::optional<Enum> (*parse)(const std::string&)) {
  if (!node.contains(key) || !node.at(key).is_string()) return;
  const auto parsed = parse(node.at(key).get<std::string>());
  if (parsed) target = *parsed;
}

json to_json(const Tool& tool) {
  json cutting = json::array();
  for (const auto& data : tool.cutting)
    cutting.push_back({{"material", data.material_id},
                       {"spindle_rpm", data.spindle_rpm},
                       {"chip_load", data.chip_load},
                       {"plunge_feed", data.plunge_feed}});
  return json{{"id", tool.id}, {"name", tool.name},
              {"diameter", tool.diameter}, {"flute_length", tool.flute_length},
              {"flutes", tool.flutes}, {"type", tool_type_name(tool.type)},
              {"cutting", std::move(cutting)},
              {"point_angle_degrees", tool.point_angle_degrees},
              {"tip_flat_diameter", tool.tip_flat_diameter}};
}

Tool tool_from(const json& node) {
  Tool tool;
  read(node, "id", tool.id);
  read(node, "name", tool.name);
  read(node, "diameter", tool.diameter);
  read(node, "flute_length", tool.flute_length);
  read(node, "flutes", tool.flutes);
  read(node, "point_angle_degrees", tool.point_angle_degrees);
  read(node, "tip_flat_diameter", tool.tip_flat_diameter);
  if (node.contains("type")) {
    const auto type = tool_type_from_name(node.at("type").get<std::string>());
    if (!type) throw std::runtime_error("Unknown tool type");
    tool.type = *type;
  }
  if (node.contains("cutting")) {
    if (!node.at("cutting").is_array())
      throw std::runtime_error("Expected a cutting data array");
    for (const auto& entry : node.at("cutting")) {
      CuttingData data;
      read(entry, "material", data.material_id);
      read(entry, "spindle_rpm", data.spindle_rpm);
      read(entry, "chip_load", data.chip_load);
      read(entry, "plunge_feed", data.plunge_feed);
      tool.cutting.push_back(std::move(data));
    }
  }
  return tool;
}

json to_json(const Machine& machine) {
  return json{{"id", machine.id},
              {"name", machine.name},
              {"specifications_unverified", machine.specifications_unverified},
              {"controller", controller_name(machine.controller)},
              {"rapid_feed", machine.rapid_feed},
              {"rapid_feed_z", machine.rapid_feed_z},
              {"min_rpm", machine.min_rpm},
              {"max_rpm", machine.max_rpm},
              {"rpm_resolution", machine.rpm_resolution},
              {"spindle_spinup_seconds", machine.spindle_spinup_seconds},
              {"tool_change", tool_change_name(machine.tool_change)},
              {"tool_change_x", machine.tool_change_x},
              {"tool_change_y", machine.tool_change_y},
              {"length_offset_mode",
               length_offset_mode_name(machine.length_offset_mode)},
              {"coolant_flood", machine.coolant_flood},
              {"coolant_mist", machine.coolant_mist},
              {"output_inches", machine.output_inches},
              {"decimals", machine.decimals},
              {"line_numbers", machine.line_numbers},
              {"line_number_increment", machine.line_number_increment},
              {"program_header", machine.program_header},
              {"program_footer", machine.program_footer},
              {"path_blending", machine.path_blending}};
}

Machine machine_from(const json& node) {
  Machine machine;
  read(node, "id", machine.id);
  read(node, "name", machine.name);
  read(node, "specifications_unverified", machine.specifications_unverified);
  read_enum(node, "controller", machine.controller, controller_from_name);
  read(node, "rapid_feed", machine.rapid_feed);
  read(node, "rapid_feed_z", machine.rapid_feed_z);
  read(node, "min_rpm", machine.min_rpm);
  read(node, "max_rpm", machine.max_rpm);
  read(node, "rpm_resolution", machine.rpm_resolution);
  read(node, "spindle_spinup_seconds", machine.spindle_spinup_seconds);
  read_enum(node, "tool_change", machine.tool_change, tool_change_from_name);
  read(node, "tool_change_x", machine.tool_change_x);
  read(node, "tool_change_y", machine.tool_change_y);
  read_enum(node, "length_offset_mode", machine.length_offset_mode,
            length_offset_mode_from_name);
  read(node, "coolant_flood", machine.coolant_flood);
  read(node, "coolant_mist", machine.coolant_mist);
  read(node, "output_inches", machine.output_inches);
  read(node, "decimals", machine.decimals);
  read(node, "line_numbers", machine.line_numbers);
  read(node, "line_number_increment", machine.line_number_increment);
  read(node, "program_header", machine.program_header);
  read(node, "program_footer", machine.program_footer);
  read(node, "path_blending", machine.path_blending);
  return machine;
}

json to_json(const Material& material) {
  return json{{"id", material.id},
              {"name", material.name}};
}

Material material_from(const json& node) {
  Material material;
  read(node, "id", material.id);
  read(node, "name", material.name);
  return material;
}

json to_json(const Operation& operation) {
  json pockets = json::array();
  for (const auto& p : operation.pockets)
    pockets.push_back({{"shape", pocket_shape_name(p.shape)},
                       {"center_x", p.center_x},
                       {"center_y", p.center_y},
                       {"top_z", p.top_z},
                       {"width", p.width},
                       {"height", p.height},
                       {"depth", p.depth},
                       {"radius", p.radius},
                       {"helix_angle_degrees", p.helix_angle_degrees}});
  json areas = json::array();
  for (const auto& area : operation.plane.areas)
    areas.push_back({{"center_x", area.center_x}, {"center_y", area.center_y},
                     {"width", area.width}, {"height", area.height},
                     {"top_z", area.top_z}, {"depth", area.depth}});
  json exclusions = json::array();
  for (const auto& exclusion : operation.pocket_auto.exclusions)
    exclusions.push_back({{"enabled", exclusion.enabled},
                          {"center_x", exclusion.center_x},
                          {"center_y", exclusion.center_y},
                          {"width", exclusion.width},
                          {"height", exclusion.height}});
  json cutting = json::object();
  if (operation.cutting.spindle_rpm)
    cutting["spindle_rpm"] = *operation.cutting.spindle_rpm;
  if (operation.cutting.chip_load)
    cutting["chip_load"] = *operation.cutting.chip_load;
  if (operation.cutting.plunge_feed)
    cutting["plunge_feed"] = *operation.cutting.plunge_feed;
  json result{{"name", operation.name},
              {"mode", operation_mode_name(operation.mode)},
              {"tool", operation.tool_id},
              {"cutting", std::move(cutting)},
              {"surface", {{"faces", operation.surface.faces},
                           {"model_fingerprint", operation.surface.model_fingerprint},
                           {"stepover", operation.surface.stepover},
                           {"chord_tolerance", operation.surface.chord_tolerance},
                           {"allowance", operation.surface.allowance},
                           {"along_v", operation.surface.along_v},
                           {"bidirectional", operation.surface.bidirectional},
                           {"flip_normals", operation.surface.flip_normals},
                           {"manual_order", operation.surface.manual_order},
                           {"order_signature", operation.surface.order_signature},
                           {"order_roots", operation.surface.order_roots}}},
              {"stepover_ratio", operation.stepover_ratio},
              {"stepdown", operation.stepdown},
              {"coolant", operation.coolant},
              {"pockets", std::move(pockets)},
              {"auto_identify_pockets", operation.auto_identify_pockets},
              {"pocket_radial_margin", operation.pocket_radial_margin},
              {"pocket_auto", {
                  {"check_depth", operation.pocket_auto.check_depth},
                  {"helix_angle_degrees",
                   operation.pocket_auto.helix_angle_degrees},
                  {"exclusions", std::move(exclusions)}}},
              {"plane", {{"areas", areas}}},
              {"finishing_subdivisions", operation.finishing_subdivisions},
              {"adaptive_cleanup_stepover", operation.adaptive_cleanup_stepover},
              {"cleanup_stepover_limit", operation.cleanup_stepover_limit},
              {"depth_breaks", operation.depth_breaks},
              {"radial_stock_to_leave", operation.radial_stock_to_leave},
              {"axial_stock_to_leave", operation.axial_stock_to_leave},
              {"depth_from", operation.depth_from},
              {"depth_to", operation.depth_to},
              {"lead_in", operation.lead_in},
              {"stock_deviation", operation.stock_deviation},
              {"path_pitch", operation.path_pitch},
              {"entry_front_match_tolerance",
               operation.entry_front_match_tolerance},
              {"tiny_split_area_factor", operation.tiny_split_area_factor},
              {"clip_contours_to_stock", operation.clip_contours_to_stock},
              {"contour_direction",
               contour_direction_name(operation.contour_direction)},
              {"rough_boundary_cleanup", operation.rough_boundary_cleanup},
              {"cleanup_tolerance", operation.cleanup_tolerance},
              {"slice_grid", operation.slice_grid}};
  if(operation.mode==OperationMode::Hole) result["holes"]=hole_parameters_json(operation.holes);
  return result;
}

json to_json(const UserSettings& settings) {
  json defaults = json::object();
  for (const OperationMode mode : kOperationModes)
    defaults[operation_mode_name(mode)] =
        to_json(operation_default(settings, mode));
  json themes = json::array();
  for (const auto& theme : settings.custom_themes)
    themes.push_back({{"name", theme.name},
                      {"font", ui_font_name(theme.font)},
                      {"style", theme.style_blob}});
  const PanelLayout& layout = settings.panel_layout;
  return json{{"ui_scale", settings.ui_scale},
              {"ui_theme", ui_theme_name(settings.ui_theme)},
              {"custom_theme", settings.custom_theme},
              {"custom_themes", std::move(themes)},
              {"panel_layout", {{"left_width", layout.left_width},
                                 {"right_width", layout.right_width},
                                 {"left_setup_fraction", layout.left_setup_fraction},
                                 {"messages_height", layout.messages_height}}},
              {"show_debug", settings.show_debug},
              {"default_machine", settings.default_machine},
              {"default_material", settings.default_material},
              {"recent_jobs", settings.recent_jobs},
              {"operation_defaults", std::move(defaults)}};
}

Operation operation_from(const json& node, std::vector<std::string>* warnings = nullptr,
                         const std::string& where = {}) {
  report_unknown(node,
                 {"name", "mode", "tool", "cutting", "surface", "stepover_ratio",
                  "stepdown", "coolant", "pockets", "holes", "auto_identify_pockets",
                  "pocket_radial_margin", "pocket_auto", "plane",
                  "finishing_subdivisions", "adaptive_cleanup_stepover",
                  "cleanup_stepover_limit", "depth_breaks", "radial_stock_to_leave",
                  "axial_stock_to_leave", "depth_from", "depth_to", "lead_in",
                  "stock_deviation", "path_pitch", "entry_front_match_tolerance",
                  "tiny_split_area_factor", "clip_contours_to_stock",
                  "contour_direction", "rough_boundary_cleanup", "cleanup_tolerance",
                  "slice_grid"},
                 where, warnings);
  Operation operation;
  read(node, "name", operation.name);
  read_enum(node, "mode", operation.mode, operation_mode_from_name);
  read(node, "tool", operation.tool_id);
  if (node.contains("cutting")) {
    const auto& cutting = node.at("cutting");
    if (!cutting.is_object()) throw std::runtime_error("Expected cutting overrides object");
    const auto optional = [&](const char* key, std::optional<double>& target) {
      if (cutting.contains(key) && !cutting.at(key).is_null())
        target = cutting.at(key).get<double>();
    };
    optional("spindle_rpm", operation.cutting.spindle_rpm);
    optional("chip_load", operation.cutting.chip_load);
    optional("plunge_feed", operation.cutting.plunge_feed);
  }
  if (node.contains("surface")) {
    const auto& surface = node.at("surface");
    read(surface, "faces", operation.surface.faces);
    read(surface, "model_fingerprint", operation.surface.model_fingerprint);
    read(surface, "stepover", operation.surface.stepover);
    read(surface, "chord_tolerance", operation.surface.chord_tolerance);
    read(surface, "allowance", operation.surface.allowance);
    read(surface, "along_v", operation.surface.along_v);
    read(surface, "bidirectional", operation.surface.bidirectional);
    read(surface, "flip_normals", operation.surface.flip_normals);
    read(surface, "manual_order", operation.surface.manual_order);
    read(surface, "order_signature", operation.surface.order_signature);
    read(surface, "order_roots", operation.surface.order_roots);
  }
  read(node, "stepover_ratio", operation.stepover_ratio);
  read(node, "stepdown", operation.stepdown);
  read(node, "coolant", operation.coolant);
  if (node.contains("holes")) operation.holes = hole_parameters_from_json(node.at("holes"));
  read(node, "auto_identify_pockets", operation.auto_identify_pockets);
  if (node.contains("pocket_auto")) {
    const auto& automatic = node.at("pocket_auto");
    read(automatic, "check_depth", operation.pocket_auto.check_depth);
    read(automatic, "helix_angle_degrees",
         operation.pocket_auto.helix_angle_degrees);
    const auto read_exclusion = [](const json& node) {
      PocketExclusionArea exclusion;
      read(node, "enabled", exclusion.enabled);
      read(node, "center_x", exclusion.center_x);
      read(node, "center_y", exclusion.center_y);
      read(node, "width", exclusion.width);
      read(node, "height", exclusion.height);
      return exclusion;
    };
    if (automatic.contains("exclusions")) {
      const auto& exclusions = automatic.at("exclusions");
      if (!exclusions.is_array())
        throw std::runtime_error("Expected pocket exclusions array");
      operation.pocket_auto.exclusions.clear();
      for (const auto& exclusion : exclusions)
        operation.pocket_auto.exclusions.push_back(read_exclusion(exclusion));
    }
  }
  if (node.contains("plane")) {
    const auto& plane = node.at("plane");
    if (plane.contains("areas")) {
      if (!plane.at("areas").is_array()) throw std::runtime_error("Expected plane areas array");
      operation.plane.areas.clear();
      for (const auto& entry : plane.at("areas")) {
        ClearingArea area;
        read(entry, "center_x", area.center_x);
        read(entry, "center_y", area.center_y);
        read(entry, "width", area.width);
        read(entry, "height", area.height);
        read(entry, "top_z", area.top_z);
        read(entry, "depth", area.depth);
        operation.plane.areas.push_back(area);
      }
    }
  }
  const auto read_pocket = [](const json& pocket) {
    PocketParameters p;
    if (pocket.contains("shape")) {
      if (!pocket.at("shape").is_string())
        throw std::runtime_error("Pocket shape must be a string");
      const auto parsed = pocket_shape_from_name(pocket.at("shape").get<std::string>());
      if (!parsed) throw std::runtime_error("Unknown pocket shape");
      p.shape = *parsed;
    }
    read(pocket, "center_x", p.center_x);
    read(pocket, "center_y", p.center_y);
    read(pocket, "top_z", p.top_z);
    read(pocket, "width", p.width);
    read(pocket, "height", p.height);
    read(pocket, "depth", p.depth);
    read(pocket, "radius", p.radius);
    read(pocket, "helix_angle_degrees", p.helix_angle_degrees);
    return p;
  };
  if (node.contains("pockets")) {
    const auto& pockets = node.at("pockets");
    if (!pockets.is_array()) throw std::runtime_error("Expected pockets array");
    operation.pockets.clear();
    for (const auto& pocket : pockets)
      operation.pockets.push_back(read_pocket(pocket));
  }
  read(node, "pocket_radial_margin", operation.pocket_radial_margin);
  read(node, "finishing_subdivisions", operation.finishing_subdivisions);
  read(node, "adaptive_cleanup_stepover", operation.adaptive_cleanup_stepover);
  read(node, "cleanup_stepover_limit", operation.cleanup_stepover_limit);
  read(node, "depth_breaks", operation.depth_breaks);
  read(node, "radial_stock_to_leave", operation.radial_stock_to_leave);
  read(node, "axial_stock_to_leave", operation.axial_stock_to_leave);
  read(node, "depth_from", operation.depth_from);
  read(node, "depth_to", operation.depth_to);
  read(node, "lead_in", operation.lead_in);
  read(node, "stock_deviation", operation.stock_deviation);
  read(node, "path_pitch", operation.path_pitch);
  read(node, "entry_front_match_tolerance",
       operation.entry_front_match_tolerance);
  read(node, "tiny_split_area_factor", operation.tiny_split_area_factor);
  read(node, "clip_contours_to_stock", operation.clip_contours_to_stock);
  read_enum(node, "contour_direction", operation.contour_direction,
            contour_direction_from_name);
  read(node, "rough_boundary_cleanup", operation.rough_boundary_cleanup);
  read(node, "cleanup_tolerance", operation.cleanup_tolerance);
  read(node, "slice_grid", operation.slice_grid);
  return operation;
}

UserSettings settings_from(const json& node) {
  UserSettings settings = default_user_settings();
  read(node, "ui_scale", settings.ui_scale);
  read_enum(node, "ui_theme", settings.ui_theme, ui_theme_from_name);
  read(node, "custom_theme", settings.custom_theme);
  if (node.contains("custom_themes")) {
    const auto& themes = node.at("custom_themes");
    if (!themes.is_array())
      throw std::runtime_error("Expected custom_themes array");
    for (const auto& item : themes) {
      if (!item.is_object())
        throw std::runtime_error("Expected custom theme object");
      CustomUiTheme theme;
      read(item, "name", theme.name);
      read_enum(item, "font", theme.font, ui_font_from_name);
      read(item, "style", theme.style_blob);
      if (!theme.name.empty() && !theme.style_blob.empty())
        settings.custom_themes.push_back(std::move(theme));
    }
  }
  if (!(settings.ui_scale >= 0.5 && settings.ui_scale <= 3.0) ||
      !std::isfinite(settings.ui_scale))
    throw std::runtime_error("UI scale must be finite and between 0.5 and 3.0");
  if (node.contains("panel_layout")) {
    const auto& layout = node.at("panel_layout");
    if (!layout.is_object())
      throw std::runtime_error("Expected panel_layout object");
    read(layout, "left_width", settings.panel_layout.left_width);
    read(layout, "right_width", settings.panel_layout.right_width);
    read(layout, "left_setup_fraction", settings.panel_layout.left_setup_fraction);
    read(layout, "messages_height", settings.panel_layout.messages_height);
  }
  if (!valid_panel_layout(settings.panel_layout))
    settings.panel_layout = default_panel_layout();
  read(node, "show_debug", settings.show_debug);
  read(node, "default_machine", settings.default_machine);
  read(node, "default_material", settings.default_material);
  read(node, "recent_jobs", settings.recent_jobs);
  if (node.contains("operation_defaults")) {
    const auto& defaults = node.at("operation_defaults");
    if (!defaults.is_object())
      throw std::runtime_error("Expected operation_defaults object");
    for (const OperationMode mode : kOperationModes) {
      const std::string key = operation_mode_name(mode);
      if (!defaults.contains(key)) continue;
      settings.operation_defaults[operation_default_index(mode)] =
          operation_from(defaults.at(key));
      settings.operation_defaults[operation_default_index(mode)].mode = mode;
    }
  }
  return settings;
}

json to_json(const Job& job) {
  json operations = json::array();
  for (const auto& operation : job.operations)
    operations.push_back(to_json(operation));
  const BoxStockMargins& margins = job.box_stock_margins;
  json tools = json::array();
  for (const auto& entry : job.tools)
    tools.push_back({{"tool", entry.tool_id},
                     {"number", entry.number},
                     {"length_offset", entry.length_offset}});
  return json{{"name", job.name},
              {"machine", job.machine_id},
              {"material", job.material_id},
              {"simulation_pitch", job.simulation_pitch},
              {"tools", std::move(tools)},
              {"box_stock", job.box_stock},
              {"box_stock_margins", {
                  {"minus_x", margins.minus_x},
                  {"plus_x", margins.plus_x},
                  {"minus_y", margins.minus_y},
                  {"plus_y", margins.plus_y},
                  {"minus_z", margins.minus_z},
                  {"plus_z", margins.plus_z}}},
              {"part_mesh", job.part_mesh.string()},
              {"stock_mesh", job.stock_mesh.string()},
              {"part_orientation", job.part_orientation.axes},
              {"stock_orientation", job.stock_orientation.axes},
              {"origin_x", job.origin_x},
              {"origin_y", job.origin_y},
              {"origin_z", job.origin_z},
              {"rapid_clearance_height", job.rapid_clearance_height},
              {"plunge_clearance_height", job.plunge_clearance_height},
              {"work_offset", job.work_offset},
              {"operations", operations}};
}

Job job_from(const json& node, std::vector<std::string>* warnings = nullptr) {
  report_unknown(node,
                 {"name", "machine", "material", "simulation_pitch", "tools",
                  "part_mesh", "stock_mesh", "part_orientation", "stock_orientation",
                  "origin_x", "origin_y", "origin_z",
                  "box_stock", "box_stock_margins", "rapid_clearance_height",
                  "plunge_clearance_height",
                  "work_offset", "operations"},
                 "Job", warnings);
  Job job;
  std::string part;
  std::string stock;
  read(node, "name", job.name);
  read(node, "machine", job.machine_id);
  read(node, "material", job.material_id);
  read(node, "simulation_pitch", job.simulation_pitch);
  if (node.contains("tools")) {
    if (!node.at("tools").is_array()) throw std::runtime_error("Expected a tools array");
    for (const auto& entry : node.at("tools")) {
      ToolAssignment assignment;
      read(entry, "tool", assignment.tool_id);
      read(entry, "number", assignment.number);
      read(entry, "length_offset", assignment.length_offset);
      job.tools.push_back(std::move(assignment));
    }
  }
  read(node, "part_mesh", part);
  read(node, "stock_mesh", stock);
  job.part_mesh = part;
  job.stock_mesh = stock;
  const auto orientation = [&](const char* key, ModelOrientation& value) {
    if (node.contains(key)) {
      const auto& entry = node.at(key);
      if (!entry.is_array() || entry.size() != 3)
        throw std::runtime_error(std::string("Expected three signed axes for ") + key);
      for (int i = 0; i < 3; ++i) {
        if (!entry[i].is_number_integer() || entry[i] < -3 || entry[i] > 3)
          throw std::runtime_error(std::string("Expected integer axes for ") + key);
        value.axes[i] = entry[i].get<int>();
      }
      value.validate();
    }
  };
  orientation("part_orientation", job.part_orientation);
  orientation("stock_orientation", job.stock_orientation);
  read(node, "origin_x", job.origin_x);
  read(node, "origin_y", job.origin_y);
  read(node, "origin_z", job.origin_z);
  read(node, "box_stock", job.box_stock);
  if (node.contains("box_stock_margins")) {
    const auto& margins = node.at("box_stock_margins");
    if (!margins.is_object())
      throw std::runtime_error("Expected box_stock_margins object");
    read(margins, "minus_x", job.box_stock_margins.minus_x);
    read(margins, "plus_x", job.box_stock_margins.plus_x);
    read(margins, "minus_y", job.box_stock_margins.minus_y);
    read(margins, "plus_y", job.box_stock_margins.plus_y);
    read(margins, "minus_z", job.box_stock_margins.minus_z);
    read(margins, "plus_z", job.box_stock_margins.plus_z);
  }
  read(node, "rapid_clearance_height", job.rapid_clearance_height);
  read(node, "plunge_clearance_height", job.plunge_clearance_height);
  read(node, "work_offset", job.work_offset);
  if (node.contains("operations") && node.at("operations").is_array())
    for (const auto& entry : node.at("operations"))
      job.operations.push_back(operation_from(
          entry, warnings, "Operation " + std::to_string(job.operations.size() + 1)));
  return job;
}

json parse(const std::string& text, const std::filesystem::path& where) {
  try {
    return json::parse(text, nullptr, true, true);
  } catch (const std::exception& error) {
    throw std::runtime_error(where.string() + ": " + error.what());
  }
}

}  // namespace

std::filesystem::path config_directory() {
#ifdef _WIN32
  if (const char* appdata = std::getenv("APPDATA"))
    return std::filesystem::path(appdata) / "TingleCAM";
#else
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME"))
    if (*xdg) return std::filesystem::path(xdg) / "tinglecam";
  if (const char* home = std::getenv("HOME"))
    return std::filesystem::path(home) / ".config" / "tinglecam";
#endif
  return std::filesystem::current_path() / ".tinglecam";
}

PanelLayout default_panel_layout() { return PanelLayout{}; }

bool valid_panel_layout(const PanelLayout& layout) {
  const auto within = [](double value, double low, double high) {
    return std::isfinite(value) && value >= low && value <= high;
  };
  return within(layout.left_width, 200.0, 2000.0) &&
         within(layout.right_width, 220.0, 2000.0) &&
         within(layout.left_setup_fraction, 0.15, 0.85) &&
         within(layout.messages_height, 60.0, 2000.0);
}

const char* ui_theme_name(UiTheme theme) {
  switch (theme) {
    case UiTheme::Default: return "Default";
    case UiTheme::Slate: return "Slate";
    case UiTheme::Ember: return "Ember";
    case UiTheme::Paper: return "Paper";
  }
  return "Default";
}

std::optional<UiTheme> ui_theme_from_name(const std::string& name) {
  if (name == "Default") return UiTheme::Default;
  if (name == "Slate") return UiTheme::Slate;
  if (name == "Ember") return UiTheme::Ember;
  if (name == "Paper") return UiTheme::Paper;
  // Accept the lowercase spelling used by hand-authored settings files too.
  if (name == "default") return UiTheme::Default;
  if (name == "slate") return UiTheme::Slate;
  if (name == "ember") return UiTheme::Ember;
  if (name == "paper") return UiTheme::Paper;
  return std::nullopt;
}

const char* ui_font_name(UiFont font) {
  switch (font) {
    case UiFont::Default: return "Default";
    case UiFont::Sans: return "Sans";
    case UiFont::Mono: return "Mono";
    case UiFont::Serif: return "Serif";
  }
  return "Default";
}

std::optional<UiFont> ui_font_from_name(const std::string& name) {
  if (name == "Default" || name == "default") return UiFont::Default;
  if (name == "Sans" || name == "sans") return UiFont::Sans;
  if (name == "Mono" || name == "mono") return UiFont::Mono;
  if (name == "Serif" || name == "serif") return UiFont::Serif;
  return std::nullopt;
}

UserSettings default_user_settings() {
  UserSettings settings;
  settings.panel_layout = default_panel_layout();
  for (const OperationMode mode : kOperationModes) {
    Operation& operation = settings.operation_defaults[operation_default_index(mode)];
    operation.mode = mode;
    if (mode == OperationMode::ContourFinishing) {
      operation.radial_stock_to_leave = 0.0;
      operation.rough_boundary_cleanup = false;
    }
  }
  return settings;
}

const Operation& operation_default(const UserSettings& settings,
                                   OperationMode mode) {
  return settings.operation_defaults[operation_default_index(mode)];
}

Operation& operation_default(UserSettings& settings, OperationMode mode) {
  return settings.operation_defaults[operation_default_index(mode)];
}

UserSettings load_user_settings(const std::filesystem::path& path) {
  std::error_code code;
  if (!std::filesystem::exists(path, code)) return default_user_settings();
  try {
    return settings_from(parse(read_file(path), path));
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

void save_user_settings(const std::filesystem::path& path,
                        const UserSettings& settings) {
  if (!(settings.ui_scale >= 0.5 && settings.ui_scale <= 3.0) ||
      !std::isfinite(settings.ui_scale))
    throw std::runtime_error("UI scale must be finite and between 0.5 and 3.0");
  if (!valid_panel_layout(settings.panel_layout))
    throw std::runtime_error("Panel layout values are outside their allowed ranges");
  write_file(path, to_json(settings).dump(2) + "\n");
}

std::filesystem::path data_directory() {
  std::error_code code;
  const std::filesystem::path beside =
      std::filesystem::current_path(code) / "data";
  if (std::filesystem::is_directory(beside, code)) return beside;
#ifdef TINGLECAM_DATA_DIR
  const std::filesystem::path built(TINGLECAM_DATA_DIR);
  if (std::filesystem::is_directory(built, code)) return built;
#endif
  return beside;
}

std::string read_file(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot open " + path.string());
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

void write_file(const std::filesystem::path& path, const std::string& text) {
  std::error_code code;
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path(), code);
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) throw std::runtime_error("Cannot open " + path.string());
  stream << text;
  if (!stream) throw std::runtime_error("Failed while writing " + path.string());
}

ToolLibrary load_tools(const std::filesystem::path& path) {
  ToolLibrary library;
  std::error_code code;
  if (!std::filesystem::exists(path, code)) return library;
  const json node = parse(read_file(path), path);
  try {
    const json& list = node.is_array() ? node : node.at("tools");
    if (!list.is_array()) throw std::runtime_error("Expected a tools array");
    for (const auto& entry : list) {
      Tool tool = tool_from(entry);
      const auto problems = validate(tool);
      if (!problems.empty())
        throw std::runtime_error(tool.name + ": " + problems.front());
      if (library.find(tool.id))
        throw std::runtime_error("Duplicate tool ID: " + tool.id);
      library.tools.push_back(std::move(tool));
    }
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
  return library;
}

void save_tools(const std::filesystem::path& path, const ToolLibrary& library) {
  json list = json::array();
  for (const auto& tool : library.tools) list.push_back(to_json(tool));
  // Stage beside the destination so replacing it cannot leave a truncated
  // library if writing fails. Keep the original file until the close succeeds.
  auto temporary = path;
  temporary += ".tmp";
  try {
    if (path.has_parent_path())
      std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) throw std::runtime_error("Cannot open " + temporary.string());
    stream << json{{"tools", list}}.dump(2) << '\n';
    stream.close();
    if (!stream) throw std::runtime_error("Failed while writing " + temporary.string());
    std::filesystem::rename(temporary, path);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}

MachineLibrary load_machines(const std::filesystem::path& directory) {
  MachineLibrary library;
  std::error_code code;
  if (!std::filesystem::is_directory(directory, code)) return library;
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(directory, code))
    if (entry.is_regular_file() && entry.path().extension() == ".json")
      files.push_back(entry.path());
  std::sort(files.begin(), files.end());
  for (const auto& file : files)
    library.upsert(machine_from(parse(read_file(file), file)));
  return library;
}

MaterialLibrary load_materials(const std::filesystem::path& directory) {
  MaterialLibrary library;
  std::error_code code;
  if (!std::filesystem::is_directory(directory, code)) return library;
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(directory, code))
    if (entry.is_regular_file() && entry.path().extension() == ".json")
      files.push_back(entry.path());
  std::sort(files.begin(), files.end());
  for (const auto& file : files)
    library.upsert(material_from(parse(read_file(file), file)));
  return library;
}

MaterialLibrary load_material_library(const std::filesystem::path& path) {
  MaterialLibrary library;
  const json node = parse(read_file(path), path);
  try {
    const json& list = node.at("materials");
    if (!list.is_array()) throw std::runtime_error("Expected a materials array");
    for (const auto& entry : list) {
      Material material = material_from(entry);
      const auto problems = validate(material);
      if (!problems.empty()) throw std::runtime_error(problems.front());
      if (library.find(material.id))
        throw std::runtime_error("Duplicate material ID: " + material.id);
      library.materials.push_back(std::move(material));
    }
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
  return library;
}

void save_material_library(const std::filesystem::path& path,
                           const MaterialLibrary& library) {
  json list = json::array();
  for (const auto& material : library.materials) list.push_back(to_json(material));
  write_file(path, json{{"materials", list}}.dump(2) + "\n");
}

void save_machine(const std::filesystem::path& path, const Machine& machine) {
  write_file(path, to_json(machine).dump(2) + "\n");
}
void save_material(const std::filesystem::path& path,
                   const Material& material) {
  write_file(path, to_json(material).dump(2) + "\n");
}
Job load_job(const std::filesystem::path& path, std::vector<std::string>* warnings) {
  return job_from(parse(read_file(path), path), warnings);
}
void save_job(const std::filesystem::path& path, const Job& job) {
  write_file(path, to_json(job).dump(2) + "\n");
}


std::string to_json_text(const Tool& tool) { return to_json(tool).dump(2); }
Tool tool_from_json_text(const std::string& text) {
  return tool_from(parse(text, "<memory>"));
}
std::string to_json_text(const Machine& machine) {
  return to_json(machine).dump(2);
}
Machine machine_from_json_text(const std::string& text) {
  return machine_from(parse(text, "<memory>"));
}
std::string to_json_text(const Material& material) {
  return to_json(material).dump(2);
}
Material material_from_json_text(const std::string& text) {
  return material_from(parse(text, "<memory>"));
}
std::string to_json_text(const Job& job) { return to_json(job).dump(2); }
std::string to_json_text(const Operation& operation) {
  return to_json(operation).dump(2);
}
Job job_from_json_text(const std::string& text) {
  return job_from(parse(text, "<memory>"));
}

}  // namespace cam
