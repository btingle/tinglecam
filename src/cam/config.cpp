#include "cam/config.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cam {
namespace {

using nlohmann::json;

// Missing keys keep whatever default the struct already carries, so a file
// written by an older build still loads and a new field simply takes its
// default rather than failing the whole library.
template <typename T>
void read(const json& node, const char* key, T& target) {
  if (!node.contains(key) || node.at(key).is_null()) return;
  target = node.at(key).get<T>();
}

template <typename Enum>
void read_enum(const json& node, const char* key, Enum& target,
               std::optional<Enum> (*parse)(const std::string&)) {
  if (!node.contains(key) || !node.at(key).is_string()) return;
  const auto parsed = parse(node.at(key).get<std::string>());
  if (parsed) target = *parsed;
}

json to_json(const CuttingData& data) {
  return json{
      {"material", data.material_id},
      {"surface_speed", data.surface_speed},
      {"feed_per_tooth", data.feed_per_tooth},
      {"min_chip_thickness", data.min_chip_thickness},
      {"max_ae_over_d", data.max_ae_over_d},
      {"max_ap_over_d", data.max_ap_over_d},
      {"max_ramp_angle", data.max_ramp_angle},
      {"helical_entry", data.helical_entry},
      {"min_helix_diameter_ratio", data.min_helix_diameter_ratio},
      {"plunge_feed_factor", data.plunge_feed_factor},
      {"preferred_direction", cutting_direction_name(data.preferred_direction)},
      {"coolant", coolant_need_name(data.coolant)},
  };
}

CuttingData cutting_data_from(const json& node) {
  CuttingData data;
  read(node, "material", data.material_id);
  read(node, "surface_speed", data.surface_speed);
  read(node, "feed_per_tooth", data.feed_per_tooth);
  read(node, "min_chip_thickness", data.min_chip_thickness);
  read(node, "max_ae_over_d", data.max_ae_over_d);
  read(node, "max_ap_over_d", data.max_ap_over_d);
  read(node, "max_ramp_angle", data.max_ramp_angle);
  read(node, "helical_entry", data.helical_entry);
  read(node, "min_helix_diameter_ratio", data.min_helix_diameter_ratio);
  read(node, "plunge_feed_factor", data.plunge_feed_factor);
  read_enum(node, "preferred_direction", data.preferred_direction,
            cutting_direction_from_name);
  read_enum(node, "coolant", data.coolant, coolant_need_from_name);
  return data;
}

json to_json(const Tool& tool) {
  json cutting = json::array();
  for (const auto& entry : tool.cutting_data) cutting.push_back(to_json(entry));
  return json{
      {"id", tool.id},
      {"name", tool.name},
      {"vendor", tool.vendor},
      {"part_number", tool.part_number},
      {"notes", tool.notes},
      {"type", tool_type_name(tool.type)},
      {"diameter", tool.diameter},
      {"corner_radius", tool.corner_radius},
      {"flutes", tool.flutes},
      {"flute_length", tool.flute_length},
      {"overall_length", tool.overall_length},
      {"shank_diameter", tool.shank_diameter},
      {"neck_diameter", tool.neck_diameter},
      {"neck_length", tool.neck_length},
      {"helix_angle", tool.helix_angle},
      {"included_angle", tool.included_angle},
      {"tip_diameter", tool.tip_diameter},
      {"center_cutting", tool.center_cutting},
      {"variable_helix", tool.variable_helix},
      {"substrate", substrate_name(tool.substrate)},
      {"coating", coating_name(tool.coating)},
      {"geometry_class", geometry_class_name(tool.geometry_class)},
      {"stickout", tool.stickout},
      {"holder", holder_type_name(tool.holder)},
      {"runout", tool.runout},
      {"tool_number", tool.tool_number},
      {"length_offset", tool.length_offset},
      {"cost", tool.cost},
      {"expected_life_minutes", tool.expected_life_minutes},
      {"cutting_data", cutting},
  };
}

Tool tool_from(const json& node) {
  Tool tool;
  read(node, "id", tool.id);
  read(node, "name", tool.name);
  read(node, "vendor", tool.vendor);
  read(node, "part_number", tool.part_number);
  read(node, "notes", tool.notes);
  read_enum(node, "type", tool.type, tool_type_from_name);
  read(node, "diameter", tool.diameter);
  read(node, "corner_radius", tool.corner_radius);
  read(node, "flutes", tool.flutes);
  read(node, "flute_length", tool.flute_length);
  read(node, "overall_length", tool.overall_length);
  read(node, "shank_diameter", tool.shank_diameter);
  read(node, "neck_diameter", tool.neck_diameter);
  read(node, "neck_length", tool.neck_length);
  read(node, "helix_angle", tool.helix_angle);
  read(node, "included_angle", tool.included_angle);
  read(node, "tip_diameter", tool.tip_diameter);
  read(node, "center_cutting", tool.center_cutting);
  read(node, "variable_helix", tool.variable_helix);
  read_enum(node, "substrate", tool.substrate, substrate_from_name);
  read_enum(node, "coating", tool.coating, coating_from_name);
  read_enum(node, "geometry_class", tool.geometry_class, geometry_class_from_name);
  read(node, "stickout", tool.stickout);
  read_enum(node, "holder", tool.holder, holder_type_from_name);
  read(node, "runout", tool.runout);
  read(node, "tool_number", tool.tool_number);
  read(node, "length_offset", tool.length_offset);
  read(node, "cost", tool.cost);
  read(node, "expected_life_minutes", tool.expected_life_minutes);
  if (node.contains("cutting_data") && node.at("cutting_data").is_array())
    for (const auto& entry : node.at("cutting_data"))
      tool.cutting_data.push_back(cutting_data_from(entry));
  return tool;
}

json to_json(const AxisLimits& axis) {
  return json{{"min", axis.minimum},
              {"max", axis.maximum},
              {"max_rapid_feed", axis.max_rapid_feed},
              {"max_acceleration", axis.max_acceleration},
              {"backlash", axis.backlash}};
}

AxisLimits axis_from(const json& node) {
  AxisLimits axis;
  read(node, "min", axis.minimum);
  read(node, "max", axis.maximum);
  read(node, "max_rapid_feed", axis.max_rapid_feed);
  read(node, "max_acceleration", axis.max_acceleration);
  read(node, "backlash", axis.backlash);
  return axis;
}

json to_json(const Machine& machine) {
  json power = json::array();
  for (const auto& point : machine.power_curve)
    power.push_back(json{{"rpm", point.rpm}, {"kw", point.kilowatts}});
  json pulleys = json::array();
  for (const auto& step : machine.pulley_steps)
    pulleys.push_back(json{{"name", step.name},
                           {"min_rpm", step.min_rpm},
                           {"max_rpm", step.max_rpm},
                           {"max_torque_nm", step.max_torque_nm}});
  return json{
      {"id", machine.id},
      {"name", machine.name},
      {"notes", machine.notes},
      {"specifications_unverified", machine.specifications_unverified},
      {"controller", controller_name(machine.controller)},
      {"x", to_json(machine.x)},
      {"y", to_json(machine.y)},
      {"z", to_json(machine.z)},
      {"max_cutting_feed", machine.max_cutting_feed},
      {"block_rate", machine.block_rate},
      {"lookahead_blocks", machine.lookahead_blocks},
      {"junction_deviation", machine.junction_deviation},
      {"min_segment_length", machine.min_segment_length},
      {"repeatability", machine.repeatability},
      {"min_rpm", machine.min_rpm},
      {"max_rpm", machine.max_rpm},
      {"power_curve", power},
      {"base_speed_rpm", machine.base_speed_rpm},
      {"drivetrain_efficiency", machine.drivetrain_efficiency},
      {"spindle_type", spindle_type_name(machine.spindle_type)},
      {"pulley_steps", pulleys},
      {"spindle_spinup_seconds", machine.spindle_spinup_seconds},
      {"rpm_resolution", machine.rpm_resolution},
      {"supports_reverse", machine.supports_reverse},
      {"max_cutting_force", machine.max_cutting_force},
      {"max_mrr", machine.max_mrr},
      {"rigidity", rigidity_class_name(machine.rigidity)},
      {"tool_change", tool_change_name(machine.tool_change)},
      {"tool_pockets", machine.tool_pockets},
      {"tool_change_x", machine.tool_change_x},
      {"tool_change_y", machine.tool_change_y},
      {"tool_change_z", machine.tool_change_z},
      {"length_offset_mode", length_offset_mode_name(machine.length_offset_mode)},
      {"coolant_flood", machine.coolant_flood},
      {"coolant_mist", machine.coolant_mist},
      {"coolant_air", machine.coolant_air},
      {"probe_support", machine.probe_support},
      {"output_inches", machine.output_inches},
      {"arc_support", arc_support_name(machine.arc_support)},
      {"decimals", machine.decimals},
      {"work_offsets", machine.work_offsets},
      {"line_numbers", machine.line_numbers},
      {"line_number_increment", machine.line_number_increment},
      {"program_header", machine.program_header},
      {"program_footer", machine.program_footer},
      {"path_blending", machine.path_blending},
  };
}

Machine machine_from(const json& node) {
  Machine machine;
  read(node, "id", machine.id);
  read(node, "name", machine.name);
  read(node, "notes", machine.notes);
  read(node, "specifications_unverified", machine.specifications_unverified);
  read_enum(node, "controller", machine.controller, controller_from_name);
  if (node.contains("x")) machine.x = axis_from(node.at("x"));
  if (node.contains("y")) machine.y = axis_from(node.at("y"));
  if (node.contains("z")) machine.z = axis_from(node.at("z"));
  read(node, "max_cutting_feed", machine.max_cutting_feed);
  read(node, "block_rate", machine.block_rate);
  read(node, "lookahead_blocks", machine.lookahead_blocks);
  read(node, "junction_deviation", machine.junction_deviation);
  read(node, "min_segment_length", machine.min_segment_length);
  read(node, "repeatability", machine.repeatability);
  read(node, "min_rpm", machine.min_rpm);
  read(node, "max_rpm", machine.max_rpm);
  if (node.contains("power_curve"))
    for (const auto& point : node.at("power_curve")) {
      PowerPoint entry;
      read(point, "rpm", entry.rpm);
      read(point, "kw", entry.kilowatts);
      machine.power_curve.push_back(entry);
    }
  read(node, "base_speed_rpm", machine.base_speed_rpm);
  read(node, "drivetrain_efficiency", machine.drivetrain_efficiency);
  read_enum(node, "spindle_type", machine.spindle_type, spindle_type_from_name);
  if (node.contains("pulley_steps"))
    for (const auto& step : node.at("pulley_steps")) {
      PulleyStep entry;
      read(step, "name", entry.name);
      read(step, "min_rpm", entry.min_rpm);
      read(step, "max_rpm", entry.max_rpm);
      read(step, "max_torque_nm", entry.max_torque_nm);
      machine.pulley_steps.push_back(entry);
    }
  read(node, "spindle_spinup_seconds", machine.spindle_spinup_seconds);
  read(node, "rpm_resolution", machine.rpm_resolution);
  read(node, "supports_reverse", machine.supports_reverse);
  read(node, "max_cutting_force", machine.max_cutting_force);
  read(node, "max_mrr", machine.max_mrr);
  read_enum(node, "rigidity", machine.rigidity, rigidity_class_from_name);
  read_enum(node, "tool_change", machine.tool_change, tool_change_from_name);
  read(node, "tool_pockets", machine.tool_pockets);
  read(node, "tool_change_x", machine.tool_change_x);
  read(node, "tool_change_y", machine.tool_change_y);
  read(node, "tool_change_z", machine.tool_change_z);
  read_enum(node, "length_offset_mode", machine.length_offset_mode,
            length_offset_mode_from_name);
  read(node, "coolant_flood", machine.coolant_flood);
  read(node, "coolant_mist", machine.coolant_mist);
  read(node, "coolant_air", machine.coolant_air);
  read(node, "probe_support", machine.probe_support);
  read(node, "output_inches", machine.output_inches);
  read_enum(node, "arc_support", machine.arc_support, arc_support_from_name);
  read(node, "decimals", machine.decimals);
  read(node, "work_offsets", machine.work_offsets);
  read(node, "line_numbers", machine.line_numbers);
  read(node, "line_number_increment", machine.line_number_increment);
  read(node, "program_header", machine.program_header);
  read(node, "program_footer", machine.program_footer);
  read(node, "path_blending", machine.path_blending);
  return machine;
}

json to_json(const Material& material) {
  return json{
      {"id", material.id},
      {"name", material.name},
      {"notes", material.notes},
      {"group", material_group_name(material.group)},
      {"hardness", material.hardness},
      {"specific_cutting_energy", material.specific_cutting_energy},
      {"kienzle_kc11", material.kienzle_kc11},
      {"kienzle_mc", material.kienzle_mc},
      {"vc_multiplier", material.vc_multiplier},
      {"fz_multiplier", material.fz_multiplier},
      {"chip_class", chip_class_name(material.chip_class)},
      {"work_hardening", material.work_hardening},
  };
}

Material material_from(const json& node) {
  Material material;
  read(node, "id", material.id);
  read(node, "name", material.name);
  read(node, "notes", material.notes);
  read_enum(node, "group", material.group, material_group_from_name);
  read(node, "hardness", material.hardness);
  read(node, "specific_cutting_energy", material.specific_cutting_energy);
  read(node, "kienzle_kc11", material.kienzle_kc11);
  read(node, "kienzle_mc", material.kienzle_mc);
  read(node, "vc_multiplier", material.vc_multiplier);
  read(node, "fz_multiplier", material.fz_multiplier);
  read_enum(node, "chip_class", material.chip_class, chip_class_from_name);
  read(node, "work_hardening", material.work_hardening);
  return material;
}

json to_json(const FeedPolicy& policy) {
  return json{
      {"chip_thinning", policy.chip_thinning},
      {"load_ceiling", policy.load_ceiling},
      {"max_thinning_factor", policy.max_thinning_factor},
      {"corner_compensation", policy.corner_compensation},
      {"min_corner_radius", policy.min_corner_radius},
      {"path_tolerance", policy.path_tolerance},
      {"lookahead_distance", policy.lookahead_distance},
      {"feed_slew_limit", policy.feed_slew_limit},
      {"min_feed", policy.min_feed},
      {"max_feed", policy.max_feed},
      {"rapid_disengaged", policy.rapid_disengaged},
      {"power_margin", policy.power_margin},
      {"enable_power", policy.enable_power},
      {"enable_force", policy.enable_force},
      {"enable_deflection", policy.enable_deflection},
      {"enable_acceleration", policy.enable_acceleration},
      {"max_deflection", policy.max_deflection},
      {"lateral_force_ratio", policy.lateral_force_ratio},
  };
}

FeedPolicy feed_policy_from(const json& node) {
  FeedPolicy policy;
  read(node, "chip_thinning", policy.chip_thinning);
  read(node, "load_ceiling", policy.load_ceiling);
  read(node, "max_thinning_factor", policy.max_thinning_factor);
  read(node, "corner_compensation", policy.corner_compensation);
  read(node, "min_corner_radius", policy.min_corner_radius);
  read(node, "path_tolerance", policy.path_tolerance);
  read(node, "lookahead_distance", policy.lookahead_distance);
  read(node, "feed_slew_limit", policy.feed_slew_limit);
  read(node, "min_feed", policy.min_feed);
  read(node, "max_feed", policy.max_feed);
  read(node, "rapid_disengaged", policy.rapid_disengaged);
  read(node, "power_margin", policy.power_margin);
  read(node, "enable_power", policy.enable_power);
  read(node, "enable_force", policy.enable_force);
  read(node, "enable_deflection", policy.enable_deflection);
  read(node, "enable_acceleration", policy.enable_acceleration);
  read(node, "max_deflection", policy.max_deflection);
  read(node, "lateral_force_ratio", policy.lateral_force_ratio);
  return policy;
}

json to_json(const Operation& operation) {
  return json{
      {"name", operation.name},
      {"type", operation_type_name(operation.type)},
      {"tool", operation.tool_id},
      {"machine", operation.machine_id},
      {"material", operation.material_id},
      {"stepover_ratio", operation.stepover_ratio},
      {"stepdown", operation.stepdown},
      {"radial_stock_to_leave", operation.radial_stock_to_leave},
      {"axial_stock_to_leave", operation.axial_stock_to_leave},
      {"top_z", operation.top_z},
      {"bottom_z", operation.bottom_z},
      {"direction", cutting_direction_name(operation.direction)},
      {"lead_in", operation.lead_in},
      {"slice_grid", operation.slice_grid},
      {"simulation_stock_inflation", operation.simulation_stock_inflation},
      {"feeds", to_json(operation.feeds)},
  };
}

Operation operation_from(const json& node) {
  Operation operation;
  read(node, "name", operation.name);
  read_enum(node, "type", operation.type, operation_type_from_name);
  read(node, "tool", operation.tool_id);
  read(node, "machine", operation.machine_id);
  read(node, "material", operation.material_id);
  read(node, "stepover_ratio", operation.stepover_ratio);
  read(node, "stepdown", operation.stepdown);
  read(node, "radial_stock_to_leave", operation.radial_stock_to_leave);
  read(node, "axial_stock_to_leave", operation.axial_stock_to_leave);
  read(node, "top_z", operation.top_z);
  read(node, "bottom_z", operation.bottom_z);
  read_enum(node, "direction", operation.direction, cutting_direction_from_name);
  read(node, "lead_in", operation.lead_in);
  read(node, "slice_grid", operation.slice_grid);
  read(node, "simulation_stock_inflation", operation.simulation_stock_inflation);
  if (node.contains("feeds")) operation.feeds = feed_policy_from(node.at("feeds"));
  return operation;
}

json to_json(const Job& job) {
  json operations = json::array();
  for (const auto& operation : job.operations) operations.push_back(to_json(operation));
  return json{
      {"name", job.name},
      {"part_mesh", job.part_mesh.string()},
      {"stock_mesh", job.stock_mesh.string()},
      {"origin_x", job.origin_x},
      {"origin_y", job.origin_y},
      {"origin_z", job.origin_z},
      {"clearance_z", job.clearance_z},
      {"retract_z", job.retract_z},
      {"work_offset", job.work_offset},
      {"operations", operations},
  };
}

Job job_from(const json& node) {
  Job job;
  read(node, "name", job.name);
  std::string path;
  read(node, "part_mesh", path);
  job.part_mesh = path;
  path.clear();
  read(node, "stock_mesh", path);
  job.stock_mesh = path;
  read(node, "origin_x", job.origin_x);
  read(node, "origin_y", job.origin_y);
  read(node, "origin_z", job.origin_z);
  read(node, "clearance_z", job.clearance_z);
  read(node, "retract_z", job.retract_z);
  read(node, "work_offset", job.work_offset);
  if (node.contains("operations"))
    for (const auto& entry : node.at("operations"))
      job.operations.push_back(operation_from(entry));
  return job;
}

json parse(const std::string& text, const std::filesystem::path& where) {
  try {
    return json::parse(text, nullptr, true, /*ignore_comments=*/true);
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
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), code);
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) throw std::runtime_error("Cannot write " + path.string());
  stream << text;
  if (!stream) throw std::runtime_error("Failed while writing " + path.string());
}

ToolLibrary load_tools(const std::filesystem::path& path) {
  ToolLibrary library;
  std::error_code code;
  if (!std::filesystem::exists(path, code)) return library;
  const json node = parse(read_file(path), path);
  const json& list = node.is_array() ? node : node.value("tools", json::array());
  for (const auto& entry : list) library.tools.push_back(tool_from(entry));
  return library;
}

void save_tools(const std::filesystem::path& path, const ToolLibrary& library) {
  json list = json::array();
  for (const auto& tool : library.tools) list.push_back(to_json(tool));
  write_file(path, json{{"tools", list}}.dump(2) + "\n");
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

void save_machine(const std::filesystem::path& path, const Machine& machine) {
  write_file(path, to_json(machine).dump(2) + "\n");
}

void save_material(const std::filesystem::path& path, const Material& material) {
  write_file(path, to_json(material).dump(2) + "\n");
}

Job load_job(const std::filesystem::path& path) {
  return job_from(parse(read_file(path), path));
}

void save_job(const std::filesystem::path& path, const Job& job) {
  write_file(path, to_json(job).dump(2) + "\n");
}

Preferences load_preferences(const std::filesystem::path& path) {
  Preferences value;
  std::error_code code;
  if (!std::filesystem::exists(path, code)) return value;
  const json node = parse(read_file(path), path);
  read(node, "display_inches", value.display_inches);
  read(node, "display_decimals", value.display_decimals);
  read(node, "default_tool", value.default_tool_id);
  read(node, "default_machine", value.default_machine_id);
  read(node, "default_material", value.default_material_id);
  read(node, "slice_grid", value.slice_grid);
  read(node, "prefer_gpu", value.prefer_gpu);
  read(node, "last_job_path", value.last_job_path);
  read(node, "recent_jobs", value.recent_jobs);
  return value;
}

void save_preferences(const std::filesystem::path& path, const Preferences& value) {
  write_file(path, json{
                       {"display_inches", value.display_inches},
                       {"display_decimals", value.display_decimals},
                       {"default_tool", value.default_tool_id},
                       {"default_machine", value.default_machine_id},
                       {"default_material", value.default_material_id},
                       {"slice_grid", value.slice_grid},
                       {"prefer_gpu", value.prefer_gpu},
                       {"last_job_path", value.last_job_path},
                       {"recent_jobs", value.recent_jobs},
                   }
                       .dump(2) +
                       "\n");
}

std::string to_json_text(const Tool& tool) { return to_json(tool).dump(2); }
Tool tool_from_json_text(const std::string& text) {
  return tool_from(parse(text, "<memory>"));
}
std::string to_json_text(const Machine& machine) { return to_json(machine).dump(2); }
Machine machine_from_json_text(const std::string& text) {
  return machine_from(parse(text, "<memory>"));
}
std::string to_json_text(const Material& material) { return to_json(material).dump(2); }
Material material_from_json_text(const std::string& text) {
  return material_from(parse(text, "<memory>"));
}
std::string to_json_text(const Job& job) { return to_json(job).dump(2); }
Job job_from_json_text(const std::string& text) {
  return job_from(parse(text, "<memory>"));
}

}  // namespace cam
