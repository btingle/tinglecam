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

std::optional<OperationMode> operation_mode_from_name(const std::string& name) {
  if (name == "roughing") return OperationMode::Roughing;
  if (name == "contour_finishing") return OperationMode::ContourFinishing;
  return std::nullopt;
}

std::optional<ContourDirectionPolicy> contour_direction_from_name(
    const std::string& name) {
  if (name == "either") return ContourDirectionPolicy::Either;
  if (name == "climb") return ContourDirectionPolicy::Climb;
  if (name == "conventional") return ContourDirectionPolicy::Conventional;
  return std::nullopt;
}

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
  return json{{"material", data.material_id},
              {"spindle_rpm", data.spindle_rpm},
              {"max_ae_over_d", data.max_ae_over_d},
              {"max_ap_over_d", data.max_ap_over_d},
              {"plunge_feed", data.plunge_feed},
              {"coolant", coolant_need_name(data.coolant)}};
}

CuttingData cutting_data_from(const json& node) {
  CuttingData data;
  read(node, "material", data.material_id);
  read(node, "spindle_rpm", data.spindle_rpm);
  read(node, "max_ae_over_d", data.max_ae_over_d);
  read(node, "max_ap_over_d", data.max_ap_over_d);
  read(node, "plunge_feed", data.plunge_feed);
  read_enum(node, "coolant", data.coolant, coolant_need_from_name);
  return data;
}

json to_json(const Tool& tool) {
  json cutting = json::array();
  for (const auto& entry : tool.cutting_data) cutting.push_back(to_json(entry));
  return json{{"id", tool.id},
              {"name", tool.name},
              {"notes", tool.notes},
              {"diameter", tool.diameter},
              {"flute_length", tool.flute_length},
              {"center_cutting", tool.center_cutting},
              {"tool_number", tool.tool_number},
              {"length_offset", tool.length_offset},
              {"cutting_data", cutting}};
}

Tool tool_from(const json& node) {
  Tool tool;
  read(node, "id", tool.id);
  read(node, "name", tool.name);
  read(node, "notes", tool.notes);
  read(node, "diameter", tool.diameter);
  read(node, "flute_length", tool.flute_length);
  read(node, "center_cutting", tool.center_cutting);
  read(node, "tool_number", tool.tool_number);
  read(node, "length_offset", tool.length_offset);
  if (node.contains("cutting_data") && node.at("cutting_data").is_array())
    for (const auto& entry : node.at("cutting_data"))
      tool.cutting_data.push_back(cutting_data_from(entry));
  return tool;
}

json to_json(const MaterialFeedRange& range) {
  return json{{"material", range.material_id},
              {"full_engagement", range.full_engagement},
              {"no_engagement", range.no_engagement}};
}

MaterialFeedRange feed_range_from(const json& node) {
  MaterialFeedRange range;
  read(node, "material", range.material_id);
  read(node, "full_engagement", range.full_engagement);
  read(node, "no_engagement", range.no_engagement);
  return range;
}

json to_json(const Machine& machine) {
  json ranges = json::array();
  for (const auto& range : machine.feed_ranges) ranges.push_back(to_json(range));
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
              {"feed_ranges", ranges},
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
  if (node.contains("feed_ranges") && node.at("feed_ranges").is_array())
    for (const auto& entry : node.at("feed_ranges"))
      machine.feed_ranges.push_back(feed_range_from(entry));
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
  return json{{"name", operation.name},
              {"mode", operation_mode_name(operation.mode)},
              {"tool", operation.tool_id},
              {"machine", operation.machine_id},
              {"material", operation.material_id},
              {"stepover_ratio", operation.stepover_ratio},
              {"stepdown", operation.stepdown},
              {"finishing_subdivisions", operation.finishing_subdivisions},
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
              {"manual_feed_range", operation.manual_feed_range},
              {"full_engagement_feed", operation.full_engagement_feed},
              {"no_engagement_feed", operation.no_engagement_feed},
              {"slice_grid", operation.slice_grid},
              {"simulation_stock_inflation",
               operation.simulation_stock_inflation}};
}

Operation operation_from(const json& node) {
  Operation operation;
  read(node, "name", operation.name);
  read_enum(node, "mode", operation.mode, operation_mode_from_name);
  read(node, "tool", operation.tool_id);
  read(node, "machine", operation.machine_id);
  read(node, "material", operation.material_id);
  read(node, "stepover_ratio", operation.stepover_ratio);
  read(node, "stepdown", operation.stepdown);
  read(node, "finishing_subdivisions", operation.finishing_subdivisions);
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
  read(node, "manual_feed_range", operation.manual_feed_range);
  read(node, "full_engagement_feed", operation.full_engagement_feed);
  read(node, "no_engagement_feed", operation.no_engagement_feed);
  read(node, "slice_grid", operation.slice_grid);
  read(node, "simulation_stock_inflation",
       operation.simulation_stock_inflation);
  return operation;
}

json to_json(const Job& job) {
  json operations = json::array();
  for (const auto& operation : job.operations)
    operations.push_back(to_json(operation));
  return json{{"name", job.name},
              {"box_stock", job.box_stock},
              {"box_stock_margin", job.box_stock_margin},
              {"part_mesh", job.part_mesh.string()},
              {"stock_mesh", job.stock_mesh.string()},
              {"origin_x", job.origin_x},
              {"origin_y", job.origin_y},
              {"origin_z", job.origin_z},
              {"rapid_clearance_height", job.rapid_clearance_height},
              {"work_offset", job.work_offset},
              {"operations", operations}};
}

Job job_from(const json& node) {
  Job job;
  std::string part;
  std::string stock;
  read(node, "name", job.name);
  read(node, "part_mesh", part);
  read(node, "stock_mesh", stock);
  job.part_mesh = part;
  job.stock_mesh = stock;
  read(node, "origin_x", job.origin_x);
  read(node, "origin_y", job.origin_y);
  read(node, "origin_z", job.origin_z);
  read(node, "box_stock", job.box_stock);
  read(node, "box_stock_margin", job.box_stock_margin);
  read(node, "rapid_clearance_height", job.rapid_clearance_height);
  read(node, "work_offset", job.work_offset);
  if (node.contains("operations") && node.at("operations").is_array())
    for (const auto& entry : node.at("operations"))
      job.operations.push_back(operation_from(entry));
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

void save_machine(const std::filesystem::path& path, const Machine& machine) {
  write_file(path, to_json(machine).dump(2) + "\n");
}
void save_material(const std::filesystem::path& path,
                   const Material& material) {
  write_file(path, to_json(material).dump(2) + "\n");
}
Job load_job(const std::filesystem::path& path) {
  return job_from(parse(read_file(path), path));
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
Job job_from_json_text(const std::string& text) {
  return job_from(parse(text, "<memory>"));
}

}  // namespace cam
