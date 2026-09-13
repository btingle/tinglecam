#pragma once

#include "cam/job.h"
#include "cam/machine.h"
#include "cam/material.h"
#include "cam/tooling.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace cam {

// Where the user's own tools, preferences and jobs live.
std::filesystem::path config_directory();

struct PanelLayout {
  // Dimensions are stored in unscaled application pixels. The UI scale is
  // applied when they are used by the renderer.
  double left_width = 440.0;
  double right_width = 480.0;
  double left_program_fraction = 0.54;
  double right_settings_fraction = 0.58;
};

PanelLayout default_panel_layout();

enum class UiTheme { Default, Slate, Ember, Paper };
const char* ui_theme_name(UiTheme theme);
std::optional<UiTheme> ui_theme_from_name(const std::string& name);

enum class UiFont { Default, Sans, Mono, Serif };
const char* ui_font_name(UiFont font);
std::optional<UiFont> ui_font_from_name(const std::string& name);

struct CustomUiTheme {
  std::string name;
  UiFont font = UiFont::Default;
  // Serialized ImGuiStyle snapshot. The GUI owns its representation so the
  // CAM core does not depend on ImGui headers.
  std::string style_blob;
};

struct UserSettings {
  double ui_scale = 1.0;
  UiTheme ui_theme = UiTheme::Default;
  std::string custom_theme;
  std::vector<CustomUiTheme> custom_themes;
  PanelLayout panel_layout{};
  std::array<Operation, 4> operation_defaults{};
};

UserSettings default_user_settings();
const Operation& operation_default(const UserSettings& settings,
                                   OperationMode mode);
Operation& operation_default(UserSettings& settings, OperationMode mode);
UserSettings load_user_settings(const std::filesystem::path& path);
void save_user_settings(const std::filesystem::path& path,
                        const UserSettings& settings);

// Where the shipped machine and material profiles live. Searched next to the
// executable first, so an installed copy works, then at the path baked in at
// build time, so a build tree works without installing.
std::filesystem::path data_directory();

std::string read_file(const std::filesystem::path& path);
void write_file(const std::filesystem::path& path, const std::string& text);

// Each of these throws std::runtime_error with a message naming the file and
// what was wrong with it. The callers turn that into a banner rather than
// letting a malformed library take the application down.
ToolLibrary load_tools(const std::filesystem::path& path);
void save_tools(const std::filesystem::path& path, const ToolLibrary& library);

// Reads every *.json in a directory, so a profile is one file and adding one is
// dropping it in.
MachineLibrary load_machines(const std::filesystem::path& directory);
MaterialLibrary load_materials(const std::filesystem::path& directory);
void save_machine(const std::filesystem::path& path, const Machine& machine);
void save_material(const std::filesystem::path& path, const Material& material);

Job load_job(const std::filesystem::path& path);
void save_job(const std::filesystem::path& path, const Job& job);

// Round-trips through the same serialisation the files use, which is what the
// tests check rather than reaching into the JSON.
std::string to_json_text(const Tool& tool);
Tool tool_from_json_text(const std::string& text);
std::string to_json_text(const Machine& machine);
Machine machine_from_json_text(const std::string& text);
std::string to_json_text(const Material& material);
Material material_from_json_text(const std::string& text);
std::string to_json_text(const Job& job);
Job job_from_json_text(const std::string& text);
// One operation on its own, so a caller comparing an operation against the one
// a plan was built from does not have to serialise the whole job to do it.
std::string to_json_text(const Operation& operation);

}  // namespace cam
