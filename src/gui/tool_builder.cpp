#include "gui_internal.h"
namespace tinglecam_gui {
// Tool Builder
// ---------------------------------------------------------------------------

// A coordinate typed where the operator reads it: in the program's own frame,
// which is the frame the machine's DRO and the posted G-code are both in. What
// the operation stores is the model frame, because that is where planning
// happens and the origin has to be free to move without moving the work. The
// two differ by exactly the offset the post adds, so moving the origin changes
// the figure in the box and leaves the pocket where it was cut.
bool origin_field(const char* label, double& stored, double origin) {
  double shown = stored + origin;
  if (!ImGui::DragScalar(label, ImGuiDataType_Double, &shown, 0.1f))
    return false;
  stored = shown - origin;
  return true;
}

void text_field(const char* label, std::string& value, std::size_t capacity) {
  std::vector<char> buffer(std::max(capacity, value.size() + 1), '\0');
  std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
  if (ImGui::InputText(label, buffer.data(), buffer.size())) value = buffer.data();
}

void invalidate_plan(App& app);

// Publish a library change only once it has been written successfully.
void commit_tools(App& app, cam::ToolLibrary library) {
  cam::save_tools(app.tools_file, library);
  app.tools = std::move(library);
  invalidate_plan(app);
}

void reload_tools(App& app) {
  if (!std::filesystem::exists(app.tools_file))
    throw std::runtime_error("Cannot open " + app.tools_file.string());
  auto library = cam::load_tools(app.tools_file);
  app.tools = std::move(library);
  invalidate_plan(app);
  say(app, "Reloaded " + app.tools_file.string());
}

void import_tools(App& app) {
  if (!std::filesystem::exists(app.tools_import_file))
    throw std::runtime_error("Cannot open " + app.tools_import_file);
  const auto imported = cam::load_tools(app.tools_import_file);
  auto library = app.tools;
  for (const auto& tool : imported.tools) library.upsert(tool);
  commit_tools(app, std::move(library));
  say(app, "Imported " + std::to_string(imported.tools.size()) + " tools into " +
               app.tools_file.string());
}

void tool_builder(App& app) {
  if (!app.builder_open) return;
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 620.0f), ui_size(app, 560.0f)),
                           ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Tool builder", &app.builder_open)) {
    ImGui::End();
    return;
  }
  cam::Tool& tool = app.builder_tool;
  text_field("Name", tool.name);
  ImGui::InputDouble("Diameter (mm)", &tool.diameter, 0, 0, "%.15g");
  ImGui::InputDouble("Flute length (mm)", &tool.flute_length, 0, 0, "%.15g");
  ImGui::InputInt("Number of flutes", &tool.flutes);
  static constexpr cam::ToolType types[]{cam::ToolType::FlatEndMill,
      cam::ToolType::BallEndMill, cam::ToolType::Drill, cam::ToolType::VBit};
  enum_combo("Type", tool.type, cam::tool_type_name, types, std::size(types));

  ImGui::Separator();
  cam::Tool candidate = tool;
  if (app.builder_is_new || candidate.id.empty())
    candidate.id = app.tools.unique_id_for(candidate.name);
  ImGui::TextDisabled("ID: %s", candidate.id.c_str());
  const auto problems = cam::validate(candidate);
  for (const auto& problem : problems)
    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f),
                       "%s", problem.c_str());
  ImGui::BeginDisabled(!problems.empty() || app.planner.running());
  if (ImGui::Button("Save tool")) {
    try {
      auto library = app.tools;
      library.upsert(candidate);
      commit_tools(app, std::move(library));
      say(app, "Saved " + tool.name);
      app.builder_open = false;
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) app.builder_open = false;
  ImGui::End();
}
}  // namespace tinglecam_gui
