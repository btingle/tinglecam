#include "gui_internal.h"
namespace tinglecam_gui {
namespace {

// Publish a library change only once it has been written successfully. Plans
// using a changed tool go out of date through their signatures; nothing else
// is thrown away.
void commit_tools(App& app, cam::ToolLibrary library) {
  cam::save_tools(app.tools_file, library);
  app.tools = std::move(library);
  app.result_preview = {};
}

bool tool_in_use(const App& app, const std::string& id) {
  return std::any_of(app.job.operations.begin(), app.job.operations.end(),
                     [&](const cam::Operation& operation) {
                       if(operation.tool_id==id) return true;
                       if(operation.mode==cam::OperationMode::Hole)
                         for(const auto& h:operation.holes.systems) for(const auto& l:h.levels)
                           if(l.tool_id==id) return true;
                       return false;
                     });
}

void tool_editor(App& app) {
  if (!app.builder_open) return;
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 640.0f), ui_size(app, 520.0f)),
                           ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Tool", &app.builder_open)) {
    ImGui::End();
    return;
  }
  cam::Tool& tool = app.builder_tool;
  if (begin_properties("tool")) {
    prop_text("Name", tool.name);
    static constexpr cam::ToolType kTypes[]{cam::ToolType::FlatEndMill,
                                            cam::ToolType::BallEndMill,
                                            cam::ToolType::Drill, cam::ToolType::VBit};
    prop_enum("Type", tool.type, cam::tool_type_name, kTypes, std::size(kTypes));
    prop_double("Diameter", tool.diameter, 0.01f, "%.3f mm", nullptr, 0.001);
    prop_double("Flute length", tool.flute_length, 0.1f, "%.2f mm", nullptr, 0.001);
    prop_int("Flutes", tool.flutes, 0.05f, 1, 16);
    if(tool.type==cam::ToolType::Drill) {
      prop_double("Point angle",tool.point_angle_degrees,.5f,"%.1f deg",
                  "Included angle of the conical drill point; 180 is flat.",1,180);
      prop_double("Tip flat diameter",tool.tip_flat_diameter,.01f,"%.3f mm",nullptr,0,tool.diameter);
      prop_value("Point length","%.3f mm",cam::drill_tip_height(tool));
    }
    end_properties();
  }

  ImGui::SeparatorText("Cutting data");
  ImGui::TextDisabled("One row per material. Feed is spindle x flutes x chip load.");
  int remove = -1;
  if (ImGui::BeginTable("cutting", 6,
                        ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_BordersInnerH)) {
    ImGui::TableSetupColumn("Material", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("Spindle rpm", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("Chip load mm", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("Plunge mm/min", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("Feed mm/min", ImGuiTableColumnFlags_WidthStretch, 1.2f);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (std::size_t i = 0; i < tool.cutting.size(); ++i) {
      cam::CuttingData& data = tool.cutting[i];
      ImGui::PushID(static_cast<int>(i));
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      const cam::Material* material = app.materials.find(data.material_id);
      ImGui::SetNextItemWidth(-FLT_MIN);
      if (ImGui::BeginCombo("##material", material ? material->name.c_str()
                                                   : (data.material_id.empty()
                                                          ? "Choose..."
                                                          : data.material_id.c_str()))) {
        for (const auto& option : app.materials.materials)
          if (ImGui::Selectable(option.name.c_str(), option.id == data.material_id))
            data.material_id = option.id;
        ImGui::EndCombo();
      }
      ImGui::TableSetColumnIndex(1);
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::DragScalar("##rpm", ImGuiDataType_Double, &data.spindle_rpm, 10.0f,
                        nullptr, nullptr, "%.0f");
      ImGui::TableSetColumnIndex(2);
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::DragScalar("##chip", ImGuiDataType_Double, &data.chip_load, 0.0005f,
                        nullptr, nullptr, "%.4f");
      ImGui::TableSetColumnIndex(3);
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::DragScalar("##plunge", ImGuiDataType_Double, &data.plunge_feed, 5.0f,
                        nullptr, nullptr, "%.0f");
      ImGui::TableSetColumnIndex(4);
      ImGui::AlignTextToFramePadding();
      ImGui::TextDisabled("%.0f", data.spindle_rpm * tool.flutes * data.chip_load);
      ImGui::TableSetColumnIndex(5);
      if (ImGui::SmallButton("Remove")) remove = static_cast<int>(i);
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (remove >= 0) tool.cutting.erase(tool.cutting.begin() + remove);
  if (ImGui::Button("Add material")) {
    cam::CuttingData data;
    if (!tool.cutting_for(app.job.material_id)) data.material_id = app.job.material_id;
    tool.cutting.push_back(std::move(data));
  }

  ImGui::Separator();
  cam::Tool candidate = tool;
  if (app.builder_is_new || candidate.id.empty())
    candidate.id = app.tools.unique_id_for(candidate.name);
  ImGui::TextDisabled("ID: %s", candidate.id.c_str());
  const auto problems = cam::validate(candidate);
  for (const auto& problem : problems)
    ImGui::TextColored(severity_colour(app, LogSeverity::Error), "%s", problem.c_str());
  ImGui::BeginDisabled(!problems.empty());
  if (ImGui::Button("Save tool")) {
    try {
      auto library = app.tools;
      library.upsert(candidate);
      commit_tools(app, std::move(library));
      say(app, "Saved tool " + candidate.name);
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

}  // namespace

void reload_tools(App& app) {
  if (!std::filesystem::exists(app.tools_file))
    throw std::runtime_error("Cannot open " + app.tools_file.string());
  app.tools = cam::load_tools(app.tools_file);
  say(app, "Reloaded " + app.tools_file.string());
}

void import_tools(App& app, const std::filesystem::path& file) {
  if (!std::filesystem::exists(file)) throw std::runtime_error("Cannot open " + file.string());
  const auto imported = cam::load_tools(file);
  auto library = app.tools;
  for (const auto& tool : imported.tools) library.upsert(tool);
  commit_tools(app, std::move(library));
  say(app, "Imported " + std::to_string(imported.tools.size()) + " tools from " +
               file.string());
}

void material_library_window(App& app) {
  if (!app.show_material_library) return;
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 520.0f), ui_size(app, 400.0f)),
                           ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Material library", &app.show_material_library)) {
    ImGui::End();
    return;
  }
  const auto save = [&](cam::MaterialLibrary library) {
    try {
      cam::save_material_library(app.materials_file, library);
      app.materials = std::move(library);
      return true;
    } catch (const std::exception& error) {
      say(app, error.what(), true);
      return false;
    }
  };
  if (ImGui::Button("New material")) {
    auto library = app.materials;
    cam::Material material{library.unique_id_for("New material"), "New material"};
    library.materials.push_back(material);
    save(std::move(library));
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%s", app.materials_file.string().c_str());

  std::string remove;
  if (ImGui::BeginTable("materials", 4,
                        ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersOuter)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch, 3.0f);
    ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("Tools with data", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (std::size_t i = 0; i < app.materials.materials.size(); ++i) {
      const cam::Material& material = app.materials.materials[i];
      ImGui::PushID(material.id.c_str());
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      // Edited in a copy and written when the field lets go, so the file is
      // not rewritten on every keystroke.
      static std::string editing_id;
      static std::array<char, 256> name{};
      if (editing_id != material.id)
        std::snprintf(name.data(), name.size(), "%s", material.name.c_str());
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::InputText("##name", name.data(), name.size());
      if (ImGui::IsItemActivated()) editing_id = material.id;
      if (ImGui::IsItemDeactivatedAfterEdit() && name[0] != '\0') {
        auto library = app.materials;
        library.materials[i].name = name.data();
        save(std::move(library));
      }
      if (ImGui::IsItemDeactivated()) editing_id.clear();
      ImGui::TableSetColumnIndex(1);
      ImGui::AlignTextToFramePadding();
      ImGui::TextDisabled("%s", material.id.c_str());
      ImGui::TableSetColumnIndex(2);
      const auto with_data = std::count_if(
          app.tools.tools.begin(), app.tools.tools.end(),
          [&](const cam::Tool& tool) { return tool.cutting_for(material.id) != nullptr; });
      ImGui::Text("%zu of %zu", static_cast<std::size_t>(with_data), app.tools.tools.size());
      ImGui::TableSetColumnIndex(3);
      if (ImGui::SmallButton("Delete")) remove = material.id;
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (!remove.empty()) {
    const cam::Material* material = app.materials.find(remove);
    const std::string name = material ? material->name : remove;
    std::string text = "Delete \"" + name + "\" from the library?";
    if (app.job.material_id == remove) text += " The open job uses it.";
    text += " Tools keep their cutting data for it.";
    confirm(app, "Delete material", text, "Delete", [&app, remove, save] {
      auto library = app.materials;
      library.erase(remove);
      if (save(std::move(library))) say(app, "Deleted material " + remove);
    });
  }
  ImGui::End();
}

void tool_library_window(App& app) {
  tool_editor(app);
  if (!app.show_tool_library) return;
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 720.0f), ui_size(app, 460.0f)),
                           ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Tool library", &app.show_tool_library)) {
    ImGui::End();
    return;
  }
  if (ImGui::Button("New tool")) {
    app.builder_tool = cam::Tool{};
    app.builder_tool.name = "New tool";
    app.builder_is_new = true;
    app.builder_open = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Import...")) {
    open_file_dialog(app, "Import tools", {".json"}, false,
                     app.tools_file.parent_path(), "",
                     [&app](const std::filesystem::path& file) {
      try { import_tools(app, file); }
      catch (const std::exception& error) { say(app, error.what(), true); }
    });
  }
  ImGui::SetItemTooltip("Add tools from another library file. Tools with the "
                        "same ID are replaced.");
  ImGui::SameLine();
  if (ImGui::Button("Reload")) {
    try { reload_tools(app); }
    catch (const std::exception& error) { say(app, error.what(), true); }
  }
  ImGui::SetItemTooltip("%s", app.tools_file.string().c_str());

  std::string remove;
  if (ImGui::BeginTable("tools", 6,
                        ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersOuter,
                        ImVec2(0, 0))) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch, 3.0f);
    ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn("Diameter", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn("Flutes", ImGuiTableColumnFlags_WidthStretch, 0.8f);
    ImGui::TableSetupColumn("Job material", ImGuiTableColumnFlags_WidthStretch, 1.2f);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (const auto& tool : app.tools.tools) {
      ImGui::PushID(tool.id.c_str());
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(tool.name.c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(cam::tool_type_name(tool.type));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f mm", tool.diameter);
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%d", tool.flutes);
      ImGui::TableSetColumnIndex(4);
      if (tool.cutting_for(app.job.material_id)) ImGui::TextColored(success_colour(app), "has data");
      else ImGui::TextColored(severity_colour(app, LogSeverity::Warning), "no data");
      ImGui::TableSetColumnIndex(5);
      if (ImGui::SmallButton("Edit")) {
        app.builder_tool = tool;
        app.builder_is_new = false;
        app.builder_open = true;
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Copy")) {
        app.builder_tool = tool;
        app.builder_tool.name += " copy";
        app.builder_tool.id.clear();
        app.builder_is_new = true;
        app.builder_open = true;
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Delete")) remove = tool.id;
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (!remove.empty()) {
    const cam::Tool* tool = app.tools.find(remove);
    const std::string name = tool ? tool->name : remove;
    confirm(app, "Delete tool",
            "Delete \"" + name + "\" from the library?" +
                (tool_in_use(app, remove) ? " Operations in this job use it." : ""),
            "Delete", [&app, remove] {
              try {
                auto library = app.tools;
                library.erase(remove);
                commit_tools(app, std::move(library));
                say(app, "Deleted tool " + remove);
              } catch (const std::exception& error) {
                say(app, error.what(), true);
              }
            });
  }
  ImGui::End();
}

}  // namespace tinglecam_gui
