#include "gui_internal.h"

#include <cctype>

namespace tinglecam_gui {
namespace {

std::string lower(std::string text) {
  for (char& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return text;
}

struct Listing {
  std::filesystem::path directory;
  std::vector<std::filesystem::path> folders;
  std::vector<std::filesystem::path> files;
  std::string error;
};

Listing list_directory(const std::filesystem::path& directory,
                       const std::vector<std::string>& extensions) {
  Listing listing;
  listing.directory = directory;
  std::error_code error;
  for (std::filesystem::directory_iterator it(directory, error), end;
       !error && it != end; it.increment(error)) {
    const auto& path = it->path();
    const std::string name = path.filename().string();
    if (name.empty() || name.front() == '.') continue;
    std::error_code kind;
    if (it->is_directory(kind)) {
      listing.folders.push_back(path);
    } else if (extensions.empty() ||
               std::find(extensions.begin(), extensions.end(),
                         lower(path.extension().string())) != extensions.end()) {
      listing.files.push_back(path);
    }
  }
  if (error) listing.error = error.message();
  const auto by_name = [](const auto& a, const auto& b) {
    return lower(a.filename().string()) < lower(b.filename().string());
  };
  std::sort(listing.folders.begin(), listing.folders.end(), by_name);
  std::sort(listing.files.begin(), listing.files.end(), by_name);
  return listing;
}

void set_location(FileDialog& dialog, const std::filesystem::path& directory) {
  dialog.directory = directory;
  std::snprintf(dialog.location.data(), dialog.location.size(), "%s",
                directory.string().c_str());
  dialog.selected.clear();
  dialog.error.clear();
}

std::string extension_list(const std::vector<std::string>& extensions) {
  std::string text;
  for (const auto& extension : extensions) {
    if (!text.empty()) text += ", ";
    std::string upper = extension.substr(1);
    for (char& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    text += upper;
  }
  return text.empty() ? "All files" : text;
}

}  // namespace

void remember_recent_job(App& app, const std::filesystem::path& file) {
  auto& recent = app.settings.recent_jobs;
  const std::string text = std::filesystem::absolute(file).lexically_normal().string();
  recent.erase(std::remove(recent.begin(), recent.end(), text), recent.end());
  recent.insert(recent.begin(), text);
  if (recent.size() > 8) recent.resize(8);
  save_settings(app);
}

namespace {

std::string file_stem(const std::string& name) {
  std::string stem;
  for (const char c : name)
    stem.push_back(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_'
                       ? c : '_');
  return stem.empty() ? "job" : stem;
}

void write_job(App& app, const std::filesystem::path& destination) {
  cam::Job saved = app.job;
  const auto directory = std::filesystem::absolute(destination).parent_path();
  const auto relative_to_job = [&](const std::filesystem::path& model) {
    if (model.empty()) return std::filesystem::path{};
    std::error_code error;
    const auto relative = std::filesystem::relative(
        std::filesystem::absolute(model), directory, error);
    return error ? std::filesystem::absolute(model) : relative;
  };
  saved.part_mesh = relative_to_job(saved.part_mesh);
  saved.stock_mesh = relative_to_job(saved.stock_mesh);
  cam::save_job(destination, saved);
  app.job_file = destination;
  mark_job_saved(app);
  remember_recent_job(app, destination);
  say(app, "Saved " + destination.string());
}

void post_and_save(App& app, bool selected_only) {
  const std::vector<std::size_t> selected = selected_operation_indices(app);
  const auto directory = app.gcode_file.empty() ? job_directory(app)
                                                : app.gcode_file.parent_path();
  const std::string name = app.gcode_file.empty()
      ? file_stem(app.job.name) + ".nc" : app.gcode_file.filename().string();
  open_file_dialog(app, "Export G-code", {".nc", ".ngc", ".gcode", ".tap"}, true,
                   directory, name, [&app, selected_only, selected](const std::filesystem::path& file) {
    app.post = selected_only
        ? cam::post_operations(app.plan, app.job, app.tools, app.machines, selected, app.export_flavor)
        : cam::post_job(app.plan, app.job, app.tools, app.machines, app.export_flavor);
    app.posted_operations.clear();
    if (selected_only && app.post.ok())
      for (const std::size_t index : selected)
        app.posted_operations.push_back(static_cast<int>(index));
    log_warnings(app, "G-code", app.post.warnings);
    app.gcode = app.post.text;
    if (!app.post.ok()) {
      say(app, app.post.error, true);
      return;
    }
    try {
      cam::write_file(file, app.gcode);
      app.gcode_file = file;
      say(app, "Wrote " + std::to_string(app.post.line_count) + " lines to " +
                   file.string());
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  });
}

void load_model(App& app, const std::filesystem::path& file, bool part) {
  start_model_loading(app, file, part);
}

}  // namespace

std::filesystem::path job_directory(const App& app) {
  if (!app.job_file.empty())
    return std::filesystem::absolute(app.job_file).parent_path();
  std::error_code error;
  return std::filesystem::current_path(error);
}

void confirm(App& app, std::string title, std::string text, std::string accept,
             std::function<void()> action) {
  app.confirmation = {true, std::move(title), std::move(text), std::move(accept),
                      std::move(action)};
}

void open_file_dialog(App& app, std::string title,
                      std::vector<std::string> extensions, bool saving,
                      std::filesystem::path directory, std::string name,
                      std::function<void(const std::filesystem::path&)> accept) {
  FileDialog& dialog = app.file_dialog;
  dialog = {};
  dialog.pending = true;
  dialog.saving = saving;
  dialog.title = std::move(title);
  dialog.extensions = std::move(extensions);
  dialog.accept = std::move(accept);
  std::error_code error;
  if (directory.empty() || !std::filesystem::is_directory(directory, error))
    directory = std::filesystem::current_path(error);
  set_location(dialog, std::filesystem::absolute(directory));
  std::snprintf(dialog.name.data(), dialog.name.size(), "%s", name.c_str());
}

void draw_dialogs(App& app) {
  FileDialog& dialog = app.file_dialog;
  const std::string file_popup = dialog.title + "###file-dialog";
  if (dialog.pending && !ImGui::IsPopupOpen(file_popup.c_str()))
    ImGui::OpenPopup(file_popup.c_str());
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 760.0f), ui_size(app, 520.0f)),
                           ImGuiCond_Appearing);
  bool open = dialog.pending;
  if (ImGui::BeginPopupModal(file_popup.c_str(), &open)) {
    static Listing listing;
    if (listing.directory != dialog.directory)
      listing = list_directory(dialog.directory, dialog.extensions);

    bool accept = false;
    if (ImGui::Button("Up") && dialog.directory.has_parent_path() &&
        dialog.directory.parent_path() != dialog.directory)
      set_location(dialog, dialog.directory.parent_path());
    ImGui::SameLine();
    if (ImGui::Button("Home"))
      if (const char* home = std::getenv("HOME")) set_location(dialog, home);
    ImGui::SameLine();
    if (ImGui::Button("Job folder")) set_location(dialog, job_directory(app));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::InputText("##location", dialog.location.data(), dialog.location.size(),
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
      std::error_code error;
      const std::filesystem::path typed(dialog.location.data());
      if (std::filesystem::is_directory(typed, error)) set_location(dialog, typed);
      else dialog.error = "No such folder";
    }

    const bool exporting=dialog.saving && dialog.title=="Export G-code";
    const float footer = ImGui::GetFrameHeightWithSpacing() * (exporting?3.0f:2.0f) +
                         ImGui::GetStyle().ItemSpacing.y;
    if (ImGui::BeginChild("entries", ImVec2(0, -footer), ImGuiChildFlags_Borders)) {
      std::filesystem::path enter;
      for (const auto& folder : listing.folders) {
        const std::string label = folder.filename().string() + "/";
        if (ImGui::Selectable(label.c_str(), false,
                              ImGuiSelectableFlags_AllowDoubleClick) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
          enter = folder;
      }
      for (const auto& file : listing.files) {
        const std::string name = file.filename().string();
        if (ImGui::Selectable(name.c_str(), dialog.selected == name,
                              ImGuiSelectableFlags_AllowDoubleClick)) {
          dialog.selected = name;
          std::snprintf(dialog.name.data(), dialog.name.size(), "%s", name.c_str());
          if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) accept = true;
        }
      }
      if (listing.folders.empty() && listing.files.empty())
        ImGui::TextDisabled("%s", listing.error.empty() ? "Nothing here" : listing.error.c_str());
      if (!enter.empty()) set_location(dialog, enter);
    }
    ImGui::EndChild();

    if(exporting) {
      int flavor=static_cast<int>(app.export_flavor);
      ImGui::SetNextItemWidth(ui_size(app,240));
      if(ImGui::Combo("Export flavor",&flavor,"Machine default\0Roland MDX-40A\0"))
        app.export_flavor=static_cast<cam::ExportFlavor>(flavor);
    }
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(dialog.saving ? "Save as" : "File");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-ImGui::CalcTextSize(extension_list(dialog.extensions).c_str()).x -
                            ImGui::GetStyle().ItemSpacing.x * 2.0f);
    if (ImGui::InputText("##name", dialog.name.data(), dialog.name.size(),
                         ImGuiInputTextFlags_EnterReturnsTrue))
      accept = true;
    ImGui::SameLine();
    ImGui::TextDisabled("%s", extension_list(dialog.extensions).c_str());
    if (!dialog.error.empty())
      ImGui::TextColored(severity_colour(app, LogSeverity::Warning), "%s",
                         dialog.error.c_str());
    else
      ImGui::NewLine();
    ImGui::SameLine(ImGui::GetContentRegionAvail().x -
                    ImGui::CalcTextSize("Cancel Overwrite").x -
                    ImGui::GetStyle().FramePadding.x * 4.0f);
    if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) open = false;
    ImGui::SameLine();
    const bool overwrite = dialog.error.rfind("Replace", 0) == 0;
    if (ImGui::Button(dialog.saving ? (overwrite ? "Overwrite" : "Save") : "Open"))
      accept = true;

    if (accept && dialog.name[0] != '\0') {
      std::filesystem::path chosen = dialog.directory / dialog.name.data();
      if (dialog.saving && !dialog.extensions.empty() &&
          std::find(dialog.extensions.begin(), dialog.extensions.end(),
                    lower(chosen.extension().string())) == dialog.extensions.end())
        chosen += dialog.extensions.front();
      std::error_code error;
      const bool exists = std::filesystem::exists(chosen, error);
      if (!dialog.saving && !exists) {
        dialog.error = "No such file";
      } else if (dialog.saving && exists && !overwrite) {
        dialog.error = "Replace " + chosen.filename().string() + "? Press Overwrite.";
      } else {
        auto action = dialog.accept;
        open = false;
        ImGui::CloseCurrentPopup();
        listing = {};
        dialog = {};
        if (action) action(chosen);
      }
    }
    if (!open) {
      ImGui::CloseCurrentPopup();
      listing = {};
    }
    ImGui::EndPopup();
  }
  if (!open) dialog.pending = false;

  Confirmation& question = app.confirmation;
  const std::string confirm_popup = question.title + "###confirm";
  if (question.pending && !ImGui::IsPopupOpen(confirm_popup.c_str()))
    ImGui::OpenPopup(confirm_popup.c_str());
  if (ImGui::BeginPopupModal(confirm_popup.c_str(), nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::PushTextWrapPos(ui_size(app, 420.0f));
    ImGui::TextUnformatted(question.text.c_str());
    ImGui::PopTextWrapPos();
    ImGui::Spacing();
    bool close = false;
    if (ImGui::Button(question.accept.c_str())) {
      auto action = question.action;
      close = true;
      if (action) action();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
      close = true;
      app.quit_requested = false;
    }
    if (close) {
      // The action may have asked a new question; keep that one.
      if (&question == &app.confirmation && question.title + "###confirm" == confirm_popup)
        question.pending = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

bool job_dirty(const App& app) { return cam::to_json_text(app.job) != app.saved_job; }

void unless_unsaved(App& app, const char* doing, std::function<void()> action) {
  if (!job_dirty(app)) {
    action();
    return;
  }
  confirm(app, "Unsaved changes",
          std::string("The job has changes that have not been saved. ") + doing +
              " anyway?",
          "Discard changes", std::move(action));
}

void new_job(App& app) {
  unless_unsaved(app, "Start a new job", [&app] {
    app.planner.stop();
    app.surface_previewer.stop();
    app.hole_detector.stop();app.hole_detection_id.clear();
    app.job = cam::Job{};
    app.geometry = cam::JobGeometry{};
    app.job_file.clear();
    adopt_default_setup(app);
    app.operation_selection.clear();
    invalidate_plan(app);
    select_operation(app, 0);
    mark_job_saved(app);
    say(app, "New job");
  });
}

void open_recent_job(App& app, const std::filesystem::path& file) {
  unless_unsaved(app, "Open another job", [&app, file] {
    try {
      start_job_loading(app, file);
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  });
}

void open_job(App& app) {
  unless_unsaved(app, "Open another job", [&app] {
    open_file_dialog(app, "Open job", {".json"}, false, job_directory(app), "",
                     [&app](const std::filesystem::path& file) {
      try {
        start_job_loading(app, file);
      } catch (const std::exception& error) {
        say(app, error.what(), true);
      }
    });
  });
}

void save_job(App& app, bool choose_file) {
  if (!choose_file && !app.job_file.empty()) {
    try { write_job(app, app.job_file); }
    catch (const std::exception& error) { say(app, error.what(), true); }
    return;
  }
  const std::string name = app.job_file.empty() ? file_stem(app.job.name) + ".json"
                                                : app.job_file.filename().string();
  open_file_dialog(app, "Save job", {".json"}, true, job_directory(app), name,
                   [&app](const std::filesystem::path& file) {
    try { write_job(app, file); }
    catch (const std::exception& error) { say(app, error.what(), true); }
  });
}

void export_gcode(App& app, bool selected_only) {
  if (!app.has_plan) {
    say(app, "Plan the job before exporting G-code", true);
    return;
  }
  const std::size_t stale = stale_operations(app);
  if (stale == 0) {
    post_and_save(app, selected_only);
    return;
  }
  confirm(app, "Plan out of date",
          std::to_string(stale) + " operation" + (stale == 1 ? " has" : "s have") +
              " changed since planning. The export will contain the last plan, "
              "not the current settings.",
          "Export last plan", [&app, selected_only] { post_and_save(app, selected_only); });
}

void request_quit(App& app) {
  app.quit_requested = true;
  unless_unsaved(app, "Quit", [&app] { app.quit = true; });
}

void choose_part_model(App& app) {
  const auto directory = app.job.part_mesh.empty()
      ? job_directory(app) : std::filesystem::absolute(app.job.part_mesh).parent_path();
  open_file_dialog(app, "Part model", {".obj", ".stl", ".step", ".stp"}, false,
                   directory, "", [&app](const std::filesystem::path& file) {
    load_model(app, file, true);
  });
}

void choose_stock_model(App& app) {
  const auto directory = app.job.stock_mesh.empty()
      ? (app.job.part_mesh.empty() ? job_directory(app)
                                   : std::filesystem::absolute(app.job.part_mesh).parent_path())
      : std::filesystem::absolute(app.job.stock_mesh).parent_path();
  open_file_dialog(app, "Stock model", {".obj", ".stl", ".step", ".stp"}, false,
                   directory, "", [&app](const std::filesystem::path& file) {
    load_model(app, file, false);
  });
}

void handle_shortcuts(App& app) {
  if (app.model_loading || app.file_dialog.pending || app.confirmation.pending) return;
  constexpr ImGuiInputFlags global = ImGuiInputFlags_RouteGlobal;
  if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_N, global)) new_job(app);
  if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_O, global)) open_job(app);
  if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_S, global)) save_job(app, false);
  if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiKey_S, global)) save_job(app, true);
  if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_E, global)) export_gcode(app, false);
  if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Q, global)) request_quit(app);
  if (ImGui::Shortcut(ImGuiKey_F5, global) && !app.planner.running()) plan_out_of_date(app);
  if (ImGui::GetIO().WantTextInput) return;
  if (ImGui::IsKeyPressed(ImGuiKey_Space, false) && app.has_plan)
    app.playback.playing = !app.playback.playing;
  if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
    app.view.needs_fit = true;
    app.orbit.pan = ImVec2(0.0f, 0.0f);
    app.orbit.zoom = 1.0f;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) {
    app.playback.distance = 0.0;
    app.playback.motion_seconds = 0.0;
  }
}

}  // namespace tinglecam_gui
