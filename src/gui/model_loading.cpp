#include "gui_internal.h"

namespace tinglecam_gui {
namespace {
LoadedModels read_job(const std::filesystem::path& file) {
  LoadedModels result;
  result.job = cam::load_job(file, &result.warnings);
  result.job_file = file;
  result.opened_job = true;
  for (auto* path : {&result.job.part_mesh, &result.job.stock_mesh})
    if (!path->empty() && path->is_relative())
      *path = std::filesystem::absolute(file).parent_path() / *path;
  return result;
}
void load_geometry(LoadedModels& result, const cam::ProgressCallback& progress = {}) {
  const auto& job = result.job;
  const int count = int(!job.part_mesh.empty()) + int(!job.stock_mesh.empty());
  int completed = 0;
  auto report = [&](double fraction, const std::string& stage) {
    const auto& file = completed == 0 && !job.part_mesh.empty() ? job.part_mesh : job.stock_mesh;
    return !progress || progress((completed + fraction) / std::max(1, count),
                                  file.filename().string() + ": " + stage);
  };
  if (!job.part_mesh.empty()) {
    result.geometry.load_part(job.part_mesh, report);
    ++completed;
  }
  if (!job.stock_mesh.empty()) {
    result.geometry.load_stock(job.stock_mesh, report);
    ++completed;
  }
  result.geometry.set_box_stock(job.box_stock, job.box_stock_margins);
  if (progress && !progress(1.0, "Models ready")) throw std::runtime_error("Cancelled");
}
void apply_loaded_models(App& app, LoadedModels result) {
  app.geometry = std::move(result.geometry);
  app.job = std::move(result.job);
  if (result.opened_job) {
    app.job_file = std::move(result.job_file);
    mark_job_saved(app);
    app.selected_operation = 0;
    app.operation_selection.assign(app.job.operations.size(), false);
    log_warnings(app, "Job", result.warnings);
    say(app, "Opened " + app.job_file.string());
    if (app.job.machine_id.empty() || app.job.material_id.empty()) {
      adopt_default_setup(app);
      log_message(app, LogSeverity::Warning, "Job",
                  "The job named no machine or material; the defaults were chosen. "
                  "Save to keep them.");
    }
  }
  if (result.reset_setup) {
    for (auto& operation : app.job.operations) depth_defaults(app, operation);
    apply_origin_pick(app);
  }
  app.surface_preview_key.clear();
  app.surface_preview = {};
  app.surface_depth_model.reset();
  invalidate_plan(app);
  app.view.needs_fit = true;
}
void begin_loading(App& app, std::string label,
                   std::function<LoadedModels(const cam::ProgressCallback&)> task) {
  app.planner.stop();
  app.surface_previewer.stop();
  app.hole_detector.stop();
  app.hole_detection_id.clear();
  app.playback.playing = false;
  app.model_loading = true;
  app.model_load_cancelled = false;
  app.model_load_label = std::move(label);
  app.model_load_error.clear();
  app.model_loader.start(std::move(task));
}
}

// Synchronous entry points remain available to command-line planning. GUI
// requests build their own result on a worker and only commit on success.
void load_job_into(App& app, const std::filesystem::path& file, bool models) {
  app.planner.stop();
  app.surface_previewer.stop();
  app.hole_detector.stop();
  app.hole_detection_id.clear();
  auto result = read_job(file);
  if (models) load_geometry(result);
  apply_loaded_models(app, std::move(result));
}
void load_meshes(App& app) {
  app.planner.stop();
  app.surface_previewer.stop();
  app.hole_detector.stop();
  app.hole_detection_id.clear();
  try {
    LoadedModels result;
    result.job = app.job;
    if (result.job.stock_mesh.empty()) result.job.box_stock = true;
    result.reset_setup = true;
    load_geometry(result);
    apply_loaded_models(app, std::move(result));
    say(app, "Models loaded");
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
}
void start_model_loading(App& app, bool reset_setup) {
  auto request = app.job;
  if (reset_setup && request.stock_mesh.empty()) request.box_stock = true;
  begin_loading(app, "Loading models", [request = std::move(request), reset_setup](
      const cam::ProgressCallback& progress) {
    LoadedModels result;
    result.job = request;
    result.reset_setup = reset_setup;
    load_geometry(result, progress);
    return result;
  });
}
void start_model_loading(App& app, const std::filesystem::path& file, bool part) {
  auto request = app.job;
  if (part) request.part_mesh = file;
  else { request.stock_mesh = file; request.box_stock = false; }
  if (request.stock_mesh.empty()) request.box_stock = true;
  begin_loading(app, file.filename().string(), [request = std::move(request)](
      const cam::ProgressCallback& progress) {
    LoadedModels result;
    result.job = request;
    result.reset_setup = true;
    load_geometry(result, progress);
    return result;
  });
}
void start_job_loading(App& app, const std::filesystem::path& file) {
  begin_loading(app, file.filename().string(), [file](const cam::ProgressCallback& progress) {
    if (!progress(0.0, "Reading job")) throw std::runtime_error("Cancelled");
    auto result = read_job(file);
    load_geometry(result, progress);
    return result;
  });
}
bool take_model_load(App& app) {
  if (!app.model_loading) return false;
  LoadedModels loaded;
  if (!app.model_loader.take(loaded)) return false;
  app.model_loading = false;
  app.model_load_error = loaded.error;
  if (!loaded.error.empty()) {
    say(app, loaded.error == "Cancelled" ? "Model loading cancelled" : loaded.error,
        loaded.error != "Cancelled");
    return true;
  }
  const bool opened_job = loaded.opened_job;
  apply_loaded_models(app, std::move(loaded));
  if (opened_job) remember_recent_job(app, app.job_file);
  else say(app, "Models loaded");
  return true;
}
void draw_model_loading(App& app) {
  constexpr const char* popup = "Loading models###model-loading";
  if (app.model_loading && !ImGui::IsPopupOpen(popup)) ImGui::OpenPopup(popup);
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, {.5f,.5f});
  if (ImGui::BeginPopupModal(popup, nullptr,
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
    if (!app.model_loading) ImGui::CloseCurrentPopup();
    else {
      ImGui::TextUnformatted(app.model_load_label.c_str());
      const auto stage = app.model_loader.stage();
      ImGui::TextUnformatted(stage.c_str());
      const float progress = float(std::clamp(app.model_loader.progress().fraction,0.0,1.0));
      ImGui::ProgressBar(progress, {ui_size(app,360),0});
      ImGui::BeginDisabled(app.model_load_cancelled);
      if (ImGui::Button(app.model_load_cancelled ? "Cancelling..." : "Cancel")) {
        app.model_load_cancelled = true;
        app.model_loader.cancel();
      }
      ImGui::EndDisabled();
    }
    ImGui::EndPopup();
  }
}
}  // namespace tinglecam_gui
