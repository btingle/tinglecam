#include "../src/gui/gui_internal.h"
#include "imgui_internal.h"

#include <future>
#include <iostream>

using namespace tinglecam_gui;

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void finish(App& app) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!take_model_load(app)) {
    check(std::chrono::steady_clock::now() < deadline, "Model load timed out");
    std::this_thread::yield();
  }
}
void frame(App& app) {
  ImGui::NewFrame();
  draw_model_loading(app);
  ImGui::Render();
}
void run(const std::filesystem::path& directory) {
  // A portable fixture also covers non-STEP model loading through the worker.
  cam::write_file(directory / "part.obj",
      "v 0 0 0\nv 8 0 0\nv 0 6 0\nv 0 0 4\n"
      "f 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n");
  std::filesystem::copy_file(directory / "part.obj", directory / "stock.obj");
  cam::Job saved;
  saved.part_mesh = "part.obj";
  saved.stock_mesh = "stock.obj";
  saved.box_stock = false;
  saved.machine_id = "machine";
  saved.material_id = "material";
  saved.origin_x = 12.5;
  saved.operations.emplace_back();
  saved.operations.front().depth_from = .75;
  saved.operations.front().depth_to = 2.25;
  const auto job_file = directory / "job.json";
  cam::save_job(job_file, saved);

  App app;
  app.settings_file = directory / "settings.json";
  start_job_loading(app, job_file);
  check(app.model_loading && !app.geometry.has_part(),
        "Starting a load must return before committing geometry");
  finish(app);
  check(app.model_load_error.empty() && app.geometry.has_part() && app.geometry.has_stock(),
        "The worker must load both models and resolve paths against the job");
  check(app.model_loader.progress().fraction == 1 && app.job.origin_x == saved.origin_x &&
            app.job.operations.front().depth_from == .75 &&
            app.job.operations.front().depth_to == 2.25 && !app.geometry.box_stock(),
        "Successful job loading must finish progress and retain saved setup");
  check(!job_dirty(app) && app.settings.recent_jobs.size() == 1,
        "A loaded job must be marked saved and remembered");
  const auto original = cam::to_json_text(app.job);
  const auto triangles = app.geometry.part().mesh().triangles.size();

  start_model_loading(app, directory / "missing.obj", true);
  finish(app);
  check(!app.model_load_error.empty() && cam::to_json_text(app.job) == original &&
            app.geometry.part().mesh().triangles.size() == triangles,
        "A failed replacement must preserve the current job and geometry");

  // Hold a worker at an intermediate progress value to drive the actual
  // popup and Cancel button without timing a model import or needing a GPU.
  std::promise<void> ready;
  auto ready_future = ready.get_future();
  app.model_loading = true;
  app.model_load_label = "part.step";
  app.model_loader.start([&](const cam::ProgressCallback& progress) -> LoadedModels {
    progress(.42, "Preparing STEP faces");
    ready.set_value();
    while (progress(.42, "Preparing STEP faces")) std::this_thread::yield();
    throw std::runtime_error("Cancelled");
  });
  ready_future.wait();
  frame(app);
  frame(app);
  const auto* popup = ImGui::FindWindowByName("Loading models###model-loading");
  check(popup && popup->Active && app.model_loader.progress().fraction == .42,
        "The loading dialog must render during background work");
  const ImVec2 cancel(popup->Pos.x + ImGui::GetStyle().WindowPadding.x + 20,
                      popup->DC.CursorMaxPos.y - ImGui::GetFrameHeight() * .5f);
  auto& io = ImGui::GetIO();
  io.AddMousePosEvent(cancel.x, cancel.y);
  frame(app);
  io.AddMouseButtonEvent(0, true);
  frame(app);
  io.AddMouseButtonEvent(0, false);
  frame(app);
  check(app.model_load_cancelled, "The Cancel button must request cancellation");
  finish(app);
  frame(app);
  check(app.model_load_error == "Cancelled" && cam::to_json_text(app.job) == original &&
            app.geometry.part().mesh().triangles.size() == triangles,
        "Cancellation must preserve the current job and geometry");

  // Interactive model replacement still sets the work depth and origin.
  start_model_loading(app, directory / "part.obj", true);
  finish(app);
  check(app.model_load_error.empty() && app.job.operations.front().depth_from == 0 &&
            app.job.operations.front().depth_to == 4,
        "Replacing a model must reset depths against the new work bounds");
}
}  // namespace

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("tinglecam-model-loading-tests-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  ImGui::CreateContext();
  auto& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.DisplaySize = {1280, 1000};
  io.DeltaTime = 1.f / 60;
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  int result = EXIT_SUCCESS;
  try {
    run(directory);
    std::cout << "Model loading GUI checks passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = EXIT_FAILURE;
  }
  ImGui::DestroyContext();
  std::filesystem::remove_all(directory);
  return result;
}
