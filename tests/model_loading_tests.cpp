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

  // Quarter turns compose exactly across all 24 cube orientations.
  std::vector<cam::ModelOrientation> rotations(1);
  for (std::size_t i = 0; i < rotations.size(); ++i) {
    const auto rotation = rotations[i];
    for (int axis = 0; axis < 3; ++axis) {
      auto turned = rotation;
      for (int count = 0; count < 4; ++count) turned.quarter_turn(axis, 1);
      check(turned == rotation, "Four quarter turns restore the exact orientation");
      turned.quarter_turn(axis, 1);
      if (std::find(rotations.begin(), rotations.end(), turned) == rotations.end())
        rotations.push_back(turned);
      turned.quarter_turn(axis, -1);
      check(turned == rotation, "Opposite turns cancel exactly");
    }
  }
  check(rotations.size() == 24, "Quarter turns produce 24 proper orientations");
  const auto mapped = cam::ModelOrientation{{1, -3, 2}}.apply(wave::Vec3f{2, 3, 5});
  check(mapped.x == 2 && mapped.y == -5 && mapped.z == 3,
        "Positive X rotation follows the right hand rule");
  for (auto axes : {std::array{1, 1, 3}, std::array{1, 2, -3}, std::array{0, 2, 3}}) {
    check(!cam::ModelOrientation{axes}.valid(), "Reject degenerate axes and reflections");
  }
  for (const char* invalid : {R"([1,2,-3])", R"([1,1,3])", R"([1,2])",
                              R"([1.0,2,3])", R"([4294967297,2,3])"}) {
    bool rejected = false;
    try { cam::job_from_json_text(std::string("{\"part_orientation\":") + invalid + "}"); }
    catch (const std::exception&) { rejected = true; }
    check(rejected, "Reject invalid persisted orientations");
  }
  check(cam::job_from_json_text("{}").part_orientation == cam::ModelOrientation{},
        "Older jobs default to the original orientation");

  const auto hole_signature = cam::hole_detection_signature(app.geometry, {});
  start_model_rotation(app, 0, 0, 1);
  finish(app);
  check(cam::hole_detection_signature(app.geometry, {}) != hole_signature,
        "Rotation invalidates automatic hole detections");
  check(app.model_load_error.empty(), "Rotate models in the background");
  const auto& rotated = app.geometry.part().mesh().bounds;
  const auto& stock = app.geometry.stock().mesh().bounds;
  check(rotated.lower.y == -4 && rotated.upper.y == 0 && rotated.upper.z == 6 &&
            stock.lower.y == -4 && stock.upper.z == 6,
        "Part and stock rotate together about the shared input origin");
  check(job_dirty(app), "Rotation marks the job dirty");
  cam::save_job(job_file, app.job);
  load_job_into(app, job_file, true);
  check(app.geometry.part().mesh().bounds.upper.z == 6 &&
            app.job.part_orientation == cam::ModelOrientation{{1, -3, 2}},
        "Job reload restores the rotated geometry");
  start_model_rotation(app, 1, 2, 1);
  finish(app);
  check(app.geometry.part().mesh().bounds.upper.x == 4 &&
            app.geometry.stock().mesh().bounds.lower.y == -4,
        "World axis composition can rotate only the part");
  start_model_rotation(app, 0, 0, 0);
  finish(app);
  check(app.geometry.part().mesh().bounds.upper.z == 4 &&
            app.geometry.stock().mesh().bounds.upper.z == 4,
        "Reset restores the imported orientation");

  const auto before_failed_rotation = cam::to_json_text(app.job);
  std::filesystem::rename(directory / "part.obj", directory / "held.obj");
  start_model_rotation(app, 0, 1, 1);
  finish(app);
  std::filesystem::rename(directory / "held.obj", directory / "part.obj");
  check(!app.model_load_error.empty() && cam::to_json_text(app.job) == before_failed_rotation &&
            app.geometry.part().mesh().bounds.upper.z == 4,
        "A failed rotation preserves the job orientation and geometry");

  app.job.stock_mesh.clear();
  app.job.box_stock = true;
  app.job.box_stock_margins.plus_z = 2;
  app.has_plan = true;
  start_model_rotation(app, 0, 0, 1);
  finish(app);
  check(app.geometry.box_stock() && cam::highest_model_z(app.geometry) == 8 && !app.has_plan,
        "Rotation rebuilds box stock and invalidates toolpaths");
  start_model_rotation(app, 0, 0, 0);
  finish(app);

  app.job.box_stock_margins = {};
  // Interactive model replacement still sets the work depth and origin.
  start_model_loading(app, directory / "part.obj", true);
  finish(app);
  check(app.model_load_error.empty() && app.job.operations.front().depth_from == 0 &&
            app.job.operations.front().depth_to == 4,
        "Replacing a model must reset depths against the new work bounds");
}
void check_model_reuse(const std::filesystem::path& directory,
                       const std::filesystem::path& part_file) {
  App app;
  start_model_loading(app, part_file, true);
  finish(app);
  check(app.model_load_error.empty(), "Load reuse fixture");
  const auto* part_triangles = app.geometry.part().mesh().triangles.data();
  const auto native_part = app.geometry.part_cad();
  const auto& bounds = app.geometry.part().mesh().bounds;
  const double slice_z = (double(bounds.lower.z) + bounds.upper.z) / 2;
  const auto* part_slice = &app.geometry.part_outline(slice_z, 8);
  const auto original_job = cam::to_json_text(app.job);

  // Removing the source proves that adding stock never tries to import the
  // part again; pointer checks also catch copying/rebuilding the loaded mesh.
  const auto held_part = directory / "held-part";
  std::filesystem::rename(part_file, held_part);
  start_model_loading(app, directory / "missing-stock.obj", false);
  finish(app);
  check(!app.model_load_error.empty() && cam::to_json_text(app.job) == original_job &&
            app.geometry.part().mesh().triangles.data() == part_triangles,
        "A failed stock replacement preserves the loaded part and job");
  start_model_loading(app, directory / "stock.obj", false);
  check(app.geometry.part().mesh().triangles.data() == part_triangles,
        "The part stays available while stock loads");
  finish(app);
  check(app.model_load_error.empty() && !app.geometry.box_stock() &&
            app.geometry.part().mesh().triangles.data() == part_triangles &&
            app.geometry.part_cad() == native_part &&
            &app.geometry.part_outline(slice_z, 8) == part_slice,
        "Loading stock preserves the part mesh, native STEP data and slice cache");
  start_model_rotation(app, 2, 2, 1);
  finish(app);
  check(app.model_load_error.empty() &&
            app.geometry.part().mesh().triangles.data() == part_triangles &&
            app.geometry.part_cad() == native_part,
        "Stock-only rotation reuses the loaded part");
  std::filesystem::rename(held_part, part_file);

  const auto* stock_triangles = app.geometry.stock().mesh().triangles.data();
  const auto* stock_slice = &app.geometry.stock_outline(2, 8);
  const auto held_stock = directory / "held-stock";
  std::filesystem::rename(directory / "stock.obj", held_stock);
  start_model_loading(app, part_file, true);
  finish(app);
  check(app.model_load_error.empty() &&
            app.geometry.stock().mesh().triangles.data() == stock_triangles &&
            &app.geometry.stock_outline(2, 8) == stock_slice,
        "Replacing the part reuses the stock mesh and slice cache");
  start_model_rotation(app, 1, 0, 1);
  finish(app);
  check(app.model_load_error.empty() &&
            app.geometry.stock().mesh().triangles.data() == stock_triangles,
        "Part-only rotation reuses the loaded stock");
  std::filesystem::rename(held_stock, directory / "stock.obj");
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
    check_model_reuse(directory, directory / "part.obj");
#ifdef TINGLECAM_HAVE_OCCT
    const auto step = directory / "part.step";
    std::filesystem::copy_file(std::filesystem::path(__FILE__).parent_path() /
        "fixtures/cad/vertical_track_wall.step", step);
    check_model_reuse(directory, step);
#endif
    std::cout << "Model loading GUI checks passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = EXIT_FAILURE;
  }
  ImGui::DestroyContext();
  std::filesystem::remove_all(directory);
  return result;
}
