#include "geometry.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using wave::Path64;
using wave::Paths64;

struct ViewState {
  ImVec2 pan{0.0f, 0.0f};
  float zoom = 1.0f;
  bool needs_fit = true;
};

struct AppState {
  wave::SvgGeometry svg;
  wave::WaveResult result;
  float allowance = 150.0f;
  float engagement = 80.0f;
  int selected_iteration = 0;
  bool show_all_iterations = true;
  bool show_subject = true;
  bool show_stock = true;
  bool animate = false;
  float animation_speed = 4.0f;
  ViewState view;
  std::string error;
};

ImU32 iteration_color(int index, int count, int alpha = 225) {
  const float t = count <= 1 ? 0.0f : static_cast<float>(index) / (count - 1);
  const ImVec4 cyan{0.10f, 0.78f, 0.94f, alpha / 255.0f};
  const ImVec4 orange{1.00f, 0.45f, 0.12f, alpha / 255.0f};
  return ImGui::ColorConvertFloat4ToU32(ImVec4(
      cyan.x + (orange.x - cyan.x) * t,
      cyan.y + (orange.y - cyan.y) * t,
      cyan.z + (orange.z - cyan.z) * t,
      alpha / 255.0f));
}

void recompute(AppState& app) {
  try {
    app.result = wave::build_wavefront(app.svg.paths, app.allowance, app.engagement);
    app.selected_iteration = std::clamp(
        app.selected_iteration, 0,
        std::max(0, static_cast<int>(app.result.generations.size()) - 1));
    app.error.clear();
  } catch (const std::exception& error) {
    app.error = error.what();
  }
}

struct CanvasTransform {
  ImVec2 origin;
  float pixels_per_unit = 1.0f;

  ImVec2 point(const Clipper2Lib::Point64& value) const {
    return {origin.x + static_cast<float>(wave::to_world(value.x)) * pixels_per_unit,
            origin.y + static_cast<float>(wave::to_world(value.y)) * pixels_per_unit};
  }
};

void draw_paths(ImDrawList* draw_list, const Paths64& paths,
                const CanvasTransform& transform, ImU32 color, float width) {
  for (const auto& path : paths) {
    if (path.size() < 2) continue;
    for (std::size_t i = 0; i < path.size(); ++i) {
      draw_list->AddLine(transform.point(path[i]),
                         transform.point(path[(i + 1) % path.size()]), color, width);
    }
  }
}

void draw_path(ImDrawList* draw_list, const Path64& path,
               const CanvasTransform& transform, ImU32 color, float width) {
  draw_paths(draw_list, Paths64{path}, transform, color, width);
}

void fit_view(AppState& app, ImVec2 canvas_size) {
  const auto bounds = Clipper2Lib::GetBounds(Paths64{app.result.stock_boundary});
  const float width = static_cast<float>(wave::to_world(bounds.right - bounds.left));
  const float height = static_cast<float>(wave::to_world(bounds.bottom - bounds.top));
  if (width <= 0.0f || height <= 0.0f) return;
  app.view.zoom = 0.88f * std::min(canvas_size.x / width, canvas_size.y / height);
  const float center_x = static_cast<float>(wave::to_world(bounds.left + bounds.right)) * 0.5f;
  const float center_y = static_cast<float>(wave::to_world(bounds.top + bounds.bottom)) * 0.5f;
  app.view.pan = {-center_x * app.view.zoom, -center_y * app.view.zoom};
  app.view.needs_fit = false;
}

void draw_canvas(AppState& app) {
  ImGui::BeginChild("Canvas", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 canvas_min = ImGui::GetCursorScreenPos();
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();
  canvas_size.x = std::max(canvas_size.x, 50.0f);
  canvas_size.y = std::max(canvas_size.y, 50.0f);
  ImGui::InvisibleButton("geometry-canvas", canvas_size,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 canvas_max{canvas_min.x + canvas_size.x, canvas_min.y + canvas_size.y};
  draw_list->AddRectFilled(canvas_min, canvas_max, IM_COL32(17, 22, 29, 255));
  draw_list->PushClipRect(canvas_min, canvas_max, true);

  if (app.view.needs_fit) fit_view(app, canvas_size);
  const ImVec2 center{canvas_min.x + canvas_size.x * 0.5f,
                      canvas_min.y + canvas_size.y * 0.5f};

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.view.pan.x += ImGui::GetIO().MouseDelta.x;
    app.view.pan.y += ImGui::GetIO().MouseDelta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const float factor = std::pow(1.15f, ImGui::GetIO().MouseWheel);
    const ImVec2 before{(mouse.x - center.x - app.view.pan.x) / app.view.zoom,
                        (mouse.y - center.y - app.view.pan.y) / app.view.zoom};
    app.view.zoom = std::clamp(app.view.zoom * factor, 0.001f, 1000.0f);
    app.view.pan = {mouse.x - center.x - before.x * app.view.zoom,
                    mouse.y - center.y - before.y * app.view.zoom};
  }

  const CanvasTransform transform{{center.x + app.view.pan.x,
                                   center.y + app.view.pan.y},
                                  app.view.zoom};

  // A subtle world-space grid gives scale and makes pan/zoom obvious.
  const float desired_grid_px = 75.0f;
  const float raw_grid = desired_grid_px / app.view.zoom;
  const float grid_power = std::pow(10.0f, std::floor(std::log10(raw_grid)));
  const float grid_unit = (raw_grid / grid_power < 2.0f ? 2.0f
                           : raw_grid / grid_power < 5.0f ? 5.0f : 10.0f) * grid_power;
  const float grid_px = grid_unit * app.view.zoom;
  const float offset_x = std::fmod(center.x + app.view.pan.x - canvas_min.x, grid_px);
  const float offset_y = std::fmod(center.y + app.view.pan.y - canvas_min.y, grid_px);
  for (float x = canvas_min.x + offset_x; x < canvas_max.x; x += grid_px)
    draw_list->AddLine({x, canvas_min.y}, {x, canvas_max.y}, IM_COL32(48, 57, 68, 100));
  for (float y = canvas_min.y + offset_y; y < canvas_max.y; y += grid_px)
    draw_list->AddLine({canvas_min.x, y}, {canvas_max.x, y}, IM_COL32(48, 57, 68, 100));

  if (app.show_stock)
    draw_path(draw_list, app.result.stock_boundary, transform, IM_COL32(125, 138, 152, 190), 2.0f);

  const int count = static_cast<int>(app.result.generations.size());
  if (app.show_all_iterations) {
    for (int i = 0; i < count; ++i) {
      const bool selected = i == app.selected_iteration;
      draw_paths(draw_list, app.result.generations[i], transform,
                 iteration_color(i, count, selected ? 255 : 145),
                 selected ? 3.0f : 1.25f);
    }
  } else if (count > 0) {
    draw_paths(draw_list, app.result.generations[app.selected_iteration], transform,
               iteration_color(app.selected_iteration, count), 2.5f);
  }

  if (app.show_subject)
    draw_paths(draw_list, app.result.subject, transform, IM_COL32(252, 226, 89, 255), 3.0f);

  draw_list->PopClipRect();
  ImGui::EndChild();
}

void controls(AppState& app, const std::filesystem::path& input) {
  ImGui::TextUnformatted("Clipper2 contour wave");
  ImGui::TextDisabled("%s", input.filename().string().c_str());
  ImGui::Separator();

  bool changed = false;
  changed |= ImGui::DragFloat("Allowance", &app.allowance, 1.0f, 0.001f, 100000.0f, "%.3f");
  changed |= ImGui::DragFloat("Engagement", &app.engagement, 1.0f, 0.001f, 100000.0f, "%.3f");
  if (changed) recompute(app);

  if (ImGui::Button("Recompute")) recompute(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) app.view.needs_fit = true;

  if (!app.error.empty()) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 100, 100, 255));
    ImGui::TextWrapped("%s", app.error.c_str());
    ImGui::PopStyleColor();
    return;
  }

  const int count = static_cast<int>(app.result.generations.size());
  if (count > 0) {
    ImGui::SliderInt("Generation", &app.selected_iteration, 0, count - 1);
  }
  ImGui::Checkbox("Show all generations", &app.show_all_iterations);
  ImGui::Checkbox("Show subject", &app.show_subject);
  ImGui::Checkbox("Show allowance box", &app.show_stock);
  ImGui::Checkbox("Animate", &app.animate);
  if (app.animate) ImGui::SliderFloat("Speed", &app.animation_speed, 0.25f, 20.0f, "%.2f gen/s");

  ImGui::Separator();
  ImGui::Text("Input paths: %zu", app.svg.source_path_count);
  ImGui::Text("Union paths: %zu", app.result.subject.size());
  ImGui::Text("Generations: %d", count);
  ImGui::Text("Stopped on: %s", wave::stop_reason_name(app.result.stop_reason));
  if (count > 0)
    ImGui::Text("Selected contours: %zu",
                app.result.generations[app.selected_iteration].size());

  ImGui::Spacing();
  ImGui::TextColored(ImVec4(0.99f, 0.89f, 0.35f, 1.0f), "Subject");
  ImGui::SameLine();
  ImGui::TextDisabled("  cyan -> orange: early -> late");
  ImGui::TextDisabled("Right-drag: pan   Wheel: zoom");
  ImGui::TextWrapped(
      "Waves are seeded only at the allowance box. The reached area expands by "
      "Engagement, is clipped to the box, and then has the subject removed. Each "
      "colored generation is only the newly reached area; subject boundaries never "
      "act as wave sources.");
}

std::filesystem::path resolve_input(int argc, char** argv) {
  if (argc > 1) return argv[1];
  const std::filesystem::path local = "wavetest.svg";
  if (std::filesystem::exists(local)) return local;
  const auto beside_executable = std::filesystem::path(argv[0]).parent_path() / "wavetest.svg";
  return beside_executable;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto input = resolve_input(argc, argv);
    AppState app;
    app.svg = wave::load_svg_closed_paths(input);
    const double span = std::max(
        wave::to_world(app.svg.bounds.right - app.svg.bounds.left),
        wave::to_world(app.svg.bounds.bottom - app.svg.bounds.top));
    app.allowance = static_cast<float>(span * 0.05);
    app.engagement = static_cast<float>(span * 0.025);
    recompute(app);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
      throw std::runtime_error(std::string("SDL initialization failed: ") + SDL_GetError());

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
    SDL_Window* window = SDL_CreateWindow(
        "Clipper2 contour wave", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 800, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) throw std::runtime_error(std::string("Window creation failed: ") + SDL_GetError());
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) throw std::runtime_error(std::string("OpenGL context failed: ") + SDL_GetError());
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;  // The demo uses a fixed workspace; don't litter the input directory.
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 150");

    bool running = true;
    double animation_accumulator = 0.0;
    auto previous_time = std::chrono::steady_clock::now();
    while (running) {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT ||
            (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
             event.window.windowID == SDL_GetWindowID(window))) {
          running = false;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - previous_time).count();
      previous_time = now;
      if (app.animate && !app.result.generations.empty()) {
        animation_accumulator += elapsed * app.animation_speed;
        while (animation_accumulator >= 1.0) {
          app.selected_iteration =
              (app.selected_iteration + 1) % static_cast<int>(app.result.generations.size());
          animation_accumulator -= 1.0;
        }
      }

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplSDL2_NewFrame();
      ImGui::NewFrame();

      const ImGuiViewport* viewport = ImGui::GetMainViewport();
      ImGui::SetNextWindowPos(viewport->WorkPos);
      ImGui::SetNextWindowSize(viewport->WorkSize);
      ImGui::Begin("Clipper2 wave workspace", nullptr,
                   ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                       ImGuiWindowFlags_NoSavedSettings);
      ImGui::BeginChild("Controls", ImVec2(330.0f, 0.0f), ImGuiChildFlags_Borders);
      controls(app, input);
      ImGui::EndChild();
      ImGui::SameLine();
      draw_canvas(app);
      ImGui::End();

      ImGui::Render();
      int display_width = 0;
      int display_height = 0;
      SDL_GL_GetDrawableSize(window, &display_width, &display_height);
      glViewport(0, 0, display_width, display_height);
      glClearColor(0.035f, 0.047f, 0.063f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "clipper2_wave_demo: " << error.what() << '\n';
    return 1;
  }
}
