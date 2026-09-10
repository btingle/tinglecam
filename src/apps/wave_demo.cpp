#include "wave_demo_state.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <stdexcept>

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

using namespace wave_demo;

int main(int argc, char** argv) {
  try {
    const auto inputs = resolve_inputs(argc, argv);
    if (inputs.help) {
      print_help(argv[0]);
      return 0;
    }
    const auto& input = inputs.svg;
    AppState app;
    app.svg = wave::load_svg_closed_paths(input);
    app.subject_paths = app.svg.paths;
    app.auto_stock_subject_bounds = app.svg.bounds;
    const double span = std::max(
        wave::to_world(app.svg.bounds.right - app.svg.bounds.left),
        wave::to_world(app.svg.bounds.bottom - app.svg.bounds.top));
    app.allowance = static_cast<float>(span * 0.05);
    app.stepover = static_cast<float>(span * 0.025);
    app.lead_in_distance = app.stepover * 0.5f;
    // Every analysis below would be thrown away: the mesh loads redefine the
    // subject, the command line then redefines the parameters, and the tail of
    // this function analyses the result. So load the meshes, resolve the
    // options, and build the wavefront exactly once, at the bottom.
    if (!inputs.subject_mesh.empty()) {
      std::snprintf(app.subject_mesh_path.data(), app.subject_mesh_path.size(), "%s",
                    inputs.subject_mesh.string().c_str());
      load_mesh(app, false, /*analyze=*/false);
    }
    if (!inputs.stock_mesh.empty()) {
      std::snprintf(app.stock_mesh_path.data(), app.stock_mesh_path.size(), "%s",
                    inputs.stock_mesh.string().c_str());
      load_mesh(app, true, /*analyze=*/false);
    }
    if (inputs.query_grid) {
      if (*inputs.query_grid < 1 || *inputs.query_grid > 256)
        throw std::runtime_error("--query-grid must be between 1 and 256");
      app.query_grid = *inputs.query_grid;
    }
    if (inputs.analysis_layers) {
      if (*inputs.analysis_layers < 1 || *inputs.analysis_layers > 128)
        throw std::runtime_error("--analysis-layers must be between 1 and 128");
      app.max_analysis_layers = *inputs.analysis_layers;
    }
    if (inputs.tiny_split_area_factor) {
      if (*inputs.tiny_split_area_factor < 0.0f)
        throw std::runtime_error(
            "--tiny-split-area-factor must be nonnegative");
      app.tiny_split_area_factor = *inputs.tiny_split_area_factor;
    }
    if (inputs.lead_in) app.lead_in = *inputs.lead_in;
    if (inputs.slice_z) app.slice_z = *inputs.slice_z;
    if (inputs.subject_inflation) app.subject_inflation = *inputs.subject_inflation;
    if (inputs.stock_inflation) app.stock_inflation = *inputs.stock_inflation;
    if (inputs.allowance) {
      if (!(*inputs.allowance > 0.0f))
        throw std::runtime_error("--allowance must be positive");
      app.allowance = *inputs.allowance;
    }
    if (inputs.stepover) {
      if (!(*inputs.stepover > 0.0f))
        throw std::runtime_error("--stepover must be positive");
      app.stepover = *inputs.stepover;
      if (!inputs.lead_in_distance)
        app.lead_in_distance = app.stepover * 0.5f;
    }
    if (inputs.lead_in_distance) {
      if (!(*inputs.lead_in_distance >= 0.0f))
        throw std::runtime_error("--lead-in-distance must not be negative");
      app.lead_in_distance = *inputs.lead_in_distance;
    }
    if (inputs.rounded_waves) app.rounded_waves = *inputs.rounded_waves;
    if (!app.subject_slicer.empty() || !app.stock_slicer.empty())
      reslice(app, /*analyze=*/false);
    else
      update_preview_outlines(app);

    // Interactive startup only prepares the input preview. Headless requests
    // necessarily need a completed analysis before they can dump or render it.
    const bool build_immediately =
        inputs.dump_analysis || !inputs.screenshot.empty() ||
        inputs.analysis_step.has_value();
    if (build_immediately) recompute(app);
    if (inputs.analysis_step) {
      app.decision_step = *inputs.analysis_step;
      app.decision_caption = apply_decision_frame(app);
      app.view.needs_fit = true;
    }

    if (inputs.dump_analysis) {
      if (!app.error.empty())
        throw std::runtime_error(app.error);
      std::cout << "initial nodes " << app.standard_topology.nodes.size()
                << ", recursive layers " << app.analysis.layers.size() << '\n';
      const wave::Rect64 stock_bounds =
          Clipper2Lib::GetBounds(app.standard_result.stock);
      std::cout << "stock bounds " << wave::to_world(stock_bounds.left) << ','
                << wave::to_world(stock_bounds.top) << " -> "
                << wave::to_world(stock_bounds.right) << ','
                << wave::to_world(stock_bounds.bottom) << '\n';
      if (!app.analysis_error.empty())
        std::cout << "analysis error: " << app.analysis_error << '\n';
      for (const wave::WavefrontDecisionLayer& layer : app.analysis.layers) {
        std::cout << "layer " << layer.layer << ": "
                  << layer.entries.size() << " entries in "
                  << layer.groups.size() << " groups, baseline "
                  << layer.baseline_topology.nodes.size() << " nodes, "
                  << layer.remaining_domain.size() << " domain rings, "
                  << layer.cached_reached.size() << " cached-state rings, "
                  << layer.baseline_wave.generations.size()
                  << " generations" << '\n';
        for (const wave::WavefrontStrategyTrial& trial : layer.trials) {
          const std::size_t candidate_entries = std::count_if(
              trial.topology.nodes.begin(), trial.topology.nodes.end(),
              [](const wave::FrontNode& node) {
                return node.kind == wave::FrontNode::Kind::Propagation &&
                       node.evaluation_order == 0;
              });
          std::cout << "  longest entry per connected group: "
                    << trial.candidate_node_count << " vs "
                    << trial.previous_node_count << " raw nodes, "
                    << trial.candidate_significant_node_count << " vs "
                    << trial.previous_significant_node_count
                    << " significant nodes (>= "
                    << trial.minimum_significant_node_area << " mm^2), "
                    << trial.retained_entries.size() << " retained / "
                    << candidate_entries << " resulting entries, "
                    << (trial.accepted ? "accepted" : "rejected") << '\n';
          auto print_areas = [](const char* label,
                                const std::vector<double>& areas) {
            std::vector<double> sorted = areas;
            std::sort(sorted.begin(), sorted.end(), std::greater<double>());
            std::cout << "    " << label << " areas:";
            for (double area : sorted) std::cout << ' ' << area;
            std::cout << " mm^2\n";
          };
          print_areas("baseline", trial.previous_node_areas);
          print_areas("candidate", trial.candidate_node_areas);
        }
        std::cout << "  selected " << layer.selected_topology.nodes.size()
                  << " nodes" << '\n';
        std::vector<std::size_t> order_counts;
        for (const wave::FrontNode& node : layer.selected_topology.nodes) {
          if (node.evaluation_order >= order_counts.size())
            order_counts.resize(node.evaluation_order + 1, 0);
          ++order_counts[node.evaluation_order];
        }
        std::cout << "  node layers";
        for (std::size_t order = 0; order < order_counts.size(); ++order)
          std::cout << ' ' << order << ':' << order_counts[order];
        std::size_t mixed_order_bands = 0;
        for (std::size_t generation = 0;
             generation < layer.selected_lineage.generations.size();
             ++generation) {
          const wave::FrontGeneration& fronts =
              layer.selected_lineage.generations[generation];
          std::vector<std::size_t> band_orders(fronts.bands.size(),
                                                wave::kNoTopologyId);
          for (std::size_t front = 0; front < fronts.fronts.size(); ++front) {
            const std::size_t node =
                layer.selected_topology.front_nodes[generation][front];
            const std::size_t order =
                layer.selected_topology.nodes[node].evaluation_order;
            std::size_t& band_order = band_orders[fronts.fronts[front].band];
            if (band_order == wave::kNoTopologyId)
              band_order = order;
            else if (band_order != order)
              band_order = std::numeric_limits<std::size_t>::max() - 1;
          }
          mixed_order_bands += static_cast<std::size_t>(std::count(
              band_orders.begin(), band_orders.end(),
              std::numeric_limits<std::size_t>::max() - 1));
        }
        std::cout << ", " << mixed_order_bands << " mixed-order bands\n";
      }
      const std::vector<wave::WavefrontStage> committed =
          wave::committed_wavefront_stages(app.analysis);
      std::size_t committed_fronts = 0;
      for (const wave::WavefrontStage& stage : committed) {
        std::size_t fronts = 0;
        for (const wave::FrontGeneration& generation : stage.lineage.generations)
          fronts += generation.fronts.size();
        committed_fronts += fronts;
        std::cout << "committed layer " << stage.analysis_layer << ": "
                  << stage.topology.nodes.size() << " nodes, " << fronts
                  << " fronts, " << stage.lineage.generations.size()
                  << " generations" << '\n';
      }
      std::cout << "committed total: " << committed.size() << " layers, "
                << committed_fronts << " fronts" << '\n';
      return 0;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
      throw std::runtime_error(std::string("SDL initialization failed: ") + SDL_GetError());

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
    SDL_Window* window = SDL_CreateWindow(
        "Wavefront analysis viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1500, 900, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
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
    bool screenshot_pending = !inputs.screenshot.empty();
    double animation_accumulator = 0.0;
    double decision_accumulator = 0.0;
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
      if (app.decision_play) {
        decision_accumulator += elapsed * app.decision_speed;
        while (decision_accumulator >= 1.0 && app.decision_play) {
          if (app.decision_step + 1 >= decision_frame_count(app)) {
            app.decision_play = false;
            decision_accumulator = 0.0;
            break;
          }
          ++app.decision_step;
          app.decision_caption = apply_decision_frame(app);
          app.selected_iteration = std::clamp(
              app.selected_iteration, 0,
              std::max(0, static_cast<int>(app.result.generations.size()) - 1));
          decision_accumulator -= 1.0;
        }
      }
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
      ImGui::Begin("Wavefront analysis", nullptr,
                   ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                       ImGuiWindowFlags_NoSavedSettings);
      ImGui::BeginChild("Controls", ImVec2(365.0f, 0.0f), ImGuiChildFlags_Borders);
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
      if (screenshot_pending) {
        save_screenshot(inputs.screenshot, display_width, display_height);
        screenshot_pending = false;
        running = false;
      }
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
