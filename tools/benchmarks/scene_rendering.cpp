// Bounded setup/result benchmark, including ImGui submission and GPU completion.
#include "gui/gui_internal.h"
#include "rendering/graphics_startup.h"
#include "wave/mesh_slicer_cuda.h"
#include <iostream>

using namespace tinglecam_gui;
int main(int argc, char** argv) {
  if (argc < 2) { std::cerr << "Usage: scene_rendering_benchmark JOB.json [IMAGE.ppm] [--result] [--size WIDTH HEIGHT]\n"; return 1; }
  bool result_view=false; int width=960,height=720; const char* image_path=nullptr;
  try { for(int i=2;i<argc;++i) {
    const std::string arg=argv[i];
    if(arg=="--result") result_view=true;
    else if(arg=="--size" && i+2<argc) {
      width=std::stoi(argv[++i]);height=std::stoi(argv[++i]);
      if(width<=0 || height<=0 || width>8192 || height>8192) return 1;
    } else if(!image_path && !arg.starts_with("--")) image_path=argv[i];
    else {std::cerr << "Unknown argument: " << arg << '\n';return 1;}
  }
  } catch(const std::exception& error) {
    std::cerr << "Invalid benchmark arguments: " << error.what() << '\n';return 1;
  }
  wave::graphics::apply_wsl_defaults(wave::graphics::running_under_wsl(), wave::cuda_slicer::current_device_name);
  if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1;
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_Window* window = SDL_CreateWindow("Scene benchmark", 0, 0, width, height,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  SDL_GLContext context = window ? SDL_GL_CreateContext(window) : nullptr;
  if (!context) { std::cerr << SDL_GetError() << '\n'; SDL_Quit(); return 1; }
  int result = 0;
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 150");
  try {
    App app; app.job = cam::load_job(argv[1]); cam::ensure_setup_ids(app.job);
    app.selected_setup = int(app.job.setups.size()) - 1;
    app.selected_operation = -1;
    auto settings = app.effective_context();
    app.geometry.configure(settings);
    app.setup_models = ModelShow::Both;
    if(result_view) {
      app.tools=cam::load_tools(cam::config_directory()/"tools.json");
      app.materials=cam::load_material_library(cam::config_directory()/"materials.json");
      app.machines=cam::load_machines(cam::data_directory()/"machines");
      const auto start=std::chrono::steady_clock::now(); auto last=start;
      app.plan=cam::plan_job(app.geometry,app.job,app.tools,app.machines,app.materials,
          [&](double fraction,const std::string& label) {
        const auto now=std::chrono::steady_clock::now();
        if(now-last>std::chrono::seconds(2)) {
          std::cerr << int(fraction*100) << "% " << label << std::endl;last=now;
        }
        return now-start<std::chrono::seconds(45);
      });
      if(!app.plan.ok()) throw std::runtime_error(app.plan.error);
      app.has_plan=true;app.selected_plan_operation=int(app.plan.operations.size())-1;
      app.selected_operation=app.selected_plan_operation;app.selected_level=0;
      app.stack_follows_playback=false;
      const auto& stock=*app.plan.operations.back().stock_after;
      std::cout << "setup=" << app.job.setups.at(app.selected_setup).name
                << " operation=" << app.plan.operations.back().operation_name
                << " pitch=" << stock.pitch() << " grid=" << stock.columns() << 'x' << stock.rows() << std::endl;
    }
    std::size_t triangles = app.geometry.part().mesh().triangles.size();
    for (const auto& fixture : app.geometry.fixtures()) triangles += fixture.geometry.mesh().triangles.size();
    std::cout << "renderer=" << glGetString(GL_RENDERER) << " triangles=" << triangles << '\n';
    const auto frame = [&] {
      ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL2_NewFrame();
      auto& io = ImGui::GetIO(); io.DisplaySize = {float(width),float(height)}; io.DeltaTime = 1.f/60;
      ImGui::NewFrame();
      ImGui::SetNextWindowPos({0,0}); ImGui::SetNextWindowSize({float(width),float(height)});
      ImGui::Begin("Scene benchmark", nullptr, ImGuiWindowFlags_NoDecoration);
      if(result_view) draw_result(app);else draw_setup(app); ImGui::End(); ImGui::Render();
      glBindFramebuffer(GL_FRAMEBUFFER, 0); glViewport(0,0,width,height);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); glFinish();
    };
    for (int i=0;i<5;++i) frame();
    const auto* renderer=result_view?heightmap_view.get():setup_view.get();
    if(renderer) std::cout << "window=" << width << 'x' << height
                          << " render_pixels=" << renderer->read_depth().size() << std::endl;
    for (bool orbit : {false,true}) {
      const auto start = std::chrono::steady_clock::now();
      for (int i=0;i<30;++i) { if (orbit) app.orbit.yaw += .01f; frame(); }
      const double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/30;
      std::cout << (orbit?"orbit_ms=":"idle_ms=") << ms << std::endl;
      if (renderer) {
        const auto stats=renderer->stats();
        std::cout << "mesh_upload_bytes=" << stats.mesh_upload_bytes << " mesh_buffers=" << stats.mesh_buffers
                  << " mesh_resident_bytes=" << stats.mesh_resident_bytes << " reused_frame=" << stats.reused_frame << '\n';
      }
    }
    if (image_path) {
      std::vector<unsigned char> rgb(std::size_t(width)*height*3);
      glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,rgb.data());
      std::ofstream out(image_path,std::ios::binary);out << "P6\n" << width << ' ' << height << "\n255\n";
      for (int y=height-1;y>=0;--y) out.write(reinterpret_cast<const char*>(rgb.data()+std::size_t(y)*width*3),width*3);
    }
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; result=1; }
  release_render_resources();
  ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext();
  SDL_GL_DeleteContext(context); SDL_DestroyWindow(window); SDL_Quit(); return result;
}
