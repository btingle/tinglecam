#include "gui/gui_internal.h"
#include <iostream>

using namespace tinglecam_gui;
namespace {
void check(bool b,const char* text) {if(!b) throw std::runtime_error(text);}
void frame(App& app, bool result = false, ImVec2 size = {800,600}) {
  auto& io=ImGui::GetIO();io.DisplaySize=size;io.DeltaTime=1.f/60;
  ImGui::NewFrame();ImGui::SetNextWindowPos({0,0});ImGui::SetNextWindowSize(size);
  ImGui::Begin("GPU setup test",nullptr,ImGuiWindowFlags_NoDecoration);
  if (result) draw_result(app); else draw_setup(app);
  ImGui::End();ImGui::Render();
  check(result?bool(heightmap_view):bool(setup_view),"The view creates its GPU renderer");
}
void run(const std::filesystem::path& directory) {
  const auto part=directory/"part.obj";
  cam::write_file(part,"v 0 0 0\nv 8 0 0\nv 0 6 0\nv 0 0 4\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n");
  App app;app.job.part_mesh=part;app.job.box_stock=true;
  cam::SetupContext::ProtectedModel fixture;fixture.mesh=part;fixture.placement.offset[0]=10;
  app.job.fixtures={fixture};
  app.geometry.configure(app.job);app.setup_models=ModelShow::Both;app.selected_operation=-1;
  frame(app);frame(app);
  check(setup_view->stats().mesh_buffers==3,"Part, box stock and fixture use persistent GPU buffers");
  frame(app);
  check(setup_view->stats().reused_frame && setup_view->stats().mesh_upload_bytes==0,
        "An unchanged setup frame is reused with all models visible");
  const auto original=setup_view->read_colour();
  app.orbit.yaw+=.5f;frame(app);
  check(!setup_view->stats().reused_frame && setup_view->stats().mesh_upload_bytes==0 &&
        setup_view->read_colour()!=original,"Orbiting updates the image without resending models");
  app.wireframe=true;frame(app);
  check(setup_view->stats().mesh_upload_bytes==0,"Wireframes retain the model buffers");
  app.wireframe=false;app.setup_models=ModelShow::Part;frame(app);
  check(setup_view->stats().mesh_buffers==2,"Hiding stock releases its GPU buffer");
  app.setup_models=ModelShow::Both;frame(app);frame(app);
  check(setup_view->stats().reused_frame,"Showing stock again restores a cacheable scene");
  app.job.fixtures.front().placement.offset[0]=12;app.geometry.configure(app.job);frame(app);
  check(!setup_view->stats().reused_frame && setup_view->stats().mesh_upload_bytes>0,
        "Replacing placed geometry updates the GPU scene");
  frame(app);check(setup_view->stats().reused_frame,"Replaced geometry has a stable display lifetime");
  frame(app);const int without_areas=ImGui::GetDrawData()->TotalVtxCount;
  cam::Operation rough;rough.roughing_areas={{"Left",2,3,12,14},{"Right",14,3,12,14}};
  app.job.operations={rough};app.selected_operation=0;
  frame(app);frame(app);
  check(ImGui::GetDrawData()->TotalVtxCount>without_areas && setup_view->stats().mesh_upload_bytes==0,
        "Numbered roughing boxes are drawn over the models without resending source meshes");
  app.job.operations.clear();app.selected_operation=-1;
  app.has_plan=true;app.plan.operations.resize(1);
  auto& result=app.plan.operations.front();result.planned=true;result.levels.resize(1);
  result.stock_before=result.stock_after=std::make_shared<const wave::Heightmap>(
      wave::Heightmap::box({{0,0,0},{8,6,4},true},.25));
  app.selected_plan_operation=0;app.selected_level=0;app.stack_follows_playback=false;
  for(float dpi:{1.f,2.f}) {
    ImGui::GetIO().DisplayFramebufferScale={dpi,dpi};frame(app);
    const auto setup_pixels=setup_view->read_colour().size()/4;
    frame(app,true);frame(app,true);
    const auto result_pixels=heightmap_view->read_colour().size()/4;
    check(std::abs(double(setup_pixels)-4*double(result_pixels))<setup_pixels*.01,
          "Result renders at half Setup's width and height at normal and high DPI");
  }
  // The cap applies to physical pixels, for both large logical windows and
  // high-DPI displays. Rounding either target axis up costs at most one row
  // and column beyond the nominal budget.
  for(const auto size:{ImVec2{3840,2160},ImVec2{1920,1080},ImVec2{2160,3840}}) {
    const float dpi=size.x==1920?2.f:1.f;
    ImGui::GetIO().DisplayFramebufferScale={dpi,dpi};
    frame(app,true,size);frame(app,true,size);
    const auto pixels=heightmap_view->read_colour().size()/4;
    check(pixels>=1024*1024 && pixels<1024*1024+4096,
          "Large and high-DPI Result targets stay near one million pixels");
    check(heightmap_view->stats().reused_frame,
          "Large unchanged Result views still reuse the completed image");
  }
  ImGui::GetIO().DisplayFramebufferScale={1,1};
  frame(app);const auto native=setup_view->read_colour().size()/4;
  app.settings.surface_render_scale=.5;frame(app);frame(app);
  const auto half_surface=setup_view->read_colour().size()/4;
  check(std::abs(double(native)-4*double(half_surface))<native*.01 &&
        setup_view->stats().reused_frame && setup_view->stats().mesh_upload_bytes==0,
        "Surface resolution changes the target without resending meshes");
  app.settings.stock_render_scale=1;frame(app,true);
  const auto full_stock=heightmap_view->read_colour().size()/4;
  app.settings.stock_render_scale=.5;frame(app,true);
  const auto half_stock=heightmap_view->read_colour().size()/4;
  check(std::abs(double(full_stock)-4*double(half_stock))<full_stock*.01,
        "Stock resolution is independent of the surface resolution");
  frame(app,true,{3200,1800});
  check(heightmap_view->read_colour().size()/4>1024*1024,
        "Explicit stock resolution overrides the automatic pixel budget");
  app.settings.surface_render_scale=1;app.settings.stock_render_scale=0;
  auto retained=app.geometry.part().shared_mesh();
  app.geometry=cam::JobGeometry{};
  check(retained->triangles.size()==4,"Immutable display handles survive clearing their source slicer");
#ifdef TINGLECAM_HAVE_OCCT
  app.job={};app.job.part_mesh=std::filesystem::path(__FILE__).parent_path()/"fixtures/cad/track_finish_faces.step";
  app.geometry.configure(app.job);app.setup_models=ModelShow::Part;
  cam::Operation op;op.mode=cam::OperationMode::SurfaceFinishing;app.job.operations={op};app.selected_operation=0;
  frame(app);frame(app);
  check(setup_view->stats().mesh_buffers==1,"All native CAD faces share one vertex buffer");
  const auto normal=setup_view->read_colour();
  auto& selection=app.job.operations[0].surface;
  selection.model_fingerprint=app.geometry.part_cad()->fingerprint();
  for(const auto& face:app.geometry.part_cad()->faces()) selection.faces.push_back(face.id);
  frame(app);
  check(setup_view->stats().mesh_upload_bytes==0 && setup_view->read_colour()!=normal,
        "Selecting CAD faces changes colours without splitting or uploading meshes");
  frame(app);check(setup_view->stats().reused_frame,"Selected faces are part of frame cache identity");
  OrbitScene scene;scene.top_left={0,0};scene.size={200,200};
  scene.light={0,0,1};
  const auto& mesh=app.geometry.part_cad()->mesh();const auto b=mesh.bounds;
  const float span=std::max({b.upper.x-b.lower.x,b.upper.y-b.lower.y,b.upper.z-b.lower.z,1.f});
  scene.projection={{100,100},{(b.lower.x+b.upper.x)/2,(b.lower.y+b.upper.y)/2,(b.lower.z+b.upper.z)/2},
                    {1,0,0},{0,1,0},{0,0,1},160/span};
  refresh_surface_depth(app,scene);
  check(bool(surface_depth_view),"Contour visibility uses a GPU depth render in the application");
  int checked=0;
  for(int y=1;y<99;y+=3) for(int x=1;x<99;x+=3) {
    float expected=-std::numeric_limits<float>::infinity();
    for(const auto& t:mesh.triangles) {
      const Vec3 a{t.a.x,t.a.y,t.a.z},b{t.b.x,t.b.y,t.b.z},c{t.c.x,t.c.y,t.c.z};
      auto p=scene.projection.point(a),q=scene.projection.point(b),r=scene.projection.point(c);
      for(auto* v:{&p,&q,&r}) {v->x/=2;v->y/=2;}
      const float det=(q.y-r.y)*(p.x-r.x)+(r.x-q.x)*(p.y-r.y);
      // Positive screen determinant is a front face in this projection.
      if(det>=-1e-6f) continue;
      const float u=((q.y-r.y)*(x+.5f-r.x)+(r.x-q.x)*(y+.5f-r.y))/det;
      const float v=((r.y-p.y)*(x+.5f-r.x)+(p.x-r.x)*(y+.5f-r.y))/det;
      if(u<0 || v<0 || u+v>1) continue;
      expected=std::max(expected,u*scene.projection.depth(a)+v*scene.projection.depth(b)+(1-u-v)*scene.projection.depth(c));
    }
    const auto actual=app.surface_depth[std::size_t(y)*100+x];
    check((std::isfinite(actual)==std::isfinite(expected)) &&
          (!std::isfinite(expected) || std::abs(actual-expected)<span*.0001f),
          "GPU contour visibility depth agrees with an independent projected surface oracle");
    if(std::isfinite(expected)) ++checked;
  }
  check(checked>10,"GPU depth verification samples the native model surface");
  const auto origin_depths=app.surface_depth;
  scene.projection.model_centre.z+=500;
  refresh_surface_depth(app,scene);
  for(std::size_t i=0;i<origin_depths.size();++i) if(std::isfinite(origin_depths[i]))
    check(std::isfinite(app.surface_depth[i]) && std::abs(app.surface_depth[i]-(origin_depths[i]-500))<.001f,
          "Tall fixtures shifting the view centre do not clip contour visibility depth");
#endif
}
}
int main() {
  if(SDL_Init(SDL_INIT_VIDEO)!=0) {std::cerr<<SDL_GetError()<<'\n';return 1;}
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,2);
  auto* window=SDL_CreateWindow("GPU scene tests",0,0,800,600,SDL_WINDOW_OPENGL|SDL_WINDOW_HIDDEN);
  auto context=window?SDL_GL_CreateContext(window):nullptr;
  if(!context) {std::cerr<<SDL_GetError()<<'\n';SDL_Quit();return 1;}
  ImGui::CreateContext();auto& io=ImGui::GetIO();io.IniFilename=nullptr;
  unsigned char* pixels;int w,h;io.Fonts->GetTexDataAsRGBA32(&pixels,&w,&h);
  const auto directory=std::filesystem::temp_directory_path()/cam::new_document_id();
  std::filesystem::create_directories(directory);int result=0;
  try {run(directory);std::cout<<"GPU setup view, immutable meshes and selection checks passed.\n";}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  release_render_resources();ImGui::DestroyContext();SDL_GL_DeleteContext(context);SDL_DestroyWindow(window);SDL_Quit();
  std::filesystem::remove_all(directory);return result;
}
