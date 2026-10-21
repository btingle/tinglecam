#include "../src/gui/gui_internal.h"
#include "imgui_internal.h"
#include <iostream>
using namespace tinglecam_gui;
namespace {
void check(bool b,const char* message) {if(!b) throw std::runtime_error(message);}
void frame(App& app) {
  ImGui::NewFrame();
  ImGui::SetNextWindowSize({720,960},ImGuiCond_Always);
  ImGui::Begin("Hole panel");holes_section(app,app.job.operations.at(std::size_t(app.selected_operation)));ImGui::End();
  ImGui::Render();
}
void finish(App& app) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(app.hole_detector.running()) {
    check(std::chrono::steady_clock::now()<deadline,"Hole identification timed out");
    std::this_thread::yield();
  }
  poll_hole_detection(app);
}
void run(const std::filesystem::path& directory) {
  // Discovery intentionally needs neither tool selection nor stock nor a plan.
  const auto model=std::filesystem::path(__FILE__).parent_path()/"fixtures/meshes/auto_pocket.obj";
  App app;app.settings_file=directory/"settings.json";
  app.geometry.load_part(model);
  cam::Operation source;source.mode=cam::OperationMode::Hole;source.holes.automatic=true;
  source.holes.systems.clear();source.holes.id="source";
  insert_operation(app,0,source);select_operation(app,0);
  frame(app);frame(app);
  start_hole_identification(app,0);
  check(app.job.operations[0].holes.detection_signature.empty(),"Discovery does not publish geometry synchronously");
  finish(app);
  check(!app.job.operations[0].holes.detection_signature.empty() && !app.has_plan && app.tools.tools.empty(),
        "Independent discovery works without planning or a tool");
  check(app.hole_detection.error.empty(),"Discovery reports a successful empty-circle result for the square fixture");
  // An edit while a worker runs must not apply its old settings' result.
  app.job.operations[0].holes.detection_signature.clear();
  start_hole_identification(app,0);app.job.operations[0].holes.inspection_depth=.5;
  finish(app);
  check(app.job.operations[0].holes.detection_signature.empty(),"Stale discovery results cannot overwrite edited inspection settings");
  auto& holes=app.job.operations[0].holes;holes.automatic=false;holes.systems={{}};
  cam::Tool drill;drill.id="drill";drill.name="Drill";drill.type=cam::ToolType::Drill;
  drill.point_angle_degrees=118;drill.diameter=2;
  app.tools.tools.push_back(drill);
  frame(app);frame(app);
  add_pre_drilling(app,0);
  check(app.job.operations.size()==2 && app.job.operations[0].holes.source_id=="source" &&
      app.job.operations[0].tool_id=="drill" && app.selected_operation==0,
      "Pre-drilling insertion selects a normal linked drilling operation ahead of its source");
  const auto before=operation_signature(app,0,job_signature(app));
  app.job.operations[1].holes.systems[0].selected=false;
  check(operation_signature(app,0,job_signature(app))!=before,"Parent selection changes invalidate linked pilot plans");
  app.job.operations[1].holes.systems[0].selected=true;
  const auto tip_before=operation_signature(app,0,job_signature(app));
  app.tools.tools[0].point_angle_degrees=90;
  check(operation_signature(app,0,job_signature(app))!=tip_before,"Drill point changes invalidate the drilling plan");
  frame(app);frame(app);
  select_operation(app,1);frame(app);
  // Rendering the panel must not dirty a newly loaded/manual job just to
  // manufacture an identity; identities are assigned by explicit actions.
  app.job.operations[1].holes.id.clear();
  const auto before_frame=cam::to_json_text(app.job);
  frame(app);
  check(cam::to_json_text(app.job)==before_frame,"Viewing hole settings does not mutate the saved job");
  // A library default can retain strategy preferences, but not detected source
  // identities when a new operation is created or copied.
  check(cam::operation_default(app.settings,cam::OperationMode::Hole).mode==cam::OperationMode::Hole,
        "Hole defaults have their own operation slot");
}
}
int main() {
  const auto directory=std::filesystem::temp_directory_path()/("tinglecam-hole-ui-"+
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(directory);ImGui::CreateContext();auto& io=ImGui::GetIO();
  io.IniFilename=nullptr;io.DisplaySize={1280,1000};io.DeltaTime=1.f/60;
  unsigned char* pixels;int w,h;io.Fonts->GetTexDataAsRGBA32(&pixels,&w,&h);
  int result=0;try {run(directory);std::cout<<"Hole GUI discovery, editable levels and linked pre-drilling passed\n";}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  ImGui::DestroyContext();std::filesystem::remove_all(directory);return result;
}
