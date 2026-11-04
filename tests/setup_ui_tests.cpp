#include "../src/gui/gui_internal.h"
#include <iostream>

using namespace tinglecam_gui;
namespace {
void check(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
void finish(App& app) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(!take_model_load(app)) {
    check(std::chrono::steady_clock::now()<deadline,"Setup load timed out");std::this_thread::yield();
  }
  check(app.model_load_error.empty(),app.model_load_error.c_str());
}
void frame(App& app) {
  ImGui::NewFrame();
  ImGui::SetNextWindowSize({1100,1600});ImGui::Begin("Setup editor");
  setup_panel(app);operations_panel(app);operation_panel(app);
  ImGui::End();ImGui::Render();
}
void run(const std::filesystem::path& directory) {
  cam::write_file(directory/"part.obj","v 0 0 0\nv 8 0 0\nv 0 6 0\nv 0 0 4\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n");
  App app;app.settings_file=directory/"settings.json";
  app.job.part_mesh=directory/"part.obj";app.job.box_stock=true;
  cam::ensure_setup_ids(app.job);
  load_meshes(app);
  insert_operation(app,0,make_operation(app,cam::OperationMode::Roughing));
  const auto source_id=app.job.operations.front().id;
  const auto* source_geometry=app.geometry.part().source_identity();
  add_setup(app);finish(app);
  check(app.job.setups.size()==2 && app.selected_setup==1,"Adding a setup selects its own context");
  check(app.context().stock_from_operation==source_id,"New setups name a stable upstream stock result");
  check(app.geometry.part().source_identity()==source_geometry,"Switching contexts shares source geometry");
  check(app.selected_operation==-1,"An empty setup does not select an operation in another setup");
  insert_operation(app,1,make_operation(app,cam::OperationMode::Roughing));select_operation(app,1);
  for(std::size_t i=0;i<app.job.operations.size();++i) {
    auto& result=app.plan.operations[i];result.planned=true;result.error.clear();
    result.operation_id=app.job.operations[i].id;result.setup_id=app.job.operations[i].setup_id;
    result.stock_after=std::make_shared<const wave::Heightmap>(wave::Heightmap::from_polygons(
        {{{0,0},{8000,0},{8000,6000},{0,6000}}},0,4,.25));
    app.planned[i]={operation_signature(app,i,job_signature(app)),1};
  }
  refresh_plan_states(app);refresh_inherited_stock(app);
  check(plan_state(app,0)==PlanState::Planned && plan_state(app,1)==PlanState::Planned,
      "An unchanged cross-setup dependency has valid result markers");
  check(bool(app.geometry.inherited_stock()),"The setup preview consumes the referenced stock result");
  app.plan.operations[0].error="Source failed";refresh_inherited_stock(app);
  check(!app.geometry.inherited_stock(),"A failed stock source cannot leave an apparently valid inherited preview");
  app.plan.operations[0].error.clear();refresh_inherited_stock(app);
  app.job.setups.front().part_offset_x=2;refresh_plan_states(app);
  check(plan_state(app,0)==PlanState::Edited && plan_state(app,1)==PlanState::Downstream,
      "Editing an upstream setup invalidates its dependent stock consumer");
  app.job.setups.front().part_offset_x=0;refresh_plan_states(app);
  app.job.setups[1].rapid_clearance_height+=1;refresh_plan_states(app);
  check(plan_state(app,0)==PlanState::Planned && plan_state(app,1)==PlanState::Edited,
      "Editing a downstream context leaves upstream results current");
  frame(app);frame(app);
  select_operation(app,0);finish(app);
  check(app.selected_setup==0 && app.geometry.part().source_identity()==source_geometry,
      "Selecting an operation switches the setup view without reimporting its part");
  const auto before=job_signature(app);
  select_setup(app,1);finish(app);
  check(job_signature(app)==before,"Selecting a setup does not change document dependencies");
  app.context().properties_from_setup=app.job.setups.front().id;
  app.job.setups.front().origin_z=-12;
  check(app.effective_context().origin_z==-12,"Displayed coordinates resolve inherited properties");
  frame(app);
  cam::Setup independent;independent.id=cam::new_document_id();independent.name="Independent";
  app.job.setups.push_back(independent);
  move_setup(app,2,1);
  check(app.job.setups[1].id==independent.id && app.selected_setup==2,
      "Independent setups can be reordered without changing the selected context");
  check(app.job.setups[2].stock_from_operation==source_id && app.plan.operations[0].operation_id==source_id,
      "Reordering retains result identities and aligns plan entries");
  move_setup(app,0,2);
  check(app.job.setups[1].id==independent.id,"A reorder that breaks earlier-stage inheritance is rejected atomically");
  frame(app);
}
}
int main() {
  const auto directory=std::filesystem::temp_directory_path()/cam::new_document_id();
  std::filesystem::create_directories(directory);ImGui::CreateContext();
  auto& io=ImGui::GetIO();io.IniFilename=nullptr;io.DisplaySize={1200,1800};io.DeltaTime=1.f/60;
  unsigned char* pixels;int w,h;io.Fonts->GetTexDataAsRGBA32(&pixels,&w,&h);
  int result=0;
  try {run(directory);std::cout<<"Setup editor and dependency checks passed.\n";}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  ImGui::DestroyContext();std::filesystem::remove_all(directory);return result;
}
