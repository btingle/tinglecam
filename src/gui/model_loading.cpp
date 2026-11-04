#include "gui_internal.h"

namespace tinglecam_gui {
namespace {
LoadedModels read_job(const std::filesystem::path& file) {
  LoadedModels result;
  result.job = cam::load_job(file, &result.warnings);
  result.job_file = file;
  result.opened_job = true;
  const auto resolve=[&](cam::SetupContext& context) {
    const auto path=[&](std::filesystem::path& value) {
      if(!value.empty() && value.is_relative()) value=std::filesystem::absolute(file).parent_path()/value;
    };
    path(context.part_mesh); path(context.stock_mesh);
    for(auto& f:context.fixtures) path(f.mesh);
  };
  resolve(result.job);
  for(auto& setup:result.job.setups) resolve(setup);
  return result;
}
void load_geometry(LoadedModels& result, const cam::ProgressCallback& progress = {}) {
  const auto job=result.job.setups.empty()?result.job:cam::job_for_setup(result.job,
      result.job.setups.at(result.setup_index).id);
  if(result.replace_part && result.replace_stock) {
    result.geometry.configure(job,[&](double f,const std::string& stage) {return !progress || progress(f,stage);});
    if(progress && !progress(1,"Models ready")) throw std::runtime_error("Cancelled");
    return;
  }
  const bool part = result.replace_part && !job.part_mesh.empty();
  const bool stock = result.replace_stock && !job.stock_mesh.empty();
  const int count = int(part) + int(stock);
  int completed = 0;
  auto report = [&](double fraction, const std::string& stage) {
    const auto& file = completed == 0 && part ? job.part_mesh : job.stock_mesh;
    return !progress || progress((completed + fraction) / std::max(1, count),
                                  file.filename().string() + ": " + stage);
  };
  if (part) {
    result.geometry.load_part(job.part_mesh, report, job.part_orientation);
    result.geometry.set_part_placement({job.part_orientation,{job.part_offset_x,job.part_offset_y,job.part_offset_z}});
    ++completed;
  }
  if (stock) {
    result.geometry.load_stock(job.stock_mesh, report, job.stock_orientation,
        {job.stock_offset_x, job.stock_offset_y, job.stock_offset_z});
    ++completed;
  }
  result.geometry.set_box_stock(job.box_stock, job.box_stock_margins);
  if (progress && !progress(1.0, "Models ready")) throw std::runtime_error("Cancelled");
}
void apply_loaded_models(App& app, LoadedModels result) {
  if (result.replace_part && result.replace_stock)
    app.geometry = std::move(result.geometry);
  else app.geometry.replace_models(std::move(result.geometry),result.replace_part,result.replace_stock);
  const auto c=result.job.setups.empty()?static_cast<const cam::SetupContext&>(result.job):
      cam::resolve_setup_context(result.job,result.job.setups.at(result.setup_index).id);
  app.geometry.set_box_stock(c.stock_from_operation.empty() && c.box_stock,c.box_stock_margins);
  app.job = std::move(result.job);
  app.selected_setup=result.setup_index;
  if(!app.job.setups.empty() && (app.selected_operation<0 || app.selected_operation>=int(app.job.operations.size()) ||
      app.job.operations[app.selected_operation].setup_id!=app.job.setups.at(app.selected_setup).id)) {
    app.selected_operation=-1;
    for(std::size_t i=0;i<app.job.operations.size();++i)
      if(app.job.operations[i].setup_id==app.job.setups.at(app.selected_setup).id) {
        app.selected_operation=int(i);app.selected_plan_operation=int(i);break;
      }
  }
  refresh_inherited_stock(app);
  if (result.opened_job) {
    app.job_file = std::move(result.job_file);
    mark_job_saved(app);
    if(app.job.setups.empty()) app.selected_operation = app.job.operations.empty()?-1:0;
    app.operation_selection.assign(app.job.operations.size(), false);
    log_warnings(app, "Job", result.warnings);
    say(app, "Opened " + app.job_file.string());
    if (app.effective_context().machine_id.empty() || app.effective_context().material_id.empty()) {
      adopt_default_setup(app);
      log_message(app, LogSeverity::Warning, "Job",
                  "The job named no machine or material; the defaults were chosen. "
                  "Save to keep them.");
    }
  }
  if (result.reset_setup) {
    for (auto& operation : app.job.operations)
      if(app.job.setups.empty() || operation.setup_id==app.job.setups[app.selected_setup].id)
        depth_defaults(app, operation);
    if (result.replace_part || !app.context().origin_relative_to_part) apply_origin_pick(app);
  }
  app.stock_position_editing = false;
  app.surface_preview_key.clear();
  app.surface_preview = {};
  app.surface_depth_model.reset();
  // Geometry edits retain existing results with their stale-state markers;
  // opening a different document establishes a new result list.
  if(result.opened_job) invalidate_plan(app);
  else refresh_plan_states(app);
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
    result.setup_index=app.selected_setup;
    auto& context=result.job.setups.empty()?static_cast<cam::SetupContext&>(result.job):result.job.setups.at(result.setup_index);
    if (context.stock_mesh.empty() && context.stock_from_operation.empty()) context.box_stock = true;
    result.geometry.share_sources(app.geometry);
    result.reset_setup = true;
    load_geometry(result);
    apply_loaded_models(app, std::move(result));
    say(app, "Models loaded");
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
}
void start_model_loading(App& app, bool reset_setup) {
  auto request=app.job;
  const int setup=app.selected_setup;
  auto& context=request.setups.empty()?static_cast<cam::SetupContext&>(request):request.setups.at(setup);
  if(reset_setup && context.stock_mesh.empty() && context.stock_from_operation.empty()) context.box_stock=true;
  auto* geometry=&app.geometry;
  begin_loading(app,"Loading setup",[request=std::move(request),setup,reset_setup,geometry](const cam::ProgressCallback& progress) {
    LoadedModels result; result.job=request; result.setup_index=setup; result.reset_setup=reset_setup;
    result.geometry.share_sources(*geometry);
    load_geometry(result,progress);
    return result;
  });
}
void start_model_loading(App& app,const std::filesystem::path& file,bool part) {
  auto request=app.job;
  const int setup=app.selected_setup;
  auto& context=request.setups.empty()?static_cast<cam::SetupContext&>(request):request.setups.at(setup);
  if(part) {context.part_mesh=file; context.part_from_setup.clear();}
  else {context.stock_mesh=file;context.box_stock=false;context.stock_from_operation.clear();}
  if(context.stock_mesh.empty() && context.stock_from_operation.empty()) context.box_stock=true;
  auto* geometry=&app.geometry;
  begin_loading(app,file.filename().string(),[request=std::move(request),setup,part,geometry](const cam::ProgressCallback& progress) {
    LoadedModels result;result.job=request;result.setup_index=setup;result.reset_setup=true;
    result.replace_part=part;result.replace_stock=!part;
    result.geometry.share_sources(*geometry);
    load_geometry(result,progress);
    return result;
  });
}
void start_model_rotation(App& app,int target,int axis,int direction) {
  auto request=app.job;
  const int setup=app.selected_setup;
  auto& context=request.setups.empty()?static_cast<cam::SetupContext&>(request):request.setups.at(setup);
  const auto rotate=[&](cam::ModelOrientation& orientation) {
    if(direction==0) orientation={}; else orientation.quarter_turn(axis,direction);
  };
  if(target!=2) rotate(context.part_orientation);
  if(target!=1) {rotate(context.stock_orientation);rotate(context.stock_transfer.orientation);}
  auto* geometry=&app.geometry;
  begin_loading(app,"Rotating model views",[request=std::move(request),setup,target,geometry](const cam::ProgressCallback& progress) {
    LoadedModels result;result.job=request;result.setup_index=setup;result.reset_setup=true;
    result.replace_part=target!=2;result.replace_stock=target!=1;
    const auto& c=request.setups.empty()?static_cast<const cam::SetupContext&>(request):request.setups.at(setup);
    if(progress && !progress(0,"Preparing model placement")) throw std::runtime_error("Cancelled");
    result.geometry.place_models_from(*geometry,result.replace_part,result.replace_stock && !c.stock_mesh.empty(),
        {c.part_orientation,{c.part_offset_x,c.part_offset_y,c.part_offset_z}},
        {c.stock_orientation,{c.stock_offset_x,c.stock_offset_y,c.stock_offset_z}});
    if(progress && !progress(1,"Model views ready")) throw std::runtime_error("Cancelled");
    return result;
  });
}
void start_stock_positioning(App& app,const std::array<double,3>& offset) {
  if(!app.geometry.has_stock() || app.geometry.box_stock()) return;
  auto request=app.job;
  const int setup=app.selected_setup;
  auto& context=request.setups.empty()?static_cast<cam::SetupContext&>(request):request.setups.at(setup);
  context.stock_offset_x=offset[0];context.stock_offset_y=offset[1];context.stock_offset_z=offset[2];
  auto* geometry=&app.geometry;
  begin_loading(app,"Positioning stock view",[request=std::move(request),setup,geometry](const cam::ProgressCallback& progress) {
    LoadedModels result;result.job=request;result.setup_index=setup;result.replace_part=false;result.reset_setup=true;
    const auto& c=request.setups.empty()?static_cast<const cam::SetupContext&>(request):request.setups.at(setup);
    if(progress && !progress(0,"Preparing stock placement")) throw std::runtime_error("Cancelled");
    result.geometry.place_models_from(*geometry,false,true,{},
        {c.stock_orientation,{c.stock_offset_x,c.stock_offset_y,c.stock_offset_z}});
    if(progress && !progress(1,"Stock positioned")) throw std::runtime_error("Cancelled");
    return result;
  });
}
void refresh_inherited_stock(App& app) {
  if(app.job.setups.empty()) return;
  try {
    const auto context=cam::resolve_setup_context(app.job,app.job.setups.at(app.selected_setup).id);
    if(context.stock_from_operation.empty()) return;
    app.geometry.set_inherited_stock(nullptr);
    for(std::size_t i=0;i<app.job.operations.size() && i<app.plan.operations.size();++i)
      if(app.job.operations[i].id==context.stock_from_operation) {
        const auto& source=app.plan.operations[i];
        if(source.planned && source.ok() && source.stock_after && context.stock_transfer.preserves_columns())
          app.geometry.set_inherited_stock(std::make_shared<const wave::Heightmap>(source.stock_after->transformed(context.stock_transfer)));
        return;
      }
  } catch(const std::exception& e) {say(app,e.what(),true);}
}
void refresh_setup_geometry(App& app) {start_model_loading(app,false);}
void select_setup(App& app,int index) {
  if(app.model_loading || app.planner.running() || index<0 || index>=int(app.job.setups.size()) || index==app.selected_setup) return;
  // Keep the current view until the target context is ready to commit.
  auto request=app.job;
  auto* geometry=&app.geometry;
  begin_loading(app,"Loading setup",[request=std::move(request),index,geometry](const cam::ProgressCallback& progress) {
    LoadedModels result;result.job=request;result.setup_index=index;
    result.geometry.share_sources(*geometry);load_geometry(result,progress);return result;
  });
}
void add_setup(App& app) {
  if(app.model_loading || app.planner.running()) return;
  cam::ensure_setup_ids(app.job);
  const auto source=app.job.setups.at(app.selected_setup).id;
  cam::Setup setup;
  static_cast<cam::SetupContext&>(setup)=cam::resolve_setup_context(app.job,source);
  setup.id=cam::new_document_id();setup.name="Setup "+std::to_string(app.job.setups.size()+1);
  setup.part_from_setup=source;setup.fixtures.clear();setup.fixtures_from_setup.clear();
  setup.properties_from_setup.clear();setup.stock_from_operation.clear();setup.stock_transfer={};
  for(auto i=app.job.operations.rbegin();i!=app.job.operations.rend();++i)
    if(i->setup_id==source) {setup.stock_from_operation=i->id;break;}
  app.job.setups.push_back(std::move(setup));
  select_setup(app,int(app.job.setups.size())-1);
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
