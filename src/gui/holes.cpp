#include "gui_internal.h"
#include <numbers>
#include <utility>
namespace tinglecam_gui {
namespace {
std::string new_hole_id() {
  static std::uint64_t serial=0;
  return "holes-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+
      "-"+std::to_string(++serial);
}
const char* strategy_label(cam::HoleStrategy s) {
  switch(s) {
    case cam::HoleStrategy::Drill:return "Drilling";
    case cam::HoleStrategy::Helical:return "Helical finishing";
    case cam::HoleStrategy::Pocket:return "Pocket clearing";
    case cam::HoleStrategy::Adaptive:return "Adaptive by radius";
  }
  return "Unknown";
}
const char* depth_label(cam::DrillDepth d) {
  return d==cam::DrillDepth::Tip?"Tip reaches bottom Z":"Full diameter reaches bottom Z";
}
constexpr cam::HoleStrategy strategies[]{cam::HoleStrategy::Drill,cam::HoleStrategy::Helical,
    cam::HoleStrategy::Pocket,cam::HoleStrategy::Adaptive};
void strategy_override(std::optional<cam::HoleStrategy>& strategy) {
  property_label("Strategy");
  if(ImGui::BeginCombo("##strategy",strategy?strategy_label(*strategy):"Operation default")) {
    if(ImGui::Selectable("Operation default",!strategy)) strategy.reset();
    for(auto s:strategies) if(ImGui::Selectable(strategy_label(s),strategy==s)) strategy=s;
    ImGui::EndCombo();
  }
}
void tool_override(App& app,std::string& id) {
  property_label("Tool");
  const auto* current=app.tools.find(id);
  if(ImGui::BeginCombo("##tool",id.empty()?"Operation default":current?current->name.c_str():"Missing tool")) {
    if(ImGui::Selectable("Operation default",id.empty())) id.clear();
    for(const auto& tool:app.tools.tools)
      if(ImGui::Selectable(tool.name.c_str(),tool.id==id)) id=tool.id;
    ImGui::EndCombo();
  }
}
}
void start_hole_identification(App& app,int index) {
  if(app.model_loading || app.planner.running() || !app.geometry.has_part()) return;
  auto& p=app.job.operations.at(std::size_t(index)).holes;
  if(p.id.empty()) p.id=new_hole_id();
  app.hole_detection_id=p.id;app.hole_detection={};
  app.hole_detector.start([&geometry=app.geometry,parameters=p](const cam::ProgressCallback& progress) {
    return cam::identify_holes(geometry,parameters,progress);
  });
}
void poll_hole_detection(App& app) {
  cam::HoleDetection result;
  if(!app.hole_detector.take(result)) return;
  const auto id=std::exchange(app.hole_detection_id,{});
  for(auto& op:app.job.operations) if(!id.empty() && op.mode==cam::OperationMode::Hole && op.holes.id==id) {
    app.hole_detection=result;
    if(!result.error.empty()) {say(app,result.error,result.error!="Cancelled");return;}
    if(result.signature!=cam::hole_detection_signature(app.geometry,op.holes)) {
      say(app,"Inspection settings changed; identify holes again.");return;
    }
    cam::apply_hole_detection(op.holes,result);
    log_warnings(app,"Hole identification",result.warnings);
    say(app,"Identified "+std::to_string(result.systems.size())+" hole systems");
    app.view.needs_fit=true;return;
  }
}
bool holes_section(App& app,cam::Operation& operation) {
  auto& p=operation.holes;
  const bool linked=!p.source_id.empty();
  if(linked) {
    ImGui::TextWrapped("Pre-drilling follows the selected milling levels of its linked Hole operation.");
    try {
      const auto resolved=cam::resolve_hole_source(app.job,std::size_t(app.selected_operation));
      ImGui::TextDisabled("%zu selected hole systems",resolved.holes.systems.size());
    } catch(const std::exception& e) {ImGui::TextWrapped("%s",e.what());}
  }
  bool milling=!linked && p.strategy!=cam::HoleStrategy::Drill;
  bool adaptive=!linked && p.strategy==cam::HoleStrategy::Adaptive;
  bool drilling=linked || p.strategy==cam::HoleStrategy::Drill;
  for(const auto& h:p.systems) for(const auto& l:h.levels) if(l.strategy) {
    milling|=*l.strategy!=cam::HoleStrategy::Drill;
    adaptive|=*l.strategy==cam::HoleStrategy::Adaptive;
    drilling|=*l.strategy==cam::HoleStrategy::Drill;
  }
  if(begin_properties("hole-settings")) {
    if(!linked) prop_check("Identify automatically",p.automatic);
    if(!linked) prop_enum("Default strategy",p.strategy,strategy_label,strategies,std::size(strategies));
    if(adaptive) prop_double("Adaptive cutoff radius",p.adaptive_radius,.05f,"%.3f mm",
                "Larger radii use pocket clearing; smaller radii use helical finishing.",.001);
    if(milling) {
      prop_double("Helix angle",p.helix_angle_degrees,.1f,"%.2f deg",nullptr,.1,10);
      prop_double("Milling stepdown",operation.stepdown,.1f,"%.3f mm",nullptr,.001);
      double percent=100*operation.stepover_ratio;
      if(prop_double("Pocket stepover",percent,.5f,"%.1f%% of tool diameter",nullptr,1,100))
        operation.stepover_ratio=percent/100;
      prop_double("Radial margin",p.radial_margin,.01f,"%.3f mm",
                  "Material left on milled walls; a negative value makes them oversize.");
    }
    end_properties();
  }
  if(!linked && p.automatic) {
    if(begin_properties("hole-identification")) {
      prop_double("Inspection depth",p.inspection_depth,.1f,"%.3f mm",
                  "Distance below part top. Zero inspects just inside the top surface.",0);
      prop_double("Circle tolerance",p.circle_tolerance,.001f,"%.3f mm",
                  "Maximum deviation of the complete contour from a fitted circle.",.001);
      prop_check("Find counterbores",p.counterbores);
      prop_double("Probe inset",p.probe_inset,.005f,"%.3f mm",
                  "Distance inside the rim for downward shoulder probes.",.001);
      prop_double("Slice offset",p.slice_epsilon,.001f,"%.3f mm",
                  "Distance below a shoulder to inspect the next opening.",.001);
      end_properties();
    }
    const bool busy=app.hole_detector.running();
    ImGui::BeginDisabled(busy || app.model_loading || app.planner.running() || !app.geometry.has_part());
    if(ImGui::Button("Identify holes")) start_hole_identification(app,app.selected_operation);
    ImGui::EndDisabled();
    if(busy) {
      ImGui::SameLine();if(ImGui::Button("Cancel identification")) app.hole_detector.cancel();
      ImGui::ProgressBar(float(app.hole_detector.progress().fraction),ImVec2(-FLT_MIN,0));
      ImGui::TextDisabled("%s",app.hole_detector.stage().c_str());
    }
    if(!p.detection_signature.empty() && p.detection_signature!=cam::hole_detection_signature(app.geometry,p))
      ImGui::TextColored(severity_colour(app,LogSeverity::Warning),"Identification is out of date. Identify again before planning.");
  }
  if(drilling && begin_properties("hole-drilling")) {
    constexpr cam::DrillDepth depths[]{cam::DrillDepth::Tip,cam::DrillDepth::FullDiameter};
    prop_enum("Drilling depth",p.drilling.depth_reference,depth_label,depths,std::size(depths),
              "Tip: the point stops at bottom Z. Full diameter: the complete cutting diameter reaches bottom Z.");
    prop_double("Depth adjustment",p.drilling.depth_adjustment,.05f,"%.3f mm",
                "Positive drills deeper; negative stops above the nominal hole floor.");
    prop_double("Peck depth",p.drilling.peck_depth,.1f,"%.3f mm","Zero drills in one feed move.",0);
    prop_double("Peck retract clearance",p.drilling.peck_clearance,.1f,"%.3f mm",nullptr,0);
    prop_double("Through breakthrough",p.drilling.breakthrough,.05f,"%.3f mm",nullptr,0);
    end_properties();
  }
  if(!linked && p.automatic && p.detection_signature.empty()) {
    ImGui::TextDisabled("Identify holes before selecting levels to cut.");return false;
  }
  if(!linked) {
    const bool exists=!p.id.empty() && std::any_of(app.job.operations.begin(),app.job.operations.end(),
        [&](const auto& op) {return op.mode==cam::OperationMode::Hole && op.holes.source_id==p.id;});
    if(milling && !exists && ImGui::Button("Insert pre-drilling operation")) return true;
    if(exists) ImGui::TextDisabled("A linked pre-drilling operation is already in this job.");
    if(ImGui::SmallButton("Select all")) for(auto& h:p.systems) {h.selected=true;for(auto& l:h.levels) l.selected=true;}
    ImGui::SameLine();
    if(ImGui::SmallButton("Deselect all")) for(auto& h:p.systems) {h.selected=false;for(auto& l:h.levels) l.selected=false;}
    int remove_hole=-1;
    for(std::size_t i=0;i<p.systems.size();++i) {
      auto& h=p.systems[i];ImGui::PushID(int(i));
      ImGui::Checkbox("##system-selected",&h.selected);
      ImGui::SetItemTooltip("Cut this hole system");ImGui::SameLine();
      const auto flags=p.systems.size()<=3?ImGuiTreeNodeFlags_DefaultOpen:ImGuiTreeNodeFlags_None;
      if(!ImGui::TreeNodeEx("system",flags,"Hole %zu: X %.2f Y %.2f (%zu levels)",
          i+1,h.center_x+app.job.origin_x,h.center_y+app.job.origin_y,h.levels.size())) {
        ImGui::PopID();continue;
      }
      if(begin_properties("hole-system")) {
        prop_coordinate("Centre X",h.center_x,app.job.origin_x);
        prop_coordinate("Centre Y",h.center_y,app.job.origin_y);
        end_properties();
      }
      int remove_level=-1;
      for(std::size_t j=0;j<h.levels.size();++j) {
        auto& l=h.levels[j];ImGui::PushID(int(j));
        const bool open=ImGui::TreeNodeEx("level",ImGuiTreeNodeFlags_DefaultOpen,"Level %zu: R %.3f, Z %.3f to %.3f",
            j+1,l.radius,l.top_z+app.job.origin_z,l.bottom_z+app.job.origin_z);
        if(open) {
          ImGui::Checkbox("Cut this level",&l.selected);
          if(begin_properties("hole-level")) {
            prop_double("Radius",l.radius,.01f,"%.3f mm",nullptr,.001);
            prop_coordinate("Top Z",l.top_z,app.job.origin_z);
            prop_coordinate("Bottom Z",l.bottom_z,app.job.origin_z);
            prop_check("Through hole",l.through);
            strategy_override(l.strategy);tool_override(app,l.tool_id);
            if(p.automatic) prop_value("Circle fit error","%.4f mm",l.fit_error);
            end_properties();
          }
          if(!p.automatic && h.levels.size()>1 && ImGui::SmallButton("Remove level")) remove_level=int(j);
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
      if(remove_level>=0) h.levels.erase(h.levels.begin()+remove_level);
      if(!p.automatic) {
        if(ImGui::SmallButton("Add deeper level")) {
          auto next=h.levels.empty()?cam::HoleLevel{}:h.levels.back();
          next.top_z=next.bottom_z;next.bottom_z-=5;next.radius*=.6;
          next.through=false;next.strategy.reset();h.levels.push_back(next);
        }
        if(ImGui::SmallButton("Remove hole")) remove_hole=int(i);
      }
      ImGui::TreePop();ImGui::PopID();
    }
    if(remove_hole>=0) p.systems.erase(p.systems.begin()+remove_hole);
    if(!p.automatic && ImGui::Button("Add hole")) {
      cam::HoleSystem h;h.center_x=-app.job.origin_x;h.center_y=-app.job.origin_y;
      h.levels.front().top_z=-app.job.origin_z;h.levels.front().bottom_z=h.levels.front().top_z-5;
      p.systems.push_back(h);
    }

  }
  return false;
}
void add_pre_drilling(App& app,int index) {
  auto& source=app.job.operations.at(std::size_t(index));
  if(source.holes.id.empty()) source.holes.id=new_hole_id();
  cam::Operation pilot=source;pilot.name=source.name+" - pre-drill";
  pilot.holes.source_id=source.holes.id;pilot.holes.id=new_hole_id();
  pilot.holes.systems.clear();pilot.holes.strategy=cam::HoleStrategy::Drill;
  pilot.tool_id.clear();pilot.cutting={};
  for(const auto& tool:app.tools.tools) if(tool.type==cam::ToolType::Drill) {pilot.tool_id=tool.id;break;}
  insert_operation(app,index,std::move(pilot));select_operation(app,index);
  say(app,"Inserted linked pre-drilling operation. Choose its drill and drilling settings.");
}
void draw_hole_contours(App& app,OrbitScene& scene,cam::Operation& operation) {
  cam::Operation resolved;
  const cam::HoleParameters* parameters=&operation.holes;
  if(!operation.holes.source_id.empty()) {
    try {resolved=cam::resolve_hole_source(app.job,std::size_t(app.selected_operation));parameters=&resolved.holes;}
    catch(...) {return;}
  }
  const auto& p=*parameters;
  if(p.automatic && p.detection_signature.empty()) return;
  std::size_t hit_h=std::size_t(-1),hit_l=std::size_t(-1);
  float best=ui_size(app,7);
  const auto mouse=ImGui::GetIO().MousePos;
  for(std::size_t i=0;i<p.systems.size();++i) {
    const auto& h=p.systems[i];
    for(std::size_t j=0;j<h.levels.size();++j) {
      const auto& l=h.levels[j];if(!(l.radius>0)) continue;
      const auto colour=h.selected && l.selected?IM_COL32(100,240,150,255):IM_COL32(155,155,155,210);
      ImVec2 previous;
      for(int a=0;a<=96;++a) {
        const double angle=2*std::numbers::pi*a/96;
        const auto q=scene.projection.point({float(h.center_x+l.radius*std::cos(angle)),
            float(h.center_y+l.radius*std::sin(angle)),float(l.top_z)});
        if(a) {
          scene.list->AddLine(previous,q,colour,ui_size(app,1.7f));
          const float dx=q.x-previous.x,dy=q.y-previous.y,d=dx*dx+dy*dy;
          const float t=d>0?std::clamp(((mouse.x-previous.x)*dx+(mouse.y-previous.y)*dy)/d,0.f,1.f):0;
          const float distance=std::hypot(mouse.x-previous.x-t*dx,mouse.y-previous.y-t*dy);
          if(distance<best) {best=distance;hit_h=i;hit_l=j;}
        }
        previous=q;
      }
    }
  }
  const auto drag=ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
  if(hit_h!=std::size_t(-1) && scene.hovered && !app.orbit.cube_pressed && !app.planner.running() &&
      operation.holes.source_id.empty() && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
      drag.x*drag.x+drag.y*drag.y<9) {
    auto& h=operation.holes.systems[hit_h];auto& l=h.levels[hit_l];
    if(!h.selected) {h.selected=true;l.selected=true;} else l.selected=!l.selected;
  }
}
}
