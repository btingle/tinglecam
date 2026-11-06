#include "gui_internal.h"
namespace tinglecam_gui {
namespace {

// A depth below the work top, shown as a program Z.
bool prop_depth(App& app, const char* label, double& depth, const char* help) {
  const double offset = work_top(app) + app.effective_context().origin_z;
  double z = offset - depth;
  if (!prop_double(label, z, 0.1f, "%.3f mm", help)) return false;
  depth = offset - z;
  return true;
}

void roughing_settings(App& app, cam::Operation& operation, const cam::Tool* tool) {
  prop_check("Boundary cleanup", operation.rough_boundary_cleanup,
             "Follow the part outline once more at each depth after the wave.");
  prop_check("Lead in from the air", operation.lead_in,
             "Start the wave outside the stock so the cutter arrives at the wall "
             "along a path rather than burying half its width.");
  prop_int("Finishing subdivisions", operation.finishing_subdivisions, 0.05f, 1, 64,
           "Cut every coarse depth first, then clean each terrace upward in this "
           "many parts.");
  ImGui::BeginDisabled(operation.finishing_subdivisions <= 1);
  prop_check("Adapt cleanup stepover", operation.adaptive_cleanup_stepover,
             "Widen the cleanup stepover as the remaining terrace gets shorter.");
  ImGui::EndDisabled();
  (void)app;
  (void)tool;
}

void depth_breaks(App& app, cam::Operation& operation) {
  int remove = -1;
  for (std::size_t i = 0; i < operation.depth_breaks.size(); ++i) {
    ImGui::PushID(static_cast<int>(i));
    property_label(i == 0 ? "Depth breaks" : "");
    const double offset = work_top(app) + app.effective_context().origin_z;
    double z = offset - operation.depth_breaks[i];
    const float button = ImGui::GetFrameHeight();
    ImGui::SetNextItemWidth(-(button + ImGui::GetStyle().ItemSpacing.x));
    if (ImGui::DragScalar("##break", ImGuiDataType_Double, &z, 0.1f, nullptr, nullptr,
                          "Z %.3f mm"))
      operation.depth_breaks[i] = offset - z;
    ImGui::SameLine();
    if (ImGui::Button("x", ImVec2(button, 0))) remove = static_cast<int>(i);
    ImGui::SetItemTooltip("Remove this break");
    ImGui::PopID();
  }
  if (remove >= 0) operation.depth_breaks.erase(operation.depth_breaks.begin() + remove);
  property_label(operation.depth_breaks.empty() ? "Depth breaks" : "",
                 operation.depth_breaks.empty()
                     ? "Intermediate floors landed exactly, each restarting the "
                       "stepdown. For a flat, enter its Z plus the axial stock to leave."
                     : nullptr);
  if (ImGui::Button("Add break")) {
    // Split the widest band so repeated additions start with distinct values.
    std::vector<double> depths{operation.depth_from, operation.depth_to};
    for (double depth : operation.depth_breaks)
      if (std::isfinite(depth) && depth > operation.depth_from &&
          depth < operation.depth_to)
        depths.push_back(depth);
    std::sort(depths.begin(), depths.end());
    std::size_t widest = 1;
    for (std::size_t i = 2; i < depths.size(); ++i)
      if (depths[i] - depths[i - 1] > depths[widest] - depths[widest - 1]) widest = i;
    operation.depth_breaks.push_back((depths[widest] + depths[widest - 1]) * 0.5);
  }
}

void pockets_section(App& app, cam::Operation& operation) {
  const double ox = app.effective_context().origin_x, oy = app.effective_context().origin_y, oz = app.effective_context().origin_z;
  if (begin_properties("pocket-common")) {
    prop_double("Radial margin", operation.pocket_radial_margin, 0.02f, "%.3f mm",
                "Stock left on the walls for a finishing pass. Negative cuts past "
                "the walls instead.");
    prop_check("Find pockets automatically", operation.auto_identify_pockets,
               "Find closed, convex holes in the part from the top down and plan "
               "one pocket for each.");
    end_properties();
  }
  if (operation.auto_identify_pockets) {
    if (begin_properties("pocket-auto")) {
      prop_double("Inspection depth", operation.pocket_auto.check_depth, 0.1f,
                  "%.2f mm", "How far below the part top the candidate outlines "
                  "are taken.");
      prop_double("Helix angle", operation.pocket_auto.helix_angle_degrees, 0.1f,
                  "%.1f deg", nullptr, 0.1, 10.0);
      end_properties();
    }
    int remove = -1;
    for (std::size_t i = 0; i < operation.pocket_auto.exclusions.size(); ++i) {
      auto& exclusion = operation.pocket_auto.exclusions[i];
      ImGui::PushID(static_cast<int>(i));
      ImGui::SeparatorText(("Exclusion " + std::to_string(i + 1)).c_str());
      if (begin_properties("exclusion")) {
        prop_check("Enabled", exclusion.enabled,
                   "Candidates overlapping an enabled rectangle are skipped.");
        ImGui::BeginDisabled(!exclusion.enabled);
        prop_coordinate("Centre X", exclusion.center_x, ox);
        prop_coordinate("Centre Y", exclusion.center_y, oy);
        prop_double("Width X", exclusion.width, 0.1f, "%.2f mm", nullptr, 0.0);
        prop_double("Height Y", exclusion.height, 0.1f, "%.2f mm", nullptr, 0.0);
        ImGui::EndDisabled();
        end_properties();
      }
      if (ImGui::SmallButton("Remove exclusion")) remove = static_cast<int>(i);
      ImGui::PopID();
    }
    if (remove >= 0)
      operation.pocket_auto.exclusions.erase(operation.pocket_auto.exclusions.begin() + remove);
    if (ImGui::Button("Add exclusion"))
      operation.pocket_auto.exclusions.push_back(cam::PocketExclusionArea{});
    const std::size_t index = static_cast<std::size_t>(app.selected_operation);
    if (index < app.plan.operations.size() && app.plan.operations[index].planned &&
        app.plan.operations[index].ok()) {
      const auto& found = app.plan.operations[index].identified_pockets;
      ImGui::SeparatorText("Found in the last plan");
      if (found.empty()) ImGui::TextDisabled("None");
      for (std::size_t i = 0; i < found.size(); ++i)
        ImGui::Text("%zu: X %.3f Y %.3f, %.2f x %.2f mm, Z %.3f to %.3f", i + 1,
                    found[i].center_x + ox, found[i].center_y + oy, found[i].width,
                    found[i].height, found[i].top_z + oz, found[i].bottom_z + oz);
    }
    return;
  }
  if (operation.pockets.empty()) operation.pockets.push_back(cam::PocketParameters{});
  int remove = -1;
  for (std::size_t i = 0; i < operation.pockets.size(); ++i) {
    ImGui::PushID(static_cast<int>(i));
    ImGui::SeparatorText(("Pocket " + std::to_string(i + 1)).c_str());
    auto& p = operation.pockets[i];
    if (begin_properties("pocket")) {
      prop_coordinate("Centre X", p.center_x, ox);
      prop_coordinate("Centre Y", p.center_y, oy);
      prop_coordinate("Top Z", p.top_z, oz);
      prop_double("Width X", p.width, 0.1f, "%.2f mm", nullptr, 0.0);
      prop_double("Height Y", p.height, 0.1f, "%.2f mm", nullptr, 0.0);
      prop_double("Corner radius", p.radius, 0.1f, "%.2f mm",
                  "0 is square; half the smaller side is round.", 0.0);
      prop_double("Depth", p.depth, 0.1f, "%.2f mm", nullptr, 0.0);
      prop_double("Helix angle", p.helix_angle_degrees, 0.1f, "%.1f deg", nullptr, 0.1, 10.0);
      prop_value("Floor Z", "%.3f mm", p.top_z - p.depth + oz);
      end_properties();
    }
    if (operation.pockets.size() > 1 && ImGui::SmallButton("Remove pocket"))
      remove = static_cast<int>(i);
    ImGui::PopID();
  }
  if (remove >= 0) operation.pockets.erase(operation.pockets.begin() + remove);
  if (ImGui::Button("Add pocket")) {
    cam::PocketParameters pocket;
    pocket.center_x = -app.effective_context().origin_x;
    pocket.center_y = -app.effective_context().origin_y;
    pocket.top_z = -app.effective_context().origin_z;
    operation.pockets.push_back(pocket);
  }
}

void areas_section(App& app, cam::Operation& operation) {
  const double ox = app.effective_context().origin_x, oy = app.effective_context().origin_y, oz = app.effective_context().origin_z;
  auto& areas = operation.plane.areas;
  if (areas.empty()) areas.push_back(cam::ClearingArea{});
  int remove = -1;
  for (std::size_t i = 0; i < areas.size(); ++i) {
    ImGui::PushID(static_cast<int>(i));
    ImGui::SeparatorText(("Area " + std::to_string(i + 1)).c_str());
    auto& area = areas[i];
    if (begin_properties("area")) {
      prop_coordinate("Centre X", area.center_x, ox);
      prop_coordinate("Centre Y", area.center_y, oy);
      prop_double("Width X", area.width, 0.1f, "%.2f mm", nullptr, 0.0);
      prop_double("Height Y", area.height, 0.1f, "%.2f mm", nullptr, 0.0);
      prop_coordinate("Top Z", area.top_z, oz);
      prop_double("Depth", area.depth, 0.1f, "%.2f mm", nullptr, 0.0);
      prop_value("Floor Z", "%.3f mm", area.top_z - area.depth + oz);
      end_properties();
    }
    if (areas.size() > 1 && ImGui::SmallButton("Remove area")) remove = static_cast<int>(i);
    ImGui::PopID();
  }
  if (remove >= 0) areas.erase(areas.begin() + remove);
  if (ImGui::Button("Add area")) {
    cam::ClearingArea area;
    area.center_x = -app.effective_context().origin_x;
    area.center_y = -app.effective_context().origin_y;
    area.top_z = -app.effective_context().origin_z;
    areas.push_back(area);
  }
}

void roughing_areas_section(App& app, cam::Operation& operation, const cam::Tool* tool) {
  ImGui::SeparatorText("Roughing areas");
  auto& areas=operation.roughing_areas;
  if(areas.empty()) ImGui::TextDisabled("Whole stock. Add areas to rough parts in order.");
  else ImGui::TextDisabled("Complete every depth and cleanup in each area, in order.");
  const auto context=app.effective_context();
  int remove=-1,duplicate=-1,move_from=-1,move_to=-1;
  for(std::size_t i=0;i<areas.size();++i) {
    ImGui::PushID(int(i));
    ImGui::SeparatorText(("Area "+std::to_string(i+1)).c_str());
    auto& area=areas[i];
    if(begin_properties("roughing-area")) {
      prop_text("Name",area.name);
      prop_coordinate("Centre X",area.center_x,context.origin_x);
      prop_coordinate("Centre Y",area.center_y,context.origin_y);
      prop_double("Width X",area.width,.1f,"%.3f mm","The full cutter stays within this box.",.001);
      prop_double("Height Y",area.height,.1f,"%.3f mm",nullptr,.001);
      end_properties();
    }
    if(ImGui::SmallButton("Duplicate")) duplicate=int(i);
    ImGui::SameLine();
    if(ImGui::SmallButton("Remove")) remove=int(i);
    ImGui::SameLine();ImGui::BeginDisabled(i==0);
    if(ImGui::SmallButton("Up")) {move_from=int(i);move_to=int(i)-1;}
    ImGui::EndDisabled();ImGui::SameLine();ImGui::BeginDisabled(i+1==areas.size());
    if(ImGui::SmallButton("Down")) {move_from=int(i);move_to=int(i)+1;}
    ImGui::EndDisabled();ImGui::PopID();
  }
  if(remove>=0) areas.erase(areas.begin()+remove);
  else if(duplicate>=0) {
    auto copy=areas[duplicate];if(!copy.name.empty()) copy.name+=" copy";
    areas.insert(areas.begin()+duplicate+1,std::move(copy));
  } else if(move_from>=0) std::swap(areas[move_from],areas[move_to]);
  if(ImGui::Button("Add roughing area")) {
    cam::RoughingArea area;area.name="Area "+std::to_string(areas.size()+1);
    area.center_x=-context.origin_x;area.center_y=-context.origin_y;
    if(app.geometry.has_part()) {
      const auto& b=app.geometry.part().mesh().bounds;
      if(b.valid) {
        const double margin=tool?2*tool->diameter:10;
        area.center_x=(double(b.lower.x)+b.upper.x)/2;
        area.center_y=(double(b.lower.y)+b.upper.y)/2;
        area.width=double(b.upper.x)-b.lower.x+2*margin;
        area.height=double(b.upper.y)-b.lower.y+2*margin;
      }
    }
    areas.push_back(std::move(area));
  }
}

void surface_section(App& app, cam::Operation& operation, const cam::Tool* tool) {
  auto& surface = operation.surface;
  const auto& cad = app.geometry.part_cad();
  if (!cad) {
    ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                       "Load a STEP part to choose faces.");
  } else {
    if (surface.model_fingerprint != cad->fingerprint() && !surface.faces.empty())
      ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                         "The model changed: choose the faces again.");
    ImGui::TextDisabled(surface.manual_order
        ? "Tick faces here; choose propagation roots in Setup."
        : "Click faces in the Setup view, or tick them here.");
    if (ImGui::SmallButton("Clear faces")) {
      surface.faces.clear();
      surface.model_fingerprint = cad->fingerprint();
    }
    if (ImGui::BeginChild("faces", ImVec2(0, ui_size(app, 150.0f)), ImGuiChildFlags_Borders)) {
      for (const auto& face : cad->faces()) {
        const bool current = surface.model_fingerprint == cad->fingerprint();
        bool selected = current && std::find(surface.faces.begin(), surface.faces.end(),
                                             face.id) != surface.faces.end();
        const auto label = "Face " + std::to_string(face.id + 1) + "  " + face.type;
        if (ImGui::Checkbox(label.c_str(), &selected)) {
          if (!current) surface.faces.clear();
          surface.model_fingerprint = cad->fingerprint();
          if (selected) surface.faces.push_back(face.id);
          else std::erase(surface.faces, face.id);
        }
      }
    }
    ImGui::EndChild();
  }
  if (begin_properties("surface")) {
    prop_double("Stepover", surface.stepover, 0.01f, "%.3f mm",
                "Distance between neighbouring passes across the surface.", 0.001);
    // Half a micron short of the radius keeps the ball centre outside the surface.
    prop_double("Stock allowance", surface.allowance, 0.01f, "%.3f mm",
                "Left along the surface normal. Negative cuts below the surface.",
                tool ? -tool->diameter / 2 + 0.0005
                     : -std::numeric_limits<double>::infinity());
    prop_check("Follow V direction", surface.along_v,
               "Passes follow the face's V parameter; off follows U.");
    prop_check("Bidirectional", surface.bidirectional,
               "Join neighbouring passes so the cutter alternates direction.");
    prop_check("Reverse normals", surface.flip_normals,
               "For open surfaces whose exported orientation faces into the cutter.");
    prop_check("Manual contour order", surface.manual_order,
               "Choose root contours and finish outwards through their neighbours.");
    end_properties();
  }
  if (surface.manual_order) {
    refresh_surface_contours(app, operation);
    ImGui::TextWrapped("Click contours in Setup to toggle propagation roots. "
                       "Use the face list above to change the finishing faces.");
    if (ImGui::SmallButton("Clear roots")) {
      surface.order_roots.clear();
      surface.order_signature.clear();
    }
    if (app.surface_previewer.running()) {
      ImGui::TextDisabled("Preparing contours: %.0f%%",
          app.surface_previewer.progress().fraction * 100.0);
    } else if (!app.surface_preview.error.empty()) {
      ImGui::TextWrapped("%s", app.surface_preview.error.c_str());
    } else if (!app.surface_preview.contours.empty()) {
      const bool current = surface.order_signature == app.surface_preview.signature;
      ImGui::Text("%zu contours, %zu roots, %zu branches",
                  app.surface_preview.contours.size(),
                  current ? surface.order_roots.size() : 0,
                  app.surface_order.branches);
      ImGui::TextDisabled("Gold: roots   Grey: unseeded   Red: ends / meetings");
      if (!current && !surface.order_roots.empty())
        ImGui::TextWrapped("Contours changed. Select propagation roots again.");
      else if (!app.surface_order.error.empty())
        ImGui::TextWrapped("%s", app.surface_order.error.c_str());
    }
  }
}

void feeds_section(App& app, cam::Operation& operation, const cam::Tool* tool) {
  const cam::Machine* machine = app.machines.find(app.effective_context().machine_id);
  const cam::Material* material = app.materials.find(app.effective_context().material_id);
  const cam::CuttingData* data = tool ? tool->cutting_for(app.effective_context().material_id) : nullptr;
  if (!tool) {
    ImGui::TextDisabled("Choose a tool first.");
    return;
  }
  const bool overridden = operation.cutting.spindle_rpm && operation.cutting.chip_load &&
                          operation.cutting.plunge_feed;
  if (data) {
    ImGui::TextDisabled("From %s in %s", tool->name.c_str(),
                        material ? material->name.c_str() : app.effective_context().material_id.c_str());
  } else if (overridden) {
    ImGui::TextDisabled("Set on this operation; %s has no data for %s", tool->name.c_str(),
                        material ? material->name.c_str() : "the job material");
  } else {
    ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                       "%s has no cutting data for %s.", tool->name.c_str(),
                       material ? material->name.c_str() : "the job material");
    if (ImGui::SmallButton("Add it to the tool")) {
      app.builder_tool = *tool;
      app.builder_tool.cutting.push_back({app.effective_context().material_id});
      app.builder_is_new = false;
      app.builder_open = true;
    }
  }
  if (begin_properties("feeds")) {
    auto inherited = [&](double cam::CuttingData::*field) {
      return data ? std::optional<double>(data->*field) : std::nullopt;
    };
    prop_override("Spindle", operation.cutting.spindle_rpm,
                  inherited(&cam::CuttingData::spindle_rpm), 10.0f, "%.0f rpm",
                  "Clamped and rounded to what the machine can command.");
    prop_override("Chip load", operation.cutting.chip_load,
                  inherited(&cam::CuttingData::chip_load), 0.0005f, "%.4f mm/tooth",
                  "Feed per tooth at full radial engagement. Lighter cuts run "
                  "faster to keep the same chip.");
    prop_override("Plunge feed", operation.cutting.plunge_feed,
                  inherited(&cam::CuttingData::plunge_feed), 5.0f, "%.0f mm/min");
    prop_check("Coolant", operation.coolant);
    end_properties();
  }
  if (machine) {
    try {
      const cam::CuttingFeeds feeds = cam::resolve_cutting_feeds(
          *tool, *machine, app.effective_context().material_id, operation.cutting);
      ImGui::TextDisabled("Full engagement %.0f mm/min, light cuts up to %.0f",
                          feeds.full_feed(), feeds.travel_feed());
      if (feeds.full_feed() >= feeds.maximum_feed)
        ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                           "Capped at the machine's %.0f mm/min.", feeds.maximum_feed);
    } catch (const std::exception&) {
    }
  }
}

void result_section(App& app, std::size_t index) {
  if (index >= app.plan.operations.size()) return;
  const cam::OperationPlan& planned = app.plan.operations[index];
  if (!planned.planned) return;
  if (!ImGui::CollapsingHeader("Result", ImGuiTreeNodeFlags_DefaultOpen)) return;
  if (!planned.ok()) {
    ImGui::PushTextWrapPos();
    ImGui::TextColored(severity_colour(app, LogSeverity::Error), "%s",
                       planned.error.c_str());
    ImGui::PopTextWrapPos();
    return;
  }
  if (out_of_date(plan_state(app, index)))
    ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                       "Settings changed since this was planned.");
  if (begin_properties("result")) {
    prop_value("Time", "%.1f min", planned.stats.cutting_minutes +
                                        planned.stats.rapid_minutes +
                                        planned.stats.tool_change_minutes);
    prop_value("Passes", "%zu of %zu cut", planned.stats.cut_levels, planned.stats.levels);
    prop_value("Removed", "%.0f mm3", planned.stats.removed_volume);
    prop_value("Peak engagement", "%.0f%%", planned.stats.maximum_load * 100.0);
    if (planned.stats.gouge_area > 0.0)
      prop_value("Gouge", "%.3g mm2", planned.stats.gouge_area);
    end_properties();
  }
  ImGui::PushTextWrapPos();
  for (const auto& warning : planned.warnings)
    ImGui::TextColored(severity_colour(app, LogSeverity::Warning), "%s", warning.c_str());
  ImGui::PopTextWrapPos();
}

}  // namespace

void add_operation_menu(App& app);

void operation_panel(App& app) {
  if (app.job.operations.empty() || app.selected_operation < 0) {
    ImGui::TextDisabled("No operation selected.");
    if (!app.planner.running() && ImGui::Button("Add operation"))
      ImGui::OpenPopup("add-first");
    if (ImGui::BeginPopup("add-first")) {
      add_operation_menu(app);
      ImGui::EndPopup();
    }
    return;
  }
  cam::Operation& operation = app.job.operations[app.selected_operation];
  bool insert_pre_drill=false;
  const cam::Tool* tool = app.tools.find(operation.tool_id);
  const bool mesh_mode = operation.mode == cam::OperationMode::Roughing ||
                         operation.mode == cam::OperationMode::ContourFinishing;

  if (begin_properties("identity")) {
    prop_text("Name", operation.name);
    prop_enum("Type", operation.mode, operation_mode_label, kOperationModes,
              std::size(kOperationModes));
    prop_library("Tool", operation.tool_id, app.tools.tools,
                 [](const cam::Tool& t) { return t.name.c_str(); });
    end_properties();
  }
  if (tool) {
    ImGui::TextDisabled("%s, %.3f mm, %d flute%s, %.1f mm flute length",
                        cam::tool_type_name(tool->type), tool->diameter, tool->flutes,
                        tool->flutes == 1 ? "" : "s", tool->flute_length);
    const bool needs_ball = operation.mode == cam::OperationMode::SurfaceFinishing;
    const bool needs_flat = operation.mode==cam::OperationMode::Pocket || operation.mode==cam::OperationMode::PlaneClearing;
    if (needs_ball && tool->type != cam::ToolType::BallEndMill)
      ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                         "Surface finishing needs a ball end mill.");
    if (needs_flat && tool->type != cam::ToolType::FlatEndMill)
      ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                         "Pockets and plane clearing need a flat end mill.");
  }

  if (ImGui::CollapsingHeader("Feeds and speeds", ImGuiTreeNodeFlags_DefaultOpen))
    feeds_section(app, operation, tool);

  if (ImGui::CollapsingHeader("Cutting", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (operation.mode == cam::OperationMode::SurfaceFinishing) {
      surface_section(app, operation, tool);
    } else if(operation.mode==cam::OperationMode::Hole) {
      insert_pre_drill=holes_section(app,operation);
    } else {
      if (begin_properties("cutting")) {
        double percent = operation.stepover_ratio * 100.0;
        char format[64];
        std::snprintf(format, sizeof format, "%%.0f%%%% of diameter (%.3f mm)",
                      operation.stepover_ratio * (tool ? tool->diameter : 0.0));
        if (prop_double("Stepover", percent, 0.5f, format,
                        "Radial width of cut as a share of the tool diameter.", 1.0,
                        100.0))
          operation.stepover_ratio = percent / 100.0;
        prop_double("Stepdown", operation.stepdown, 0.1f, "%.3f mm",
                    "Axial depth of each pass.", 0.001);
        if (operation.mode == cam::OperationMode::Roughing)
          roughing_settings(app, operation, tool);
        if (operation.mode == cam::OperationMode::ContourFinishing) {
          static constexpr cam::ContourDirectionPolicy kDirections[]{
              cam::ContourDirectionPolicy::Either, cam::ContourDirectionPolicy::Climb,
              cam::ContourDirectionPolicy::Conventional};
          prop_enum("Direction", operation.contour_direction,
                    cam::contour_direction_name, kDirections, std::size(kDirections));
          prop_check("Clip to stock", operation.clip_contours_to_stock,
                     "Only cut contours where there is stock.");
        }
        end_properties();
      }
      if (operation.mode == cam::OperationMode::Roughing) roughing_areas_section(app,operation,tool);
      if (operation.mode == cam::OperationMode::Pocket) pockets_section(app, operation);
      if (operation.mode == cam::OperationMode::PlaneClearing) areas_section(app, operation);
    }
  }

  if (mesh_mode && ImGui::CollapsingHeader("Depths", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_properties("depths")) {
    prop_depth(app, "Top Z", operation.depth_from,
               "Where cutting starts, in program coordinates.");
    prop_depth(app, "Bottom Z", operation.depth_to, "The deepest floor.");
    prop_double("Radial stock", operation.radial_stock_to_leave, 0.02f, "%.3f mm",
                "Left on walls: the part outline is offset by the cutter radius "
                "plus this.", 0.0);
    prop_double("Axial stock", operation.axial_stock_to_leave, 0.02f, "%.3f mm",
                "Left on floors the passes come down onto.");
    if (operation.mode == cam::OperationMode::Roughing) depth_breaks(app, operation);
    end_properties();
    if (tool) {
      const std::size_t layers =
          cam::level_floors(operation, operation.stepdown, cam::highest_model_z(app.geometry)).size();
      ImGui::TextDisabled("%zu depth%s", layers, layers == 1 ? "" : "s");
    }
  }

  if (ImGui::CollapsingHeader("Advanced") && begin_properties("advanced")) {
    prop_double("Sample spacing", operation.path_pitch, 0.01f, "%.3f mm",
                "Paths are sampled at this spacing to measure engagement and "
                "assign feeds.", 0.001);
    if (mesh_mode) {
      prop_int("Slice grid", operation.slice_grid, 1.0f, 1, 256,
               "Resolution of the mesh slicer's acceleration grid.");
    }
    if (operation.mode == cam::OperationMode::Roughing) {
      prop_double("Stock boundary dilation", operation.stock_boundary_dilation, 0.05f, "%.3f mm",
                  "Extra XY planning margin around stock at every roughing depth. "
                  "1 mm expands a 20 x 20 mm square to 22 x 22 mm. "
                  "Material-removal simulation uses the loaded stock geometry.", 0.0);
      prop_double("Stock oversize", operation.stock_deviation, 0.01f, "%.3f mm",
                  "How much larger than its model the blank may be; the lead-in "
                  "leaves this much headroom on the first bite.", 0.0);
      prop_double("Cleanup tolerance", operation.cleanup_tolerance, 0.01f, "%.3f mm",
                  "Allowance taken off the cutter radius when deciding whether a "
                  "cleanup contour is close enough to remaining stock.", 0.0,
                  tool ? tool->diameter * 0.5 : 10.0);
      prop_double("Cleanup stepover limit", operation.cleanup_stepover_limit, 0.01f,
                  "%.2f x diameter", "Ceiling for the adapted cleanup stepover.",
                  0.01, 1.0);
      prop_double("Entry-front Z match", operation.entry_front_match_tolerance, 0.01f,
                  "%.2f mm", "Envelope for associating entry curves between depths.",
                  0.001);
      prop_double("Tiny split area", operation.tiny_split_area_factor, 0.01f,
                  "%.2f x stepover^2", "Split branches smaller than this stay in the "
                  "wave without becoming separate nodes. Zero disables.", 0.0);
    }
    end_properties();
  }

  ImGui::Spacing();
  if (ImGui::Button("Use as default")) {
    cam::Operation defaults = operation;
    cam::operation_default(app.settings, operation.mode) = defaults;
    save_settings(app);
    say(app, std::string("New ") + operation_mode_label(operation.mode) +
                 " operations will start from these settings");
  }
  ImGui::SetItemTooltip("New operations of this type start from these settings.");

  result_section(app, static_cast<std::size_t>(app.selected_operation));
  if(insert_pre_drill) add_pre_drilling(app,app.selected_operation);
}

// Plans operations [first, last]. The operations above the range are handed
// over as pointers into the plan already on screen.
void start_plan(App& app, int first, int last) {
  if (app.model_loading || app.hole_detector.running()) return;
  poll_hole_detection(app);
  cam::ensure_setup_ids(app.job);
  const int count = static_cast<int>(app.job.operations.size());
  if (count == 0) return;
  first = std::clamp(first, 0, count - 1);
  last = std::clamp(last, first, count - 1);

  app.gcode.clear();
  app.post = {};
  log_message(app, LogSeverity::Info, "Planning",
              "Planning operations " + std::to_string(first + 1) + " to " +
                  std::to_string(last + 1));
  app.planning_first = static_cast<std::size_t>(first);
  app.planning_signatures.clear();
  const std::string job_part = job_signature(app);
  for (int i = first; i <= last; ++i)
    app.planning_signatures.push_back(
        operation_signature(app, static_cast<std::size_t>(i), job_part));

  cam::PriorOperations earlier;
  earlier.reserve(static_cast<std::size_t>(first));
  for (int i = 0; i < first; ++i) earlier.push_back(&app.plan.operations[i]);

  cam::JobGeometry& geometry = app.geometry;
  app.planner.start([&geometry, job = app.job, tools = app.tools,
                     machines = app.machines, materials = app.materials,
                     earlier = std::move(earlier), first, last](
                        const cam::ProgressCallback& progress) {
    return cam::plan_operation_range(geometry, job, tools, machines, materials,
                                     progress, static_cast<std::size_t>(first),
                                     static_cast<std::size_t>(last), earlier);
  });
}

void take_plan_result(App& app, cam::PartialPlan produced, bool screenshot) {
  if (!produced.error.empty()) {
    if (produced.error != "Cancelled") {
      std::fprintf(stderr, "Planning failed:\n%s\n", produced.error.c_str());
      say(app, produced.error, true);
    } else {
      say(app, "Planning cancelled");
    }
    app.planning_signatures.clear();
    return;
  }
  // Each planned operation takes its own entry. The signature recorded is the
  // one taken when the run was asked for, so an edit made meanwhile shows.
  const std::uint64_t serial = ++app.plan_serial;
  app.result_preview = {};
  std::size_t planned_count = 0;
  for (std::size_t i = 0; i < produced.operations.size(); ++i) {
    const std::size_t at = produced.first + i;
    if (at >= app.plan.operations.size()) break;
    app.plan.operations[at] = std::move(produced.operations[i]);
    app.planned[at] = {i < app.planning_signatures.size() ? app.planning_signatures[i]
                                                          : std::string(),
                       serial};
    if (app.plan.operations[at].ok()) ++planned_count;
    else
      log_message(app, LogSeverity::Error, app.plan.operations[at].operation_name,
                  app.plan.operations[at].error);
  }
  app.planning_signatures.clear();
  app.plan.rapid_plane = produced.rapid_plane;
  cam::summarise_job(app.plan);
  for (const auto& operation : app.plan.operations)
    if (operation.planned && operation.ok()) {
      log_warnings(app, operation.operation_name.c_str(), operation.warnings);
      for (const auto& message : operation.debug_messages)
        log_message(app, LogSeverity::Debug, operation.operation_name, message);
    }
  refresh_plan_states(app);
  refresh_inherited_stock(app);
  select_operation(app, app.selected_operation);
  app.view.needs_fit = true;
  app.material.clear();
  app.material_distance = -1.0;
  app.material_level = -1;
  if (planned_count == 0) {
    say(app, "No operation in that range could be planned", true);
  } else if (!screenshot) {
    app.force_tab = 2;
    char text[128];
    std::snprintf(text, sizeof text, "Planned %zu operation%s: %.1f min",
                  planned_count, planned_count == 1 ? "" : "s",
                  app.plan.stats.cutting_minutes + app.plan.stats.rapid_minutes +
                      app.plan.stats.tool_change_minutes);
    say(app, text);
  }
}

bool plannable(const App& app, const cam::Operation& operation) {
  if(cam::is_parametric(operation.mode)) return true;
  if(app.job.setups.empty()) return app.geometry.has_part() && app.geometry.has_stock();
  try {
    const auto c=cam::resolve_setup_context(app.job,operation.setup_id);
    return !c.part_mesh.empty() && (c.box_stock || !c.stock_mesh.empty() || !c.stock_from_operation.empty()) &&
        (operation.mode!=cam::OperationMode::SurfaceFinishing || cam::step_filename(c.part_mesh));
  } catch(const std::exception&) {return false;}
}

namespace {
bool plannable_range(const App& app, int first, int last) {
  for (int i = first; i <= last && i < static_cast<int>(app.job.operations.size()); ++i)
    if (!plannable(app, app.job.operations[i])) return false;
  return true;
}
}  // namespace

void plan_out_of_date(App& app) {
  if (app.job.operations.empty()) return;
  const int last = static_cast<int>(app.job.operations.size()) - 1;
  const int from = first_out_of_date(app, last);
  if (from < 0) {
    say(app, "Every operation already matches the plan");
    return;
  }
  if (!plannable_range(app, from, last)) {
    say(app, "Load a part to plan mesh operations", true);
    return;
  }
  start_plan(app, from, last);
}

void plan_buttons(App& app) {
  if (app.planner.running()) {
    const auto progress = app.planner.progress();
    const float completed = static_cast<float>(std::clamp(progress.fraction, 0.0, 1.0));
    const float cancel_width = ImGui::CalcTextSize("Cancel").x +
                               ImGui::GetStyle().FramePadding.x * 2.0f;
    char label[32];
    std::snprintf(label, sizeof label, "%.0f%%", 100.0 * completed);
    ImGui::ProgressBar(completed,
                       ImVec2(ImGui::GetContentRegionAvail().x - cancel_width -
                                  ImGui::GetStyle().ItemSpacing.x, 0),
                       label);
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) app.planner.cancel();
    ImGui::TextDisabled("%s", app.planner.stage().c_str());
    return;
  }
  if (app.job.operations.empty()) return;
  const int selected = std::clamp(app.selected_operation, 0,
                                  static_cast<int>(app.job.operations.size()) - 1);
  const std::vector<std::size_t> chosen = selected_operation_indices(app);
  const int chosen_first = chosen.empty()?0:static_cast<int>(chosen.front());
  const int chosen_last = chosen.empty()?0:static_cast<int>(chosen.back());
  const int last = static_cast<int>(app.job.operations.size()) - 1;
  // Planning one on its own is only meaningful when everything above it has a
  // result to hand it stock.
  int missing = -1;
  for (int i = 0; i < chosen_first; ++i)
    if (!app.plan.operations[i].planned) { missing = i; break; }
  const int upto_from = first_out_of_date(app, selected);
  const int all_from = first_out_of_date(app, last);
  const bool can_checked = !chosen.empty() && missing < 0 && plannable_range(app, chosen_first, chosen_last);
  const bool can_upto = app.selected_operation>=0 && upto_from >= 0 && plannable_range(app, upto_from, selected);
  const bool can_all = all_from >= 0 && plannable_range(app, all_from, last);
  const bool any_checked = std::any_of(app.operation_selection.begin(),
                                       app.operation_selection.end(),
                                       [](char c) { return c != 0; });

  const float width = (ImGui::GetContentRegionAvail().x -
                       ImGui::GetStyle().ItemSpacing.x * 2.0f) / 3.0f;
  ImGui::BeginDisabled(!can_all);
  if (ImGui::Button("Plan all", ImVec2(width, 0))) start_plan(app, all_from, last);
  ImGui::EndDisabled();
  ImGui::SetItemTooltip("Plan every out-of-date operation (F5).");
  ImGui::SameLine();
  ImGui::BeginDisabled(!can_upto);
  if (ImGui::Button("Up to selected", ImVec2(width, 0))) start_plan(app, upto_from, selected);
  ImGui::EndDisabled();
  ImGui::SetItemTooltip("Plan out-of-date operations down to the selected one.");
  ImGui::SameLine();
  ImGui::BeginDisabled(!can_checked);
  if (ImGui::Button(any_checked ? "Checked" : "Selected", ImVec2(width, 0)))
    start_plan(app, chosen_first, chosen_last);
  ImGui::EndDisabled();
  ImGui::SetItemTooltip("Plan only these, plus anything between them, using the "
                        "stock the operations above already left.");
  if (missing >= 0)
    ImGui::TextDisabled("\"%s\" has no result yet, so nothing below it can be "
                        "planned on its own.", app.job.operations[missing].name.c_str());
  else if (!plannable_range(app, 0, last))
    ImGui::TextDisabled("Mesh operations need a part model.");
}

void plan_summary(App& app) {
  if (!app.has_plan) return;
  if (!ImGui::CollapsingHeader("Plan summary", ImGuiTreeNodeFlags_DefaultOpen)) return;
  const std::size_t stale = stale_operations(app);
  if (stale > 0)
    ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                       "%zu of %zu operations changed since planning.", stale,
                       app.job.operations.size());
  const cam::PlanStats& stats = app.plan.stats;
  if (begin_properties("summary")) {
    prop_value("Job time", "%.1f min",
               stats.cutting_minutes + stats.rapid_minutes + stats.tool_change_minutes);
    prop_value("Cutting", "%.0f mm in %.1f min", stats.cutting_distance,
               stats.cutting_minutes);
    prop_value("Travel", "%.0f mm in %.1f min", stats.rapid_distance, stats.rapid_minutes);
    prop_value("Removed", "%.0f mm3", stats.removed_volume);
    prop_value("Peak engagement", "%.0f%%", stats.maximum_load * 100.0);
    end_properties();
  }
  const ImVec4 bad = severity_colour(app, LogSeverity::Error);
  const ImVec4 caution = severity_colour(app, LogSeverity::Warning);
  if (stats.gouge_area > 0.0) ImGui::TextColored(bad, "Gouges: %.3g mm2", stats.gouge_area);
  if (stats.unsafe_plunges > 0)
    ImGui::TextColored(bad, "Roughing plunges onto stock: %zu", stats.unsafe_plunges);
  if (stats.skipped_entries > 0)
    ImGui::TextColored(caution, "Roughing entries skipped: %zu", stats.skipped_entries);
  if (stats.lifts_over_subject > 0)
    ImGui::TextColored(caution, "Rapids over the part: %zu", stats.lifts_over_subject);
  if (stats.unreachable_fronts > 0)
    ImGui::TextColored(caution, "Fronts never reached: %zu", stats.unreachable_fronts);
}

}  // namespace tinglecam_gui
