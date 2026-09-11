#include "gui_internal.h"
namespace tinglecam_gui {
void operation_settings_panel(App& app) {
  ImGui::SeparatorText("Selected operation");
  if (app.job.operations.empty()) {
    ImGui::TextDisabled("Add an operation first");
    return;
  }

  cam::Operation& operation = app.job.operations[app.selected_operation];
  text_field("Operation name", operation.name);
  enum_combo("Mode", operation.mode, operation_mode_label, kOperationModes,
             std::size(kOperationModes));
  if (operation.mode == cam::OperationMode::ContourFinishing) {
    static constexpr cam::ContourDirectionPolicy kDirections[]{
        cam::ContourDirectionPolicy::Either,
        cam::ContourDirectionPolicy::Climb,
        cam::ContourDirectionPolicy::Conventional,
    };
    enum_combo("Contour direction", operation.contour_direction,
               cam::contour_direction_name, kDirections,
               std::size(kDirections));
    ImGui::Checkbox("Clip contours to nominal stock",
                    &operation.clip_contours_to_stock);
  } else if (operation.mode == cam::OperationMode::Roughing) {
    ImGui::Checkbox("Boundary cleanup contour",
                    &operation.rough_boundary_cleanup);
  }

  const auto pick = [&](const char* label, std::string& id, const auto& entries,
                        auto name_of) {
    int index = -1;
    std::vector<const char*> labels;
    for (std::size_t i = 0; i < entries.size(); ++i) {
      labels.push_back(name_of(entries[i]));
      if (entries[i].id == id) index = static_cast<int>(i);
    }
    if (labels.empty()) {
      ImGui::TextDisabled("%s: nothing available", label);
      return;
    }
    if (index < 0) index = 0;
    if (ImGui::Combo(label, &index, labels.data(), static_cast<int>(labels.size())))
      id = entries[index].id;
    else if (id.empty())
      id = entries[index].id;
  };

  pick("Tool", operation.tool_id, app.tools.tools,
       [](const cam::Tool& tool) { return tool.name.c_str(); });
  pick("Machine", operation.machine_id, app.machines.machines,
       [](const cam::Machine& machine) { return machine.name.c_str(); });
  pick("Material", operation.material_id, app.materials.materials,
       [](const cam::Material& material) { return material.name.c_str(); });

  const cam::Tool* tool = app.tools.find(operation.tool_id);
  const cam::Machine* machine = app.machines.find(operation.machine_id);
  if (machine && machine->specifications_unverified)
    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f),
                       "This machine profile carries unverified limits");

  ImGui::InputDouble("Stepover (of diameter)", &operation.stepover_ratio, 0, 0, "%.6g");
  ImGui::InputDouble("Stepdown (mm)", &operation.stepdown, 0, 0, "%.6g");
  if (cam::is_parametric(operation.mode)) {
    if (operation.mode == cam::OperationMode::Pocket) {
      if (operation.pockets.empty()) operation.pockets.push_back(cam::PocketParameters{});
      ImGui::TextWrapped(
          "Rounded rectangles cover square, rounded, and circular pockets. "
          "Each pocket may have its own position and depth.");
      int remove = -1;
      for (std::size_t i = 0; i < operation.pockets.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        ImGui::SeparatorText(("Pocket " + std::to_string(i + 1)).c_str());
        auto& p = operation.pockets[i];
        origin_field("Center X (mm)", p.center_x, app.job.origin_x);
        origin_field("Center Y (mm)", p.center_y, app.job.origin_y);
        origin_field("Top Z (mm)", p.top_z, app.job.origin_z);
        ImGui::InputDouble("Width X (mm)", &p.width, 0, 0, "%.6g");
        ImGui::InputDouble("Height Y (mm)", &p.height, 0, 0, "%.6g");
        ImGui::InputDouble("Corner radius (mm)", &p.radius, 0, 0, "%.6g");
        ImGui::InputDouble("Pocket depth (mm)", &p.depth, 0, 0, "%.6g");
        ImGui::InputDouble("Helix angle (degrees)", &p.helix_angle_degrees,
                           0, 0, "%.6g");
        ImGui::TextWrapped("Floor Z: %.6g mm. Every level starts with a helical entry.",
                           p.top_z - p.depth + app.job.origin_z);
        if (ImGui::Button("Remove pocket")) remove = static_cast<int>(i);
        ImGui::PopID();
      }
      if (remove >= 0) operation.pockets.erase(operation.pockets.begin() + remove);
      if (ImGui::Button("Add pocket")) {
        cam::PocketParameters pocket;
        pocket.center_x = -app.job.origin_x;
        pocket.center_y = -app.job.origin_y;
        pocket.top_z = -app.job.origin_z;
        operation.pockets.push_back(pocket);
      }
    } else {
      auto& p = operation.plane;
      ImGui::TextWrapped(
          "Each area has its own top Z and depth. The cutter descends beside "
          "the area and faces the whole surface, corners included.");
      int remove = -1;
      for (std::size_t i = 0; i < p.areas.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        ImGui::SeparatorText(("Area " + std::to_string(i+1)).c_str());
        auto& area = p.areas[i];
        origin_field("Center X (mm)", area.center_x, app.job.origin_x);
        origin_field("Center Y (mm)", area.center_y, app.job.origin_y);
        ImGui::InputDouble("Width X (mm)", &area.width, 0, 0, "%.6g");
        ImGui::InputDouble("Height Y (mm)", &area.height, 0, 0, "%.6g");
        origin_field("Top Z (mm)", area.top_z, app.job.origin_z);
        ImGui::InputDouble("Clearing depth (mm)", &area.depth, 0, 0, "%.6g");
        ImGui::TextWrapped("Floor Z: %.6g mm.",
                           area.top_z - area.depth + app.job.origin_z);
        if (ImGui::Button("Remove area")) remove = static_cast<int>(i);
        ImGui::PopID();
      }
      if (remove >= 0) p.areas.erase(p.areas.begin()+remove);
      if (ImGui::Button("Add area")) p.areas.push_back(cam::ClearingArea{});
    }
    ImGui::TextWrapped(
        "Positions are measured from the program origin, so they are the "
        "numbers the machine will be sent. Moving the origin restates them and "
        "leaves the operation where it is; sizes and depths are not measured "
        "from anywhere and do not change. No part mesh is required.");
    ImGui::InputDouble("Simulation/feed pitch (mm)", &operation.path_pitch, 0, 0, "%.6g");
    feeds_panel(app);
    return;
  }
  const std::size_t base_layers =
      cam::level_floors(operation, operation.stepdown,
                        cam::highest_model_z(app.geometry))
          .size();
  if (operation.mode == cam::OperationMode::Roughing) {
    if (ImGui::DragInt("Finishing subdivisions",
                       &operation.finishing_subdivisions, 0.05f, 1, 64))
      operation.finishing_subdivisions =
          std::clamp(operation.finishing_subdivisions, 1, 64);
    ImGui::SetItemTooltip(
        "Runs this roughing operation repeatedly at evenly interleaved depth "
        "offsets. Each pass receives the stock left by the preceding pass.");
    const double minimum_cleanup_tolerance = 0.0;
    const double maximum_cleanup_tolerance =
        tool ? tool->diameter * 0.5 : 10.0;
    ImGui::BeginDisabled(!operation.rough_boundary_cleanup);
    if (ImGui::SliderScalar(
            "Cleanup tolerance (mm)", ImGuiDataType_Double,
            &operation.cleanup_tolerance, &minimum_cleanup_tolerance,
            &maximum_cleanup_tolerance, "%.3f"))
      operation.cleanup_tolerance = std::clamp(
          operation.cleanup_tolerance, minimum_cleanup_tolerance,
          maximum_cleanup_tolerance);
    ImGui::SetItemTooltip(
        "Numerical allowance excluded from the cutter radius when deciding "
        "whether a finishing-subdivision cleanup contour is close enough to "
        "remaining stock. The effective distance is cutter radius minus this "
        "value; setting it to the cutter radius keeps the strict wave-only "
        "rule.");
    ImGui::EndDisabled();
  }
  if (tool) {
    const std::string preview_layers =
        operation.mode == cam::OperationMode::Roughing &&
                operation.finishing_subdivisions > 1
            ? ", " +
                  std::to_string(base_layers *
                                 operation.finishing_subdivisions) +
                  " preview layers"
            : "";
    ImGui::TextWrapped(
        "= %.3f mm wide, %zu base layers%s",
        operation.stepover_ratio * tool->diameter, base_layers,
        preview_layers.c_str());
  }
  ImGui::DragScalar("Radial stock to leave", ImGuiDataType_Double,
                    &operation.radial_stock_to_leave, 0.02f);
  ImGui::SetItemTooltip(
      "Stock left on walls: the part outline is offset by the cutter radius "
      "plus this.");
  ImGui::DragScalar("Axial stock to leave", ImGuiDataType_Double,
                    &operation.axial_stock_to_leave, 0.02f);
  ImGui::SetItemTooltip(
      "When considering accessible cutting area, the part's Z is shifted up "
      "by this amount.");
  ImGui::DragScalar("Depth from (mm)", ImGuiDataType_Double,
                    &operation.depth_from, 0.1f);
  ImGui::DragScalar("Depth to (mm)", ImGuiDataType_Double, &operation.depth_to,
                    0.1f);
  ImGui::SetItemTooltip(
      "Measured down from the top of the work -- the taller of the part and the "
      "blank -- so a blank standing proud of the part is faced off rather than "
      "left standing. Where the origin sits does not enter into it.");
  if (operation.mode == cam::OperationMode::Roughing) {
    ImGui::Checkbox("Lead in from the air", &operation.lead_in);
    ImGui::SetItemTooltip(
        "Seeds the wave outside the stock so the cutter arrives at the wall along "
        "a path rather than at a standstill. Without it the first pass runs down "
        "the stock wall itself and buries half the cutter.");
    ImGui::BeginDisabled(!operation.lead_in);
    ImGui::DragScalar("Stock oversize (mm)", ImGuiDataType_Double,
                      &operation.stock_deviation, 0.01f);
    ImGui::SetItemTooltip(
        "How much larger than its model the blank may be. The run-in is placed so "
        "the first pass to touch the stock aims for a full stepover less this, "
        "which leaves that much headroom for an oversize blank.");
    if (tool) {
      const double bite = operation.stepover_ratio * tool->diameter;
      const double first = std::max(
          bite - std::clamp(operation.stock_deviation, 0.0, bite * 0.9), 0.0);
      ImGui::TextDisabled("= target first bite %.3f mm, then %.3f mm", first,
                          bite);
    }
    ImGui::EndDisabled();
  }

  ImGui::DragScalar("Simulation/feed pitch (mm)", ImGuiDataType_Double,
                    &operation.path_pitch, 0.01f);
  ImGui::SetItemTooltip(
      "The complete at-depth path is resampled at this travelled-distance "
      "interval before simulation and feed assignment.");
  if (operation.mode == cam::OperationMode::Roughing) {
    ImGui::DragScalar("Entry-front Z match (mm)", ImGuiDataType_Double,
                      &operation.entry_front_match_tolerance, 0.01f);
    ImGui::SetItemTooltip(
        "The complete lower-layer entry curve must fit in this envelope around "
        "the upper curve. Node-area Z matching remains exact.");
    ImGui::DragScalar("Tiny split area (stepover^2)", ImGuiDataType_Double,
                      &operation.tiny_split_area_factor, 0.01f);
    ImGui::SetItemTooltip(
        "A terminal split branch smaller than this multiple of stepover squared "
        "stays in the wave but does not create a separate node. Zero disables "
        "the filter.");
  }

  feeds_panel(app);
}

void feeds_panel(App& app) {
  if (app.job.operations.empty()) return;
  cam::Operation& operation = app.job.operations[app.selected_operation];
  const cam::Machine* machine = app.machines.find(operation.machine_id);
  const cam::MaterialFeedRange* library =
      machine ? machine->feeds_for(operation.material_id) : nullptr;
  ImGui::SeparatorText("Operation cutting settings");
  ImGui::InputDouble("Spindle (rpm)", &operation.spindle_rpm, 0, 0, "%.6g");
  ImGui::InputDouble("Plunge / ramp feed (mm/min)", &operation.plunge_feed, 0, 0, "%.6g");
  ImGui::Checkbox("Coolant", &operation.coolant);
  if (ImGui::CollapsingHeader("Machine tool assignment")) {
    ImGui::InputInt("Tool number (0 = auto)", &operation.tool_number);
    ImGui::InputInt("H offset (0 = tool)", &operation.length_offset);
  }
  ImGui::SeparatorText("Engagement feeds");
  ImGui::TextWrapped(
      "Feed is interpolated directly between these endpoints using the "
      "simulation's engagement measurement.");
  ImGui::Checkbox("Enter manually", &operation.manual_feed_range);
  if (!operation.manual_feed_range && library) {
    ImGui::Text("Full engagement  %.0f mm/min", library->full_engagement);
    ImGui::Text("No engagement    %.0f mm/min", library->no_engagement);
  } else if (!operation.manual_feed_range) {
    ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                       "No library range for this machine and material");
  }
  ImGui::BeginDisabled(!operation.manual_feed_range);
  ImGui::DragScalar("Full engagement (mm/min)", ImGuiDataType_Double,
                    &operation.full_engagement_feed, 10.0f);
  ImGui::DragScalar("No engagement (mm/min)", ImGuiDataType_Double,
                    &operation.no_engagement_feed, 10.0f);
  ImGui::EndDisabled();
}

// Plans operations [first, last] and nothing else. The operations above the
// range are handed over as pointers into the plan already on screen: they are
// where the stock the range inherits comes from, and copying them would cost a
// good part of the work not re-planning them exists to save. Nothing may
// disturb that list until the run finishes, which is why the buttons that
// reorder it are disabled while it does.
void start_plan(App& app, int first, int last) {
  const int count = static_cast<int>(app.job.operations.size());
  if (count == 0) return;
  first = std::clamp(first, 0, count - 1);
  last = std::clamp(last, first, count - 1);

  app.gcode.clear();
  app.post = {};
  app.planning_first = static_cast<std::size_t>(first);
  app.planning_signatures.clear();
  const std::string job_part = job_signature(app);
  for (int i = first; i <= last; ++i)
    app.planning_signatures.push_back(
        job_part + cam::to_json_text(app.job.operations[i]));

  cam::PriorOperations earlier;
  earlier.reserve(static_cast<std::size_t>(first));
  for (int i = 0; i < first; ++i) earlier.push_back(&app.plan.operations[i]);

  cam::JobGeometry& geometry = app.geometry;
  cam::Job job = app.job;
  cam::ToolLibrary tools = app.tools;
  cam::MachineLibrary machines = app.machines;
  cam::MaterialLibrary materials = app.materials;
  app.planner.start([&geometry, job = std::move(job), tools = std::move(tools),
                     machines = std::move(machines),
                     materials = std::move(materials),
                     earlier = std::move(earlier), first, last](
                        const cam::ProgressCallback& progress) {
    return cam::plan_operation_range(geometry, job, tools, machines, materials,
                                     progress, static_cast<std::size_t>(first),
                                     static_cast<std::size_t>(last), earlier);
  });
}

// Can this operation be planned at all, whatever the state of the rest?
bool plannable(const App& app, const cam::Operation& operation) {
  return cam::is_parametric(operation.mode) ||
         (app.geometry.has_part() && app.geometry.has_stock());
}

bool plannable_range(const App& app, int first, int last) {
  for (int i = first; i <= last && i < static_cast<int>(app.job.operations.size());
       ++i)
    if (!plannable(app, app.job.operations[i])) return false;
  return true;
}

// Planning, at the operation list, because what gets planned is a question
// about that list. Checked rows form one ordered planning span; any operations
// between the first and last checked row are included because later operations
// inherit their stock from everything above them.
void plan_buttons(App& app) {
  ImGui::SeparatorText("Plan");
  if (app.planner.running()) {
    ImGui::ProgressBar(static_cast<float>(app.planner.fraction()), ImVec2(-1, 0));
    ImGui::TextWrapped("%s", app.planner.stage().c_str());
    if (ImGui::Button("Cancel")) app.planner.cancel();
    return;
  }
  if (app.job.operations.empty()) return;

  const int selected = std::clamp(app.selected_operation, 0,
                                  static_cast<int>(app.job.operations.size()) - 1);
  const std::vector<std::size_t> chosen = selected_operation_indices(app);
  const int chosen_first = static_cast<int>(chosen.front());
  const int chosen_last = static_cast<int>(chosen.back());
  const int last = static_cast<int>(app.job.operations.size()) - 1;

  // An operation is cut out of the stock the ones above it left, so planning
  // one on its own is only meaningful when all of them have a result to leave
  // it. A result that failed is not a gap: a failed operation removed nothing,
  // which is exactly what the planner takes it to have done.
  int missing = -1;
  for (int i = 0; i < chosen_first; ++i)
    if (!app.plan.operations[i].planned) { missing = i; break; }

  const int upto_from = first_out_of_date(app, selected);
  const int all_from = first_out_of_date(app, last);
  const bool can_select = missing < 0 &&
                          plannable_range(app, chosen_first, chosen_last);
  const bool can_upto = upto_from >= 0 && plannable_range(app, upto_from, selected);
  const bool can_all = all_from >= 0 && plannable_range(app, all_from, last);

  // Sized to their own labels with the slack shared out, rather than a third
  // of the row each: three equal thirds cut "Plan up to selected" in half.
  static const char* const kLabels[3] = {"Plan selected", "Plan up to selected",
                                         "Plan all"};
  float width[3];
  float needed = 0.0f;
  for (int i = 0; i < 3; ++i) {
    width[i] = ImGui::CalcTextSize(kLabels[i]).x +
               ImGui::GetStyle().FramePadding.x * 2.0f;
    needed += width[i];
  }
  const float slack = (ImGui::GetContentRegionAvail().x -
                       ImGui::GetStyle().ItemSpacing.x * 2.0f - needed) / 3.0f;
  for (int i = 0; i < 3; ++i) width[i] = std::max(width[i] + slack, width[i]);

  ImGui::BeginDisabled(!can_select);
  if (ImGui::Button(kLabels[0], ImVec2(width[0], 0)))
    start_plan(app, chosen_first, chosen_last);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!can_upto);
  if (ImGui::Button(kLabels[1], ImVec2(width[1], 0)))
    start_plan(app, upto_from, selected);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!can_all);
  if (ImGui::Button(kLabels[2], ImVec2(width[2], 0)))
    start_plan(app, all_from, last);
  ImGui::EndDisabled();

  ImGui::PushStyleColor(ImGuiCol_Text,
                        ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
  if (!plannable_range(app, 0, last))
    ImGui::TextWrapped(
        "Mesh operations require a part and stock; pocket/plane operations use "
        "their dimensions.");
  if (missing >= 0)
    ImGui::TextWrapped(
        "\"%s\" has no result yet, so the operations below it have no stock to "
        "be cut out of. Plan up to the selected operation instead.",
        app.job.operations[missing].name.c_str());
  if (chosen.size() > 1)
    ImGui::TextWrapped(
        "%zu operations selected; planning includes every operation from %d "
        "through %d so stock handoff remains valid.",
        chosen.size(), chosen_first + 1, chosen_last + 1);
  const std::size_t stale = stale_operations(app);
  if (stale == 0)
    ImGui::TextWrapped("Every operation matches the plan.");
  else
    ImGui::TextWrapped(
        "%zu operation%s out of date. The last plan stays on screen until they "
        "are planned again, and each button plans only what its span still "
        "needs.",
        stale, stale == 1 ? " is" : "s are");
  ImGui::PopStyleColor();
}

void plan_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("Plan");
  const std::size_t stale = stale_operations(app);
  if (stale > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f),
                       "%zu of %zu operations have changed since this plan. "
                       "The figures below are the plan's, not the job's.",
                       stale, app.job.operations.size());

  const cam::PlanStats& stats = app.plan.stats;
  ImGui::Separator();
  ImGui::Text("Passes        %zu of %zu cut", stats.cut_levels, stats.levels);
  ImGui::Text("Cutting       %.0f mm in %.1f min", stats.cutting_distance,
              stats.cutting_minutes);
  ImGui::Text("Travel        %.0f mm in %.1f min", stats.rapid_distance,
              stats.rapid_minutes);
  ImGui::Text("Removed       %.0f mm3", stats.removed_volume);
  ImGui::Text("Peak load     %.2f of a full slot", stats.maximum_load);
  ImGui::Text("Left standing %.0f mm2", stats.excess_area);
  ImGui::SetItemTooltip(
      "Stock still there when the operation finishes that is not part of the "
      "finished shape. Roughing always leaves some: the radial allowance is "
      "exactly this. A figure well above it is material the passes missed.");
  if (stats.gouge_area > 0.0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Gouges        %.3f mm2",
                       stats.gouge_area);
  if (stats.unsafe_plunges > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Unsafe plunges %zu",
                       stats.unsafe_plunges);
  if (stats.unresolved_entries > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Entries into solid %zu",
                       stats.unresolved_entries);
  if (stats.lifts_over_subject > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "Rapids over the part %zu",
                       stats.lifts_over_subject);
  if (stats.unreachable_fronts > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "Fronts never reached %zu",
                       stats.unreachable_fronts);

  show_warnings("plan-warnings", app.plan.warnings);

  const cam::LevelPlan* level = current_level(app);
  if (level)
    ImGui::Text("Spindle %.0f rpm", level->feeds.stats.spindle_rpm);
}
}  // namespace tinglecam_gui
