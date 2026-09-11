#include "gui_internal.h"
namespace tinglecam_gui {
void output_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("Output");
  // The plan is what gets posted, not the job. Keeping a plan through an edit
  // is what makes it possible to post one that no longer says what the panel
  // beside it says, so this is the one place that has to be blunt about it.
  if (stale_operations(app) > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f),
                       "This posts the plan as it was last computed. Edited "
                       "operations are not in it; re-plan first to include "
                       "them.");
  if (app.plan.operations.empty()) return;
  if (ImGui::Button("Generate all G-code", ImVec2(-1, 0))) {
    app.post = cam::post_job(app.plan, app.job, app.tools, app.machines);
    app.posted_operations.clear();
    log_warnings("G-code", app.post.warnings);
    app.gcode = app.post.text;
    if (!app.post.ok()) say(app, app.post.error, true);
    else say(app, "Generated " + std::to_string(app.post.line_count) + " lines");
  }
  const std::vector<std::size_t> selected_operations =
      selected_operation_indices(app);
  const bool selected_planned =
      !selected_operations.empty() &&
      std::all_of(selected_operations.begin(), selected_operations.end(),
                  [&](std::size_t index) {
                    return index < app.plan.operations.size() &&
                           app.plan.operations[index].planned &&
                           app.plan.operations[index].ok();
                  });
  ImGui::BeginDisabled(!selected_planned);
  if (ImGui::Button("Generate selected operations", ImVec2(-1, 0))) {
    app.post = cam::post_operations(app.plan, app.job, app.tools, app.machines,
                                    selected_operations);
    app.posted_operations.clear();
    if (app.post.ok())
      for (const std::size_t index : selected_operations)
        app.posted_operations.push_back(static_cast<int>(index));
    log_warnings("G-code", app.post.warnings);
    app.gcode = app.post.text;
    if (!app.post.ok()) say(app, app.post.error, true);
    else say(app, "Generated " + std::to_string(app.post.line_count) +
                       " lines for " +
                       std::to_string(selected_operations.size()) +
                       " selected operation(s)");
  }
  ImGui::EndDisabled();
  if (app.gcode.empty()) return;

  if (app.posted_operations.empty()) {
    ImGui::Text("Output: complete job");
  } else if (app.posted_operations.size() == 1) {
    const int index = app.posted_operations.front();
    ImGui::Text("Output: operation %d, %s", index + 1,
                app.job.operations[index].name.c_str());
  } else {
    ImGui::Text("Output: %zu selected operations", app.posted_operations.size());
  }
  ImGui::Text("%zu lines, %zu cutting moves, %zu rapids", app.post.line_count,
              app.post.cutting_moves, app.post.rapid_moves);
  ImGui::Text("Estimated %.1f minutes", app.post.estimated_minutes);
  show_warnings("post-warnings", app.post.warnings);

  ImGui::InputText("File", app.save_path.data(), app.save_path.size());
  if (ImGui::Button("Save")) {
    try {
      cam::write_file(app.save_path.data(), app.gcode);
      say(app, std::string("Wrote ") + app.save_path.data());
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  if (ImGui::CollapsingHeader("Preview")) {
    ImGui::BeginChild("gcode", ImVec2(0, ui_size(app, 220.0f)),
                      ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::TextUnformatted(app.gcode.c_str(),
                           app.gcode.c_str() + std::min<std::size_t>(app.gcode.size(), 60000));
    ImGui::EndChild();
  }
}

void library_panel(App& app) {
  ImGui::SeparatorText("Tools");
  ImGui::TextWrapped("Library file: %s", app.tools_file.string().c_str());
  if (ImGui::SmallButton("Copy library path"))
    ImGui::SetClipboardText(app.tools_file.string().c_str());
  ImGui::SameLine();
  ImGui::BeginDisabled(app.builder_open || app.planner.running());
  if (ImGui::SmallButton("Reload from disk")) {
    try {
      reload_tools(app);
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::TextWrapped("Starter tools are copied only on first run. To apply edits "
                     "to another JSON file, import it below. Matching IDs are "
                     "replaced; other tools are kept.");
  text_field("Import JSON", app.tools_import_file, 1024);
  if (ImGui::Button("Import tools")) {
    try {
      import_tools(app);
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::EndDisabled();
  ImGui::Separator();
  for (const auto& tool : app.tools.tools) {
    ImGui::PushID(tool.id.c_str());
    ImGui::Text("%s", tool.name.c_str());
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 90.0f);
    if (ImGui::SmallButton("Edit")) {
      app.builder_tool = tool;
      app.builder_is_new = false;
      app.builder_open = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) {
      app.builder_tool = tool;
      app.builder_tool.name += " copy";
      app.builder_is_new = true;
      app.builder_open = true;
    }
    ImGui::PopID();
  }
  if (ImGui::Button("New tool")) {
    app.builder_tool = cam::Tool{};
    app.builder_tool.name = "New tool";
    app.builder_is_new = true;
    app.builder_open = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%zu machines, %zu materials", app.machines.machines.size(),
                      app.materials.materials.size());
}

void view_panel(App& app) {
  ImGui::SeparatorText("Playback / Display");
  if (!app.has_plan || app.plan.operations.empty()) {
    ImGui::TextDisabled("Plan the job to enable playback and result display.");
    return;
  }
  app.selected_plan_operation = std::clamp(
      app.selected_plan_operation, 0,
      static_cast<int>(app.plan.operations.size()) - 1);
  const cam::OperationPlan& shown =
      app.plan.operations[app.selected_plan_operation];
  ImGui::Text("Operation: %s", shown.operation_name.c_str());
  if (!shown.planned) {
    ImGui::TextDisabled(
        "This operation has no result yet. Plan it from the operation list.");
    return;
  }
  if (stale_operations(app) > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f),
                       "From the last plan; the job has been edited since.");
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (!levels.empty()) {
    int last = static_cast<int>(levels.size()) - 1;
    app.selected_level = std::clamp(app.selected_level, 0, last);
    if (ImGui::SliderInt("Pass", &app.selected_level, 0, last)) {
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
    }
    ImGui::TextDisabled("Depth %.3f mm, cutting %.3f mm",
                        levels[app.selected_level].floor_z,
                        levels[app.selected_level].axial_depth);
    const cam::LevelPlan& selected = levels[app.selected_level];
    const auto& subdivisions =
        app.plan.operations[app.selected_plan_operation].subdivisions;
    if (subdivisions.size() > 1 &&
        selected.subdivision < subdivisions.size()) {
      ImGui::TextDisabled(
          "Subdivision %zu/%zu, layer %zu/%zu", selected.subdivision + 1,
          subdivisions.size(), selected.subdivision_level + 1,
          subdivisions[selected.subdivision].level_count);
    }
  }
  static const char* kColourings[]{"Feed rate", "Limiting reason", "Engagement",
                                   "Move type"};
  int colouring = static_cast<int>(app.colouring);
  if (ImGui::Combo("Colour by", &colouring, kColourings, 4))
    app.colouring = static_cast<Colouring>(colouring);
  ImGui::Checkbox("Stock", &app.show_stock);
  ImGui::SameLine();
  ImGui::Checkbox("Part", &app.show_part);
  ImGui::SameLine();
  ImGui::Checkbox("Material##show", &app.show_material);
  ImGui::Checkbox("Cutting paths", &app.show_cutting_paths);
  ImGui::SetItemTooltip(
      "Everything cut at a feed: the passes themselves, their links, and the "
      "entries. Shown in both the pass view and over the stack.");
  ImGui::SameLine();
  ImGui::Checkbox("Rapid travel", &app.show_rapid_travel);
  ImGui::SetItemTooltip(
      "Everything moved at machine rapid: retracts, the traverses across the "
      "safe plane, and the descents back to depth. Over the stack these are "
      "the moves the pass view cannot show, being the ones that leave its "
      "depth.");
  if (ImGui::TreeNode("Debug view")) {
    const auto& topology =
        app.plan.operations[app.selected_plan_operation].inter_layer;
    if (!levels.empty()) {
      const cam::LevelPlan& selected = levels[app.selected_level];
      ImGui::TextDisabled(
          "%zu recursive wave layer%s, %zu/%zu trial%s accepted",
          selected.wave_stages.size(),
          selected.wave_stages.size() == 1 ? "" : "s",
          selected.accepted_analysis_trials, selected.analysis_trials,
          selected.analysis_trials == 1 ? "" : "s");
      ImGui::TextDisabled(
          "%zu current node%s, %zu current entr%s; overlays use the combined "
          "committed result.",
          selected.topology.nodes.size(),
          selected.topology.nodes.size() == 1 ? "" : "s",
          selected.lineage.entry_fronts.size(),
          selected.lineage.entry_fronts.size() == 1 ? "y" : "ies");
    }
    ImGui::TextDisabled("%zu node Z-group%s, %zu entry-front Z-group%s",
                        topology.node_chains.size(),
                        topology.node_chains.size() == 1 ? "" : "s",
                        topology.entry_front_chains.size(),
                        topology.entry_front_chains.size() == 1 ? "" : "s");
    static const char* kNodeColourings[]{"Off", "By node", "By Z-group"};
    int node_colouring = static_cast<int>(app.node_debug_colouring);
    if (ImGui::Combo("Node boundaries", &node_colouring, kNodeColourings,
                     std::size(kNodeColourings)))
      app.node_debug_colouring =
          static_cast<NodeDebugColouring>(node_colouring);
    ImGui::Checkbox("Entry-front Z-groups", &app.show_entry_front_chains);
    ImGui::Checkbox("Boundary contacts", &app.show_boundary_contacts);
    ImGui::TextDisabled("Closed node areas use the simulated-stock fill path.");
    ImGui::TreePop();
  }
  ImGui::Checkbox("Stock follows the cutter", &app.stack_follows_playback);
  ImGui::SetItemTooltip(
      "Off, the stacked view shows the finished part. On, it shows how far the "
      "job has got: passes above the cutter finished, the one it is in part way "
      "through, and the ones below it untouched.");

  ImGui::SeparatorText("Playback");
  const cam::LevelPlan* level = current_level(app);
  const double total = level_length(level);
  if (ImGui::Button(app.playback.playing ? "Pause" : "Play"))
    app.playback.playing = !app.playback.playing;
  ImGui::SameLine();
  if (ImGui::Button("Restart")) {
    app.playback.distance = 0.0;
    app.playback.motion_seconds = 0.0;
    app.playback.playing = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("End")) {
    app.playback.distance = total;
    app.playback.motion_seconds =
        level ? cam::feed_schedule_seconds(level->feeds) : 0.0;
    app.playback.playing = false;
  }
  ImGui::SameLine();
  ImGui::Checkbox("Run on", &app.playback.follow);
  ImGui::SetItemTooltip("Fall into the next pass instead of looping this one.");

  float where = static_cast<float>(app.playback.distance);
  ImGui::SetNextItemWidth(-160.0f);
  if (ImGui::SliderFloat("Along pass", &where, 0.0f,
                         static_cast<float>(total), "%.1f mm")) {
    app.playback.distance = where;
    sync_playback_time(app);
  }

  if (ImGui::Checkbox("Machine time", &app.playback.real_time))
    sync_playback_time(app);
  ImGui::SetItemTooltip(
      "Run the cutter at the feeds the plan commands. Those came out of what "
      "the simulation measured it removing, so the pass crawls where the "
      "engagement is heavy and opens out where there is nothing in the way. "
      "Off, it runs at a fixed distance per second instead.");
  if (app.playback.real_time) {
    float scale = static_cast<float>(app.playback.time_scale);
    ImGui::SetNextItemWidth(-160.0f);
    if (ImGui::SliderFloat("Rate", &scale, 0.1f, 20.0f, "%.2fx",
                           ImGuiSliderFlags_Logarithmic))
      app.playback.time_scale = scale;
    if (level && !level->nothing_to_cut) {
      const double elapsed = app.playback.motion_seconds;
      const double whole = cam::feed_schedule_seconds(level->feeds);
      ImGui::Text("%s of %s in this pass, %.1f min in the job",
                  clock_text(elapsed).c_str(), clock_text(whole).c_str(),
                  app.plan.stats.cutting_minutes + app.plan.stats.rapid_minutes);
      const cam::MotionCursor motion = cam::feed_motion_at_seconds(
          level->feeds, app.playback.motion_seconds);
      const double motion_total = cam::feed_motion_length(level->feeds);
      ImGui::Text("Machine XYZ travel %.1f of %.1f mm",
                  motion.motion_distance, motion_total);
      ImGui::Text("%s %.1f of %.1f mm at %.0f mm/min (%.2fx view)",
                  motion_phase_name(motion), motion.phase_distance,
                  motion.phase_length, motion.rate, app.playback.time_scale);
      ImGui::TextDisabled("Tool Z %.3f mm; rapid plane %.3f mm",
                          motion.z, app.plan.rapid_plane);
      const cam::FeedSegment* segment = nullptr;
      if (!level->feeds.segments.empty())
        segment = &level->feeds.segments[segment_at(level->feeds.segments,
                                                    app.playback.distance)];
      const bool positioning = motion.rapid || motion.from_z != motion.to_z;
      if (segment != nullptr && !positioning)
        ImGui::Text("Now %.0f mm/min, %.0f%% engaged, %s", segment->rate,
                    segment->engagement * 100.0, cam::feed_limit_name(segment->limit));
    }
  } else {
    float speed = static_cast<float>(app.playback.speed);
    ImGui::SetNextItemWidth(-160.0f);
    if (ImGui::SliderFloat("Speed", &speed, 2.0f, 400.0f, "%.0f mm/s"))
      app.playback.speed = speed;
  }

  if (level && !level->nothing_to_cut) {
    const double area = wave::material_area(app.material);
    const double started = level->simulation.stats.initial_stock_area;
    ImGui::Text("Stock left %.1f mm2 of %.1f", area, started);
    if (started > 0.0)
      ImGui::ProgressBar(static_cast<float>(1.0 - area / started),
                         ImVec2(-1, ui_size(app, 12.0f)),
                         "removed");
  }
}

}  // namespace tinglecam_gui
