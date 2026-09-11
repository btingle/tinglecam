#include "gui_internal.h"
namespace tinglecam_gui {
// ---------------------------------------------------------------------------
// Panels
// ---------------------------------------------------------------------------

// Defined below with the origin panel, which is where the box it places the
// origin against is worked out.
void apply_origin_pick(App& app);

void depth_defaults(App& app, cam::Operation& operation) {
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.work_range(low, high)) return;
  // The whole of the work, top to bottom. Depths measured from its top, so the
  // near end is zero whatever the models' own coordinates say.
  operation.depth_from = 0.0;
  operation.depth_to = high - low;
}

void load_meshes(App& app) {
  // The planner slices this same geometry as it runs, so it has to be stopped
  // before the meshes under it are replaced rather than after.
  app.planner.stop();
  try {
    if (app.part_path[0] != '\0') {
      app.geometry.load_part(app.part_path.data());
      app.job.part_mesh = app.part_path.data();
    }
    if (app.stock_path[0] != '\0') {
      app.geometry.load_stock(app.stock_path.data());
      app.job.stock_mesh = app.stock_path.data();
    } else {
      // No model named, so a box around the part is the only blank on offer.
      // Any model loaded earlier goes, or it would go on standing in for one
      // the job no longer names.
      app.geometry.clear_stock();
      app.job.stock_mesh.clear();
      app.job.box_stock = true;
    }
    app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margin);
    for (auto& operation : app.job.operations) depth_defaults(app, operation);
    // The blank has changed, so the picked corner of it is somewhere new.
    apply_origin_pick(app);
    double low = 0.0;
    double high = 0.0;
    if (app.geometry.work_range(low, high))
      say(app, "Loaded. Z runs " + std::to_string(low) + " to " + std::to_string(high));
    // Not merely stale: the models a plan was cut against have been replaced
    // under it, and the planner reads this same geometry as it runs.
    invalidate_plan(app);
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
}

void load_job_into(App& app, const std::filesystem::path& file) {
  app.planner.stop();
  cam::Job job = cam::load_job(file);
  cam::JobGeometry geometry;
  const auto resolve = [&](std::filesystem::path& path) {
    if (!path.empty() && path.is_relative())
      path = std::filesystem::absolute(file).parent_path() / path;
  };
  resolve(job.part_mesh);
  resolve(job.stock_mesh);
  if (!job.part_mesh.empty()) geometry.load_part(job.part_mesh);
  if (!job.stock_mesh.empty()) geometry.load_stock(job.stock_mesh);
  geometry.set_box_stock(job.box_stock, job.box_stock_margin);
  app.geometry = std::move(geometry);
  app.job = std::move(job);
  app.job_file = file.string();
  std::snprintf(app.part_path.data(), app.part_path.size(), "%s", app.job.part_mesh.string().c_str());
  std::snprintf(app.stock_path.data(), app.stock_path.size(), "%s", app.job.stock_mesh.string().c_str());
  app.selected_operation = 0;
  app.operation_selection.assign(app.job.operations.size(), false);
  invalidate_plan(app);
  say(app, "Loaded " + file.string());
}

// The box the program origin is placed against: the blank, because the blank is
// what sits on the machine and what the operator touches off. A rectangular
// blank has no mesh behind it, so its footprint comes from the outline it is
// defined by, and its Z from the part it was built around.
struct JobBounds {
  double low[3]{};
  double high[3]{};
  bool valid = false;
  const char* what = "stock";

  double size(int axis) const { return high[axis] - low[axis]; }
  // Where a low/middle/high pick on one axis puts the origin, in model
  // coordinates.
  double pick(int axis, int choice) const {
    if (choice == 0) return low[axis];
    if (choice == 2) return high[axis];
    return (low[axis] + high[axis]) * 0.5;
  }
};

JobBounds job_bounds(const App& app) {
  JobBounds result;
  const auto take = [&](const wave::Bounds3f& box, const char* what) {
    if (!box.valid) return false;
    result.low[0] = box.lower.x;
    result.low[1] = box.lower.y;
    result.low[2] = box.lower.z;
    result.high[0] = box.upper.x;
    result.high[1] = box.upper.y;
    result.high[2] = box.upper.z;
    result.what = what;
    result.valid = true;
    return true;
  };
  if (app.geometry.box_stock()) {
    // The blank stood in for: the part's own Z, and a footprint this asks the
    // geometry for rather than recomputing, so the box drawn and the box the
    // origin is placed against cannot drift apart.
    double low = 0.0;
    double high = 0.0;
    if (!app.geometry.has_part() || !app.geometry.work_range(low, high))
      return result;
    const wave::Paths64& outline = app.geometry.stock_outline(low, 32);
    if (outline.empty()) return result;
    const Clipper2Lib::Rect64 box = Clipper2Lib::GetBounds(outline);
    result.low[0] = wave::to_world(box.left);
    result.low[1] = wave::to_world(box.top);
    result.low[2] = low;
    result.high[0] = wave::to_world(box.right);
    result.high[1] = wave::to_world(box.bottom);
    result.high[2] = high;
    result.what = "box stock";
    result.valid = true;
    return result;
  }
  if (app.geometry.has_stock() &&
      take(app.geometry.stock().mesh().bounds, "stock"))
    return result;
  if (app.geometry.has_part()) take(app.geometry.part().mesh().bounds, "part");
  return result;
}

// Put the origin where the picks say, on the blank as it stands now. Called
// when a model loads as well as from the button, so the origin the gizmo shows
// is the one the picks describe from the outset rather than wherever the mesh
// happens to carry its own zero.
void apply_origin_pick(App& app) {
  const JobBounds bounds = job_bounds(app);
  if (!bounds.valid) return;
  app.job.origin_x = -bounds.pick(0, app.origin_pick[0]);
  app.job.origin_y = -bounds.pick(1, app.origin_pick[1]);
  app.job.origin_z = -bounds.pick(2, app.origin_pick[2]);
}

// Placing the program origin. Everything here is in the model's own
// coordinates, measured from the bottom-back-left corner of the blank, because
// that is the corner an operator can find with an edge finder. What the job
// carries is the other half of the same statement -- the offset added to every
// posted coordinate, which is the negative of the point picked here -- and none
// of the planning ever sees this side of it.
void origin_panel(App& app) {
  ImGui::SeparatorText("Origin");
  const JobBounds bounds = job_bounds(app);
  if (!bounds.valid) {
    ImGui::TextDisabled("Load a model to place the program origin");
    return;
  }

  static const char* const kLabels[3][3] = {
      {"Left", "Centre *", "Right"},
      {"Back", "Centre *", "Front"},
      {"Bottom", "Centre", "Top *"},
  };
  static const char* const kAxes[3] = {"-/+X", "-/+Y", "-/+Z"};

  if (ImGui::BeginTable("origin_pick", 4, ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableSetupColumn("l", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("m", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("h", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("a", ImGuiTableColumnFlags_WidthFixed);
    for (int axis = 0; axis < 3; ++axis) {
      ImGui::TableNextRow();
      for (int choice = 0; choice < 3; ++choice) {
        ImGui::TableSetColumnIndex(choice);
        const bool picked = app.origin_pick[axis] == choice;
        if (picked)
          ImGui::PushStyleColor(ImGuiCol_Button,
                                ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        ImGui::PushID(axis * 3 + choice);
        if (ImGui::Button(kLabels[axis][choice], ImVec2(-FLT_MIN, 0.0f)))
          app.origin_pick[axis] = choice;
        ImGui::PopID();
        if (picked) ImGui::PopStyleColor();
      }
      ImGui::TableSetColumnIndex(3);
      ImGui::AlignTextToFramePadding();
      ImGui::TextDisabled("%s", kAxes[axis]);
    }
    ImGui::EndTable();
  }
  ImGui::TextDisabled("* the usual choice");

  if (ImGui::Button("Apply")) apply_origin_pick(app);
  ImGui::SetItemTooltip(
      "Move the origin to the picked place on the blank. No re-plan: the "
      "origin is only where the posted numbers are measured from, and the plan "
      "is the same motion whichever corner that is.");

  // The three figures are the origin itself, stated where it can be checked
  // against the work rather than in the sign the post wants it in.
  double offset[3] = {
      -app.job.origin_x - bounds.low[0],
      -app.job.origin_y - bounds.low[1],
      -app.job.origin_z - bounds.low[2],
  };
  static const char* const kFields[3] = {"X", "Y", "Z"};
  bool edited = false;
  if (ImGui::BeginTable("origin_value", 3, ImGuiTableFlags_SizingStretchSame)) {
    for (int axis = 0; axis < 3; ++axis) {
      ImGui::TableNextColumn();
      ImGui::PushID(axis);
      // Labelled on the left. ImGui puts a widget's label on its right, which
      // a full-width field in a table cell pushes off the end of the cell.
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(kFields[axis]);
      ImGui::SameLine();
      ImGui::SetNextItemWidth(-FLT_MIN);
      if (ImGui::InputDouble("##value", &offset[axis], 0.0, 0.0, "%.3f"))
        edited = true;
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (edited) {
    app.job.origin_x = -(offset[0] + bounds.low[0]);
    app.job.origin_y = -(offset[1] + bounds.low[1]);
    app.job.origin_z = -(offset[2] + bounds.low[2]);
  }
  ImGui::TextDisabled("From the bottom-back-left corner of the %s",
                      bounds.what);
  ImGui::TextDisabled("Bounds %.1f x %.1f x %.1f mm", bounds.size(0),
                      bounds.size(1), bounds.size(2));
}

void job_panel(App& app) {
  ImGui::SeparatorText("Job");
  text_field("Name", app.job.name);
  text_field("Job JSON", app.job_file, 1024);
  ImGui::BeginDisabled(app.planner.running());
  if (ImGui::Button("Save job")) {
    try {
      cam::Job saved = app.job;
      const std::filesystem::path destination =
          std::filesystem::absolute(app.job_file);
      const auto relative_to_job = [&](const std::filesystem::path& model) {
        if (model.empty()) return std::filesystem::path{};
        std::error_code error;
        const auto relative = std::filesystem::relative(
            std::filesystem::absolute(model), destination.parent_path(), error);
        return error ? std::filesystem::absolute(model) : relative;
      };
      saved.part_mesh = relative_to_job(saved.part_mesh);
      saved.stock_mesh = relative_to_job(saved.stock_mesh);
      cam::save_job(app.job_file, saved);
      say(app, "Saved " + app.job_file);
    } catch (const std::exception& error) { say(app, error.what(), true); }
  }
  ImGui::SameLine();
  if (ImGui::Button("Load job")) {
    try { load_job_into(app, app.job_file); }
    catch (const std::exception& error) { say(app, error.what(), true); }
  }
  ImGui::EndDisabled();
  // The planner is slicing these while it runs; the blank cannot be changed
  // under it any more than the models can.
  ImGui::BeginDisabled(app.planner.running());
  ImGui::InputText("Part model", app.part_path.data(), app.part_path.size());
  ImGui::InputText("Stock model", app.stock_path.data(), app.stock_path.size());
  // A named model always wins, so the fallback is only offered without one.
  ImGui::BeginDisabled(app.stock_path[0] != '\0');
  if (ImGui::Checkbox("Box stock around the part", &app.job.box_stock))
    app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margin);
  ImGui::SetItemTooltip(
      "Stand a rectangular blank in for a stock model: the part's bounding box "
      "grown in X and Y. Z stays at the part's own range.");
  if (app.job.box_stock) {
    if (ImGui::DragScalar("Box margin (mm)", ImGuiDataType_Double,
                          &app.job.box_stock_margin, 0.1f))
      app.geometry.set_box_stock(app.job.box_stock,
                                 std::max(app.job.box_stock_margin, 0.0));
  }
  ImGui::EndDisabled();
  if (ImGui::Button("Load models")) load_meshes(app);
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) {
    app.view.needs_fit = true;
    // The 3D views frame themselves, so there is nothing to fit there -- but a
    // view panned off into the corner is exactly as lost as one badly zoomed,
    // and this is the button to reach for.
    app.orbit.pan = ImVec2(0.0f, 0.0f);
  }

  origin_panel(app);

  ImGui::DragScalar("Rapid plane (above highest)", ImGuiDataType_Double,
                    &app.job.rapid_clearance_height, 0.1f);
  ImGui::SetItemTooltip(
      "How far the safe plane clears the tallest thing in the way, part or "
      "blank, not the distance of each retract: a retract travels from its "
      "pass depth to that one plane. Measured over the work rather than up "
      "from the origin, so moving the origin cannot move it. Re-plan after "
      "changing it -- it is what every lift is costed against.");
  text_field("Work offset", app.job.work_offset, 16);
}

cam::Operation make_operation(App& app, cam::OperationMode mode) {
  cam::Operation operation = cam::operation_default(app.settings, mode);
  operation.mode = mode;
  switch (mode) {
    case cam::OperationMode::Roughing: operation.name = "Rough"; break;
    case cam::OperationMode::ContourFinishing: operation.name = "Contour finish"; break;
    case cam::OperationMode::Pocket: operation.name = "Pocket"; break;
    case cam::OperationMode::PlaneClearing: operation.name = "Plane clearing"; break;
  }
  if (operation.tool_id.empty() && !app.tools.tools.empty())
    operation.tool_id = app.tools.tools.front().id;
  if (operation.machine_id.empty() && !app.machines.machines.empty())
    operation.machine_id = app.machines.machines.front().id;
  if (operation.material_id.empty() && !app.materials.materials.empty())
    operation.material_id = app.materials.materials.front().id;
  depth_defaults(app, operation);
  for (auto& pocket : operation.pockets) {
    pocket.center_x = -app.job.origin_x;
    pocket.center_y = -app.job.origin_y;
    pocket.top_z = -app.job.origin_z;
  }
  for (auto& area : operation.plane.areas) {
    area.center_x = -app.job.origin_x;
    area.center_y = -app.job.origin_y;
    area.top_z = -app.job.origin_z;
  }
  if (mode == cam::OperationMode::ContourFinishing) {
    operation.radial_stock_to_leave = 0.0;
    operation.rough_boundary_cleanup = false;
  }

  const std::string base = operation.name;
  int suffix = 2;
  const auto name_exists = [&](const std::string& name) {
    return std::any_of(app.job.operations.begin(), app.job.operations.end(),
                       [&](const cam::Operation& candidate) {
                         return candidate.name == name;
                       });
  };
  while (name_exists(operation.name))
    operation.name = base + " " + std::to_string(suffix++);
  return operation;
}

void select_operation(App& app, int index) {
  if (app.job.operations.empty()) {
    app.selected_operation = -1;
    return;
  }
  index = std::clamp(index, 0,
                     static_cast<int>(app.job.operations.size()) - 1);
  app.selected_operation = index;
  if (app.plan.operations.empty()) return;

  // The plan holds one entry per operation, in the same order, so the two are
  // the same index. An operation with no result yet has an entry saying so
  // rather than no entry at all, which is what keeps that true -- and what
  // lets the preview follow the selection whether or not it has been planned.
  app.selected_plan_operation = std::clamp(
      index, 0, static_cast<int>(app.plan.operations.size()) - 1);
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  app.selected_level = 0;
  while (app.selected_level < static_cast<int>(levels.size()) &&
         levels[app.selected_level].nothing_to_cut)
    ++app.selected_level;
  if (app.selected_level >= static_cast<int>(levels.size()))
    app.selected_level = 0;
  app.playback.playing = false;
  app.playback.distance = 0.0;
  app.playback.motion_seconds = 0.0;
  app.material.clear();
  app.material_distance = -1.0;
  app.material_level = -1;
  app.material_operation = -1;
  app.stock_cache_operation = -1;
  app.view.needs_fit = true;
}

// Everything outside an operation that decides what it cuts. The resolved
// rapid plane is in here rather than the clearance height it comes from: it
// also folds in how tall the models are and how high a pocket's own top
// surface sits, and it is what every lift in every operation was costed
// against. An operation planned against a different one is out of date however
// little else has changed.
//
// Where the origin sits is deliberately absent. It is added to posted
// coordinates and nothing else, so moving it cannot make a plan wrong.
std::string job_signature(const App& app) {
  return app.job.part_mesh.string() + '\n' + app.job.stock_mesh.string() +
         (app.job.box_stock ? "\nbox " : "\nmodel ") +
         std::to_string(app.job.box_stock_margin) + ' ' +
         std::to_string(cam::job_rapid_plane(app.job, app.geometry)) + '\n';
}

// A blank entry for an operation nothing has been asked of yet. It carries the
// operation's identity so the list, the post's skip comment and the preview
// can all name it without having to reach back into the job.
cam::OperationPlan unplanned(const cam::Operation& operation) {
  cam::OperationPlan blank;
  blank.operation_name = operation.name;
  blank.tool_id = operation.tool_id;
  blank.machine_id = operation.machine_id;
  blank.material_id = operation.material_id;
  blank.error = kUnplanned;
  return blank;
}

// The job's operations, the plan's entry for each, and the record of what each
// was planned from are three lists in step. Every structural edit goes through
// these, or an operation's passes end up filed under its neighbour.
void insert_operation(App& app, int index, cam::Operation operation) {
  index = std::clamp(index, 0, static_cast<int>(app.job.operations.size()));
  app.plan.operations.insert(app.plan.operations.begin() + index,
                             unplanned(operation));
  app.planned.insert(app.planned.begin() + index, PlannedRecord{});
  app.operation_selection.insert(app.operation_selection.begin() + index, false);
  app.job.operations.insert(app.job.operations.begin() + index,
                            std::move(operation));
}

void erase_operation(App& app, int index) {
  if (index < 0 || index >= static_cast<int>(app.job.operations.size())) return;
  app.job.operations.erase(app.job.operations.begin() + index);
  app.plan.operations.erase(app.plan.operations.begin() + index);
  app.planned.erase(app.planned.begin() + index);
  if (index < static_cast<int>(app.operation_selection.size()))
    app.operation_selection.erase(app.operation_selection.begin() + index);
}

void swap_operations(App& app, int a, int b) {
  const int count = static_cast<int>(app.job.operations.size());
  if (a < 0 || b < 0 || a >= count || b >= count || a == b) return;
  std::swap(app.job.operations[a], app.job.operations[b]);
  std::swap(app.plan.operations[a], app.plan.operations[b]);
  std::swap(app.planned[a], app.planned[b]);
  if (a < static_cast<int>(app.operation_selection.size()) &&
      b < static_cast<int>(app.operation_selection.size()))
    std::swap(app.operation_selection[a], app.operation_selection[b]);
}

// A last line of defence. Everything that edits the list goes through the
// helpers above, so this should never have anything to do; if it ever does,
// losing the results is far better than showing one operation's passes under
// another operation's name.
void align_plan(App& app) {
  const std::size_t count = app.job.operations.size();
  app.operation_selection.resize(count, false);
  if (app.plan.operations.size() == count && app.planned.size() == count) return;
  app.plan.operations.clear();
  for (const cam::Operation& operation : app.job.operations)
    app.plan.operations.push_back(unplanned(operation));
  app.planned.assign(count, PlannedRecord{});
}

std::vector<std::size_t> selected_operation_indices(const App& app,
                                                     bool fallback_to_focus) {
  std::vector<std::size_t> selected;
  for (std::size_t i = 0; i < app.job.operations.size(); ++i)
    if (i < app.operation_selection.size() && app.operation_selection[i])
      selected.push_back(i);
  if (selected.empty() && fallback_to_focus && app.selected_operation >= 0 &&
      app.selected_operation < static_cast<int>(app.job.operations.size()))
    selected.push_back(static_cast<std::size_t>(app.selected_operation));
  return selected;
}

// An operation is out of date when its own settings have moved on, when it has
// no result at all, or when something above it has been planned more recently
// than it was -- because what it was handed to cut has changed underneath it.
void refresh_plan_states(App& app) {
  align_plan(app);
  const std::string job_part = job_signature(app);
  app.plan_states.assign(app.job.operations.size(), PlanState::None);
  app.has_plan = false;
  bool downstream = false;
  std::uint64_t newest_above = 0;
  for (std::size_t i = 0; i < app.job.operations.size(); ++i) {
    const cam::OperationPlan& planned = app.plan.operations[i];
    if (!planned.planned) {
      downstream = true;
      continue;
    }
    if (planned.ok()) app.has_plan = true;
    const PlannedRecord& record = app.planned[i];
    if (record.signature != job_part + cam::to_json_text(app.job.operations[i])) {
      app.plan_states[i] = PlanState::Edited;
      downstream = true;
    } else if (downstream || record.serial < newest_above) {
      app.plan_states[i] = PlanState::Downstream;
    } else {
      app.plan_states[i] =
          planned.ok() ? PlanState::Planned : PlanState::Failed;
    }
    newest_above = std::max(newest_above, record.serial);
  }
}

PlanState plan_state(const App& app, std::size_t index) {
  return index < app.plan_states.size() ? app.plan_states[index]
                                        : PlanState::None;
}

bool out_of_date(PlanState state) {
  return state == PlanState::Edited || state == PlanState::Downstream ||
         state == PlanState::None;
}

// Anything the plan on screen no longer describes.
std::size_t stale_operations(const App& app) {
  return static_cast<std::size_t>(
      std::count_if(app.plan_states.begin(), app.plan_states.end(), out_of_date));
}

// The first operation at or before `last` that has to be planned again.
// Everything after it in the job has to be too, so this one index is the whole
// of what a re-plan needs to be told: an operation is only current while every
// operation above it is.
int first_out_of_date(const App& app, int last) {
  for (int i = 0; i <= last && i < static_cast<int>(app.plan_states.size()); ++i)
    if (out_of_date(app.plan_states[i])) return i;
  return -1;
}

void invalidate_plan(App& app) {
  // The worker reads the plans of the operations above the range it is
  // planning, straight out of the list about to be emptied.
  app.planner.stop();
  app.has_plan = false;
  app.plan = {};
  for (const cam::Operation& operation : app.job.operations)
    app.plan.operations.push_back(unplanned(operation));
  app.planned.assign(app.job.operations.size(), PlannedRecord{});
  app.plan_states.assign(app.job.operations.size(), PlanState::None);
  app.playback.playing = false;
  app.gcode.clear();
  app.post = {};
  app.posted_operations.clear();
  app.material.clear();
  app.material_distance = -1.0;
  app.material_level = -1;
  app.material_operation = -1;
  app.stock_cache_operation = -1;
}

void feeds_panel(App& app);
void plan_buttons(App& app);

void operations_list_panel(App& app) {
  ImGui::SeparatorText("Ordered operations");
  app.operation_selection.resize(app.job.operations.size(), false);
  if (app.job.operations.empty())
    app.selected_operation = -1;
  else
    app.selected_operation =
        std::clamp(app.selected_operation, 0,
                   static_cast<int>(app.job.operations.size()) - 1);
  if (ImGui::BeginListBox("##operation-order",
                          ImVec2(-FLT_MIN, ui_size(app, 190.0f)))) {
    for (std::size_t i = 0; i < app.job.operations.size(); ++i) {
      const cam::Operation& operation = app.job.operations[i];
      const PlanState state = plan_state(app, i);
      // The row itself carries the mark, because the mark is about the row.
      // An edited operation reads warm, one that only has to be replanned
      // because of something above it reads quiet, and a failure reads red --
      // and where there is no plan at all nothing is marked, since then
      // nothing is out of date with anything.
      const char* note = "";
      switch (state) {
        case PlanState::Edited: note = "   edited"; break;
        case PlanState::Downstream: note = "   replan"; break;
        case PlanState::Failed: note = "   failed"; break;
        // Worth saying only once something else has a result. Before anything
        // has been planned nothing is out of date with anything, and a mark on
        // every row would say only that the job is new.
        case PlanState::None: note = app.has_plan ? "   no result" : ""; break;
        case PlanState::Planned: break;
      }
      const std::string label = std::to_string(i + 1) + ". " + operation.name +
                                "  [" + operation_mode_label(operation.mode) +
                                "]" + note + "##" + std::to_string(i);
      const bool coloured = note[0] != '\0';
      if (coloured)
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            state == PlanState::Edited ? ImVec4(1.0f, 0.72f, 0.35f, 1.0f)
            : state == PlanState::Failed ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f)
                                         : ImVec4(0.62f, 0.68f, 0.78f, 1.0f));
      ImGui::PushID(static_cast<int>(i));
      bool selected_for_action = app.operation_selection[i];
      if (ImGui::Checkbox("##action", &selected_for_action))
        app.operation_selection[i] = selected_for_action;
      ImGui::SameLine();
      if (ImGui::Selectable(label.c_str(),
                            static_cast<int>(i) == app.selected_operation))
        select_operation(app, static_cast<int>(i));
      ImGui::PopID();
      if (coloured) ImGui::PopStyleColor();
    }
    ImGui::EndListBox();
  }

  ImGui::TextDisabled("Checked operations are used by multi-operation actions.");
  if (ImGui::SmallButton("Select all"))
    std::fill(app.operation_selection.begin(), app.operation_selection.end(), true);
  ImGui::SameLine();
  if (ImGui::SmallButton("Clear selection"))
    std::fill(app.operation_selection.begin(), app.operation_selection.end(), false);

  ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x -
                          ImGui::CalcTextSize("Add").x -
                          ImGui::GetStyle().FramePadding.x * 2.0f -
                          ImGui::GetStyle().ItemSpacing.x);
  enum_combo("##add-mode", app.add_mode, operation_mode_label, kOperationModes,
             std::size(kOperationModes));
  ImGui::SameLine();
  // Reordering the list while a re-plan is in flight would move the operations
  // it is reading its inherited stock out of. Editing settings stays live:
  // that only changes what the plan will be compared against afterwards.
  ImGui::BeginDisabled(app.planner.running());
  if (ImGui::Button("Add")) {
    insert_operation(app, static_cast<int>(app.job.operations.size()),
                     make_operation(app, app.add_mode));
    select_operation(app, static_cast<int>(app.job.operations.size()) - 1);
  }
  ImGui::BeginDisabled(app.job.operations.empty());
  if (ImGui::Button("Duplicate")) {
    cam::Operation copy = app.job.operations[app.selected_operation];
    copy.name += " copy";
    insert_operation(app, app.selected_operation + 1, std::move(copy));
    select_operation(app, app.selected_operation + 1);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(app.selected_operation <= 0);
  if (ImGui::Button("Move up")) {
    swap_operations(app, app.selected_operation, app.selected_operation - 1);
    select_operation(app, app.selected_operation - 1);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(
      app.selected_operation < 0 ||
      app.selected_operation + 1 >= static_cast<int>(app.job.operations.size()));
  if (ImGui::Button("Move down")) {
    swap_operations(app, app.selected_operation, app.selected_operation + 1);
    select_operation(app, app.selected_operation + 1);
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(app.job.operations.empty());
  if (ImGui::Button("Remove")) {
    erase_operation(app, app.selected_operation);
    const int next = std::min(app.selected_operation,
                              static_cast<int>(app.job.operations.size()) - 1);
    app.selected_operation = -1;
    select_operation(app, next);
  }
  ImGui::EndDisabled();
  ImGui::EndDisabled();

  plan_buttons(app);

  ImGui::TextWrapped(
      "Operations run from top to bottom. Simulation stock is handed to each "
      "operation from every operation above it.");
}
}  // namespace tinglecam_gui
