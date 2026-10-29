#include "gui_internal.h"
namespace tinglecam_gui {

void apply_origin_pick(App& app);

double work_top(const App& app) {
  const double top = cam::highest_model_z(app.geometry);
  return std::isfinite(top) ? top : 0.0;
}

void depth_defaults(App& app, cam::Operation& operation) {
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.work_range(low, high)) return;
  // Depths are measured from the top of the work, so the near end is zero.
  operation.depth_from = 0.0;
  operation.depth_to = high - low;
}

void mark_job_saved(App& app) { app.saved_job = cam::to_json_text(app.job); }

std::string window_title(const App& app) {
  std::string title = "TingleCAM - " + (app.job.name.empty() ? "Untitled" : app.job.name);
  if (job_dirty(app)) title += " *";
  if (!app.job_file.empty()) title += " - " + app.job_file.string();
  return title;
}

void adopt_default_setup(App& app) {
  const auto pick = [](std::string& id, const std::string& preferred,
                       const auto& entries) {
    if (!id.empty()) return;
    for (const auto& entry : entries)
      if (entry.id == preferred) id = preferred;
    if (id.empty() && !entries.empty()) id = entries.front().id;
  };
  pick(app.job.machine_id, app.settings.default_machine, app.machines.machines);
  pick(app.job.material_id, app.settings.default_material, app.materials.materials);
}

// The box the program origin is placed against: the blank, because the blank
// is what sits on the machine and what the operator touches off.
struct JobBounds {
  double low[3]{};
  double high[3]{};
  bool valid = false;
  const char* what = "stock";

  double size(int axis) const { return high[axis] - low[axis]; }
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

// Put the origin where the picks say, on the blank as it stands now.
void apply_origin_pick(App& app) {
  const JobBounds bounds = job_bounds(app);
  if (!bounds.valid) return;
  app.job.origin_x = -bounds.pick(0, app.origin_pick[0]);
  app.job.origin_y = -bounds.pick(1, app.origin_pick[1]);
  app.job.origin_z = -bounds.pick(2, app.origin_pick[2]);
}

namespace {

std::string file_label(const std::filesystem::path& path) {
  return path.empty() ? std::string("None") : path.filename().string();
}

void model_row(App& app, const char* label, const std::filesystem::path& path,
               bool part) {
  property_label(label);
  ImGui::PushID(label);
  ImGui::BeginDisabled(app.planner.running());
  if (ImGui::Button("Choose...")) {
    if (part) choose_part_model(app);
    else choose_stock_model(app);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (path.empty()) ImGui::TextDisabled("None");
  else ImGui::TextUnformatted(file_label(path).c_str());
  if (!path.empty()) ImGui::SetItemTooltip("%s", path.string().c_str());
  ImGui::PopID();
}

void model_rotation_controls(App& app) {
  ImGui::BeginDisabled(app.model_loading ||
      (!app.geometry.has_part() && app.job.stock_mesh.empty()));
  ImGui::TextUnformatted("Model orientation");
  ImGui::SetNextItemWidth(ui_size(app, 180));
  ImGui::Combo("Rotate", &app.model_rotation_target, "Part and stock\0Part only\0Stock only\0");
  ImGui::BeginDisabled((app.model_rotation_target == 2 && app.job.stock_mesh.empty()) ||
                       (app.model_rotation_target == 1 && !app.geometry.has_part()));
  const char* axes[]{"X", "Y", "Z"};
  for (int axis = 0; axis < 3; ++axis) {
    ImGui::PushID(axis);
    if (axis) ImGui::SameLine();
    ImGui::TextUnformatted(axes[axis]);
    ImGui::SameLine();
    if (ImGui::SmallButton("-90")) start_model_rotation(app, app.model_rotation_target, axis, -1);
    ImGui::SameLine();
    if (ImGui::SmallButton("+90")) start_model_rotation(app, app.model_rotation_target, axis, 1);
    ImGui::PopID();
  }
  if (ImGui::SmallButton("Reset orientation"))
    start_model_rotation(app, app.model_rotation_target, 0, 0);
  ImGui::SetItemTooltip("Restore the selected models' original orientation.\n"
      "Turns use world axes about the input coordinate origin.\n"
      "Rotation resets work depths and the program origin from the new bounds.");
  ImGui::EndDisabled();
  ImGui::EndDisabled();
}

void origin_section(App& app) {
  const JobBounds bounds = job_bounds(app);
  if (!bounds.valid) {
    ImGui::TextDisabled("Load a part to place the program origin.");
    return;
  }
  static const char* const kLabels[3][3] = {
      {"Left", "Centre", "Right"},
      {"Back", "Centre", "Front"},
      {"Bottom", "Centre", "Top"},
  };
  static const char* const kAxes[3] = {"X", "Y", "Z"};
  if (ImGui::BeginTable("origin_pick", 4, ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableSetupColumn("a", ImGuiTableColumnFlags_WidthFixed);
    for (int axis = 0; axis < 3; ++axis) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(kAxes[axis]);
      for (int choice = 0; choice < 3; ++choice) {
        ImGui::TableSetColumnIndex(choice + 1);
        const bool picked = app.origin_pick[axis] == choice;
        if (picked)
          ImGui::PushStyleColor(ImGuiCol_Button,
                                ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        ImGui::PushID(axis * 3 + choice);
        if (ImGui::Button(kLabels[axis][choice], ImVec2(-FLT_MIN, 0.0f))) {
          app.origin_pick[axis] = choice;
          apply_origin_pick(app);
        }
        ImGui::PopID();
        if (picked) ImGui::PopStyleColor();
      }
    }
    ImGui::EndTable();
  }
  // The origin itself, measured from the bottom-back-left corner of the blank:
  // the corner an operator finds with an edge finder.
  double offset[3] = {-app.job.origin_x - bounds.low[0],
                      -app.job.origin_y - bounds.low[1],
                      -app.job.origin_z - bounds.low[2]};
  bool edited = false;
  if (begin_properties("origin_values")) {
    static const char* const kFields[3] = {"Origin X", "Origin Y", "Origin Z"};
    for (int axis = 0; axis < 3; ++axis)
      edited |= prop_double(kFields[axis], offset[axis], 0.1f, "%.3f mm",
                            axis == 0 ? "Measured from the bottom-back-left corner of "
                                        "the blank. Moving the origin needs no re-plan."
                                      : nullptr);
    end_properties();
  }
  if (edited) {
    app.job.origin_x = -(offset[0] + bounds.low[0]);
    app.job.origin_y = -(offset[1] + bounds.low[1]);
    app.job.origin_z = -(offset[2] + bounds.low[2]);
  }
  ImGui::TextDisabled("%s %.1f x %.1f x %.1f mm", bounds.what, bounds.size(0),
                      bounds.size(1), bounds.size(2));
}

void tool_numbers_section(App& app) {
  const auto table = cam::resolved_tool_table(app.job);
  if (table.empty()) {
    ImGui::TextDisabled("No operation uses a tool yet.");
    return;
  }
  if (!ImGui::BeginTable("tool_numbers", 3,
                         ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
    return;
  ImGui::TableSetupColumn("Tool", ImGuiTableColumnFlags_WidthStretch, 3.0f);
  ImGui::TableSetupColumn("T", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("H", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableHeadersRow();
  for (const auto& entry : table) {
    ImGui::PushID(entry.tool_id.c_str());
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    const cam::Tool* tool = app.tools.find(entry.tool_id);
    ImGui::TextUnformatted(tool ? tool->name.c_str() : entry.tool_id.c_str());
    int number = entry.number;
    int offset = entry.length_offset;
    ImGui::TableSetColumnIndex(1);
    ImGui::SetNextItemWidth(-FLT_MIN);
    const bool numbered = ImGui::DragInt("##t", &number, 0.1f, 1, 999);
    ImGui::TableSetColumnIndex(2);
    ImGui::SetNextItemWidth(-FLT_MIN);
    const bool offset_changed = ImGui::DragInt("##h", &offset, 0.1f, 1, 999);
    if (numbered || offset_changed) {
      // Pin the whole table as shown, so other numbers stay where they are.
      app.job.tools = table;
      for (auto& pinned : app.job.tools)
        if (pinned.tool_id == entry.tool_id) {
          pinned.number = std::max(number, 1);
          pinned.length_offset = std::max(offset, 1);
        }
    }
    ImGui::PopID();
  }
  ImGui::EndTable();
}

}  // namespace

void setup_panel(App& app) {
  ImGui::BeginDisabled(app.planner.running());
  if (ImGui::CollapsingHeader("Job", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_properties("job")) {
    prop_text("Name", app.job.name);
    model_row(app, "Part", app.job.part_mesh, true);
    property_label("Stock");
    if (ImGui::RadioButton("Box around part", app.job.box_stock && app.job.stock_mesh.empty())) {
      app.job.stock_mesh.clear();
      app.job.box_stock = true;
      if (app.geometry.has_part()) load_meshes(app);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Model", !app.job.stock_mesh.empty()))
      choose_stock_model(app);
    if (!app.job.stock_mesh.empty()) model_row(app, "Stock model", app.job.stock_mesh, false);
    end_properties();
    model_rotation_controls(app);
    if (app.job.box_stock && app.job.stock_mesh.empty() &&
        ImGui::BeginTable("margins", 4, ImGuiTableFlags_SizingStretchProp)) {
      bool changed = false;
      auto& m = app.job.box_stock_margins;
      const auto margin = [&](const char* label, double& value) {
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::PushID(label);
        const double zero = 0.0;
        changed |= ImGui::DragScalar("##m", ImGuiDataType_Double, &value, 0.1f,
                                     &zero, nullptr, "%.2f mm",
                                     ImGuiSliderFlags_AlwaysClamp);
        ImGui::PopID();
      };
      margin("X-", m.minus_x); margin("X+", m.plus_x);
      margin("Y-", m.minus_y); margin("Y+", m.plus_y);
      margin("Z-", m.minus_z); margin("Z+", m.plus_z);
      ImGui::EndTable();
      if (changed) {
        app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margins);
        apply_origin_pick(app);
      }
    }
  }

  if (ImGui::CollapsingHeader("Machine", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_properties("machine")) {
    if (prop_library("Machine", app.job.machine_id, app.machines.machines,
                     [](const cam::Machine& m) { return m.name.c_str(); })) {
      app.settings.default_machine = app.job.machine_id;
      save_settings(app);
    }
    if (prop_library("Material", app.job.material_id, app.materials.materials,
                     [](const cam::Material& m) { return m.name.c_str(); },
                     "Feeds and speeds come from each tool's cutting data for "
                     "this material.")) {
      app.settings.default_material = app.job.material_id;
      save_settings(app);
    }
    prop_double("Stock grid pitch", app.job.simulation_pitch, 0.001f, "%.3f mm",
                "XY spacing of the stock simulation. Smaller is more exact and "
                "slower.", 0.001, 10.0);
    prop_double("Rapid clearance", app.job.rapid_clearance_height, 0.1f, "%.2f mm",
                "Height of the safe plane above the tallest of part and blank. "
                "Every lift goes to this one plane.", 0.1, 1000.0);
    prop_double("Plunge clearance", app.job.plunge_clearance_height, 0.1f, "%.2f mm",
                "Switch from rapid to plunge feed this far above the remaining "
                "stock beneath the cutter.", 0.0, 1000.0);
    static constexpr const char* kOffsets[]{"G54", "G55", "G56", "G57", "G58", "G59"};
    property_label("Work offset");
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##work_offset", app.job.work_offset.c_str())) {
      for (const char* offset : kOffsets)
        if (ImGui::Selectable(offset, app.job.work_offset == offset))
          app.job.work_offset = offset;
      ImGui::EndCombo();
    }
    end_properties();
    if (const cam::Machine* machine = app.machines.find(app.job.machine_id);
        machine && machine->specifications_unverified)
      ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                         "This machine profile has unverified limits.");
  }

  if (ImGui::CollapsingHeader("Tool numbers")) tool_numbers_section(app);
  ImGui::EndDisabled();
  if (ImGui::CollapsingHeader("Origin", ImGuiTreeNodeFlags_DefaultOpen))
    origin_section(app);
}

// ---------------------------------------------------------------------------
// Operations and their plan state

cam::Operation make_operation(App& app, cam::OperationMode mode) {
  cam::Operation operation = cam::operation_default(app.settings, mode);
  operation.mode = mode;
  switch (mode) {
    case cam::OperationMode::Roughing: operation.name = "Rough"; break;
    case cam::OperationMode::ContourFinishing: operation.name = "Contour finish"; break;
    case cam::OperationMode::Hole: operation.name = "Holes"; break;
    case cam::OperationMode::Pocket: operation.name = "Pocket"; break;
    case cam::OperationMode::PlaneClearing: operation.name = "Plane clearing"; break;
    case cam::OperationMode::SurfaceFinishing: operation.name = "Surface finish"; break;
  }
  if (operation.tool_id.empty() && !app.tools.tools.empty())
    operation.tool_id = app.tools.tools.front().id;
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
  if(mode==cam::OperationMode::Hole) {
    operation.holes.id.clear();operation.holes.source_id.clear();operation.holes.detection_signature.clear();
    operation.holes.systems=operation.holes.automatic?std::vector<cam::HoleSystem>{}:std::vector<cam::HoleSystem>{{}};
    for(auto& h:operation.holes.systems) {
      h.center_x=-app.job.origin_x;h.center_y=-app.job.origin_y;
      for(auto& l:h.levels) {l.top_z=-app.job.origin_z;l.bottom_z=l.top_z-5;}
    }
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
  index = std::clamp(index, 0, static_cast<int>(app.job.operations.size()) - 1);
  app.selected_operation = index;
  if (app.plan.operations.empty()) return;
  // The plan holds one entry per operation, in the same order.
  app.selected_plan_operation =
      std::clamp(index, 0, static_cast<int>(app.plan.operations.size()) - 1);
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  app.selected_level = 0;
  while (app.selected_level < static_cast<int>(levels.size()) &&
         levels[app.selected_level].nothing_to_cut)
    ++app.selected_level;
  if (app.selected_level >= static_cast<int>(levels.size())) app.selected_level = 0;
  app.playback.playing = false;
  app.playback.distance = 0.0;
  app.playback.motion_seconds = 0.0;
  app.material.clear();
  app.material_distance = -1.0;
  app.material_level = -1;
  app.material_operation = -1;
  app.view.needs_fit = true;
}

// Everything outside an operation that decides what it cuts. Where the origin
// sits is deliberately absent: it moves posted coordinates and nothing else.
std::string job_signature(const App& app) {
  const auto& m = app.job.box_stock_margins;
  std::string orientation;
  for (const auto& rotation : {app.job.part_orientation, app.job.stock_orientation})
    for (int axis : rotation.axes) orientation += std::to_string(axis) + ',';
  return orientation + '\n' + app.job.part_mesh.string() + '\n' + app.job.stock_mesh.string() +
         (app.job.box_stock ? "\nbox " : "\nmodel ") + std::to_string(m.minus_x) +
         ' ' + std::to_string(m.plus_x) + ' ' + std::to_string(m.minus_y) + ' ' +
         std::to_string(m.plus_y) + ' ' + std::to_string(m.minus_z) + ' ' +
         std::to_string(m.plus_z) + ' ' +
         std::to_string(cam::job_rapid_plane(app.job, app.geometry)) + '\n' +
         app.job.machine_id + '\n' + app.job.material_id + '\n' +
         std::to_string(app.job.simulation_pitch) + '\n' +
         std::to_string(app.job.plunge_clearance_height) + '\n';
}

// Everything one operation's plan depends on: the job, its own settings, and
// its tool, cutting data included.
std::string operation_signature(const App& app, std::size_t index,
                                const std::string& job_part) {
  cam::Operation operation=app.job.operations[index];
  std::string dependency;
  try {operation=cam::resolve_hole_source(app.job,index);}
  catch(const std::exception& e) {dependency=e.what();}
  const cam::Tool* tool=app.tools.find(operation.tool_id);
  if(operation.mode==cam::OperationMode::Hole) {
    for(const auto& h:operation.holes.systems) for(const auto& l:h.levels)
      if(const auto* selected=app.tools.find(l.tool_id)) dependency+=cam::to_json_text(*selected);
    if(!operation.holes.source_id.empty()) for(const auto& parent:app.job.operations)
      if(parent.holes.id==operation.holes.source_id) dependency+=cam::to_json_text(parent);
  }
  return job_part+cam::to_json_text(operation)+dependency+
      (tool?cam::to_json_text(*tool):std::string());
}

cam::OperationPlan unplanned(const cam::Operation& operation) {
  cam::OperationPlan blank;
  blank.operation_name = operation.name;
  blank.tool_id = operation.tool_id;
  blank.error = kUnplanned;
  return blank;
}

// The job's operations, the plan's entry for each, and the record of what each
// was planned from are three lists in step. Every structural edit goes here.
void insert_operation(App& app, int index, cam::Operation operation) {
  index = std::clamp(index, 0, static_cast<int>(app.job.operations.size()));
  app.result_preview = {};
  app.plan.operations.insert(app.plan.operations.begin() + index, unplanned(operation));
  app.planned.insert(app.planned.begin() + index, PlannedRecord{});
  app.operation_selection.insert(app.operation_selection.begin() + index, false);
  app.job.operations.insert(app.job.operations.begin() + index, std::move(operation));
}

void erase_operation(App& app, int index) {
  if (index < 0 || index >= static_cast<int>(app.job.operations.size())) return;
  app.job.operations.erase(app.job.operations.begin() + index);
  app.result_preview = {};
  app.plan.operations.erase(app.plan.operations.begin() + index);
  app.planned.erase(app.planned.begin() + index);
  if (index < static_cast<int>(app.operation_selection.size()))
    app.operation_selection.erase(app.operation_selection.begin() + index);
}

void move_operation(App& app, int from, int to) {
  const int count = static_cast<int>(app.job.operations.size());
  if (from < 0 || to < 0 || from >= count || to >= count || from == to) return;
  const auto rotate = [&](auto& list) {
    if (from < to)
      std::rotate(list.begin() + from, list.begin() + from + 1, list.begin() + to + 1);
    else
      std::rotate(list.begin() + to, list.begin() + from, list.begin() + from + 1);
  };
  rotate(app.job.operations);
  rotate(app.plan.operations);
  rotate(app.planned);
  app.operation_selection.resize(count, false);
  rotate(app.operation_selection);
}

// A last line of defence: losing the results is better than showing one
// operation's passes under another's name.
void align_plan(App& app) {
  const std::size_t count = app.job.operations.size();
  app.operation_selection.resize(count, false);
  if (app.plan.operations.size() == count && app.planned.size() == count) return;
  app.result_preview = {};
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

// Out of date when its own settings moved on, when it has no result, or when
// something above it has been planned more recently than it was.
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
    if (record.signature != operation_signature(app, i, job_part)) {
      app.plan_states[i] = PlanState::Edited;
      downstream = true;
    } else if (downstream || record.serial < newest_above) {
      app.plan_states[i] = PlanState::Downstream;
    } else {
      app.plan_states[i] = planned.ok() ? PlanState::Planned : PlanState::Failed;
    }
    newest_above = std::max(newest_above, record.serial);
  }
}

PlanState plan_state(const App& app, std::size_t index) {
  return index < app.plan_states.size() ? app.plan_states[index] : PlanState::None;
}

bool out_of_date(PlanState state) {
  return state == PlanState::Edited || state == PlanState::Downstream ||
         state == PlanState::None;
}

std::size_t stale_operations(const App& app) {
  return static_cast<std::size_t>(
      std::count_if(app.plan_states.begin(), app.plan_states.end(), out_of_date));
}

// The first operation at or before `last` that has to be planned again.
int first_out_of_date(const App& app, int last) {
  for (int i = 0; i <= last && i < static_cast<int>(app.plan_states.size()); ++i)
    if (out_of_date(app.plan_states[i])) return i;
  return -1;
}

void invalidate_plan(App& app) {
  app.planner.stop();
  app.has_plan = false;
  app.plan = {};
  app.result_preview = {};
  for (const cam::Operation& operation : app.job.operations)
    app.plan.operations.push_back(unplanned(operation));
  app.planned.assign(app.job.operations.size(), PlannedRecord{});
  app.plan_states.assign(app.job.operations.size(), PlanState::None);
  app.playback.playing = false;
  app.gcode.clear();
  app.post = {};
  app.posted_operations.clear();
  app.material.clear();
  app.material_heightmap.reset();
  app.material_distance = -1.0;
  app.material_level = -1;
  app.material_operation = -1;
}

namespace {

void remove_with_confirmation(App& app, int index) {
  if (index < 0 || index >= static_cast<int>(app.job.operations.size())) return;
  const std::string name = app.job.operations[index].name;
  confirm(app, "Remove operation", "Remove \"" + name + "\" from the job?", "Remove",
          [&app, index] {
            if (app.planner.running()) return;
            erase_operation(app, index);
            const int next = std::min(index, static_cast<int>(app.job.operations.size()) - 1);
            app.selected_operation = -1;
            select_operation(app, next);
          });
}

void duplicate_operation(App& app, int index) {
  cam::Operation copy = app.job.operations[index];
  if(copy.mode==cam::OperationMode::Hole) copy.holes.id.clear();
  copy.name += " copy";
  insert_operation(app, index + 1, std::move(copy));
  select_operation(app, index + 1);
}

double operation_minutes(const cam::OperationPlan& plan) {
  return plan.stats.cutting_minutes + plan.stats.rapid_minutes +
         plan.stats.tool_change_minutes;
}

}  // namespace

void add_operation_menu(App& app) {
  for (const cam::OperationMode mode : kOperationModes)
    if (ImGui::MenuItem(operation_mode_label(mode))) {
      insert_operation(app, static_cast<int>(app.job.operations.size()),
                       make_operation(app, mode));
      select_operation(app, static_cast<int>(app.job.operations.size()) - 1);
    }
}

void operations_panel(App& app) {
  app.operation_selection.resize(app.job.operations.size(), false);
  if (app.job.operations.empty()) app.selected_operation = -1;
  else
    app.selected_operation = std::clamp(app.selected_operation, 0,
                                        static_cast<int>(app.job.operations.size()) - 1);
  const bool busy = app.planner.running();

  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Operations");
  ImGui::SameLine();
  ImGui::BeginDisabled(busy);
  const float add_width = ImGui::CalcTextSize("Add operation").x +
                          ImGui::GetStyle().FramePadding.x * 2.0f;
  ImGui::SameLine(ImGui::GetContentRegionMax().x - add_width);
  if (ImGui::Button("Add operation")) ImGui::OpenPopup("add-operation");
  if (ImGui::BeginPopup("add-operation")) {
    add_operation_menu(app);
    ImGui::EndPopup();
  }
  ImGui::EndDisabled();

  const float row = ImGui::GetFrameHeight() + ImGui::GetStyle().CellPadding.y * 2.0f;
  const float wanted = row * static_cast<float>(std::max<std::size_t>(3, app.job.operations.size() + 1));
  const float room = ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing() * 4.5f;
  const float list_height = std::clamp(wanted, row * 3.0f, std::max(row * 3.0f, room));
  int drag_to = -1, drag_from = -1;
  if (ImGui::BeginTable("operation-list", 4,
                        ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_BordersOuter,
                        ImVec2(0, list_height))) {
    ImGui::TableSetupColumn("check", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("time", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("000.0 min").x);
    ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("no result").x);
    for (std::size_t i = 0; i < app.job.operations.size(); ++i) {
      const cam::Operation& operation = app.job.operations[i];
      const PlanState state = plan_state(app, i);
      const cam::OperationPlan* planned =
          i < app.plan.operations.size() ? &app.plan.operations[i] : nullptr;
      ImGui::PushID(static_cast<int>(i));
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      bool checked = app.operation_selection[i];
      if (ImGui::Checkbox("##check", &checked)) app.operation_selection[i] = checked;
      ImGui::SetItemTooltip("Checked operations are what \"Plan checked\" and "
                            "\"Export checked\" use.");
      ImGui::TableSetColumnIndex(1);
      const std::string label = std::to_string(i + 1) + ". " + operation.name;
      if (ImGui::Selectable(label.c_str(), static_cast<int>(i) == app.selected_operation,
                            ImGuiSelectableFlags_SpanAllColumns |
                                ImGuiSelectableFlags_AllowOverlap))
        select_operation(app, static_cast<int>(i));
      if (!busy && ImGui::BeginDragDropSource()) {
        const int source = static_cast<int>(i);
        ImGui::SetDragDropPayload("operation", &source, sizeof source);
        ImGui::TextUnformatted(label.c_str());
        ImGui::EndDragDropSource();
      }
      if (!busy && ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("operation")) {
          drag_from = *static_cast<const int*>(payload->Data);
          drag_to = static_cast<int>(i);
        }
        ImGui::EndDragDropTarget();
      }
      if (ImGui::BeginItemTooltip()) {
        ImGui::TextUnformatted(operation_mode_label(operation.mode));
        const cam::Tool* tool = app.tools.find(operation.tool_id);
        ImGui::TextDisabled("Tool: %s", tool ? tool->name.c_str()
                                             : (operation.tool_id.empty() ? "none"
                                                                          : operation.tool_id.c_str()));
        if (planned && planned->planned && !planned->ok())
          ImGui::TextColored(severity_colour(app, LogSeverity::Error), "%s",
                             planned->error.c_str());
        else if (planned && planned->planned && !planned->warnings.empty())
          ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                             "%zu warning%s", planned->warnings.size(),
                             planned->warnings.size() == 1 ? "" : "s");
        if (state == PlanState::Edited)
          ImGui::TextDisabled("Edited since it was planned");
        else if (state == PlanState::Downstream)
          ImGui::TextDisabled("An operation above it was planned again");
        ImGui::EndTooltip();
      }
      if (ImGui::BeginPopupContextItem("row-menu")) {
        select_operation(app, static_cast<int>(i));
        ImGui::BeginDisabled(busy);
        if (ImGui::MenuItem("Duplicate")) duplicate_operation(app, static_cast<int>(i));
        if (ImGui::MenuItem("Move up", nullptr, false, i > 0)) {
          move_operation(app, static_cast<int>(i), static_cast<int>(i) - 1);
          select_operation(app, static_cast<int>(i) - 1);
        }
        if (ImGui::MenuItem("Move down", nullptr, false,
                            i + 1 < app.job.operations.size())) {
          move_operation(app, static_cast<int>(i), static_cast<int>(i) + 1);
          select_operation(app, static_cast<int>(i) + 1);
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Remove...", "Del"))
          remove_with_confirmation(app, static_cast<int>(i));
        ImGui::EndDisabled();
        ImGui::EndPopup();
      }
      ImGui::TableSetColumnIndex(2);
      if (planned && planned->planned && planned->ok())
        ImGui::TextDisabled("%.1f min", operation_minutes(*planned));
      ImGui::TableSetColumnIndex(3);
      switch (state) {
        case PlanState::Planned:
          ImGui::TextColored(success_colour(app), "ok");
          break;
        case PlanState::Failed:
          ImGui::TextColored(severity_colour(app, LogSeverity::Error), "failed");
          break;
        case PlanState::Edited:
          ImGui::TextColored(severity_colour(app, LogSeverity::Warning), "edited");
          break;
        case PlanState::Downstream:
          ImGui::TextColored(muted_colour(app), "replan");
          break;
        case PlanState::None:
          if (app.has_plan) ImGui::TextDisabled("no result");
          break;
      }
      ImGui::PopID();
    }
    if (app.job.operations.empty()) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(1);
      ImGui::TextDisabled("No operations yet. Use Add operation.");
    }
    ImGui::EndTable();
  }
  if (drag_from >= 0 && drag_to >= 0 && drag_from != drag_to) {
    move_operation(app, drag_from, drag_to);
    select_operation(app, drag_to);
  }
  if (!busy && !ImGui::GetIO().WantTextInput && ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows) &&
      ImGui::IsKeyPressed(ImGuiKey_Delete, false))
    remove_with_confirmation(app, app.selected_operation);

  ImGui::BeginDisabled(app.job.operations.empty() || busy);
  if (ImGui::SmallButton("Check all"))
    std::fill(app.operation_selection.begin(), app.operation_selection.end(), true);
  ImGui::SameLine();
  if (ImGui::SmallButton("Uncheck all"))
    std::fill(app.operation_selection.begin(), app.operation_selection.end(), false);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::TextDisabled("Drag to reorder, right-click for more");
  plan_buttons(app);
  plan_summary(app);
}

}  // namespace tinglecam_gui
