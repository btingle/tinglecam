// TingleCAM: adaptive roughing over the wavefront engine, with tool and machine
// libraries and feeds driven by what the material simulation says is actually
// being removed.

#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;

// ---------------------------------------------------------------------------
// Background planning
// ---------------------------------------------------------------------------

// Planning a job of any size takes long enough that doing it on the frame
// thread would freeze the window, so it runs on its own thread and reports back
// through here. Everything the worker touches is either atomic or under the
// mutex; the plan itself is handed over once, at the end.
class Planner {
 public:
  ~Planner() { cancel_and_join(); }

  bool running() const { return running_.load(); }
  double fraction() const { return fraction_.load(); }
  std::string stage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stage_;
  }

  void start(cam::JobGeometry& geometry, cam::Job job, cam::ToolLibrary tools,
             cam::MachineLibrary machines, cam::MaterialLibrary materials) {
    cancel_and_join();
    cancelled_.store(false);
    running_.store(true);
    fraction_.store(0.0);
    finished_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stage_ = "Starting";
    }
    worker_ = std::thread([this, &geometry, job = std::move(job),
                           tools = std::move(tools), machines = std::move(machines),
                           materials = std::move(materials)]() mutable {
      cam::JobPlan result;
      try {
        result = cam::plan_job(
            geometry, job, tools, machines, materials,
            [this](double fraction, const std::string& stage) {
              fraction_.store(fraction);
              std::lock_guard<std::mutex> lock(mutex_);
              stage_ = stage;
              return !cancelled_.load();
            });
      } catch (const std::exception& error) {
        result = {};
        result.error = error.what();
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        plan_ = std::move(result);
      }
      finished_.store(true);
      running_.store(false);
    });
  }

  void cancel() { cancelled_.store(true); }

  // True once, when a finished plan is ready to be taken.
  bool take(cam::JobPlan& into) {
    if (!finished_.exchange(false)) return false;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    into = std::move(plan_);
    return true;
  }

 private:
  void cancel_and_join() {
    cancelled_.store(true);
    if (worker_.joinable()) worker_.join();
    running_.store(false);
  }

  mutable std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<bool> cancelled_{false};
  std::atomic<double> fraction_{0.0};
  std::string stage_;
  cam::JobPlan plan_;
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class Colouring { Feed, Limit, Load, Move };

struct View {
  ImVec2 pan{0.0f, 0.0f};
  float zoom = 4.0f;
  bool needs_fit = true;
};

struct App {
  cam::ToolLibrary tools;
  cam::MachineLibrary machines;
  cam::MaterialLibrary materials;
  cam::Preferences preferences;
  cam::Job job;
  cam::JobGeometry geometry;
  cam::JobPlan plan;
  bool has_plan = false;

  Planner planner;
  std::string banner;
  bool banner_is_error = false;

  std::array<char, 512> part_path{};
  std::array<char, 512> stock_path{};
  int selected_operation = 0;
  int selected_level = 0;
  int selected_plan_operation = 0;
  Colouring colouring = Colouring::Feed;
  bool show_stock = true;
  bool show_part = true;
  bool show_rapids = true;
  View view;

  // Tool Builder
  bool builder_open = false;
  bool builder_is_new = false;
  cam::Tool builder_tool;
  std::string builder_original_id;
  int builder_material = 0;

  std::string gcode;
  cam::PostResult post;
  std::array<char, 512> save_path{};

  std::filesystem::path tools_file;
  std::filesystem::path preferences_file;
};

void say(App& app, std::string text, bool error = false) {
  app.banner = std::move(text);
  app.banner_is_error = error;
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

struct Transform {
  ImVec2 origin;
  float scale = 1.0f;
  ImVec2 operator()(const Point64& point) const {
    return {origin.x + static_cast<float>(wave::to_world(point.x)) * scale,
            origin.y - static_cast<float>(wave::to_world(point.y)) * scale};
  }
};

void draw_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 colour, float width) {
  for (const auto& ring : paths) {
    if (ring.size() < 2) continue;
    for (std::size_t i = 0; i < ring.size(); ++i)
      list->AddLine(to_screen(ring[i]), to_screen(ring[(i + 1) % ring.size()]), colour,
                    width);
  }
}

ImU32 blend(ImU32 low, ImU32 high, float t) {
  const ImVec4 a = ImGui::ColorConvertU32ToFloat4(low);
  const ImVec4 b = ImGui::ColorConvertU32ToFloat4(high);
  t = std::clamp(t, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(
      ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1.0f));
}

ImU32 limit_colour(cam::FeedLimit limit) {
  switch (limit) {
    case cam::FeedLimit::ChipLoad: return IM_COL32(90, 200, 140, 255);
    case cam::FeedLimit::CornerGeometry: return IM_COL32(120, 170, 255, 255);
    case cam::FeedLimit::SpindlePower: return IM_COL32(255, 96, 80, 255);
    case cam::FeedLimit::CuttingForce: return IM_COL32(255, 140, 60, 255);
    case cam::FeedLimit::ToolDeflection: return IM_COL32(255, 90, 190, 255);
    case cam::FeedLimit::MachineFeed: return IM_COL32(220, 200, 90, 255);
    case cam::FeedLimit::Acceleration: return IM_COL32(160, 120, 255, 255);
    case cam::FeedLimit::BlockRate: return IM_COL32(255, 210, 120, 255);
    case cam::FeedLimit::PolicyMaximum: return IM_COL32(190, 190, 130, 255);
    case cam::FeedLimit::PolicyMinimum: return IM_COL32(150, 150, 150, 255);
    case cam::FeedLimit::MinimumChip: return IM_COL32(120, 230, 220, 255);
    case cam::FeedLimit::Lookahead: return IM_COL32(110, 190, 190, 255);
    case cam::FeedLimit::SlewLimit: return IM_COL32(150, 200, 170, 255);
    case cam::FeedLimit::Rapid: return IM_COL32(80, 95, 115, 255);
    case cam::FeedLimit::Plunge: return IM_COL32(255, 170, 60, 255);
  }
  return IM_COL32(200, 200, 200, 255);
}

ImU32 move_colour(wave::MoveKind kind) {
  switch (kind) {
    case wave::MoveKind::Cut: return IM_COL32(120, 220, 160, 255);
    case wave::MoveKind::Step: return IM_COL32(255, 190, 90, 255);
    case wave::MoveKind::Link: return IM_COL32(110, 180, 255, 255);
    case wave::MoveKind::Retract: return IM_COL32(90, 105, 130, 255);
    case wave::MoveKind::Entry: return IM_COL32(255, 100, 100, 255);
  }
  return IM_COL32(200, 200, 200, 255);
}

const cam::LevelPlan* current_level(const App& app) {
  if (!app.has_plan) return nullptr;
  if (app.selected_plan_operation < 0 ||
      app.selected_plan_operation >= static_cast<int>(app.plan.operations.size()))
    return nullptr;
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (app.selected_level < 0 || app.selected_level >= static_cast<int>(levels.size()))
    return nullptr;
  return &levels[app.selected_level];
}

void fit_view(App& app, ImVec2 canvas) {
  const cam::LevelPlan* level = current_level(app);
  if (!level) return;
  // Everything the pass involves, not just the stock: a lead-in stands outside
  // the stock and a part can reach past it, and framing on the stock alone
  // crops both off the edge of the view.
  Paths64 reference = level->stock_slice;
  reference.insert(reference.end(), level->part_slice.begin(),
                   level->part_slice.end());
  for (const auto& segment : level->feeds.segments)
    reference.push_back(Clipper2Lib::Path64{segment.from, segment.to});
  if (reference.empty()) return;
  const auto bounds = Clipper2Lib::GetBounds(reference);
  const float width = static_cast<float>(wave::to_world(bounds.right - bounds.left));
  const float height = static_cast<float>(wave::to_world(bounds.bottom - bounds.top));
  if (!(width > 0.0f) || !(height > 0.0f)) return;
  app.view.zoom = 0.85f * std::min(canvas.x / width, canvas.y / height);
  const float centre_x =
      static_cast<float>(wave::to_world(bounds.left + bounds.right)) * 0.5f;
  const float centre_y =
      static_cast<float>(wave::to_world(bounds.top + bounds.bottom)) * 0.5f;
  app.view.pan = {-centre_x * app.view.zoom, centre_y * app.view.zoom};
  app.view.needs_fit = false;
}

void draw_canvas(App& app) {
  ImGui::BeginChild("canvas", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  ImGui::InvisibleButton("plot", size,
                         ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  const ImVec2 bottom_right{top_left.x + size.x, top_left.y + size.y};
  list->AddRectFilled(top_left, bottom_right, IM_COL32(16, 19, 25, 255));
  list->PushClipRect(top_left, bottom_right, true);

  if (app.view.needs_fit) fit_view(app, size);

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.view.pan.x += ImGui::GetIO().MouseDelta.x;
    app.view.pan.y += ImGui::GetIO().MouseDelta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
    const float factor = std::pow(1.12f, ImGui::GetIO().MouseWheel);
    app.view.zoom = std::clamp(app.view.zoom * factor, 0.02f, 4000.0f);
    app.view.pan.x *= factor;
    app.view.pan.y *= factor;
  }

  const Transform to_screen{
      {top_left.x + size.x * 0.5f + app.view.pan.x,
       top_left.y + size.y * 0.5f + app.view.pan.y},
      app.view.zoom};

  const cam::LevelPlan* level = current_level(app);
  if (level) {
    if (app.show_stock)
      draw_paths(list, level->stock_slice, to_screen, IM_COL32(70, 130, 165, 200), 1.6f);
    if (app.show_part)
      draw_paths(list, level->part_slice, to_screen, IM_COL32(235, 200, 90, 220), 2.0f);

    double lowest = 0.0;
    double highest = 0.0;
    for (const auto& segment : level->feeds.segments) {
      if (segment.rapid) continue;
      highest = std::max(highest, segment.feed);
      lowest = lowest > 0.0 ? std::min(lowest, segment.feed) : segment.feed;
    }
    const double span = std::max(highest - lowest, 1e-6);

    for (const auto& segment : level->feeds.segments) {
      if (segment.rapid && !app.show_rapids) continue;
      ImU32 colour = IM_COL32(150, 150, 150, 255);
      float width = 1.7f;
      switch (app.colouring) {
        case Colouring::Feed:
          colour = segment.rapid
                       ? IM_COL32(70, 82, 100, 255)
                       : blend(IM_COL32(60, 110, 230, 255), IM_COL32(120, 235, 140, 255),
                               static_cast<float>((segment.feed - lowest) / span));
          break;
        case Colouring::Limit:
          colour = limit_colour(segment.limit);
          break;
        case Colouring::Load:
          colour = segment.rapid
                       ? IM_COL32(70, 82, 100, 255)
                       : blend(IM_COL32(70, 150, 255, 255), IM_COL32(255, 80, 70, 255),
                               static_cast<float>(segment.engagement));
          break;
        case Colouring::Move:
          colour = move_colour(segment.kind);
          break;
      }
      if (segment.rapid) width = 1.0f;
      if (segment.below_minimum_chip) width = 2.6f;
      list->AddLine(to_screen(segment.from), to_screen(segment.to), colour, width);
    }
  } else {
    const char* text = app.has_plan ? "This level has nothing to cut"
                                    : "Load a part and a stock model, then plan";
    const ImVec2 extent = ImGui::CalcTextSize(text);
    list->AddText({top_left.x + (size.x - extent.x) * 0.5f,
                   top_left.y + (size.y - extent.y) * 0.5f},
                  IM_COL32(120, 130, 145, 255), text);
  }

  list->PopClipRect();
  ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Tool Builder
// ---------------------------------------------------------------------------

template <typename Enum>
bool enum_combo(const char* label, Enum& value, const char* (*name)(Enum),
                const Enum* values, std::size_t count) {
  bool changed = false;
  if (ImGui::BeginCombo(label, name(value))) {
    for (std::size_t i = 0; i < count; ++i) {
      const bool selected = values[i] == value;
      if (ImGui::Selectable(name(values[i]), selected)) {
        value = values[i];
        changed = true;
      }
      if (selected) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  return changed;
}

void text_field(const char* label, std::string& value, std::size_t capacity = 128) {
  std::vector<char> buffer(std::max(capacity, value.size() + 1), '\0');
  std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
  if (ImGui::InputText(label, buffer.data(), buffer.size())) value = buffer.data();
}

void tool_builder(App& app) {
  if (!app.builder_open) return;
  ImGui::SetNextWindowSize(ImVec2(720, 640), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Tool builder", &app.builder_open)) {
    ImGui::End();
    return;
  }
  cam::Tool& tool = app.builder_tool;

  if (ImGui::BeginTabBar("builder-tabs")) {
    if (ImGui::BeginTabItem("Shape")) {
      text_field("Name", tool.name);
      text_field("Vendor", tool.vendor);
      text_field("Part number", tool.part_number);

      static const cam::ToolType kTypes[]{
          cam::ToolType::FlatEndmill, cam::ToolType::BallEndmill,
          cam::ToolType::BullNose,    cam::ToolType::Chamfer,
          cam::ToolType::VBit,        cam::ToolType::Drill,
          cam::ToolType::FaceMill,    cam::ToolType::ThreadMill,
          cam::ToolType::Engraver};
      if (enum_combo("Type", tool.type, cam::tool_type_name, kTypes,
                     std::size(kTypes))) {
        // A ball nose is defined by its corner radius, so keep them agreeing
        // rather than making the operator remember to.
        if (tool.type == cam::ToolType::BallEndmill)
          tool.corner_radius = tool.diameter * 0.5;
        else if (tool.type == cam::ToolType::FlatEndmill)
          tool.corner_radius = 0.0;
      }
      if (ImGui::DragScalar("Diameter (mm)", ImGuiDataType_Double, &tool.diameter, 0.05f) &&
          tool.type == cam::ToolType::BallEndmill)
        tool.corner_radius = tool.diameter * 0.5;
      ImGui::DragScalar("Corner radius (mm)", ImGuiDataType_Double, &tool.corner_radius, 0.05f);
      ImGui::InputInt("Flutes", &tool.flutes);
      ImGui::DragScalar("Flute length (mm)", ImGuiDataType_Double, &tool.flute_length, 0.5f);
      ImGui::DragScalar("Overall length (mm)", ImGuiDataType_Double, &tool.overall_length, 0.5f);
      ImGui::DragScalar("Shank diameter (mm)", ImGuiDataType_Double, &tool.shank_diameter, 0.05f);
      ImGui::DragScalar("Neck diameter (mm)", ImGuiDataType_Double, &tool.neck_diameter, 0.05f);
      ImGui::DragScalar("Neck length (mm)", ImGuiDataType_Double, &tool.neck_length, 0.5f);
      ImGui::DragScalar("Helix angle (deg)", ImGuiDataType_Double, &tool.helix_angle, 0.5f);
      ImGui::DragScalar("Included angle (deg)", ImGuiDataType_Double, &tool.included_angle, 0.5f);
      ImGui::DragScalar("Tip diameter (mm)", ImGuiDataType_Double, &tool.tip_diameter, 0.05f);
      ImGui::Checkbox("Centre cutting", &tool.center_cutting);
      ImGui::SameLine();
      ImGui::Checkbox("Variable helix", &tool.variable_helix);
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("Material and holder")) {
      static const cam::Substrate kSubstrates[]{
          cam::Substrate::HighSpeedSteel, cam::Substrate::Cobalt, cam::Substrate::Carbide,
          cam::Substrate::MicrograinCarbide, cam::Substrate::Ceramic};
      enum_combo("Substrate", tool.substrate, cam::substrate_name, kSubstrates,
                 std::size(kSubstrates));
      static const cam::Coating kCoatings[]{
          cam::Coating::None,  cam::Coating::TiN,   cam::Coating::TiCN,
          cam::Coating::TiAlN, cam::Coating::AlTiN, cam::Coating::ZrN,
          cam::Coating::DLC,   cam::Coating::Diamond};
      enum_combo("Coating", tool.coating, cam::coating_name, kCoatings,
                 std::size(kCoatings));
      static const cam::GeometryClass kClasses[]{
          cam::GeometryClass::General, cam::GeometryClass::HighHelixAluminium,
          cam::GeometryClass::RoughingChipbreaker, cam::GeometryClass::Finisher,
          cam::GeometryClass::HardMilling};
      enum_combo("Geometry", tool.geometry_class, cam::geometry_class_name, kClasses,
                 std::size(kClasses));
      static const cam::HolderType kHolders[]{
          cam::HolderType::ERCollet, cam::HolderType::ShrinkFit, cam::HolderType::Weldon,
          cam::HolderType::Hydraulic};
      enum_combo("Holder", tool.holder, cam::holder_type_name, kHolders,
                 std::size(kHolders));
      ImGui::DragScalar("Stickout (mm)", ImGuiDataType_Double, &tool.stickout, 0.5f);
      ImGui::SetItemTooltip(
          "Length from the collet face to the tip. Deflection goes as the cube of "
          "this, so it is the single number most worth getting right.");
      ImGui::DragScalar("Runout (mm TIR)", ImGuiDataType_Double, &tool.runout, 0.001f);
      ImGui::InputInt("Tool number", &tool.tool_number);
      ImGui::InputInt("Length offset", &tool.length_offset);
      ImGui::DragScalar("Cost", ImGuiDataType_Double, &tool.cost, 0.5f);
      ImGui::DragScalar("Expected life (min)", ImGuiDataType_Double,
                        &tool.expected_life_minutes, 1.0f);
      text_field("Notes", tool.notes, 512);
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("Cutting data")) {
      ImGui::TextWrapped(
          "One entry per material this tool is characterised for. An operation in "
          "a material with no entry is refused rather than guessed at.");
      ImGui::Separator();

      int remove = -1;
      for (std::size_t i = 0; i < tool.cutting_data.size(); ++i) {
        cam::CuttingData& data = tool.cutting_data[i];
        ImGui::PushID(static_cast<int>(i));
        const cam::Material* material = app.materials.find(data.material_id);
        const std::string title =
            material ? material->name : data.material_id + " (unknown)";
        if (ImGui::CollapsingHeader(title.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::DragScalar("Surface speed (m/min)", ImGuiDataType_Double,
                            &data.surface_speed, 1.0f);
          ImGui::DragScalar("Feed per tooth (mm)", ImGuiDataType_Double,
                            &data.feed_per_tooth, 0.001f);
          ImGui::DragScalar("Minimum chip (mm)", ImGuiDataType_Double,
                            &data.min_chip_thickness, 0.001f);
          ImGui::SetItemTooltip(
              "Below this the edge rubs instead of cutting. In a work-hardening "
              "material that is how a tool dies.");
          ImGui::DragScalar("Max radial (of diameter)", ImGuiDataType_Double,
                            &data.max_ae_over_d, 0.01f);
          ImGui::DragScalar("Max axial (of diameter)", ImGuiDataType_Double,
                            &data.max_ap_over_d, 0.05f);
          ImGui::DragScalar("Max ramp angle (deg)", ImGuiDataType_Double,
                            &data.max_ramp_angle, 0.1f);
          ImGui::DragScalar("Plunge feed factor", ImGuiDataType_Double,
                            &data.plunge_feed_factor, 0.01f);
          ImGui::Checkbox("Helical entry", &data.helical_entry);
          static const cam::CuttingDirection kDirections[]{
              cam::CuttingDirection::Climb, cam::CuttingDirection::Conventional,
              cam::CuttingDirection::Either};
          enum_combo("Direction", data.preferred_direction, cam::cutting_direction_name,
                     kDirections, std::size(kDirections));
          static const cam::CoolantNeed kCoolants[]{
              cam::CoolantNeed::Required, cam::CoolantNeed::Recommended,
              cam::CoolantNeed::Dry, cam::CoolantNeed::AirBlast};
          enum_combo("Coolant", data.coolant, cam::coolant_need_name, kCoolants,
                     std::size(kCoolants));

          // What the numbers above actually come to, live. Forty fields with no
          // feedback is a wall; forty fields that answer back is a form.
          if (material) {
            const double rpm =
                cam::requested_spindle_rpm(tool, data, *material, tool.diameter * 0.5);
            ImGui::Separator();
            ImGui::TextDisabled("At full depth on this material");
            ImGui::Text("Spindle          %.0f rpm", rpm);
            ImGui::Text("Feed at 50%% wide %.0f mm/min",
                        data.feed_per_tooth * tool.flutes * rpm);
            ImGui::Text("Feed at 10%% wide %.0f mm/min",
                        data.feed_per_tooth * cam::chip_thinning_factor(0.1) *
                            tool.flutes * rpm);
            ImGui::Text("Deepest pass     %.2f mm",
                        std::min(tool.flute_length, data.max_ap_over_d * tool.diameter));
            const double force = material->specific_cutting_energy * 1000.0 *
                                 std::min(tool.flute_length,
                                          data.max_ap_over_d * tool.diameter) *
                                 data.feed_per_tooth * tool.flutes * 0.5 / 3.14159265;
            ImGui::Text("Deflection       %.3f mm at that depth and half width",
                        cam::tip_deflection(tool, force * 1.2));
          }
          if (ImGui::Button("Remove this material")) remove = static_cast<int>(i);
        }
        ImGui::PopID();
      }
      if (remove >= 0)
        tool.cutting_data.erase(tool.cutting_data.begin() + remove);

      ImGui::Separator();
      std::vector<const char*> names;
      for (const auto& material : app.materials.materials)
        names.push_back(material.name.c_str());
      if (!names.empty()) {
        app.builder_material =
            std::clamp(app.builder_material, 0, static_cast<int>(names.size()) - 1);
        ImGui::SetNextItemWidth(240.0f);
        ImGui::Combo("##add-material", &app.builder_material, names.data(),
                     static_cast<int>(names.size()));
        ImGui::SameLine();
        if (ImGui::Button("Add material")) {
          const auto& material = app.materials.materials[app.builder_material];
          if (tool.data_for(material.id)) {
            say(app, material.name + " is already on this tool", true);
          } else {
            cam::CuttingData data;
            data.material_id = material.id;
            // Somewhere sensible to start from, by material group, rather than
            // an empty form.
            switch (material.group) {
              case cam::MaterialGroup::Aluminium:
              case cam::MaterialGroup::Brass:
                data.surface_speed = 250.0;
                data.feed_per_tooth = tool.diameter * 0.008;
                break;
              case cam::MaterialGroup::Plastic:
              case cam::MaterialGroup::Wood:
              case cam::MaterialGroup::Foam:
                data.surface_speed = 300.0;
                data.feed_per_tooth = tool.diameter * 0.014;
                break;
              case cam::MaterialGroup::Stainless:
              case cam::MaterialGroup::Titanium:
                data.surface_speed = 55.0;
                data.feed_per_tooth = tool.diameter * 0.004;
                data.max_ae_over_d = 0.3;
                break;
              default:
                data.surface_speed = 90.0;
                data.feed_per_tooth = tool.diameter * 0.005;
                data.max_ae_over_d = 0.4;
                break;
            }
            data.min_chip_thickness = data.feed_per_tooth * 0.3;
            tool.cutting_data.push_back(data);
          }
        }
      }
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::Separator();
  const auto problems = cam::validate(tool);
  for (const auto& problem : problems)
    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", problem.c_str());

  ImGui::BeginDisabled(!problems.empty());
  if (ImGui::Button("Save tool")) {
    if (app.builder_is_new || tool.id.empty())
      tool.id = app.tools.unique_id_for(tool.name);
    app.tools.upsert(tool);
    try {
      cam::save_tools(app.tools_file, app.tools);
      say(app, "Saved " + tool.name);
      app.builder_open = false;
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) app.builder_open = false;
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Panels
// ---------------------------------------------------------------------------

void depth_defaults(App& app, cam::Operation& operation) {
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.z_range(low, high)) return;
  operation.top_z = high;
  operation.bottom_z = low;
}

void load_meshes(App& app) {
  try {
    if (app.part_path[0] != '\0') {
      app.geometry.load_part(app.part_path.data());
      app.job.part_mesh = app.part_path.data();
    }
    if (app.stock_path[0] != '\0') {
      app.geometry.load_stock(app.stock_path.data());
      app.job.stock_mesh = app.stock_path.data();
    }
    for (auto& operation : app.job.operations) depth_defaults(app, operation);
    double low = 0.0;
    double high = 0.0;
    if (app.geometry.z_range(low, high)) {
      // Put the safe planes above the work now that there is work to be above.
      // Left at their defaults they would sit inside a part whose top is not at
      // zero, which is most parts.
      if (app.job.retract_z < high + 0.5) app.job.retract_z = high + 2.0;
      if (app.job.clearance_z < high + 1.0) app.job.clearance_z = high + 10.0;
      say(app, "Loaded. Z runs " + std::to_string(low) + " to " + std::to_string(high));
    }
    app.has_plan = false;
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
}

void job_panel(App& app) {
  ImGui::SeparatorText("Job");
  text_field("Name", app.job.name);
  ImGui::InputText("Part model", app.part_path.data(), app.part_path.size());
  ImGui::InputText("Stock model", app.stock_path.data(), app.stock_path.size());
  if (ImGui::Button("Load models")) load_meshes(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) app.view.needs_fit = true;

  ImGui::DragScalar("Origin X", ImGuiDataType_Double, &app.job.origin_x, 0.1f);
  ImGui::DragScalar("Origin Y", ImGuiDataType_Double, &app.job.origin_y, 0.1f);
  ImGui::DragScalar("Origin Z", ImGuiDataType_Double, &app.job.origin_z, 0.1f);
  ImGui::DragScalar("Clearance Z", ImGuiDataType_Double, &app.job.clearance_z, 0.1f);
  ImGui::DragScalar("Retract Z", ImGuiDataType_Double, &app.job.retract_z, 0.1f);
  text_field("Work offset", app.job.work_offset, 16);
}

void operations_panel(App& app) {
  ImGui::SeparatorText("Operations");
  if (app.job.operations.empty()) {
    cam::Operation operation;
    if (!app.tools.tools.empty()) operation.tool_id = app.tools.tools.front().id;
    if (!app.machines.machines.empty())
      operation.machine_id = app.machines.machines.front().id;
    if (!app.materials.materials.empty())
      operation.material_id = app.materials.materials.front().id;
    depth_defaults(app, operation);
    app.job.operations.push_back(operation);
  }

  std::vector<const char*> names;
  for (const auto& operation : app.job.operations) names.push_back(operation.name.c_str());
  app.selected_operation =
      std::clamp(app.selected_operation, 0, static_cast<int>(names.size()) - 1);
  ImGui::SetNextItemWidth(200.0f);
  ImGui::Combo("##operation", &app.selected_operation, names.data(),
               static_cast<int>(names.size()));
  ImGui::SameLine();
  if (ImGui::Button("Add")) {
    cam::Operation copy = app.job.operations[app.selected_operation];
    copy.name += " copy";
    app.job.operations.push_back(copy);
    app.selected_operation = static_cast<int>(app.job.operations.size()) - 1;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(app.job.operations.size() < 2);
  if (ImGui::Button("Remove")) {
    app.job.operations.erase(app.job.operations.begin() + app.selected_operation);
    app.selected_operation = 0;
  }
  ImGui::EndDisabled();

  cam::Operation& operation = app.job.operations[app.selected_operation];
  text_field("Operation name", operation.name);

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
  if (tool && !tool->data_for(operation.material_id))
    ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                       "This tool has no cutting data for that material");
  if (machine && machine->specifications_unverified)
    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f),
                       "This machine profile carries unverified limits");

  ImGui::DragScalar("Stepover (of diameter)", ImGuiDataType_Double,
                    &operation.stepover_ratio, 0.005f);
  ImGui::DragScalar("Stepdown (mm)", ImGuiDataType_Double, &operation.stepdown, 0.1f);
  if (tool)
    ImGui::TextDisabled("= %.3f mm wide, %zu passes", operation.stepover_ratio * tool->diameter,
                        cam::level_floors(operation).size());
  ImGui::DragScalar("Radial stock to leave", ImGuiDataType_Double,
                    &operation.radial_stock_to_leave, 0.02f);
  ImGui::DragScalar("Axial stock to leave", ImGuiDataType_Double,
                    &operation.axial_stock_to_leave, 0.02f);
  ImGui::DragScalar("Top Z", ImGuiDataType_Double, &operation.top_z, 0.1f);
  ImGui::DragScalar("Bottom Z", ImGuiDataType_Double, &operation.bottom_z, 0.1f);
  static const cam::CuttingDirection kDirections[]{cam::CuttingDirection::Climb,
                                                   cam::CuttingDirection::Conventional};
  enum_combo("Direction", operation.direction, cam::cutting_direction_name, kDirections,
             std::size(kDirections));
  ImGui::Checkbox("Lead in from the air", &operation.lead_in);
  ImGui::SetItemTooltip(
      "Seeds the wave one tool radius outside the stock so the first passes take "
      "up their bite over the run-in rather than at first contact.");
}

void feeds_panel(App& app) {
  if (app.job.operations.empty()) return;
  cam::FeedPolicy& policy = app.job.operations[app.selected_operation].feeds;
  ImGui::SeparatorText("Feed policy");
  ImGui::Checkbox("Radial chip thinning", &policy.chip_thinning);
  ImGui::DragScalar("Thinning ceiling", ImGuiDataType_Double, &policy.max_thinning_factor,
                    0.05f);
  ImGui::DragScalar("Load ceiling", ImGuiDataType_Double, &policy.load_ceiling, 0.01f);
  ImGui::SetItemTooltip(
      "A swept disc overcounts engagement at a direction change, so the measured "
      "load can pass 1 where the geometry is benign. This clamps it.");
  ImGui::Checkbox("Corner compensation", &policy.corner_compensation);
  ImGui::DragScalar("Lookahead (mm)", ImGuiDataType_Double, &policy.lookahead_distance,
                    0.1f);
  ImGui::DragScalar("Feed slew (mm/min per mm)", ImGuiDataType_Double,
                    &policy.feed_slew_limit, 10.0f);
  ImGui::DragScalar("Minimum feed", ImGuiDataType_Double, &policy.min_feed, 1.0f);
  ImGui::DragScalar("Maximum feed", ImGuiDataType_Double, &policy.max_feed, 10.0f);
  ImGui::DragScalar("Power margin", ImGuiDataType_Double, &policy.power_margin, 0.01f);
  ImGui::DragScalar("Deflection limit (mm)", ImGuiDataType_Double, &policy.max_deflection,
                    0.005f);
  ImGui::Checkbox("Spindle power", &policy.enable_power);
  ImGui::SameLine();
  ImGui::Checkbox("Cutting force", &policy.enable_force);
  ImGui::Checkbox("Deflection", &policy.enable_deflection);
  ImGui::SameLine();
  ImGui::Checkbox("Acceleration", &policy.enable_acceleration);
  ImGui::Checkbox("Travel disengaged moves fast", &policy.rapid_disengaged);
}

void plan_panel(App& app) {
  ImGui::SeparatorText("Plan");
  if (app.planner.running()) {
    ImGui::ProgressBar(static_cast<float>(app.planner.fraction()), ImVec2(-1, 0));
    ImGui::TextWrapped("%s", app.planner.stage().c_str());
    if (ImGui::Button("Cancel")) app.planner.cancel();
    return;
  }

  const bool ready = app.geometry.has_part() && app.geometry.has_stock();
  ImGui::BeginDisabled(!ready);
  if (ImGui::Button("Plan job", ImVec2(-1, 0))) {
    app.gcode.clear();
    app.post = {};
    app.planner.start(app.geometry, app.job, app.tools, app.machines, app.materials);
  }
  ImGui::EndDisabled();
  if (!ready) ImGui::TextDisabled("Load a part and a stock model first");

  if (!app.has_plan) return;

  const cam::PlanStats& stats = app.plan.stats;
  ImGui::Separator();
  ImGui::Text("Passes        %zu of %zu cut", stats.cut_levels, stats.levels);
  ImGui::Text("Cutting       %.0f mm in %.1f min", stats.cutting_distance,
              stats.cutting_minutes);
  ImGui::Text("Travel        %.0f mm in %.1f min", stats.rapid_distance,
              stats.rapid_minutes);
  ImGui::Text("Removed       %.0f mm3", stats.removed_volume);
  ImGui::Text("Peak load     %.2f of a full slot", stats.maximum_load);
  ImGui::Text("Peak power    %.3f kW", stats.maximum_power_kw);
  ImGui::Text("Peak force    %.0f N", stats.maximum_force);
  ImGui::Text("Peak deflect  %.3f mm", stats.maximum_deflection);
  if (stats.gouge_area > 0.0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Gouges        %.3f mm2",
                       stats.gouge_area);

  for (const auto& warning : app.plan.warnings)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "! %s", warning.c_str());

  // What actually held the feed down, weighted by how much of the job each one
  // cost rather than by how many segments it touched.
  const cam::LevelPlan* level = current_level(app);
  if (level && !level->feeds.stats.limit_distance.empty()) {
    ImGui::SeparatorText("What limits this pass");
    double total = 0.0;
    for (const auto& entry : level->feeds.stats.limit_distance) total += entry.second;
    for (const auto& entry : level->feeds.stats.limit_distance) {
      if (total <= 0.0) break;
      const float share = static_cast<float>(entry.second / total);
      ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                            ImGui::ColorConvertU32ToFloat4(limit_colour(entry.first)));
      ImGui::ProgressBar(share, ImVec2(-1, 14), "");
      ImGui::PopStyleColor();
      ImGui::SameLine(0.0f, 8.0f);
      ImGui::Text("%s  %.0f%%", cam::feed_limit_name(entry.first), share * 100.0);
    }
    ImGui::Text("Spindle %.0f rpm at %.0f m/min", level->feeds.stats.spindle_rpm,
                level->feeds.stats.surface_speed);
  }
}

void output_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("Output");
  if (ImGui::Button("Generate G-code", ImVec2(-1, 0))) {
    app.post = cam::post_job(app.plan, app.job, app.tools, app.machines);
    app.gcode = app.post.text;
    if (!app.post.ok()) say(app, app.post.error, true);
    else say(app, "Generated " + std::to_string(app.post.line_count) + " lines");
  }
  if (app.gcode.empty()) return;

  ImGui::Text("%zu lines, %zu cutting moves, %zu rapids", app.post.line_count,
              app.post.cutting_moves, app.post.rapid_moves);
  ImGui::Text("Estimated %.1f minutes", app.post.estimated_minutes);
  for (const auto& warning : app.post.warnings)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "! %s", warning.c_str());

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
    ImGui::BeginChild("gcode", ImVec2(0, 220), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::TextUnformatted(app.gcode.c_str(),
                           app.gcode.c_str() + std::min<std::size_t>(app.gcode.size(), 60000));
    ImGui::EndChild();
  }
}

void library_panel(App& app) {
  ImGui::SeparatorText("Tools");
  for (const auto& tool : app.tools.tools) {
    ImGui::PushID(tool.id.c_str());
    ImGui::Text("%d  %s", tool.tool_number, tool.name.c_str());
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 90.0f);
    if (ImGui::SmallButton("Edit")) {
      app.builder_tool = tool;
      app.builder_original_id = tool.id;
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
    app.builder_tool.tool_number = static_cast<int>(app.tools.tools.size()) + 1;
    app.builder_tool.length_offset = app.builder_tool.tool_number;
    app.builder_is_new = true;
    app.builder_open = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%zu machines, %zu materials", app.machines.machines.size(),
                      app.materials.materials.size());
}

void view_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("View");
  std::vector<const char*> operations;
  for (const auto& operation : app.plan.operations)
    operations.push_back(operation.operation_name.c_str());
  if (!operations.empty()) {
    app.selected_plan_operation =
        std::clamp(app.selected_plan_operation, 0, static_cast<int>(operations.size()) - 1);
    if (ImGui::Combo("Operation", &app.selected_plan_operation, operations.data(),
                     static_cast<int>(operations.size())))
      app.selected_level = 0;
  }
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (!levels.empty()) {
    int last = static_cast<int>(levels.size()) - 1;
    app.selected_level = std::clamp(app.selected_level, 0, last);
    ImGui::SliderInt("Pass", &app.selected_level, 0, last);
    ImGui::TextDisabled("Depth %.3f mm, cutting %.3f mm",
                        levels[app.selected_level].floor_z,
                        levels[app.selected_level].axial_depth);
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
  ImGui::Checkbox("Travel", &app.show_rapids);
}

void sidebar(App& app) {
  if (!app.banner.empty()) {
    const ImVec4 colour = app.banner_is_error ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f)
                                              : ImVec4(0.55f, 0.85f, 0.65f, 1.0f);
    ImGui::TextColored(colour, "%s", app.banner.c_str());
    ImGui::Separator();
  }
  job_panel(app);
  operations_panel(app);
  plan_panel(app);
  view_panel(app);
  output_panel(app);
  if (ImGui::CollapsingHeader("Feed policy")) feeds_panel(app);
  if (ImGui::CollapsingHeader("Libraries")) library_panel(app);
}

// ---------------------------------------------------------------------------

void bootstrap(App& app) {
  const std::filesystem::path data = cam::data_directory();
  const std::filesystem::path config = cam::config_directory();
  app.tools_file = config / "tools.json";
  app.preferences_file = config / "preferences.json";

  try {
    app.machines = cam::load_machines(data / "machines");
    app.materials = cam::load_materials(data / "materials");
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  if (app.machines.machines.empty())
    say(app, "No machine profiles found under " + (data / "machines").string(), true);

  try {
    app.tools = cam::load_tools(app.tools_file);
    if (app.tools.tools.empty()) {
      // First run: start from the shipped tools rather than an empty crib, and
      // write them into the user's own file so editing them is normal.
      app.tools = cam::load_tools(data / "starter-tools.json");
      if (!app.tools.tools.empty()) cam::save_tools(app.tools_file, app.tools);
    }
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  try {
    app.preferences = cam::load_preferences(app.preferences_file);
  } catch (const std::exception&) {
  }

  std::snprintf(app.save_path.data(), app.save_path.size(), "%s",
                (std::filesystem::current_path() / "tinglecam.nc").string().c_str());
}

}  // namespace

int main(int argc, char** argv) {
  App app;
  bootstrap(app);

  std::filesystem::path screenshot;
  std::filesystem::path gcode_out;
  double stepdown = 0.0;
  double stepover = 0.0;
  std::string tool_id;
  std::string machine_id;
  std::string material_id;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    const auto value = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(argument + " needs a value");
      return argv[++i];
    };
    try {
      if (argument == "--part") std::snprintf(app.part_path.data(), app.part_path.size(), "%s", value().c_str());
      else if (argument == "--stock") std::snprintf(app.stock_path.data(), app.stock_path.size(), "%s", value().c_str());
      else if (argument == "--screenshot") screenshot = value();
      else if (argument == "--gcode") gcode_out = value();
      else if (argument == "--tool") tool_id = value();
      else if (argument == "--machine") machine_id = value();
      else if (argument == "--material") material_id = value();
      else if (argument == "--stepdown") stepdown = std::stod(value());
      else if (argument == "--stepover") stepover = std::stod(value());
      else if (argument == "--help" || argument == "-h") {
        std::printf(
            "TingleCAM -- adaptive roughing over the wavefront engine\n\n"
            "  --part PATH        Part model (OBJ or STL)\n"
            "  --stock PATH       Stock model (OBJ or STL)\n"
            "  --tool ID          Tool from the library\n"
            "  --machine ID       Machine profile\n"
            "  --material ID      Workpiece material\n"
            "  --stepover RATIO   Radial engagement as a fraction of diameter\n"
            "  --stepdown MM      Axial depth per pass\n"
            "  --gcode PATH       Plan and post without opening a window, then exit\n"
            "  --screenshot PATH  Render one frame to a binary PPM and exit\n"
            "  -h, --help         Show this help and exit\n\n"
            "Tools live in %s\n"
            "Machines and materials are read from %s\n",
            app.tools_file.string().c_str(), cam::data_directory().string().c_str());
        return EXIT_SUCCESS;
      } else {
        std::fprintf(stderr, "Unknown option: %s\n", argument.c_str());
        return EXIT_FAILURE;
      }
    } catch (const std::exception& error) {
      std::fprintf(stderr, "%s\n", error.what());
      return EXIT_FAILURE;
    }
  }
  if (app.part_path[0] != '\0' || app.stock_path[0] != '\0') load_meshes(app);

  // Anything the command line set has to reach the operation the panels would
  // otherwise have created, so make it here and apply them to it.
  if (!app.job.operations.empty() || app.geometry.has_part()) {
    if (app.job.operations.empty()) {
      cam::Operation operation;
      if (!app.tools.tools.empty()) operation.tool_id = app.tools.tools.front().id;
      if (!app.machines.machines.empty())
        operation.machine_id = app.machines.machines.front().id;
      if (!app.materials.materials.empty())
        operation.material_id = app.materials.materials.front().id;
      depth_defaults(app, operation);
      app.job.operations.push_back(operation);
    }
    cam::Operation& operation = app.job.operations.front();
    if (!tool_id.empty()) operation.tool_id = tool_id;
    if (!machine_id.empty()) operation.machine_id = machine_id;
    if (!material_id.empty()) operation.material_id = material_id;
    if (stepdown > 0.0) operation.stepdown = stepdown;
    if (stepover > 0.0) operation.stepover_ratio = stepover;
  }

  // Planning and posting with no window at all, which is how this gets checked
  // without a display and how it fits into anything scripted.
  if (!gcode_out.empty()) {
    if (!app.geometry.has_part() || !app.geometry.has_stock()) {
      std::fprintf(stderr, "--gcode needs both --part and --stock\n");
      return EXIT_FAILURE;
    }
    const cam::JobPlan plan = cam::plan_job(
        app.geometry, app.job, app.tools, app.machines, app.materials,
        [](double fraction, const std::string& stage) {
          std::fprintf(stderr, "\r%3.0f%%  %-56s", fraction * 100.0, stage.c_str());
          std::fflush(stderr);
          return true;
        });
    std::fprintf(stderr, "\r%-64s\n", "");
    for (const auto& warning : plan.warnings)
      std::fprintf(stderr, "warning: %s\n", warning.c_str());
    if (!plan.ok()) {
      std::fprintf(stderr, "%s\n", plan.error.c_str());
      return EXIT_FAILURE;
    }
    const cam::PostResult post = cam::post_job(plan, app.job, app.tools, app.machines);
    for (const auto& warning : post.warnings)
      std::fprintf(stderr, "warning: %s\n", warning.c_str());
    if (!post.ok()) {
      std::fprintf(stderr, "%s\n", post.error.c_str());
      return EXIT_FAILURE;
    }
    cam::write_file(gcode_out, post.text);
    // What actually held the feed down, weighted by the distance it cost, which
    // is the first thing worth knowing about a plan.
    std::vector<std::pair<cam::FeedLimit, double>> limits;
    double total = 0.0;
    for (const auto& operation : plan.operations)
      for (const auto& level : operation.levels)
        for (const auto& entry : level.feeds.stats.limit_distance) {
          auto it = std::find_if(limits.begin(), limits.end(),
                                 [&](const auto& row) { return row.first == entry.first; });
          if (it == limits.end()) limits.emplace_back(entry.first, entry.second);
          else it->second += entry.second;
          total += entry.second;
        }
    std::sort(limits.begin(), limits.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    if (total > 0.0) {
      std::printf("What limits the feed:\n");
      for (const auto& entry : limits)
        std::printf("  %-20s %5.1f%%  %8.0f mm\n", cam::feed_limit_name(entry.first),
                    entry.second / total * 100.0, entry.second);
    }
    std::printf(
        "%zu passes cut of %zu, %.0f mm of cutting in %.1f min, peak load %.2f, "
        "peak power %.3f kW\n%zu lines written to %s\n",
        plan.stats.cut_levels, plan.stats.levels, plan.stats.cutting_distance,
        plan.stats.cutting_minutes + plan.stats.rapid_minutes, plan.stats.maximum_load,
        plan.stats.maximum_power_kw, post.line_count, gcode_out.string().c_str());
    return EXIT_SUCCESS;
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL initialisation failed: %s\n", SDL_GetError());
    return EXIT_FAILURE;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_Window* window = SDL_CreateWindow(
      "TingleCAM", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1500, 940,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) {
    std::fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
    return EXIT_FAILURE;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  SDL_GL_MakeCurrent(window, context);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::GetStyle().FrameRounding = 3.0f;
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 150");

  bool running = true;
  bool screenshot_pending = !screenshot.empty();
  int screenshot_delay = 0;
  if (screenshot_pending && app.geometry.has_part() && app.geometry.has_stock())
    app.planner.start(app.geometry, app.job, app.tools, app.machines, app.materials);
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT ||
          (event.type == SDL_WINDOWEVENT &&
           event.window.event == SDL_WINDOWEVENT_CLOSE &&
           event.window.windowID == SDL_GetWindowID(window)))
        running = false;
    }

    if (app.planner.take(app.plan)) {
      app.has_plan = app.plan.ok();
      app.selected_level = 0;
      app.selected_plan_operation = 0;
      app.view.needs_fit = true;
      if (!app.plan.ok()) say(app, app.plan.error, true);
      else say(app, "Planned " + std::to_string(app.plan.stats.cut_levels) + " passes");
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("TingleCAM", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::BeginChild("sidebar", ImVec2(400.0f, 0.0f), ImGuiChildFlags_Borders);
    sidebar(app);
    ImGui::EndChild();
    ImGui::SameLine();
    draw_canvas(app);
    ImGui::End();
    tool_builder(app);

    ImGui::Render();
    int width = 0;
    int height = 0;
    SDL_GL_GetDrawableSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(0.03f, 0.04f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (screenshot_pending && app.planner.running()) {
      // Wait for the plan, so the frame that gets captured has something in it.
    } else if (screenshot_pending && ++screenshot_delay < 3) {
      // One more frame, so the panels have laid out against the finished plan.
    } else if (screenshot_pending) {
      std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
      std::ofstream file(screenshot, std::ios::binary);
      file << "P6\n" << width << ' ' << height << "\n255\n";
      for (int row = height - 1; row >= 0; --row)
        file.write(reinterpret_cast<const char*>(
                       pixels.data() + static_cast<std::size_t>(row) * width * 3),
                   static_cast<std::streamsize>(width) * 3);
      screenshot_pending = false;
      running = false;
    }

    SDL_GL_SwapWindow(window);
  }

  app.planner.cancel();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return EXIT_SUCCESS;
}
