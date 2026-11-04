#include "gui_internal.h"

#include <cctype>

namespace tinglecam_gui {
namespace {

// The rate slider: 1x in the middle, 20x at three quarters, 100x at the end,
// each span logarithmic.
constexpr float kMinimumPlaybackRate = 0.01f;
constexpr float kNormalPlaybackRate = 1.0f;
constexpr float kFastPlaybackRate = 20.0f;
constexpr float kMaximumPlaybackRate = 100.0f;

float playback_rate_at(float position) {
  position = std::clamp(position, 0.0f, 1.0f);
  if (position <= 0.5f)
    return kMinimumPlaybackRate *
           std::pow(kNormalPlaybackRate / kMinimumPlaybackRate, position / 0.5f);
  if (position <= 0.75f)
    return kNormalPlaybackRate *
           std::pow(kFastPlaybackRate / kNormalPlaybackRate, (position - 0.5f) / 0.25f);
  return kFastPlaybackRate *
         std::pow(kMaximumPlaybackRate / kFastPlaybackRate, (position - 0.75f) / 0.25f);
}

float playback_rate_position(float rate) {
  rate = std::clamp(rate, kMinimumPlaybackRate, kMaximumPlaybackRate);
  if (rate <= kNormalPlaybackRate)
    return 0.5f * std::log(rate / kMinimumPlaybackRate) /
           std::log(kNormalPlaybackRate / kMinimumPlaybackRate);
  if (rate <= kFastPlaybackRate)
    return 0.5f + 0.25f * std::log(rate / kNormalPlaybackRate) /
                      std::log(kFastPlaybackRate / kNormalPlaybackRate);
  return 0.75f + 0.25f * std::log(rate / kFastPlaybackRate) /
                     std::log(kMaximumPlaybackRate / kFastPlaybackRate);
}

void playback_options(App& app) {
  if (ImGui::Checkbox("Machine time", &app.playback.real_time)) sync_playback_time(app);
  ImGui::SetItemTooltip("Run at the commanded feeds; off runs at a fixed speed.");
  if (app.playback.real_time) {
    float position = playback_rate_position(static_cast<float>(app.playback.time_scale));
    ImGui::SetNextItemWidth(ui_size(app, 180.0f));
    char label[32];
    std::snprintf(label, sizeof label, "%.2gx", app.playback.time_scale);
    if (ImGui::SliderFloat("Rate", &position, 0.0f, 1.0f, label, ImGuiSliderFlags_NoInput))
      app.playback.time_scale = playback_rate_at(position);
  } else {
    float speed = static_cast<float>(app.playback.speed);
    ImGui::SetNextItemWidth(ui_size(app, 180.0f));
    if (ImGui::SliderFloat("Speed", &speed, 2.0f, 400.0f, "%.0f mm/s"))
      app.playback.speed = speed;
  }
  ImGui::Checkbox("Continue into the next pass", &app.playback.follow);
}

void legend_box(ImDrawList* list, ImVec2 at, ImVec2 size, ImU32 background) {
  list->AddRectFilled(at, ImVec2(at.x + size.x, at.y + size.y), background, 4.0f);
}

void gradient_legend(const App& app, ImDrawList* list, ImVec2 corner, const char* title,
                     ImU32 low, ImU32 high, const char* low_text, const char* high_text) {
  const float bar = ui_size(app, 140.0f);
  const float line = ImGui::GetTextLineHeight();
  const float pad = ui_size(app, 6.0f);
  const ImVec2 size(bar + pad * 2.0f, line * 3.0f + pad * 2.0f);
  const ImVec2 at(corner.x, corner.y - size.y);
  const CanvasColours colours = canvas_colours(app);
  legend_box(list, at, size, (colours.background & 0x00ffffff) | 0xd0000000);
  list->AddText(ImVec2(at.x + pad, at.y + pad), colours.caption, title);
  const ImVec2 a(at.x + pad, at.y + pad + line * 1.1f);
  const ImVec2 b(a.x + bar, a.y + line * 0.8f);
  list->AddRectFilledMultiColor(a, b, low, high, high, low);
  list->AddText(ImVec2(a.x, b.y + 2.0f), colours.caption, low_text);
  const float high_width = ImGui::CalcTextSize(high_text).x;
  list->AddText(ImVec2(b.x - high_width, b.y + 2.0f), colours.caption, high_text);
}

void swatch_legend(const App& app, ImDrawList* list, ImVec2 corner, const char* title,
                   const std::vector<std::pair<ImU32, const char*>>& entries) {
  const float line = ImGui::GetTextLineHeight();
  const float pad = ui_size(app, 6.0f);
  float width = ImGui::CalcTextSize(title).x;
  for (const auto& entry : entries)
    width = std::max(width, line * 1.4f + ImGui::CalcTextSize(entry.second).x);
  const ImVec2 size(width + pad * 2.0f, line * (entries.size() + 1) + pad * 2.0f);
  const ImVec2 at(corner.x, corner.y - size.y);
  const CanvasColours colours = canvas_colours(app);
  legend_box(list, at, size, (colours.background & 0x00ffffff) | 0xd0000000);
  list->AddText(ImVec2(at.x + pad, at.y + pad), colours.caption, title);
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const float y = at.y + pad + line * (i + 1);
    list->AddRectFilled(ImVec2(at.x + pad, y + line * 0.25f),
                        ImVec2(at.x + pad + line * 0.9f, y + line * 0.75f),
                        entries[i].first, 2.0f);
    list->AddText(ImVec2(at.x + pad + line * 1.4f, y), colours.caption, entries[i].second);
  }
}

void draw_legend(App& app, ImDrawList* list, ImVec2 corner) {
  switch (app.colouring) {
    case Colouring::Feed: {
      double lowest = 0.0, highest = 0.0;
      feed_range(app, lowest, highest);
      char low[32], high[32];
      std::snprintf(low, sizeof low, "%.0f", lowest);
      std::snprintf(high, sizeof high, "%.0f mm/min", highest);
      gradient_legend(app, list, corner, "Feed", feed_colour(lowest, lowest, highest),
                      feed_colour(highest, lowest, highest), low, high);
      break;
    }
    case Colouring::Load:
      gradient_legend(app, list, corner, "Radial engagement", engagement_colour(0.0),
                      engagement_colour(1.0), "0%", "full slot");
      break;
    case Colouring::Limit:
      swatch_legend(app, list, corner, "Feed set by",
                    {{limit_colour(cam::FeedLimit::Engagement), "Engagement"},
                     {limit_colour(cam::FeedLimit::Machine), "Machine maximum"},
                     {limit_colour(cam::FeedLimit::Plunge), "Plunge feed"},
                     {limit_colour(cam::FeedLimit::Rapid), "Rapid"}});
      break;
    case Colouring::Move: {
      std::vector<std::pair<ImU32, const char*>> entries;
      for (const auto kind : {wave::MoveKind::Cut, wave::MoveKind::Step,
                              wave::MoveKind::Link, wave::MoveKind::Entry,
                              wave::MoveKind::Retract})
        entries.push_back({move_colour(kind), wave::move_kind_name(kind)});
      swatch_legend(app, list, corner, "Move", entries);
      break;
    }
  }
}

void display_options(App& app, ViewKind kind) {
  static const char* kColourings[]{"Feed rate", "Feed limited by", "Engagement",
                                   "Move type"};
  static constexpr ModelShow kModels[]{ModelShow::None, ModelShow::Part,
                                       ModelShow::Stock, ModelShow::Both};
  if (kind == ViewKind::Setup) {
    if (ImGui::BeginCombo("Models", model_show_name(app.setup_models))) {
      for (const ModelShow value : kModels)
        if (ImGui::Selectable(model_show_name(value), value == app.setup_models))
          app.setup_models = value;
      ImGui::EndCombo();
    }
    ImGui::Checkbox("Wireframe", &app.wireframe);
    return;
  }
  int colouring = static_cast<int>(app.colouring);
  if (ImGui::Combo("Colour by", &colouring, kColourings, 4))
    app.colouring = static_cast<Colouring>(colouring);
  ImGui::Checkbox("Cutting moves", &app.show_cutting_paths);
  ImGui::Checkbox("Rapid moves", &app.show_rapid_travel);
  ImGui::Checkbox("All passes", &app.show_all_passes);
  ImGui::SetItemTooltip("Draw every pass of the operation, not only the one playing.");
  if (kind == ViewKind::Pass) {
    ImGui::Checkbox("Stock outline", &app.show_stock);
    ImGui::Checkbox("Part outline", &app.show_part);
    ImGui::Checkbox("Remaining material", &app.show_material);
  } else {
    if (ImGui::BeginCombo("Models", model_show_name(app.result_models))) {
      for (const ModelShow value : kModels)
        if (ImGui::Selectable(model_show_name(value), value == app.result_models))
          app.result_models = value;
      ImGui::EndCombo();
    }
    ImGui::Checkbox("Wireframe models", &app.wireframe);
    ImGui::Checkbox("Stock follows the cutter", &app.stack_follows_playback);
    ImGui::SetItemTooltip("Off shows the stock when the operation is finished.");
  }
  if (app.settings.show_debug && kind == ViewKind::Pass) {
    ImGui::SeparatorText("Debug");
    static const char* kNodeColourings[]{"Off", "By node", "By Z-group"};
    int node = static_cast<int>(app.node_debug_colouring);
    if (ImGui::Combo("Node boundaries", &node, kNodeColourings, 3))
      app.node_debug_colouring = static_cast<NodeDebugColouring>(node);
    ImGui::Checkbox("Entry-front Z-groups", &app.show_entry_front_chains);
    ImGui::Checkbox("Boundary contacts", &app.show_boundary_contacts);
  }
}

}  // namespace

void feed_range(const App& app, double& lowest, double& highest) {
  lowest = highest = 0.0;
  if (!app.has_plan || app.selected_plan_operation < 0 ||
      app.selected_plan_operation >= static_cast<int>(app.plan.operations.size()))
    return;
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  const auto include = [&](const cam::LevelPlan& level) {
    if (level.feeds.authoritative_motion) {
      const auto& stats = level.feeds.stats;
      highest = std::max(highest,stats.maximum_feed);
      if (stats.minimum_feed > 0)
        lowest = lowest > 0 ? std::min(lowest,stats.minimum_feed) : stats.minimum_feed;
      return;
    }
    for (const auto& phase : level.feeds.motion) {
      if (phase.rapid) continue;
      highest = std::max(highest, phase.rate);
      lowest = lowest > 0.0 ? std::min(lowest, phase.rate) : phase.rate;
    }
  };
  if (app.show_all_passes) {
    for (const auto& level : levels) include(level);
  } else if (const cam::LevelPlan* level = current_level(app)) {
    include(*level);
  }
}

float overlay_width(const App& app) {
  const float padding = ImGui::GetStyle().FramePadding.x * 2.0f;
  return ImGui::CalcTextSize("Display").x + ImGui::CalcTextSize("Fit").x + padding * 2.0f +
         ImGui::GetStyle().ItemSpacing.x + ui_size(app, 16.0f);
}

void view_caption(const App& app, ImDrawList* list, ImVec2 top_left, ImVec2 size, int line,
                  ImU32 colour, const char* text) {
  const ImVec2 at(top_left.x + 12.0f,
                  top_left.y + ui_size(app, 10.0f) + line * ImGui::GetTextLineHeightWithSpacing());
  // Clear of the buttons on the first line; the full width below them.
  const float right = top_left.x + size.x - (line == 0 ? overlay_width(app) : 12.0f);
  list->PushClipRect(at, ImVec2(right, at.y + ImGui::GetTextLineHeightWithSpacing()), true);
  list->AddText(at, colour, text);
  list->PopClipRect();
}

void view_overlay(App& app, ViewKind kind, ImVec2 top_left, ImVec2 size) {
  const float pad = ui_size(app, 8.0f);
  const float display = ImGui::CalcTextSize("Display").x + ImGui::GetStyle().FramePadding.x * 2.0f;
  const float fit = ImGui::CalcTextSize("Fit").x + ImGui::GetStyle().FramePadding.x * 2.0f;
  ImGui::SetCursorScreenPos(ImVec2(top_left.x + size.x - display - fit - pad -
                                       ImGui::GetStyle().ItemSpacing.x,
                                   top_left.y + pad));
  if (ImGui::Button("Fit")) {
    app.view.needs_fit = true;
    app.orbit.pan = ImVec2(0.0f, 0.0f);
    app.orbit.zoom = 1.0f;
  }
  ImGui::SetItemTooltip("Fit the view (F)");
  ImGui::SameLine();
  if (ImGui::Button("Display")) ImGui::OpenPopup("display");
  if (ImGui::BeginPopup("display")) {
    display_options(app, kind);
    ImGui::EndPopup();
  }
  if (kind != ViewKind::Setup && app.has_plan)
    draw_legend(app, ImGui::GetWindowDrawList(),
                ImVec2(top_left.x + pad, top_left.y + size.y - pad));
  // Back inside the canvas, so the cursor cannot grow the parent.
  ImGui::SetCursorScreenPos(top_left);
}

namespace {

// The move at the playback position as numbers, and the pass it belongs to.
void motion_readout(const App& app, const cam::LevelPlan* level) {
  if (!level) {
    ImGui::TextDisabled("No plan");
    return;
  }
  if (level->nothing_to_cut || level->feeds.motion.empty()) {
    ImGui::TextDisabled("Nothing to cut on this pass");
    return;
  }
  const float gap = ImGui::GetStyle().ItemSpacing.x * 2.0f;
  const cam::MotionCursor motion =
      cam::feed_motion_at_seconds(level->feeds, app.playback.motion_seconds);
  std::string name = motion_phase_name(motion);
  name[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));
  if (motion.rapid) {
    ImGui::TextUnformatted(name.c_str());
  } else {
    ImGui::Text("%s %.0f mm/min", name.c_str(), motion.rate);
    // Engagement is only measured on moves at one height.
    if (motion.from_z == motion.to_z) {
      ImGui::SameLine(0.0f, gap);
      ImGui::Text("Engagement %.1f%%", motion.engagement * 100.0);
      const cam::Machine* machine = app.machines.find(app.effective_context().machine_id);
      if (machine && motion.rate >= cam::rapid_feed(*machine) - 1e-6) {
        ImGui::SameLine();
        ImGui::TextDisabled("(machine maximum)");
      }
    }
  }
  ImGui::SameLine(0.0f, gap);
  ImGui::Text("Z %.3f mm", motion.z + app.effective_context().origin_z);
  if (motion.fixture_contact) {
    ImGui::SameLine(0.0f, gap);
    ImGui::TextColored(severity_colour(app, LogSeverity::Error), "Fixture / keep-away contact");
  }
  if (app.settings.show_debug && ImGui::BeginItemTooltip()) {
    ImGui::Text("XYZ travel %.1f of %.1f mm", motion.motion_distance,
                cam::feed_motion_length(level->feeds));
    ImGui::Text("Rapid plane Z %.3f mm", app.plan.operations[app.selected_plan_operation].rapid_plane + app.effective_context().origin_z);
    ImGui::EndTooltip();
  }

  const cam::FeedStats& stats = level->feeds.stats;
  ImGui::TextDisabled("Feed %.0f-%.0f mm/min", stats.minimum_feed, stats.maximum_feed);
  ImGui::SameLine(0.0f, gap);
  ImGui::TextDisabled("Engagement mean %.1f%%, peak %.1f%%",
                      stats.mean_engagement * 100.0,
                      stats.maximum_engagement * 100.0);
  ImGui::SetItemTooltip("The mean is over moves that removed material");
}

}  // namespace

void transport_bar(App& app) {
  const cam::LevelPlan* level = current_level(app);
  const bool ready = app.has_plan && level != nullptr;
  ImGui::BeginDisabled(!ready);
  if (ImGui::Button("|<")) {
    app.playback.distance = 0.0;
    app.playback.motion_seconds = 0.0;
  }
  ImGui::SetItemTooltip("Start of the pass (Home)");
  ImGui::SameLine();
  if (ImGui::Button(app.playback.playing ? "Pause" : "Play", ImVec2(ui_size(app, 56.0f), 0)))
    app.playback.playing = !app.playback.playing;
  ImGui::SetItemTooltip("Space");
  ImGui::SameLine();
  if (ImGui::Button(">|") && level) {
    app.playback.distance = level_length(level);
    app.playback.motion_seconds = cam::feed_schedule_seconds(level->feeds);
    app.playback.playing = false;
  }
  ImGui::SetItemTooltip("End of the pass");
  ImGui::SameLine();

  const auto* levels = ready ? &app.plan.operations[app.selected_plan_operation].levels : nullptr;
  const int count = levels ? static_cast<int>(levels->size()) : 1;
  int pass = std::clamp(app.selected_level, 0, std::max(0, count - 1)) + 1;
  char format[48];
  std::snprintf(format, sizeof format, "Pass %%d of %d", count);
  ImGui::SetNextItemWidth(ui_size(app, 150.0f));
  if (ImGui::SliderInt("##pass", &pass, 1, std::max(1, count), format,
                       ImGuiSliderFlags_AlwaysClamp)) {
    app.selected_level = pass - 1;
    app.playback.distance = 0.0;
    app.playback.motion_seconds = 0.0;
  }
  if (level && ImGui::BeginItemTooltip()) {
    ImGui::Text("Floor Z %.3f mm, cutting %.3f mm", level->floor_z + app.effective_context().origin_z,
                level->axial_depth);
    const auto& subdivisions = app.plan.operations[app.selected_plan_operation].subdivisions;
    if (subdivisions.size() > 1)
      ImGui::Text("Terrace %zu, %s", level->coarse_level + 1,
                  level->finishing_index == 0 ? "coarse descent" : "upward cleanup");
    ImGui::EndTooltip();
  }
  ImGui::SameLine();

  const double total = level_length(level);
  float where = static_cast<float>(app.playback.distance);
  const double whole = level ? cam::feed_schedule_seconds(level->feeds) : 0.0;
  const std::string time = clock_text(app.playback.motion_seconds) + " / " + clock_text(whole);
  const float time_width = ImGui::CalcTextSize("00:00.0 / 00:00.0").x;
  const float options = ImGui::CalcTextSize("...").x + ImGui::GetStyle().FramePadding.x * 2.0f;
  ImGui::SetNextItemWidth(std::max(ui_size(app, 60.0f),
                                   ImGui::GetContentRegionAvail().x - time_width - options -
                                       ImGui::GetStyle().ItemSpacing.x * 2.0f));
  if (ImGui::SliderFloat("##along", &where, 0.0f, static_cast<float>(std::max(total, 1e-6)),
                         "%.0f mm")) {
    app.playback.distance = where;
    sync_playback_time(app);
  }
  ImGui::SetItemTooltip("Distance along this pass");
  ImGui::SameLine();
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(time.c_str());
  ImGui::SameLine();
  ImGui::EndDisabled();
  if (ImGui::Button("...")) ImGui::OpenPopup("playback-options");
  ImGui::SetItemTooltip("Playback options");
  if (ImGui::BeginPopup("playback-options")) {
    playback_options(app);
    ImGui::EndPopup();
  }
  motion_readout(app, ready ? level : nullptr);
}

void gcode_preview_window(App& app) {
  if (!app.show_gcode_preview) return;
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 520.0f), ui_size(app, 560.0f)),
                           ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("G-code", &app.show_gcode_preview)) {
    ImGui::End();
    return;
  }
  if (app.gcode.empty()) {
    ImGui::TextDisabled("Nothing exported yet. Export G-code to see it here.");
  } else {
    if (app.posted_operations.empty()) ImGui::Text("Whole job");
    else ImGui::Text("%zu checked operation%s", app.posted_operations.size(),
                     app.posted_operations.size() == 1 ? "" : "s");
    ImGui::SameLine();
    ImGui::TextDisabled("%zu lines, %zu cutting moves, %zu rapids, about %.1f min",
                        app.post.line_count, app.post.cutting_moves, app.post.rapid_moves,
                        app.post.estimated_minutes);
    if (ImGui::SmallButton("Copy all")) ImGui::SetClipboardText(app.gcode.c_str());
    ImGui::BeginChild("gcode", ImVec2(0, 0), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);
    ImGuiListClipper clipper;
    static std::vector<std::size_t> starts;
    static const std::string* indexed = nullptr;
    static std::size_t indexed_size = 0;
    if (indexed != &app.gcode || indexed_size != app.gcode.size()) {
      starts.assign(1, 0);
      for (std::size_t i = 0; i < app.gcode.size(); ++i)
        if (app.gcode[i] == '\n') starts.push_back(i + 1);
      indexed = &app.gcode;
      indexed_size = app.gcode.size();
    }
    clipper.Begin(static_cast<int>(starts.size()));
    while (clipper.Step())
      for (int line = clipper.DisplayStart; line < clipper.DisplayEnd; ++line) {
        const std::size_t begin = starts[line];
        const std::size_t end = line + 1 < static_cast<int>(starts.size())
                                    ? starts[line + 1] - 1 : app.gcode.size();
        ImGui::TextUnformatted(app.gcode.data() + begin, app.gcode.data() + end);
      }
    ImGui::EndChild();
  }
  ImGui::End();
}

}  // namespace tinglecam_gui
