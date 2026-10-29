#pragma once
#include "cam/job.h"
#include "imgui.h"

namespace tinglecam_gui {
inline bool auto_pocket_strategy_controls(cam::PocketAutoParameters& settings) {
  bool changed=false;
  int strategy=static_cast<int>(settings.strategy);
  if (ImGui::Combo("Auto-pocket strategy",&strategy,
                   "Layered pocket\0Centre drilling\0Continuous helix\0")) {
    settings.strategy=static_cast<cam::PocketAutoStrategy>(strategy);changed=true;
  }
  if (settings.strategy==cam::PocketAutoStrategy::Layered) {
    changed |= ImGui::DragScalar("Helix angle (degrees)",ImGuiDataType_Double,
                                 &settings.helix_angle_degrees,0.1f);
  } else if (settings.strategy==cam::PocketAutoStrategy::ContinuousHelix) {
    ImGui::TextWrapped("One revolution descends one stepdown, followed by a full bottom cleanup turn. "
                       "Circular pockets only; each hole appears as one full-depth preview pass.");
  } else {
    ImGui::TextWrapped("Excavates only the tool-sized centre, leaving larger pocket walls. "
                       "Use a centre-cutting end mill or drill. One preview pass per hole.");
    ImGui::TextDisabled("Same-size drill fits allow 0.002 mm mesh/faceting tolerance.");
    int cycle=static_cast<int>(settings.drilling_cycle);
    if (ImGui::Combo("Drilling cycle",&cycle,"Feed drilling\0Peck: full retract\0Chip break: short retract\0")) {
      settings.drilling_cycle=static_cast<cam::DrillingCycle>(cycle);changed=true;
    }
    if (settings.drilling_cycle!=cam::DrillingCycle::Feed)
      changed |= ImGui::DragScalar("Peck depth (mm)",ImGuiDataType_Double,&settings.peck_depth,0.1f);
    if (settings.drilling_cycle==cam::DrillingCycle::ChipBreak)
      changed |= ImGui::DragScalar("Chip-break retract (mm)",ImGuiDataType_Double,&settings.chip_break_retract,0.1f);
    changed |= ImGui::DragScalar("Drilling clearance above opening (mm)",ImGuiDataType_Double,&settings.drilling_clearance,0.1f);
    changed |= ImGui::DragScalar("Bottom dwell (seconds)",ImGuiDataType_Double,&settings.bottom_dwell_seconds,0.1f);
  }
  return changed;
}
}
