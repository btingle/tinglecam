#include "cam/hole_config.h"
#include <nlohmann/json.hpp>
#include <stdexcept>
namespace cam {
using nlohmann::json;
namespace {
template<class T> void read(const json& j, const char* key, T& value) {
  if(j.contains(key) && !j.at(key).is_null()) j.at(key).get_to(value);
}
HoleStrategy strategy(const json& j) {
  const auto s=hole_strategy_from_name(j.get<std::string>());
  if(!s) throw std::runtime_error("Unknown hole strategy");
  return *s;
}
}
json hole_parameters_json(const HoleParameters& p) {
  json systems=json::array();
  for(const auto& h:p.systems) {
    json levels=json::array();
    for(const auto& l:h.levels) levels.push_back({{"radius",l.radius},{"top_z",l.top_z},
        {"bottom_z",l.bottom_z},{"through",l.through},{"selected",l.selected},
        {"strategy",l.strategy?json(hole_strategy_name(*l.strategy)):json(nullptr)},
        {"tool",l.tool_id},{"fit_error",l.fit_error}});
    systems.push_back({{"center_x",h.center_x},{"center_y",h.center_y},
        {"selected",h.selected},{"levels",std::move(levels)}});
  }
  return {{"automatic",p.automatic},{"inspection_depth",p.inspection_depth},
      {"circle_tolerance",p.circle_tolerance},{"counterbores",p.counterbores},
      {"probe_inset",p.probe_inset},{"slice_epsilon",p.slice_epsilon},
      {"strategy",hole_strategy_name(p.strategy)},{"adaptive_radius",p.adaptive_radius},
      {"helix_angle_degrees",p.helix_angle_degrees},{"radial_margin",p.radial_margin},
      {"drilling",{{"depth_reference",drill_depth_name(p.drilling.depth_reference)},
          {"peck_depth",p.drilling.peck_depth},{"peck_clearance",p.drilling.peck_clearance},
          {"breakthrough",p.drilling.breakthrough},{"depth_adjustment",p.drilling.depth_adjustment}}},
      {"systems",std::move(systems)},{"detection_signature",p.detection_signature},
      {"id",p.id},{"source_id",p.source_id}};
}
HoleParameters hole_parameters_from_json(const json& j) {
  if(!j.is_object()) throw std::runtime_error("Expected hole parameters object");
  HoleParameters p;
  read(j,"automatic",p.automatic);read(j,"inspection_depth",p.inspection_depth);
  read(j,"circle_tolerance",p.circle_tolerance);read(j,"counterbores",p.counterbores);
  read(j,"probe_inset",p.probe_inset);read(j,"slice_epsilon",p.slice_epsilon);
  read(j,"adaptive_radius",p.adaptive_radius);read(j,"helix_angle_degrees",p.helix_angle_degrees);
  read(j,"radial_margin",p.radial_margin);read(j,"detection_signature",p.detection_signature);
  read(j,"id",p.id);read(j,"source_id",p.source_id);
  if(j.contains("strategy")) p.strategy=strategy(j.at("strategy"));
  if(j.contains("drilling")) {
    const auto& d=j.at("drilling");
    if(d.contains("depth_reference")) {
      const auto mode=drill_depth_from_name(d.at("depth_reference").get<std::string>());
      if(!mode) throw std::runtime_error("Unknown drilling depth reference");
      p.drilling.depth_reference=*mode;
    }
    read(d,"peck_depth",p.drilling.peck_depth);read(d,"peck_clearance",p.drilling.peck_clearance);
    read(d,"breakthrough",p.drilling.breakthrough);read(d,"depth_adjustment",p.drilling.depth_adjustment);
  }
  if(j.contains("systems")) {
    if(!j.at("systems").is_array()) throw std::runtime_error("Expected hole systems array");
    p.systems.clear();
    for(const auto& h:j.at("systems")) {
      if(!h.is_object() || !h.contains("levels") || !h.at("levels").is_array())
        throw std::runtime_error("Expected a hole system with a levels array");
      HoleSystem system;read(h,"center_x",system.center_x);read(h,"center_y",system.center_y);
      read(h,"selected",system.selected);system.levels.clear();
      for(const auto& l:h.at("levels")) {
        if(!l.is_object()) throw std::runtime_error("Expected a hole level object");
        HoleLevel level;read(l,"radius",level.radius);read(l,"top_z",level.top_z);
        read(l,"bottom_z",level.bottom_z);read(l,"through",level.through);
        read(l,"selected",level.selected);read(l,"fit_error",level.fit_error);
        read(l,"tool",level.tool_id);
        if(l.contains("strategy") && !l.at("strategy").is_null()) level.strategy=strategy(l.at("strategy"));
        system.levels.push_back(std::move(level));
      }
      p.systems.push_back(std::move(system));
    }
  }
  return p;
}
}
