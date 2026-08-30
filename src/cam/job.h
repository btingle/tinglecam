#pragma once

#include "cam/feeds.h"
#include "cam/machine.h"
#include "cam/material.h"
#include "cam/tooling.h"
#include "geometry.h"
#include "mesh_slicer.h"
#include "simulation.h"
#include "topology.h"
#include "toolpath.h"

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace cam {

struct Operation {
  std::string name = "Rough";
  std::string tool_id;
  std::string machine_id;
  std::string material_id;

  // Engagement. The radial figure is a fraction of the cutter diameter and is
  // capped by whatever the tool says it will take in this material; the axial
  // one is millimetres and is capped by the flute length.
  double stepover_ratio = 0.4;
  double stepdown = 1.0;

  double radial_stock_to_leave = 0.2;
  double axial_stock_to_leave = 0.0;

  // Depth range, in job coordinates. Top is where the cutter starts, bottom is
  // the deepest floor the operation reaches.
  double top_z = 0.0;
  double bottom_z = -5.0;

  bool lead_in = true;
  // How much larger than its model the raw blank may be. The run-in is placed
  // so the first pass that touches the stock aims for a full stepover less
  // this, which leaves exactly this much headroom: an oversize blank pushes the
  // first bite back up towards the stepover instead of past it.
  double stock_deviation = 0.2;
  // Uniform distance between samples used by simulation and feed assignment.
  double path_pitch = 0.2;
  // When false, use the selected machine's range for this material.
  bool manual_feed_range = false;
  double full_engagement_feed = 500.0;
  double no_engagement_feed = 2000.0;
  int slice_grid = 32;
  double simulation_stock_inflation = 0.0;
};

struct Job {
  std::string name = "Untitled";
  std::filesystem::path part_mesh;
  std::filesystem::path stock_mesh;
  // Where the program's origin sits relative to the mesh coordinates. Added to
  // every posted coordinate.
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
  double clearance_z = 10.0;  // rapid plane above the work
  double retract_z = 2.0;     // where a lift inside the job goes
  std::string work_offset = "G54";
  std::vector<Operation> operations;
};

// One depth of cut: everything the engine produced for it, kept so the UI can
// draw any level and the post can walk them in order.
struct LevelPlan {
  double floor_z = 0.0;    // the depth this pass cuts down to
  double axial_depth = 0.0;  // a_p actually taken, which the last pass may trim
  wave::Paths64 part_slice;
  wave::Paths64 stock_slice;
  wave::WaveResult wave;
  wave::FrontLineage lineage;
  wave::Toolpath toolpath;
  wave::SimulationResult simulation;
  FeedSchedule feeds;
  bool nothing_to_cut = false;
};

struct PlanStats {
  std::size_t levels = 0;
  std::size_t cut_levels = 0;
  double cutting_distance = 0.0;
  double rapid_distance = 0.0;
  double cutting_minutes = 0.0;
  double rapid_minutes = 0.0;
  double tool_change_minutes = 0.0;
  double maximum_load = 0.0;
  double removed_volume = 0.0;  // mm^3
  double gouge_area = 0.0;
  std::size_t unsafe_plunges = 0;
  std::size_t unresolved_entries = 0;
  // Rapids the planner had to route over the part because the free space
  // around it is disconnected. A slice says nothing about how far the part
  // rises above it, so each one is an assumption about the clearance plane.
  std::size_t lifts_over_subject = 0;
  // Fronts the sequencer never reached. The material they bound is still
  // there when the operation finishes.
  std::size_t unreachable_fronts = 0;
  // Stock still standing that is not part of the finished shape. Roughing
  // always leaves some -- the radial allowance is exactly this -- so it is
  // reported rather than warned about.
  double excess_area = 0.0;
};

struct OperationPlan {
  std::string operation_name;
  std::string tool_id;
  double spindle_rpm = 0.0;
  std::vector<LevelPlan> levels;
  PlanStats stats;
  std::vector<std::string> warnings;
  std::string error;  // non-empty when the operation could not be planned

  bool ok() const { return error.empty(); }
};

struct JobPlan {
  std::vector<OperationPlan> operations;
  PlanStats stats;
  std::vector<std::string> warnings;
  std::string error;

  bool ok() const { return error.empty(); }
};

// Called as planning proceeds. `fraction` runs 0 to 1 across the whole job and
// `stage` says what is happening. Returning false asks planning to stop, and a
// cancelled plan comes back with an error saying so.
using ProgressCallback = std::function<bool(double fraction, const std::string& stage)>;

// Holds the sliced meshes so a replan does not rebuild the BVH. One instance
// per job; slicing is not called from more than one thread at a time.
class JobGeometry {
 public:
  void load_part(const std::filesystem::path& file);
  void load_stock(const std::filesystem::path& file);
  bool has_part() const { return !part_.empty(); }
  bool has_stock() const { return !stock_.empty(); }
  const wave::MeshSlicer& part() const { return part_; }
  const wave::MeshSlicer& stock() const { return stock_; }
  // The Z range shared by both meshes, which is what an operation may cover.
  bool z_range(double& low, double& high) const;

 private:
  wave::MeshSlicer part_;
  wave::MeshSlicer stock_;
};

// The depths an operation will cut to, deepest last. The first pass takes a
// full stepdown below the top; the last is trimmed so it lands exactly on the
// floor rather than overshooting it.
std::vector<double> level_floors(const Operation& operation);

OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const Material& material,
                             const ProgressCallback& progress,
                             double progress_from = 0.0,
                             double progress_to = 1.0);

JobPlan plan_job(const JobGeometry& geometry,
                 const Job& job,
                 const ToolLibrary& tools,
                 const MachineLibrary& machines,
                 const MaterialLibrary& materials,
                 const ProgressCallback& progress);

}  // namespace cam
