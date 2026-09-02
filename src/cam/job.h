#pragma once

#include "cam/feeds.h"
#include "cam/contours.h"
#include "cam/machine.h"
#include "cam/material.h"
#include "cam/tooling.h"
#include "geometry.h"
#include "mesh_slicer.h"
#include "simulation.h"
#include "topology.h"
#include "toolpath.h"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace cam {

enum class OperationMode {
  Roughing,
  ContourFinishing,
};

struct Operation {
  std::string name = "Rough";
  std::string tool_id;
  std::string machine_id;
  std::string material_id;
  OperationMode mode = OperationMode::Roughing;

  // Engagement. The radial figure is a fraction of the cutter diameter and is
  // capped by whatever the tool says it will take in this material; the axial
  // one is millimetres and is capped by the flute length.
  double stepover_ratio = 0.4;
  double stepdown = 1.0;

  double radial_stock_to_leave = 0.2;
  double axial_stock_to_leave = 0.0;

  // Depth range, measured downwards from the top of the work -- the taller of
  // the part and the blank. Depths rather than heights because the top of the
  // work is the one reference that is neither the part's own (a blank standing
  // proud of the part would never be cut) nor the origin's (which moves, and
  // must not move what gets cut). `depth_from` is where the cutter starts,
  // `depth_to` the deepest floor the operation reaches; loading a model sets
  // them to the whole of it.
  double depth_from = 0.0;
  double depth_to = 5.0;

  bool lead_in = true;
  // How much larger than its model the raw blank may be. The run-in is placed
  // so the first pass that touches the stock aims for a full stepover less
  // this, which leaves exactly this much headroom: an oversize blank pushes the
  // first bite back up towards the stepover instead of past it.
  double stock_deviation = 0.2;
  // Uniform distance between samples used by simulation and feed assignment.
  double path_pitch = 0.2;
  // Entry fronts are associated between adjacent Z layers only when the whole
  // lower curve lies inside this envelope around the upper curve.
  double entry_front_match_tolerance = 3.0;
  // Contour derivation is independent of routing. In finishing mode the
  // already-sliced subject is offset to the cutter centre and may be clipped
  // against the available stock before the generic traversal planner sees it.
  bool clip_contours_to_stock = true;
  ContourDirectionPolicy contour_direction = ContourDirectionPolicy::Either;
  bool rough_boundary_cleanup = true;
  // When false, use the selected machine's range for this material.
  bool manual_feed_range = false;
  double full_engagement_feed = 500.0;
  double no_engagement_feed = 2000.0;
  int slice_grid = 32;
  double simulation_stock_inflation = 0.0;
  // Entry fronts the wave must not propagate from: one group of anchor points
  // per declined entry, a point on each layer that entry appeared at. The
  // wavefront pass discovers what is on offer; this is the answer fed back into
  // it. Anchors are matched geometrically rather than by index, so a selection
  // survives a change of stepdown or stepover that renumbers everything. The
  // grouping is kept because a declined entry is absent from the plan it was
  // declined out of, so this is the only remaining record that it exists at all.
  // Empty means every entry propagates.
  std::vector<std::vector<wave::Point64>> muted_entries;
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
  // With no stock model, the blank is the part's bounding box grown by this
  // much in X and Y. Z is left at the part's own range: a blank oversize in Z
  // means facing the top off, which is a pass in its own right rather than
  // something a fallback should decide to add.
  bool box_stock = false;
  double box_stock_margin = 5.0;
  // Every disconnected move retracts to Z0 plus this height. Keeping a single
  // safe plane avoids pretending a 2D slice can prove a lower traverse safe.
  double rapid_clearance_height = 10.0;
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
  // This pass reaches the next one without lifting: `transition` is the way
  // across at this pass's own floor, ending at the next pass's first point, so
  // all that separates them is one pass depth of descent.
  bool stays_down_after = false;
  wave::Path64 transition;
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
  // Entry fronts the operation was told not to propagate from.
  std::size_t muted_entries = 0;
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
  wave::InterLayerTopology inter_layer;
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
  // The safe plane this plan was costed against, in job coordinates. Recorded
  // rather than recomputed, so the post cannot write a different one.
  double rapid_plane = 0.0;

  bool ok() const { return error.empty(); }
};

// Called as planning proceeds. `fraction` runs 0 to 1 across the whole job and
// `stage` says what is happening. Returning false asks planning to stop, and a
// cancelled plan comes back with an error saying so.
using ProgressCallback = std::function<bool(double fraction, const std::string& stage)>;

// Holds the sliced meshes so a replan does not rebuild the BVH, and the slices
// themselves so choosing entry fronts does not re-cut them. Slicing dominates a
// wavefront pass and depends on nothing the selection can change, so a pass run
// again for a different set of entries costs only the wave. One instance per
// job; slicing is not called from more than one thread at a time.
class JobGeometry {
 public:
  void load_part(const std::filesystem::path& file);
  void load_stock(const std::filesystem::path& file);
  // Stand a rectangular blank in for a stock model: the part's bounding box
  // grown by `margin` in X and Y. A loaded stock model always wins.
  // Drops a loaded stock model, so a blank named and then unnamed does not go
  // on winning over the box that replaced it.
  void clear_stock();
  void set_box_stock(bool enabled, double margin);
  bool box_stock() const { return box_stock_ && stock_.empty(); }
  bool has_part() const { return !part_.empty(); }
  bool has_stock() const { return !stock_.empty() || (box_stock_ && !part_.empty()); }
  const wave::MeshSlicer& part() const { return part_; }
  const wave::MeshSlicer& stock() const { return stock_; }
  // The Z range spanned by either mesh, which is what an operation may cover.
  // The union rather than the overlap: a blank standing proud of the part is
  // material to be faced off, and a part reaching below the blank is a hole
  // the operation still has to descend into. Restricting an operation to the
  // depths both models occupy silently drops both.
  bool work_range(double& low, double& high) const;

  // Unioned outlines at a depth, remembered between passes. Loading a mesh
  // drops what was remembered for it.
  const wave::Paths64& part_outline(double z, int grid) const;
  const wave::Paths64& stock_outline(double z, int grid) const;
  void forget_slices();

 private:
  // Keyed on the exact depth and grid asked for; nothing is interpolated.
  struct SliceKey {
    std::int64_t z = 0;
    int grid = 0;
    bool operator<(const SliceKey& other) const {
      return z != other.z ? z < other.z : grid < other.grid;
    }
  };
  const wave::Paths64& outline(const wave::MeshSlicer& slicer,
                               std::map<SliceKey, wave::Paths64>& cache,
                               double z, int grid) const;

  wave::MeshSlicer part_;
  wave::MeshSlicer stock_;
  bool box_stock_ = false;
  double box_margin_ = 5.0;
  mutable std::map<SliceKey, wave::Paths64> part_slices_;
  mutable std::map<SliceKey, wave::Paths64> stock_slices_;
  // The stand-in blank, which is one rectangle at every depth.
  mutable wave::Paths64 box_;
};

// The depths an operation will cut to, deepest last. The first pass takes a
// full stepdown below the top; the last is trimmed so it lands exactly on the
// floor rather than overshooting it.
// The floors of every pass, in job coordinates, deepest last. `work_top` is the
// top of the work the operation's depths are measured down from; `stepdown` is
// the figure after the tool's ceiling has been applied to it.
std::vector<double> level_floors(const Operation& operation, double stepdown,
                                 double work_top);

// Slices, propagates the wave, and builds the lineage and inter-layer topology
// for every depth. Which entries were declined is an input here, not an output:
// a declined entry is one the wave is never seeded from.
OperationPlan plan_wavefronts(const JobGeometry& geometry,
                              const Operation& operation,
                              const Tool& tool,
                              const Machine& machine,
                              const Material& material,
                              const ProgressCallback& progress,
                              double progress_from = 0.0,
                              double progress_to = 1.0);

// The one plane every disconnected move traverses at, in job coordinates; the
// post adds origin_z to it. A 2D slice cannot certify a lower traverse, so this
// is the only height at which a rapid ever crosses the job.
//
// `highest` is the top of the tallest thing in the way, part or blank, in model
// coordinates. The plane is that plus the job's clearance -- a distance over the
// work rather than a height over the origin, so moving the origin cannot move
// the plane, and cannot therefore change what a lift costs or oblige a re-plan
// to say where the job's zero is.
//
// The planner resolves it once and `JobPlan::rapid_plane` records it, so the
// post writes the height every lift was costed against instead of working out
// a second opinion.
double job_rapid_plane(const Job& job, double highest);

// The top of the tallest loaded model, or negative infinity if none is loaded.
// This is what an operation's depths are measured down from.
double highest_model_z(const JobGeometry& geometry);

// Job-coordinate Z of a depth below the top of the work.
inline double z_at_depth(double work_top, double depth) { return work_top - depth; }

// Carries a wavefront plan the rest of the way: toolpath, simulation, feeds,
// and the rolled-up statistics. The plan is modified in place.
void plan_toolpaths(OperationPlan& plan,
                    const Operation& operation,
                    const Tool& tool,
                    const Machine& machine,
                    const Material& material,
                    const ProgressCallback& progress,
                    double progress_from = 0.0,
                    double progress_to = 1.0,
                    double rapid_plane = 10.0);

// Both stages back to back.
OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const Material& material,
                             const ProgressCallback& progress,
                             double progress_from = 0.0,
                             double progress_to = 1.0,
                             double rapid_plane = 10.0);

// One appearance of an entry front: the curve the wave would start from on one
// level, and the slab of depth that level stands for.
struct EntryCurve {
  wave::Path64 path;
  bool closed = false;
  double top_z = 0.0;
  double bottom_z = 0.0;
};

// An entry front on offer, gathered down its Z chain. Fronts that matched no
// chain stand on their own, or a one-level entry could never be declined.
// This is the unit the interface offers and the unit `Operation::muted_entries`
// records.
struct EntryOption {
  // One anchor per appearance, which is what muting stores. Anchors are matched
  // by position rather than by index, so a choice survives renumbering.
  std::vector<wave::Point64> anchors;
  std::vector<EntryCurve> curves;
  double top_z = 0.0;
  double bottom_z = 0.0;
  bool muted = false;
};

// The blank at one depth, so the fronts can be seen against the material they
// start from without a plan having been made.
struct EntryLevel {
  wave::Paths64 stock_slice;
  double floor_z = 0.0;
  double axial_depth = 0.0;
};

struct EntryDiscovery {
  std::vector<EntryLevel> levels;
  std::vector<EntryOption> entries;
  std::vector<std::string> warnings;
  std::string error;

  bool ok() const { return error.empty(); }
};

// Every entry front the operation could start from, found by seeding the wave
// on each level and stopping there. Seeding is the whole of entry discovery:
// the fronts are generation zero, which is built before the first propagation
// step and does not depend on any of them.
//
// Nothing here is muted. Declining an entry drops a whole seed component and
// leaves every other component exactly where it was, so this list is the same
// list whatever has been declined -- which is what lets a choice be made and
// unmade without planning anything. `Operation::muted_entries` is matched
// against it by position and sets the `muted` flags.
EntryDiscovery discover_entries(const JobGeometry& geometry,
                                const Operation& operation,
                                const Tool& tool,
                                const Machine& machine,
                                const Material& material,
                                const ProgressCallback& progress = {});

JobPlan plan_job(const JobGeometry& geometry,
                 const Job& job,
                 const ToolLibrary& tools,
                 const MachineLibrary& machines,
                 const MaterialLibrary& materials,
                 const ProgressCallback& progress);

const char* operation_mode_name(OperationMode value);

}  // namespace cam
