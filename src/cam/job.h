#pragma once

#include "cam/feeds.h"
#include "cam/contours.h"
#include "cam/machine.h"
#include "cam/material.h"
#include "cam/tooling.h"
#include "analysis.h"
#include "geometry.h"
#include "mesh_slicer.h"
#include "simulation.h"
#include "topology.h"
#include "toolpath.h"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class OperationMode {
  Roughing,
  ContourFinishing,
  Pocket,
  PlaneClearing,
};

enum class PocketShape { Circle, Rectangle, RoundedRectangle };
const char* pocket_shape_name(PocketShape shape);
std::optional<PocketShape> pocket_shape_from_name(const std::string& name);
bool is_parametric(OperationMode mode);

struct ClearingArea {
  double center_x = 0.0;
  double center_y = 0.0;
  double width = 30.0;
  double height = 30.0;
};

struct PocketParameters {
  PocketShape shape = PocketShape::Rectangle;
  double center_x = 0.0;
  double center_y = 0.0;
  double top_z = 0.0;
  double width = 30.0;
  double height = 30.0;
  double depth = 5.0;
  // Circle radius, or corner radius for a rounded rectangle.
  double radius = 5.0;
  double helix_angle_degrees = 3.0;
};

struct PlaneParameters {
  double top_z = 0.0;
  double depth = 1.0;
  std::vector<ClearingArea> areas{{}};
};

struct Operation {
  std::string name = "Rough";
  std::string tool_id;
  std::string machine_id;
  std::string material_id;
  OperationMode mode = OperationMode::Roughing;

  // Strategies belong to operations, never to the tool library.
  double stepover_ratio = 0.4;
  double stepdown = 1.0;
  double spindle_rpm = 12000.0;
  double plunge_feed = 100.0;
  bool coolant = false;
  // Zero selects the tool's library position (1-based), or the tool number
  // for H. Explicit machine assignments belong to this operation.
  int tool_number = 0;
  int length_offset = 0;
  PocketParameters pocket;
  PlaneParameters plane;
  // Roughing may be repeated at evenly interleaved Z offsets. One is the
  // ordinary pass; N adds N-1 stock-aware finishing passes while keeping this
  // operation as a single editable/postable unit.
  int finishing_subdivisions = 1;

  double radial_stock_to_leave = 0.2;
  // Stock left on the surfaces a pass comes down onto. The part is sliced this
  // far below each floor, so a pass stops above anything within that distance.
  // Floors stay on their own grid, so what is left is this to this plus a
  // stepdown. Walls are the radial allowance's job.
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
  double path_pitch = 1.0;
  // Entry fronts are associated between adjacent Z layers only when the whole
  // lower curve lies inside this envelope around the upper curve.
  double entry_front_match_tolerance = 3.0;
  // A terminal split branch below this many square stepovers remains part of
  // the propagated wave but does not introduce a lineage/node boundary.
  double tiny_split_area_factor = wave::kDefaultTinySplitAreaFactor;
  // Contour derivation is independent of routing. In finishing mode the
  // already-sliced subject is offset to the cutter centre and may be clipped
  // against the available stock before the generic traversal planner sees it.
  bool clip_contours_to_stock = true;
  ContourDirectionPolicy contour_direction = ContourDirectionPolicy::Either;
  bool rough_boundary_cleanup = true;
  // Numerical allowance excluded from the cutter radius when deciding how far
  // outside remaining stock a later subdivision's cleanup centreline may run.
  // Planning clamps the effective distance to zero at the cutter radius.
  double cleanup_tolerance = 0.15;
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
struct StockSlab {
  double bottom = 0, top = 0;
  std::shared_ptr<const wave::Paths64> region;
};

struct LevelPlan {
  std::vector<StockSlab> local_stock;
  // Parametric operations carry their actual XYZ path in feeds.motion. The
  // planar projection remains available for stock simulation and 2D views.
  bool explicit_motion = false;
  double floor_z = 0.0;    // the depth this pass cuts down to
  double axial_depth = 0.0;  // a_p actually taken, which the last pass may trim
  // Execution group and position inside it. Subdivision groups are flattened
  // into OperationPlan::levels for posting and playback.
  std::size_t subdivision = 0;
  std::size_t subdivision_level = 0;
  wave::Paths64 part_slice;
  wave::Paths64 stock_slice;
  // Material present when this operation reaches the level. The nominal stock
  // slice above is kept for planning and reference; this one has the swept
  // material from every earlier operation removed and is the stock actually
  // handed to simulation. It may be empty when an earlier operation already
  // cleared the whole slab.
  wave::Paths64 incoming_stock;
  wave::WaveResult wave;
  wave::FrontLineage lineage;
  wave::LayerTopology topology;
  // The recursively committed wave layers used for actual roughing motion.
  // `wave`, `lineage`, and `topology` above are their combined final result for
  // topology/debug consumers.
  std::vector<wave::WavefrontStage> wave_stages;
  std::size_t analysis_trials = 0;
  std::size_t accepted_analysis_trials = 0;
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

struct OperationSubdivision {
  std::size_t first_level = 0;
  std::size_t level_count = 0;
  // How far this group's floors are raised from the base roughing floors.
  double depth_shift = 0.0;
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

// Wall-clock measurements and scale counters for one planning run. These are
// deliberately part of the result rather than printed from deep inside the
// geometry code: command-line benchmarks can report them, while GUI plans and
// tests remain quiet and can inspect the same authoritative numbers.
struct PlanningMetrics {
  double slicing_milliseconds = 0.0;
  double preparation_milliseconds = 0.0;
  double wavefront_milliseconds = 0.0;
  double lineage_milliseconds = 0.0;
  double inter_layer_milliseconds = 0.0;
  double toolpath_milliseconds = 0.0;
  double cleanup_milliseconds = 0.0;
  double stitching_milliseconds = 0.0;
  double resampling_milliseconds = 0.0;
  double simulation_milliseconds = 0.0;
  double feeds_milliseconds = 0.0;
  double total_milliseconds = 0.0;

  std::size_t wave_generations = 0;
  std::size_t fronts = 0;
  std::size_t analysis_layers = 0;
  std::size_t analysis_trials = 0;
  std::size_t accepted_analysis_trials = 0;
  std::size_t toolpath_moves = 0;
  std::size_t simulation_intervals = 0;
  std::size_t empty_simulation_intervals = 0;
  std::size_t gouge_bounds_rejections = 0;
};

struct OperationPlan {
  // False for an operation the plan holds no result for at all: one added
  // since the plan was made, or one a partial re-plan has not reached. Its
  // error says so, so every reader that already skips a failed operation skips
  // this one too; the flag is what tells a failure apart from a gap.
  bool planned = false;
  bool composed_stock = false;
  // Prior stacks, clipped once during planning; playback changes local slabs only.
  std::vector<StockSlab> preview_context;
  std::string operation_name;
  std::string tool_id;
  std::string machine_id;
  std::string material_id;
  double spindle_rpm = 0.0;
  std::vector<LevelPlan> levels;
  std::vector<OperationSubdivision> subdivisions;
  wave::InterLayerTopology inter_layer;
  PlanStats stats;
  PlanningMetrics metrics;
  std::vector<std::string> warnings;
  std::string error;  // non-empty when the operation could not be planned

  bool ok() const { return error.empty(); }
};

// The operations already planned, in job order, as the source of the stock an
// operation inherits. Pointers rather than the plans themselves, because a
// partial re-plan reads its predecessors out of the plan already on screen and
// appends the ones it has just computed, and those two do not sit next to each
// other in memory. Copying an operation's plan to put them side by side would
// cost a good part of the planning this exists to avoid.
using PriorOperations = std::vector<const OperationPlan*>;

struct JobPlan {
  std::vector<OperationPlan> operations;
  PlanStats stats;
  PlanningMetrics metrics;
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
// themselves so repeated planning does not re-cut them. One instance per job.
//
// Slicing may be called from more than one thread: a plan runs on a worker
// while the window draws the same models, and both fill the same cache. What
// is shared is the remembering, and that is locked. Replacing a mesh is not:
// it empties the caches, and a slice already handed out by reference would go
// with them, so a caller loading a model has to have stopped its planner
// first.
class JobGeometry {
 public:
  JobGeometry() = default;
  // Movable, because loading a job builds a fresh one and hands it over. The
  // mutex below is not part of what moves: it guards this instance's caches,
  // and the instance being moved into has its own to guard what arrives with.
  JobGeometry(JobGeometry&& other) noexcept { take(std::move(other)); }
  JobGeometry& operator=(JobGeometry&& other) noexcept {
    if (this != &other) take(std::move(other));
    return *this;
  }

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
  void take(JobGeometry&& other) noexcept {
    std::scoped_lock lock(slices_, other.slices_);
    part_ = std::move(other.part_);
    stock_ = std::move(other.stock_);
    box_stock_ = other.box_stock_;
    box_margin_ = other.box_margin_;
    part_slices_ = std::move(other.part_slices_);
    stock_slices_ = std::move(other.stock_slices_);
    box_ = std::move(other.box_);
  }

  // The slice caches are filled lazily, and a plan running on the worker
  // thread fills them while the window draws the same models from the frame
  // thread. Both are reads as far as the caller is concerned; only the
  // remembering underneath is a write, and this is what makes it one.
  mutable std::mutex slices_;
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
// for every depth. When prior operations are supplied, roughing propagates
// through the stock they actually left instead of reconstructing the original
// blank for each operation.
OperationPlan plan_wavefronts(const JobGeometry& geometry,
                              const Operation& operation,
                              const Tool& tool,
                              const Machine& machine,
                              const Material& material,
                              const ProgressCallback& progress,
                              double progress_from = 0.0,
                              double progress_to = 1.0,
                              const PriorOperations& prior_operations = {},
                              const OperationPlan* prior_subdivisions = nullptr);

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

// The same plane, resolved from everything in the job that stands in the way:
// the models, and the top surface of every dimension-driven operation, which
// has no mesh to be read off. Every operation is costed against this one
// figure, so a partial re-plan has to arrive at the same one the operations it
// is not re-planning were costed against -- and when it does not, those
// operations are out of date, which is exactly what recording it lets a caller
// notice.
double job_rapid_plane(const Job& job, const JobGeometry& geometry);

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
                    double rapid_plane = 10.0,
                    bool finishing_subdivision = false);

// Both stages back to back.
OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const Material& material,
                             const ProgressCallback& progress,
                             double progress_from = 0.0,
                             double progress_to = 1.0,
                             double rapid_plane = 10.0,
                             const PriorOperations& prior_operations = {},
                             const OperationPlan* prior_subdivisions = nullptr);

// Resolves the material present at one Z plane after a sequence of operations.
// A level removes material only through the axial slab it represents. This is
// deliberately public: pocketing and stock-aware contour clipping can consume
// the same authoritative handoff as simulation instead of rebuilding a second
// stock model later.
wave::Paths64 stock_after_operations_at_z(
    const wave::Paths64& nominal_stock, double z,
    const PriorOperations& operations);
wave::Paths64 stock_after_planned_operation_at_z(
    const wave::Paths64& nominal_stock, double z, const OperationPlan& operation);

// Chooses the execution-record layer that supplies a physical stack layer
// while subdivision playback is in progress. Completed subdivisions retain
// their own stock; layers belonging to a future subdivision mirror the active
// subdivision at the same local depth until that future group begins.
std::size_t subdivision_preview_source(const OperationPlan& operation,
                                       std::size_t active_level,
                                       std::size_t target_level);

// What one re-planning run produced. `operations` covers job operations
// [first, first + operations.size()) and nothing else; the caller splices them
// into the plan it already has.
struct PartialPlan {
  std::size_t first = 0;
  std::vector<OperationPlan> operations;
  double rapid_plane = 0.0;
  // Non-empty when the run could not be made at all -- cancelled, or asked for
  // a range the job does not have. Individual operations carry their own
  // failures instead.
  std::string error;
};

// Plans operations [first, last] of the job and nothing else, reading the
// stock handed to the first of them out of `earlier`: the plans of the
// operations above it, which are not recomputed. `earlier` holds one entry per
// operation above `first`, in order.
//
// Planning an operation against a predecessor that was never planned is not
// possible -- there is no stock to inherit -- and is the caller's to refuse
// before asking. An entry that is present but failed is not a gap: the planner
// already treats a failed operation as one that removed nothing, which is what
// it did.
PartialPlan plan_operation_range(const JobGeometry& geometry,
                                 const Job& job,
                                 const ToolLibrary& tools,
                                 const MachineLibrary& machines,
                                 const MaterialLibrary& materials,
                                 const ProgressCallback& progress,
                                 std::size_t first, std::size_t last,
                                 const PriorOperations& earlier);

// Rebuilds the job-wide statistics, metrics, warnings and error from whatever
// the operations now say. Called after a partial re-plan has replaced some of
// them, so the totals describe the plan as it now stands rather than the run
// that last touched part of it.
void summarise_job(JobPlan& plan);

JobPlan plan_job(const JobGeometry& geometry,
                 const Job& job,
                 const ToolLibrary& tools,
                 const MachineLibrary& machines,
                 const MaterialLibrary& materials,
                 const ProgressCallback& progress);

const char* operation_mode_name(OperationMode value);

}  // namespace cam
