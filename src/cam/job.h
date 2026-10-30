#pragma once

#include "cam/feeds.h"
#include "cam/cad.h"
#include "cam/contours.h"
#include "cam/machine.h"
#include "cam/material.h"
#include "cam/tooling.h"
#include "planning/toolpath.h"
#include "simulation/heightmap.h"
#include "wave/analysis.h"
#include "wave/geometry.h"
#include "wave/mesh_slicer.h"
#include "wave/topology.h"

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
  SurfaceFinishing,
  Hole,
};

// A rounded rectangle covers the former circle and square cases as well:
// radius == half the smaller dimension is a circle, and radius == 0 is a
// square-cornered pocket (the cutter still rounds its internal corners).
enum class PocketShape { RoundedRectangle };
const char* pocket_shape_name(PocketShape shape);
std::optional<PocketShape> pocket_shape_from_name(const std::string& name);
bool is_parametric(OperationMode mode);

struct ClearingArea {
  double center_x = 0.0;
  double center_y = 0.0;
  double width = 30.0;
  double height = 30.0;
  double top_z = 0.0;
  double depth = 1.0;
};

struct PocketParameters {
  PocketShape shape = PocketShape::RoundedRectangle;
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
  std::vector<ClearingArea> areas{{}};
};

struct PocketExclusionArea {
  bool enabled = false;
  double center_x = 0.0;
  double center_y = 0.0;
  double width = 30.0;
  double height = 30.0;
};

struct PocketAutoParameters {
  // Measured down from the top of the part. The downward-envelope slice at
  // this depth supplies the candidate pocket contours.
  double check_depth = 1.0;
  double helix_angle_degrees = 3.0;
  std::vector<PocketExclusionArea> exclusions{{}};
};

// A native CAD face selection is bound to its source file, since face numbers
// can change after a model is edited or re-exported.
enum class HoleStrategy { Drill, Helical, Pocket, Adaptive };
const char* hole_strategy_name(HoleStrategy);
std::optional<HoleStrategy> hole_strategy_from_name(const std::string&);
enum class DrillDepth { Tip, FullDiameter };
const char* drill_depth_name(DrillDepth);
std::optional<DrillDepth> drill_depth_from_name(const std::string&);
struct DrillParameters {
  DrillDepth depth_reference = DrillDepth::Tip;
  double peck_depth = 0.0; // Zero drills in one feed move.
  double peck_clearance = 1.0;
  bool partial_peck_retract = false;
  double peck_retract_distance = 0.5; // Lift from the last peck floor, capped at hole top.
  double breakthrough = 0.2; // Applied only to through holes.
  double depth_adjustment = 0.0; // Positive extends below the nominal hole floor.
};
struct HoleLevel {
  double radius = 3.0;
  double top_z = 0.0;
  double bottom_z = -5.0;
  bool through = false;
  bool selected = true;
  std::optional<HoleStrategy> strategy;
  std::string tool_id; // Empty inherits the operation tool.
  double fit_error = 0.0;
};
struct HoleSystem {
  double center_x = 0.0, center_y = 0.0;
  bool selected = true;
  std::vector<HoleLevel> levels{{}}; // Shallowest/widest first.
};
struct HoleParameters {
  bool automatic = false;
  double inspection_depth = 0.0; // Down from part top; zero inspects just inside it.
  double circle_tolerance = 0.03; // Maximum radial boundary deviation, mm.
  bool counterbores = true;
  double probe_inset = 0.05;
  double slice_epsilon = 0.01;
  HoleStrategy strategy = HoleStrategy::Helical;
  double adaptive_radius = 3.0;
  double helix_angle_degrees = 3.0;
  double radial_margin = 0.0;
  DrillParameters drilling;
  std::vector<HoleSystem> systems{{}};
  std::string detection_signature;
  std::string id; // Stable identity for a linked pre-drilling operation.
  std::string source_id; // Read selected milling holes from this Hole operation.
};

struct SurfaceParameters {
  std::vector<std::size_t> faces;
  std::string model_fingerprint;
  double stepover = 0.3;
  double chord_tolerance = 0.01;
  double allowance = 0.0;
  bool along_v = true;
  bool bidirectional = true;
  bool flip_normals = false;
  bool manual_order = false;
  // Indices refer to the joined contour preview identified by this signature.
  // Changed geometry/settings require explicit reselection rather than silently
  // transferring a root to a different contour.
  std::string order_signature;
  std::vector<std::size_t> order_roots;
};

struct Operation {
  std::string name = "Rough";
  std::string tool_id;
  OperationMode mode = OperationMode::Roughing;

  // Strategies belong to operations, never to the tool library.
  double stepover_ratio = 0.4;
  double stepdown = 1.0;
  // Feeds and speeds come from the tool's cutting data for the job material.
  CuttingOverrides cutting;
  bool coolant = false;
  std::vector<PocketParameters> pockets{{}};
  bool auto_identify_pockets = false;
  PocketAutoParameters pocket_auto;
  HoleParameters holes;
  // Pocket mode only, and applied to identified pockets as well as entered
  // ones. Walls finish this far inside the requested outline, which leaves
  // that much for a finishing pass; a negative margin cuts that far past them,
  // for a pocket that has to end up oversize.
  double pocket_radial_margin = 0.0;
  PlaneParameters plane;
  SurfaceParameters surface;
  // Descend through all coarse floors, then clean each terrace upward,
  // deepest first. N divides each actual coarse interval into N equal parts.
  int finishing_subdivisions = 1;
  bool adaptive_cleanup_stepover = true;
  // Fraction of tool diameter. The effective ceiling is at least the base
  // stepover, so enabling adaptation never narrows an existing roughing cut.
  double cleanup_stepover_limit = 0.5;
  // Intermediate depths below work top; each ends a coarse depth band and
  // restarts its stepdown grid. Cleanup waits until every band has descended.
  std::vector<double> depth_breaks;

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

  // Additional XY planning margin around each roughing stock slice. Applied
  // once to the current incoming outline, independently of lead-in headroom.
  double stock_boundary_dilation = 0.0;
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
  int slice_grid = 32;
};

struct BoxStockMargins {
  double minus_x = 5.0;
  double plus_x = 5.0;
  double minus_y = 5.0;
  double plus_y = 5.0;
  double minus_z = 0.0;
  double plus_z = 0.0;
};

// A tool's place in the machine for this job. Length offset 0 means the same
// number as the tool.
struct ToolAssignment {
  std::string tool_id;
  int number = 0;
  int length_offset = 0;
};

struct Job {
  std::string name = "Untitled";
  std::filesystem::path part_mesh;
  std::filesystem::path stock_mesh;
  ModelOrientation part_orientation, stock_orientation;
  // Stock translation in setup axes, applied after its model orientation.
  double stock_offset_x = 0.0, stock_offset_y = 0.0, stock_offset_z = 0.0;
  // One machine runs the program and one material is on it.
  std::string machine_id;
  std::string material_id;
  // XY spacing of the stock simulation grid, mm.
  double simulation_pitch = 0.02;
  // Every tool the operations use, with its pocket number. Tools missing from
  // the table are numbered after it in order of first use.
  std::vector<ToolAssignment> tools;
  // Where the program's origin sits relative to the mesh coordinates. Added to
  // every posted coordinate.
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
  // Geometry used by the GUI's origin picker; posted coordinates use the
  // explicit offsets above regardless of the selected reference.
  bool origin_relative_to_part = false;
  // With no stock model, the blank is the part's bounding box grown by these
  // directional margins.
  bool box_stock = false;
  BoxStockMargins box_stock_margins;
  // Every disconnected move retracts to Z0 plus this height. Keeping a single
  // safe plane avoids pretending a 2D slice can prove a lower traverse safe.
  double rapid_clearance_height = 10.0;
  // Vertical entries rapid through air, then use plunge feed this far above
  // first contact with the current remaining stock under the whole cutter.
  double plunge_clearance_height = 1.0;
  std::string work_offset = "G54";
  std::vector<Operation> operations;
};

// The job's tool table with every tool the operations use, missing entries
// numbered after the highest number already taken, in order of first use.
std::vector<ToolAssignment> resolved_tool_table(const Job& job);
const ToolAssignment* find_tool_assignment(const std::vector<ToolAssignment>& table,
                                           const std::string& tool_id);

struct StockCheckpoint {
  std::size_t phase = 0;
  std::shared_ptr<const wave::Heightmap> stock;
  double motion_distance = 0.0;
};

// One depth of cut: everything the engine produced for it, kept so the UI can
// draw any level and the post can walk them in order.
struct LevelPlan {
  std::vector<StockCheckpoint> stock_checkpoints;
  std::vector<double> motion_distances;
  // Parametric operations carry their actual XYZ path in feeds.motion. The
  // planar projection remains available for stock simulation and 2D views.
  bool explicit_motion = false;
  std::string tool_id; // Optional tool override for a hole/counterbore level.
  double floor_z = 0.0;    // the depth this pass cuts down to
  // Axial slab height; cleanup uses the remaining coarse terrace as its bound.
  double axial_depth = 0.0;
  double radial_width = 0.0; // planned stepover in mm
  std::size_t coarse_level = 0;
  int finishing_index = 0; // 0 = coarse descent; 1..N-1 = upward cleanup
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
  // Everywhere the cutter's footprint passes over at this floor. The passes
  // of an operation still being planned hand stock to each other through it.
  wave::Paths64 swept;
  FeedSchedule feeds;
  bool nothing_to_cut = false;
  // This pass leaves the cutter at its floor. A roughing pass links to the next
  // through the stock simulation; within one depth, `transition` carries it.
  bool stays_down_after = false;
  // Pocket layers share a known opening, so prefer that stay-down connection
  // whenever the router can prove it is safe.
  bool stay_down_preferred = false;
  wave::Path64 transition;
};

struct OperationSubdivision {
  std::size_t first_level = 0;
  std::size_t level_count = 0;
  // Cleanup height above its coarse floor; zero for the initial descent.
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
  // Swept cutter area reaching into the part slice, roughing and contours.
  double gouge_area = 0.0;
  // Roughing descents that land on stock and have to plunge.
  std::size_t unsafe_plunges = 0;
  std::size_t skipped_entries = 0;
  // Rapids the planner had to route over the part because the free space
  // around it is disconnected. A slice says nothing about how far the part
  // rises above it, so each one is an assumption about the clearance plane.
  std::size_t lifts_over_subject = 0;
  // Fronts the sequencer never reached. The material they bound is still
  // there when the operation finishes.
  std::size_t unreachable_fronts = 0;
};

// Timing measurements and scale counters for one planning run. These are
// deliberately part of the result rather than printed from deep inside the
// geometry code: command-line benchmarks can report them, while GUI plans and
// tests remain quiet and can inspect the same authoritative numbers.
struct PlanningMetrics {
  // Sampling time sums work across finishing workers.
  double surface_sampling_milliseconds = 0.0;
  double surface_linking_milliseconds = 0.0;
  std::size_t surface_samples = 0;
  std::size_t surface_seam_joins = 0;
  std::size_t surface_stay_down_links = 0;
  std::size_t surface_retracts = 0;


  double slicing_milliseconds = 0.0;
  double preparation_milliseconds = 0.0;
  double wavefront_milliseconds = 0.0;
  double lineage_milliseconds = 0.0;
  double inter_layer_milliseconds = 0.0;
  double toolpath_milliseconds = 0.0;
  double cleanup_milliseconds = 0.0;
  double stitching_milliseconds = 0.0;
  double resampling_milliseconds = 0.0;
  double xyz_simulation_milliseconds = 0.0;
  double feeds_milliseconds = 0.0;
  // Phase times sum elapsed work across concurrently planned cleanup passes.
  // Total and parallel_cleanup remain wall-clock time, not summed worker time.
  double parallel_cleanup_milliseconds = 0.0;
  double cleanup_work_milliseconds = 0.0;
  double total_milliseconds = 0.0;

  std::size_t wave_generations = 0;
  std::size_t fronts = 0;
  std::size_t analysis_layers = 0;
  std::size_t analysis_trials = 0;
  std::size_t accepted_analysis_trials = 0;
  std::size_t toolpath_moves = 0;
};

struct OperationPlan {
  // False for an operation the plan holds no result for at all: one added
  // since the plan was made, or one a partial re-plan has not reached. Its
  // error says so, so every reader that already skips a failed operation skips
  // this one too; the flag is what tells a failure apart from a gap.
  bool planned = false;
  std::shared_ptr<const wave::Heightmap> stock_before, stock_after;
  std::string operation_name;
  std::string tool_id;
  struct IdentifiedPocket {
    wave::Paths64 boundary;
    double center_x = 0.0;
    double center_y = 0.0;
    double width = 0.0;
    double height = 0.0;
    double top_z = 0.0;
    double bottom_z = 0.0;
  };
  // Derived by an auto-pocket operation during planning; never serialized.
  std::vector<IdentifiedPocket> identified_pockets;
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

// Fractions cover the whole job. The interval from fraction to active_until
// represents work currently assigned to workers, not completed work or an ETA.
struct PlanningProgress {
  double fraction = 0.0;
  double active_until = 0.0;
  std::size_t active_workers = 0;
  std::size_t completed_levels = 0;
  std::size_t total_levels = 0;

  PlanningProgress(double value = 0.0) : fraction(value), active_until(value) {}
  // Keep existing callers interested only in the overall fraction compatible.
  operator double() const { return fraction; }
};

// Called only on the planning coordinator, including during parallel work.
// Returning false cancels planning; all workers finish/stop before it returns.
using ProgressCallback = std::function<bool(PlanningProgress, const std::string& stage)>;

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

  void load_part(const std::filesystem::path& file, const ModelLoadProgress& = {},
                 const ModelOrientation& orientation = {});
  void load_stock(const std::filesystem::path& file, const ModelLoadProgress& = {},
                 const ModelOrientation& orientation = {},
                 const std::array<double, 3>& offset = {});
  // Rebuild stock from an already loaded mesh, without importing its source.
  void set_stock_mesh(wave::Mesh mesh, const std::array<double, 3>& offset = {});
  // Stand a rectangular blank in for a stock model: the part's bounding box
  // grown by directional margins. A loaded stock model always wins.
  // Drops a loaded stock model, so a blank named and then unnamed does not go
  // on winning over the box that replaced it.
  void clear_stock();
  // Commit a background import without rebuilding the unchanged model. Callers
  // must stop geometry users before replacing either model, as with load_part.
  void replace_models(JobGeometry&& loaded, bool part, bool stock);
  void set_box_stock(bool enabled, const BoxStockMargins& margins);
  bool box_stock() const { return box_stock_ && stock_.empty(); }
  bool has_part() const { return !part_.empty(); }
  bool has_stock() const { return !stock_.empty() || (box_stock_ && !part_.empty()); }
  double stock_bottom() const;
  const wave::MeshSlicer& part() const { return part_; }
  const std::shared_ptr<CadModel>& part_cad() const { return part_cad_; }
  const ModelOrientation& part_orientation() const { return part_orientation_; }
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
    part_cad_ = std::move(other.part_cad_);
    part_orientation_ = other.part_orientation_;
    stock_ = std::move(other.stock_);
    box_stock_ = other.box_stock_;
    box_margins_ = other.box_margins_;
    part_slices_ = std::move(other.part_slices_);
    stock_slices_ = std::move(other.stock_slices_);
    box_ = std::move(other.box_);
  }

  // The slice caches are filled lazily, and a plan running on the worker
  // thread fills them while the window draws the same models from the frame
  // thread. Both are reads as far as the caller is concerned; only the
  // remembering underneath is a write, and this is what makes it one.
  mutable std::mutex slices_;
  std::shared_ptr<CadModel> part_cad_;
  ModelOrientation part_orientation_;
  wave::MeshSlicer part_;
  wave::MeshSlicer stock_;
  bool box_stock_ = false;
  BoxStockMargins box_margins_;
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
// the maximum axial step. Roughing depth breaks restart the grid.
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
                              const CuttingFeeds& feeds,
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
                    const CuttingFeeds& feeds,
                    const ProgressCallback& progress,
                    double progress_from = 0.0,
                    double progress_to = 1.0,
                    double rapid_plane = 10.0,
                    const PriorOperations& prior_operations = {},
                    const OperationPlan* prior_subdivisions = nullptr,
                    bool defer_links = false);

// Both stages back to back.
OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const CuttingFeeds& feeds,
                             const ProgressCallback& progress,
                             double progress_from = 0.0,
                             double progress_to = 1.0,
                             double rapid_plane = 10.0,
                             const PriorOperations& prior_operations = {},
                             const OperationPlan* prior_subdivisions = nullptr,
                             bool defer_links = false);

// The material at one Z plane after a sequence of operations: the nominal
// stock clipped to the newest simulated stock among them.
wave::Paths64 stock_after_operations_at_z(
    const wave::Paths64& nominal_stock, double z,
    const PriorOperations& operations);

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
