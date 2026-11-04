#include "job_internal.h"
#include <bit>
namespace cam {
using namespace detail;
const char* operation_mode_name(OperationMode value) {
  switch (value) {
    case OperationMode::Roughing: return "roughing";
    case OperationMode::ContourFinishing: return "contour_finishing";
    case OperationMode::Hole: return "hole";
    case OperationMode::Pocket: return "pocket";
    case OperationMode::PlaneClearing: return "plane_clearing";
    case OperationMode::SurfaceFinishing: return "surface_finishing";
  }
  return "unknown";
}

struct JobGeometry::ModelLibrary {
  struct Asset {
    std::filesystem::file_time_type modified{};
    std::uintmax_t size=0;
    wave::MeshSlicer geometry;
    std::shared_ptr<CadModel> cad;
  };
  std::mutex mutex;
  std::map<std::filesystem::path,Asset> assets;
  Asset read(const std::filesystem::path& file,const ModelLoadProgress& progress) {
    const auto key=std::filesystem::absolute(file).lexically_normal();
    std::error_code ec;
    const auto modified=std::filesystem::last_write_time(key,ec);
    const auto size=ec?0:std::filesystem::file_size(key,ec);
    std::lock_guard lock(mutex);
    auto found=assets.find(key);
    if(found!=assets.end() && (ec || (found->second.modified==modified && found->second.size==size)))
      return found->second;
    Asset asset;
    asset.modified=modified; asset.size=size;
    asset.cad=step_filename(file)?CadModel::load(file,.01,[&](double f,const std::string& stage) {
      return !progress || progress(.85*f,stage);
    }):nullptr;
    if(progress && !progress(.9,"Preparing source geometry")) throw std::runtime_error("Cancelled");
    asset.geometry.set_mesh(asset.cad?asset.cad->mesh():wave::load_triangle_mesh(file));
    assets.insert_or_assign(key,asset);
    return asset;
  }
};

JobGeometry::JobGeometry() : library_(std::make_shared<ModelLibrary>()) {}

void JobGeometry::load_part(const std::filesystem::path& file,
                            const ModelLoadProgress& progress,
                            const ModelOrientation& orientation) {
  orientation.validate();
  if(!library_) library_=std::make_shared<ModelLibrary>();
  auto source=library_->read(file,progress);
  auto placed=source.geometry.placed({orientation,{}});
  auto cad=source.cad?source.cad->placed({orientation,{}}):nullptr;
  if(progress && !progress(1,"Part model ready")) throw std::runtime_error("Cancelled");
  part_=std::move(placed); part_source_cad_=std::move(source.cad);
  part_cad_=std::move(cad); part_orientation_=orientation;
  part_slices_.clear(); box_.clear();
}
void JobGeometry::load_stock(const std::filesystem::path& file,
                             const ModelLoadProgress& progress,
                             const ModelOrientation& orientation,
                             const std::array<double,3>& offset) {
  if(!library_) library_=std::make_shared<ModelLibrary>();
  auto source=library_->read(file,progress);
  auto placed=source.geometry.placed({orientation,offset});
  if(progress && !progress(1,"Stock model ready")) throw std::runtime_error("Cancelled");
  stock_=std::move(placed); inherited_stock_.reset(); stock_slices_.clear();
}
void JobGeometry::place_models_from(const JobGeometry& source,bool part,bool stock,
                                    const ModelTransform& part_placement,
                                    const ModelTransform& stock_placement) {
  std::scoped_lock lock(slices_,source.slices_);
  library_=source.library_; fixtures_=source.fixtures_; fixture_slices_=source.fixture_slices_;
  if(part) {part_=source.part_;part_source_cad_=source.part_source_cad_;set_part_placement(part_placement);}
  if(stock) {stock_=source.stock_;set_stock_placement(stock_placement);}
}
void JobGeometry::set_part_placement(const ModelTransform& placement) {
  auto placed=part_.placed(placement);
  auto cad=part_source_cad_?part_source_cad_->placed(placement):nullptr;
  part_=std::move(placed); part_cad_=std::move(cad); part_orientation_=placement.orientation;
  part_slices_.clear(); box_.clear();
}
void JobGeometry::set_stock_placement(const ModelTransform& placement) {
  stock_=stock_.placed(placement); stock_slices_.clear();
}
void JobGeometry::set_inherited_stock(std::shared_ptr<const wave::Heightmap> stock) {
  inherited_stock_=std::move(stock); inherited_slices_.clear();
}
void JobGeometry::configure(const SetupContext& context,const ModelLoadProgress& progress) {
  const bool part=!context.part_mesh.empty();
  const bool stock=context.stock_from_operation.empty() && !context.stock_mesh.empty();
  const int count=int(part)+int(stock)+int(context.fixtures.size());
  int completed=0;
  const auto report=[&](double f,const std::string& stage) {
    return !progress || progress((completed+f)/std::max(1,count),stage);
  };
  if(part) {
    load_part(context.part_mesh,report,context.part_orientation);
    set_part_placement({context.part_orientation,{context.part_offset_x,context.part_offset_y,context.part_offset_z}});
    ++completed;
  } else {part_.clear();part_cad_.reset();part_source_cad_.reset();part_slices_.clear();}
  clear_stock();
  if(stock) {
    load_stock(context.stock_mesh,report,context.stock_orientation,
        {context.stock_offset_x,context.stock_offset_y,context.stock_offset_z});
    ++completed;
  }
  set_box_stock(context.stock_from_operation.empty() && context.box_stock,context.box_stock_margins);
  fixtures_.clear();fixture_slices_.clear();
  for(const auto& fixture:context.fixtures) {
    if(!std::isfinite(fixture.clearance) || fixture.clearance<0)
      throw std::invalid_argument("Fixture clearance must be finite and nonnegative");
    auto source=library_->read(fixture.mesh,report);
    fixtures_.push_back({fixture,source.geometry.placed(fixture.placement)});
    ++completed;
  }
  if(progress && !progress(1,"Setup models ready")) throw std::runtime_error("Cancelled");
}
JobGeometry JobGeometry::context_view(const SetupContext& context,const ModelLoadProgress& progress) const {
  JobGeometry result;
  result.library_=library_?library_:std::make_shared<ModelLibrary>();
  result.configure(context,progress);
  return result;
}
const wave::Paths64& JobGeometry::fixture_keep_out(double z,int grid,double radius) const {
  const auto key=std::tuple{std::bit_cast<std::int64_t>(z),std::bit_cast<std::int64_t>(radius),grid};
  {std::lock_guard lock(slices_);const auto found=fixture_slices_.find(key);if(found!=fixture_slices_.end()) return found->second;}
  wave::Paths64 result;
  for(const auto& fixture:fixtures_) {
    // Inclusive clearance: touching a fixture is never a finishing target.
    const auto slice=fixture.geometry.slice(float(z-fixture.model.clearance),grid).polygons;
    auto inflated=offset_outward(slice,radius+fixture.model.clearance+1.0/wave::kIntegerScale);
    result.insert(result.end(),inflated.begin(),inflated.end());
  }
  result=Union(result,FillRule::NonZero);
  std::lock_guard lock(slices_);
  return fixture_slices_.emplace(key,std::move(result)).first->second;
}
std::size_t JobGeometry::fixture_slice_count() const {
  std::lock_guard lock(slices_);
  return fixture_slices_.size();
}
bool JobGeometry::fixture_clear(wave::Vec3d from,wave::Vec3d to,const wave::MillingCutter& cutter) const {
  // Preserve the downward-envelope/cylindrical-footprint constraint used by
  // roughing, but query local triangles instead of rebuilding whole fixture
  // polygons at every continuously varying finishing height. The small guard
  // covers the former micron-grid rounding and swept-centreline padding.
  constexpr double numerical_guard=.006;
  for(const auto& fixture:fixtures_)
    if(!fixture.geometry.projected_sweep_clear(from,to,
        std::min(from.z,to.z)-fixture.model.clearance,
        cutter.radius+fixture.model.clearance+numerical_guard)) return false;
  return true;
}

void JobGeometry::set_stock_mesh(wave::Mesh mesh, const std::array<double, 3>& offset) {
  wave::MeshSlicer source; source.set_mesh(std::move(mesh));
  auto imported = source.placed({{},offset});
  stock_ = std::move(imported);
  stock_slices_.clear();
}

void JobGeometry::replace_models(JobGeometry&& loaded, bool part, bool stock) {
  if (this == &loaded) return;
  if(loaded.library_) library_=loaded.library_;
  std::scoped_lock lock(slices_, loaded.slices_);
  if (part) {
    part_ = std::move(loaded.part_);
    part_cad_ = std::move(loaded.part_cad_);
    part_source_cad_ = std::move(loaded.part_source_cad_);
    part_orientation_ = loaded.part_orientation_;
    part_slices_ = std::move(loaded.part_slices_);
    box_.clear();
  }
  if (stock) {
    stock_ = std::move(loaded.stock_);
    stock_slices_ = std::move(loaded.stock_slices_);
  }
}

void JobGeometry::clear_stock() {
  inherited_stock_.reset(); inherited_slices_.clear();
  stock_.clear();
  stock_slices_.clear();
}

namespace {

double nonnegative_margin(double value) {
  return std::isfinite(value) ? std::max(value, 0.0) : 0.0;
}

BoxStockMargins normalize_box_margins(const BoxStockMargins& margins) {
  return {nonnegative_margin(margins.minus_x),
          nonnegative_margin(margins.plus_x),
          nonnegative_margin(margins.minus_y),
          nonnegative_margin(margins.plus_y),
          nonnegative_margin(margins.minus_z),
          nonnegative_margin(margins.plus_z)};
}

bool same_box_margins(const BoxStockMargins& a, const BoxStockMargins& b) {
  return a.minus_x == b.minus_x && a.plus_x == b.plus_x &&
         a.minus_y == b.minus_y && a.plus_y == b.plus_y &&
         a.minus_z == b.minus_z && a.plus_z == b.plus_z;
}

}  // namespace

void JobGeometry::set_box_stock(bool enabled, const BoxStockMargins& margins) {
  const BoxStockMargins normalized = normalize_box_margins(margins);
  if (box_stock_ == enabled && same_box_margins(box_margins_, normalized)) return;
  box_stock_ = enabled;
  box_margins_ = normalized;
  box_.clear();
  stock_slices_.clear();
}

void JobGeometry::forget_slices() {
  part_slices_.clear();
  stock_slices_.clear();
  inherited_slices_.clear();
}

const wave::Paths64& JobGeometry::outline(const wave::MeshSlicer& slicer,
                                          std::map<SliceKey, wave::Paths64>& cache,
                                          double z, int grid) const {
  // The float the slicer takes is the key, not the double asked for, so two
  // depths that reach the same slice share one entry rather than cutting twice.
  const SliceKey key{
      std::bit_cast<std::uint32_t>(static_cast<float>(z)),
      grid};
  {
    std::lock_guard<std::mutex> lock(slices_);
    const auto found = cache.find(key);
    if (found != cache.end()) return found->second;
  }
  // Sliced outside the lock: it is the expensive half, and a second thread
  // arriving at the same depth may as well cut it twice as wait. The map is
  // what needs protecting, not the work.
  auto slice = slicer.slice(static_cast<float>(z), grid);
  auto region = Union(slice.polygons, FillRule::NonZero);
  std::lock_guard<std::mutex> lock(slices_);
  return cache.emplace(key, std::move(region)).first->second;
}

const wave::Paths64& JobGeometry::part_outline(double z, int grid) const {
  return outline(part_, part_slices_, z, grid);
}

const wave::Paths64& JobGeometry::stock_outline(double z, int grid) const {
  if(inherited_stock_) {
    const SliceKey key{std::bit_cast<std::int64_t>(z),grid};
    std::lock_guard lock(slices_);
    if(const auto found=inherited_slices_.find(key);found!=inherited_slices_.end())
      return found->second;
    // Match mesh stock's downward envelope: a cutter at Z also encounters
    // material whose underside is above Z, such as a flipped ball-mill lip.
    return inherited_slices_.emplace(key,inherited_stock_->project_above(z)).first->second;
  }
  if (!box_stock()) return outline(stock_, stock_slices_, z, grid);
  // One rectangle, the same at every depth, so the depth is not part of the
  // question. An empty part leaves it empty and the level reports nothing to
  // cut, which is what a missing model should look like.
  std::lock_guard<std::mutex> lock(slices_);
  if (box_.empty() && !part_.empty()) {
    const auto& bounds = part_.mesh().bounds;
    if (bounds.valid) {
      const std::int64_t left =
          wave::to_integer(bounds.lower.x - box_margins_.minus_x);
      const std::int64_t right =
          wave::to_integer(bounds.upper.x + box_margins_.plus_x);
      const std::int64_t bottom =
          wave::to_integer(bounds.lower.y - box_margins_.minus_y);
      const std::int64_t top =
          wave::to_integer(bounds.upper.y + box_margins_.plus_y);
      box_ = wave::Paths64{wave::Path64{{left, bottom},
                                        {right, bottom},
                                        {right, top},
                                        {left, top}}};
    }
  }
  return box_;
}

bool JobGeometry::work_range(double& low, double& high) const {
  // The union, not the overlap. Either model on its own is a reason to be at a
  // depth: blank above the part is material to face off, part below the blank
  // is a hole to descend into. Taking the overlap quietly clipped both away,
  // which is how a blank standing proud of its part came to be left standing.
  if (box_stock() && !part_.empty() && part_.mesh().bounds.valid) {
    const auto& bounds = part_.mesh().bounds;
    low = bounds.lower.z - box_margins_.minus_z;
    high = bounds.upper.z + box_margins_.plus_z;
    return high > low;
  }

  bool any = inherited_stock_ && std::isfinite(inherited_stock_->lowest()) && std::isfinite(inherited_stock_->highest());
  double lowest = any?inherited_stock_->lowest():0.0;
  double highest = any?inherited_stock_->highest():0.0;
  for (const wave::MeshSlicer* slicer : {&part_, &stock_}) {
    if (slicer->empty()) continue;
    const auto& bounds = slicer->mesh().bounds;
    if (!bounds.valid) continue;
    if (!any) {
      lowest = bounds.lower.z;
      highest = bounds.upper.z;
      any = true;
      continue;
    }
    lowest = std::min<double>(lowest, bounds.lower.z);
    highest = std::max<double>(highest, bounds.upper.z);
  }
  if (!any || !(highest > lowest)) return false;
  low = lowest;
  high = highest;
  return true;
}

double JobGeometry::stock_bottom() const {
  if(inherited_stock_) return inherited_stock_->lowest();
  if (!stock_.empty() && stock_.mesh().bounds.valid)
    return stock_.mesh().bounds.lower.z;
  if (box_stock_ && !part_.empty() && part_.mesh().bounds.valid)
    return part_.mesh().bounds.lower.z - box_margins_.minus_z;
  return std::numeric_limits<double>::quiet_NaN();
}

std::vector<double> level_floors(const Operation& operation, double stepdown,
                                 double work_top) {
  std::vector<double> floors;
  if (!std::isfinite(work_top) || !std::isfinite(operation.depth_from) ||
      !std::isfinite(operation.depth_to) || !std::isfinite(stepdown) ||
      !(stepdown > 0.0)) return floors;
  const double top = z_at_depth(work_top, operation.depth_from);
  const double bottom = z_at_depth(work_top, operation.depth_to);
  if (!(top > bottom)) return floors;
  const double step = std::max(stepdown, 1e-6);
  std::vector<double> boundaries;
  if (operation.mode == OperationMode::Roughing) {
    for (double depth : operation.depth_breaks) {
      if (!std::isfinite(depth) || !(depth > operation.depth_from &&
                                   depth < operation.depth_to)) return {};
      boundaries.push_back(z_at_depth(work_top, depth));
    }
  }
  boundaries.push_back(bottom);
  std::sort(boundaries.begin(), boundaries.end(), std::greater<double>());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
      [](double a, double b) { return std::abs(a-b) < 1e-9; }), boundaries.end());
  double z = top;
  // A guard rather than a limit: at a sane stepdown no real part gets close.
  for (double boundary : boundaries) {
    while (z > boundary + 1e-9) {
      if (floors.size() >= 20000) return {}; // Never return a truncated descent.
      const double next = z - step;
      z = next <= boundary + 1e-9 ? boundary : next;
      floors.push_back(z);
    }
  }
  return floors;
}

}  // namespace cam
