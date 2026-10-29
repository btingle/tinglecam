#include "cam/cad.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef TINGLECAM_HAVE_OCCT
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomConvert_BSplineSurfaceToBezierSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Curve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <TopLoc_Location.hxx>
#endif

namespace cam {
namespace {
thread_local CadQueryStatistics query_statistics;
}
CadQueryStatistics cad_query_statistics() { return query_statistics; }
bool step_filename(const std::filesystem::path& path) {
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  return extension == ".step" || extension == ".stp";
}
struct CadModel::Impl {
  wave::Mesh mesh;
  std::vector<std::size_t> triangle_faces;
  std::vector<CadFace> faces;
  std::string fingerprint;
  ModelOrientation orientation;
  bool solid = false;
  std::set<std::pair<std::size_t, std::size_t>> adjacent_faces;
#ifdef TINGLECAM_HAVE_OCCT
  TopoDS_Shape shape, loose_geometry;
  Bnd_Box loose_bounds;
  std::vector<TopoDS_Solid> solids;
  std::vector<TopoDS_Face> native_faces;
  std::vector<std::array<double, 4>> domains;
  struct Patch {
    Handle(Geom_BezierSurface) surface;
    std::vector<gp_Pnt> poles;
    Bnd_Box bounds;
    gp_XYZ normal;
    bool has_normal = false, split_u = false;
    mutable std::once_flag split_once, face_once;
    mutable std::array<std::unique_ptr<Patch>, 2> children;
    mutable TopoDS_Face native_face;
    void initialize(const Handle(Geom_BezierSurface) & source) {
      surface = source;
      for (int i = 1; i <= surface->NbUPoles(); ++i)
        for (int j = 1; j <= surface->NbVPoles(); ++j) {
          if (!(surface->Weight(i, j) > 0))
            throw std::runtime_error("Nonpositive rational weight");
          const auto p = surface->Pole(i, j);
          poles.push_back(p);
          bounds.Add(p);
        }
      bounds.Enlarge(1e-7);
      gp_Pnt p;
      gp_Vec du, dv;
      surface->D1(.5, .5, p, du, dv);
      const auto n = du.Crossed(dv);
      has_normal = n.SquareMagnitude() > 1e-24;
      if (has_normal) normal = n.XYZ() / n.Magnitude();
      split_u =
          surface->Pole(1, 1).Distance(surface->Pole(surface->NbUPoles(), 1)) >=
          surface->Pole(1, 1).Distance(surface->Pole(1, surface->NbVPoles()));
    }
    void initialize_child(const Patch& parent, int half) {
      auto source =
          Handle(Geom_BezierSurface)::DownCast(parent.surface->Copy());
      if (parent.split_u)
        source->Segment(half * .5, (half + 1) * .5, 0, 1);
      else
        source->Segment(0, 1, half * .5, (half + 1) * .5);
      initialize(source);
    }
    const TopoDS_Face& face() const {
      std::call_once(face_once, [&] {
        native_face = BRepBuilderAPI_MakeFace(surface, 1e-9).Face();
      });
      return native_face;
    }
  };
  struct CollisionFace {
    Bnd_Box bounds;
    mutable std::vector<std::unique_ptr<Patch>> patches;
    std::unique_ptr<std::once_flag> patches_once = std::make_unique<std::once_flag>();
  };
  // Shared once-initialized refinements are bounded per model. After this
  // budget, queries use temporary refinements with identical geometry/tests.
  static constexpr std::size_t kCachedPatchChildren = 8192;
  mutable std::atomic<std::size_t> cached_patch_children{0};
  static constexpr std::size_t kCachedPatchPoles = 131072;
  mutable std::atomic<std::size_t> cached_patch_poles{0};
  std::vector<CollisionFace> collision_faces;
  void prepare_collision_face(std::size_t index) const;
  bool boundary_clear(const TopoDS_Shape& query, double clearance) const;
#endif
};
CadModel::CadModel() : impl_(std::make_unique<Impl>()) {}
CadModel::~CadModel() = default;
const wave::Mesh& CadModel::mesh() const { return impl_->mesh; }
const std::vector<std::size_t>& CadModel::triangle_faces() const {
  return impl_->triangle_faces;
}
const std::vector<CadFace>& CadModel::faces() const { return impl_->faces; }
const std::string& CadModel::fingerprint() const { return impl_->fingerprint; }
const ModelOrientation& CadModel::orientation() const { return impl_->orientation; }
bool CadModel::solid() const { return impl_->solid; }
bool CadModel::adjacent(std::size_t a, std::size_t b) const {
  return impl_->adjacent_faces.contains(std::minmax(a, b));
}
std::optional<std::array<std::array<double, 2>, 2>> CadModel::shared_boundary(
    std::size_t a, std::size_t b, wave::Vec3d near) const {
#ifndef TINGLECAM_HAVE_OCCT
  (void)a;
  (void)b;
  (void)near;
  return std::nullopt;
#else
  if (!adjacent(a, b)) return std::nullopt;
  try {
    const auto& fa = impl_->native_faces.at(a);
    const auto& fb = impl_->native_faces.at(b);
    const gp_Pnt point(near.x, near.y, near.z);
    double best = std::numeric_limits<double>::infinity();
    std::optional<std::array<std::array<double, 2>, 2>> result;
    for (TopExp_Explorer ea(fa, TopAbs_EDGE); ea.More(); ea.Next())
      for (TopExp_Explorer eb(fb, TopAbs_EDGE); eb.More(); eb.Next()) {
        if (!ea.Current().IsSame(eb.Current())) continue;
        const auto edge = TopoDS::Edge(ea.Current());
        double first, last;
        const auto curve = BRep_Tool::Curve(edge, first, last);
        if (curve.IsNull()) continue;
        GeomAPI_ProjectPointOnCurve projection(point, curve, first, last);
        double parameter = first;
        double distance = point.Distance(curve->Value(first));
        if (point.Distance(curve->Value(last)) < distance) {
          parameter = last;
          distance = point.Distance(curve->Value(last));
        }
        if (projection.NbPoints() && projection.LowerDistance() < distance) {
          distance = projection.LowerDistance();
          parameter = projection.LowerDistanceParameter();
        }
        if (distance >= best) continue;
        const auto ca = BRep_Tool::CurveOnSurface(edge, fa, first, last);
        const auto cb = BRep_Tool::CurveOnSurface(edge, fb, first, last);
        if (ca.IsNull() || cb.IsNull()) continue;
        const auto pa = ca->Value(parameter), pb = cb->Value(parameter);
        const auto& da = impl_->domains[a];
        const auto& db = impl_->domains[b];
        result = std::array{std::array{(pa.X() - da[0]) / (da[1] - da[0]),
                                       (pa.Y() - da[2]) / (da[3] - da[2])},
                            std::array{(pb.X() - db[0]) / (db[1] - db[0]),
                                       (pb.Y() - db[2]) / (db[3] - db[2])}};
        best = distance;
      }
    return result;
  } catch (const Standard_Failure&) {
    return std::nullopt;
  }
#endif
}

std::vector<std::array<double, 2>> CadModel::matching_contour_parameters(
    std::size_t a, std::size_t b, bool along_v,
    const std::vector<double>& a_lanes,
    const std::vector<double>& b_lanes) const {
  std::vector<std::array<double, 2>> matches;
#ifdef TINGLECAM_HAVE_OCCT
  if (!adjacent(a, b)) return matches;
  try {
    const auto& fa = impl_->native_faces.at(a);
    const auto& fb = impl_->native_faces.at(b);
    for (TopExp_Explorer ea(fa, TopAbs_EDGE); ea.More(); ea.Next())
      for (TopExp_Explorer eb(fb, TopAbs_EDGE); eb.More(); eb.Next()) {
        if (!ea.Current().IsSame(eb.Current())) continue;
        const auto edge = TopoDS::Edge(ea.Current());
        double af, al, bf, bl;
        const auto ca = BRep_Tool::CurveOnSurface(edge, fa, af, al);
        const auto cb = BRep_Tool::CurveOnSurface(edge, fb, bf, bl);
        if (ca.IsNull() || cb.IsNull()) continue;
        const double first = std::max(af, bf), last = std::min(al, bl);
        if (!std::isfinite(first) || !std::isfinite(last) || first >= last) continue;
        auto uv = [&](bool second, double t) {
          const auto p = (second ? cb : ca)->Value(t);
          const auto& d = impl_->domains[second ? b : a];
          const double u = (p.X() - d[0]) / (d[1] - d[0]),
                       v = (p.Y() - d[2]) / (d[3] - d[2]);
          return along_v ? std::array{u, v} : std::array{v, u};
        };
        // Only edges at constant along=0/1 are ends of the requested passes.
        // Check monotonicity before inverting the across coordinate.
        constexpr double epsilon = 1e-9;
        bool compatible = true;
        for (bool second : {false, true}) {
          const auto start = uv(second, first), end = uv(second, last);
          if (std::abs(start[0] - end[0]) < epsilon ||
              std::min(std::abs(start[1]), std::abs(start[1] - 1)) > epsilon) {
            compatible = false;
            break;
          }
          auto previous = start;
          for (int i = 1; i <= 16; ++i) {
            const auto p = uv(second, first + (last - first) * i / 16.0);
            if (std::abs(p[1] - start[1]) > epsilon || p[0] < -epsilon ||
                p[0] > 1 + epsilon ||
                (p[0] - previous[0]) * (end[0] - start[0]) < -epsilon)
              compatible = false;
            previous = p;
          }
        }
        if (!compatible) continue;
        for (bool second : {false, true}) {
          const auto start = uv(second, first), end = uv(second, last);
          for (double lane : second ? b_lanes : a_lanes) {
            if (lane < std::min(start[0], end[0]) - epsilon ||
                lane > std::max(start[0], end[0]) + epsilon) continue;
            double low = first, high = last;
            for (int i = 0; i < 48; ++i) {
              const double middle = (low + high) / 2;
              if ((uv(second, middle)[0] < lane) == (end[0] > start[0]))
                low = middle;
              else high = middle;
            }
            const double t = (low + high) / 2;
            matches.push_back({std::clamp(uv(false, t)[0], 0.0, 1.0),
                               std::clamp(uv(true, t)[0], 0.0, 1.0)});
          }
        }
      }
  } catch (const Standard_Failure& failure) {
    throw std::runtime_error(std::string("CAD contour seam mapping failed: ") +
                             failure.GetMessageString());
  }
#else
  (void)a; (void)b; (void)along_v; (void)a_lanes; (void)b_lanes;
#endif
  return matches;
}

std::shared_ptr<CadModel> CadModel::load(const std::filesystem::path& path,
                                         double tolerance,
                                         const ModelLoadProgress& progress,
                                         const ModelOrientation& orientation) {
  orientation.validate();
#ifndef TINGLECAM_HAVE_OCCT
  (void)path;
  (void)tolerance;
  (void)progress;
  throw std::runtime_error(
      "STEP support requires Open CASCADE; rebuild with "
      "TINGLECAM_WITH_OCCT=ON");
#else
  try {
    if (!std::isfinite(tolerance) || tolerance <= 0)
      throw std::invalid_argument("Invalid CAD triangulation tolerance");
    const auto report = [&](double fraction, const char* stage) {
      if (progress && !progress(fraction, stage)) throw std::runtime_error("Cancelled");
    };
    report(0, "Reading STEP file");
    STEPControl_Reader reader;
    if (reader.ReadFile(path.string().c_str()) != IFSelect_RetDone)
      throw std::runtime_error("Could not import STEP: " + path.string());
    report(.12, "Converting STEP geometry");
    if (reader.TransferRoots() == 0)
      throw std::runtime_error("Could not import STEP: " + path.string());
    auto model = std::shared_ptr<CadModel>(new CadModel);
    auto& data = *model->impl_;
    data.orientation = orientation;
    data.shape = reader.OneShape();
    if (data.shape.IsNull()) throw std::runtime_error("STEP contains no shape");
    if (orientation != ModelOrientation{}) {
      double matrix[3][3]{};
      for (int row = 0; row < 3; ++row) {
        const int a = orientation.axes[row];
        matrix[row][std::abs(a) - 1] = a < 0 ? -1 : 1;
      }
      gp_Trsf transform;
      transform.SetValues(matrix[0][0], matrix[0][1], matrix[0][2], 0,
                          matrix[1][0], matrix[1][1], matrix[1][2], 0,
                          matrix[2][0], matrix[2][1], matrix[2][2], 0);
      data.shape = data.shape.Moved(TopLoc_Location(transform));
    }
    for (TopExp_Explorer solids(data.shape, TopAbs_SOLID); solids.More();
         solids.Next())
      data.solids.push_back(TopoDS::Solid(solids.Current()));
    data.solid = !data.solids.empty();
    report(.2, "Triangulating STEP surfaces");
    BRepMesh_IncrementalMesh mesher(data.shape, tolerance, false, 0.12, true);
    if (!mesher.IsDone())
      throw std::runtime_error("Could not triangulate STEP geometry");
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(data.shape, TopAbs_FACE, faces);
    const char* names[]{"Plane",     "Cylinder", "Cone",     "Sphere",
                        "Torus",     "Bezier",   "B-spline", "Revolution",
                        "Extrusion", "Offset",   "Other"};
    Bnd_Box box;
    for (int index = 1; index <= faces.Extent(); ++index) {
      report(.45 + .45 * (index - 1) / faces.Extent(), "Preparing STEP faces");
      const auto face = TopoDS::Face(faces(index));
      BRepAdaptor_Surface surface(face, true);
      GProp_GProps properties;
      // Area is display metadata. Reuse the tessellation rather than
      // integrating every native spline just to populate the face list.
      BRepGProp::SurfaceProperties(face, properties, false, true);
      data.native_faces.push_back(face);
      std::array<double, 4> bounds;
      BRepTools::UVBounds(face, bounds[0], bounds[1], bounds[2], bounds[3]);
      if (!std::all_of(bounds.begin(), bounds.end(), [](double x) {
            return std::isfinite(x) && std::abs(x) < 1e100;
          }))
        throw std::runtime_error("STEP face has an unbounded parameter domain");
      data.domains.push_back(bounds);
      Impl::CollisionFace collision;
      if (surface.GetType() == GeomAbs_BSplineSurface ||
          surface.GetType() == GeomAbs_BezierSurface) {
        // Native spline extrema can dominate import (seconds per face).
        // Control-hull bounds conservatively cover the native collision
        // geometry; mesher bounds include deflection and keep stock close to
        // the displayed surface without a global extrema search.
        BRepBndLib::Add(face, collision.bounds, false);
        BRepBndLib::Add(face, box, true);
      } else {
        BRepBndLib::AddOptimal(face, collision.bounds, false, false);
        box.Add(collision.bounds);
      }
      data.collision_faces.push_back(std::move(collision));
      const auto type = static_cast<std::size_t>(surface.GetType());
      data.faces.push_back({std::size_t(index - 1),
                            names[std::min(type, std::size_t{10})],
                            properties.Mass()});
      TopLoc_Location location;
      auto triangles = BRep_Tool::Triangulation(face, location);
      if (triangles.IsNull()) continue;
      const auto point = [&](int i) {
        const auto p =
            triangles->Node(i).Transformed(location.Transformation());
        return wave::Vec3f{float(p.X()), float(p.Y()), float(p.Z())};
      };
      for (int i = 1; i <= triangles->NbTriangles(); ++i) {
        int a, b, c;
        triangles->Triangle(i).Get(a, b, c);
        if (face.Orientation() == TopAbs_REVERSED) std::swap(b, c);
        data.mesh.triangles.push_back({point(a), point(b), point(c)});
        data.triangle_faces.push_back(index - 1);
      }
    }
    report(.9, "Connecting STEP faces");
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(data.shape, TopAbs_EDGE, edges);
    std::vector<std::vector<std::size_t>> owners(edges.Extent());
    for (std::size_t i = 0; i < data.native_faces.size(); ++i) {
      TopTools_IndexedMapOfShape face_edges;
      TopExp::MapShapes(data.native_faces[i], TopAbs_EDGE, face_edges);
      for (int j = 1; j <= face_edges.Extent(); ++j) {
        auto& list = owners[edges.FindIndex(face_edges(j)) - 1];
        for (auto other : list) data.adjacent_faces.insert({other, i});
        list.push_back(i);
      }
    }
    // Preserve obstacles exported as free edges or vertices as well. Face
    // bounds already cover their own wires, so only unattached geometry needs
    // a separate native-distance fallback.
    TopTools_IndexedMapOfShape covered;
    for (const auto& face : data.native_faces) TopExp::MapShapes(face, covered);
    BRep_Builder builder;
    TopoDS_Compound loose;
    builder.MakeCompound(loose);
    bool has_loose = false;
    for (auto type : {TopAbs_EDGE, TopAbs_VERTEX}) {
      TopTools_IndexedMapOfShape pieces;
      TopExp::MapShapes(data.shape, type, pieces);
      for (int i = 1; i <= pieces.Extent(); ++i)
        if (!covered.Contains(pieces(i))) {
          builder.Add(loose, pieces(i));
          TopExp::MapShapes(pieces(i), covered);
          has_loose = true;
        }
    }
    if (has_loose) {
      data.loose_geometry = loose;
      BRepBndLib::AddOptimal(loose, data.loose_bounds, false, false);
      box.Add(data.loose_bounds);
    }
    if (data.mesh.triangles.empty())
      throw std::runtime_error("STEP contains no triangulatable faces");
    // Analytic faces retain exact bounds. Spline bounds are conservative,
    // including tessellation deflection, so a box blank always covers them.
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    data.mesh.bounds = {{float(x0), float(y0), float(z0)},
                        {float(x1), float(y1), float(z1)},
                        true};
    data.mesh.source = path;
    report(.97, "Identifying STEP model");
    // File content, not face numbering, binds a persisted selection to an
    // import.
    std::ifstream file(path, std::ios::binary);
    std::uint64_t hash = 1469598103934665603ull;
    char buffer[8192];
    while (file.read(buffer, sizeof buffer) || file.gcount())
      for (std::streamsize i = 0; i < file.gcount(); ++i) {
        hash ^= static_cast<unsigned char>(buffer[i]);
        hash *= 1099511628211ull;
      }
    std::ostringstream text;
    text << std::hex << hash;
    data.fingerprint = text.str();
    report(1, "STEP model ready");
    return model;
  } catch (const Standard_Failure& failure) {
    throw std::runtime_error(std::string("STEP import failed: ") +
                             failure.GetMessageString());
  }
#endif
}
SurfacePoint CadModel::evaluate(std::size_t index, double u, double v) const {
#ifndef TINGLECAM_HAVE_OCCT
  (void)index;
  (void)u;
  (void)v;
  throw std::runtime_error("STEP support is unavailable");
#else
  try {
    if (index >= impl_->native_faces.size())
      throw std::out_of_range("CAD face selection is out of range");
    const auto& face = impl_->native_faces[index];
    const auto& domain = impl_->domains[index];
    const double U = domain[0] + u * (domain[1] - domain[0]),
                 V = domain[2] + v * (domain[3] - domain[2]);
    BRepAdaptor_Surface surface(face, true);
    gp_Pnt p;
    gp_Vec du, dv;
    surface.D1(U, V, p, du, dv);
    auto normal = du.Crossed(dv);
    SurfacePoint sample;
    sample.point = {p.X(), p.Y(), p.Z()};
    if (normal.SquareMagnitude() < 1e-24) {
      // Pole derivatives can vanish even on a smooth sphere. Evaluate the
      // limiting normal just inside the parameter domain, retaining the exact
      // contact point and trim classification at the original parameters.
      const double eu = (domain[1] - domain[0]) * 1e-7,
                   ev = (domain[3] - domain[2]) * 1e-7;
      gp_Pnt nearby;
      surface.D1(std::clamp(U, domain[0] + eu, domain[1] - eu),
                 std::clamp(V, domain[2] + ev, domain[3] - ev), nearby, du, dv);
      normal = du.Crossed(dv);
      if (normal.SquareMagnitude() < 1e-24) return sample;
    }
    normal.Normalize();
    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
    sample.normal = {normal.X(), normal.Y(), normal.Z()};
    BRepClass_FaceClassifier classifier(face, gp_Pnt2d(U, V), 1e-8);
    sample.inside =
        classifier.State() == TopAbs_IN || classifier.State() == TopAbs_ON;
    return sample;
  } catch (const Standard_Failure& failure) {
    throw std::runtime_error(std::string("CAD surface evaluation failed: ") +
                             failure.GetMessageString());
  }
#endif
}
#ifdef TINGLECAM_HAVE_OCCT
namespace {
// OCCT can retain the model's face/edge maps and bounds between queries.
// Workspaces belong to a thread, so immutable models remain safe to share.
double native_distance(const TopoDS_Shape& model, const TopoDS_Shape& query) {
  ++query_statistics.native_distance_tests;
  struct Workspace {
    TopoDS_Shape model;
    BRepExtrema_DistShapeShape distance;
  };
  thread_local Workspace workspace;
  if (workspace.model.IsNull() || !workspace.model.IsSame(model)) {
    workspace.model = model;
    workspace.distance.LoadS2(model);
  }
  workspace.distance.LoadS1(query);
  if (!workspace.distance.Perform() || !workspace.distance.IsDone()) return -1;
  return workspace.distance.Value();
}
TopAbs_State containment(const TopoDS_Solid& solid, const gp_Pnt& p,
                         double tolerance) {
  ++query_statistics.containment_tests;
  struct Workspace {
    TopoDS_Shape solid;
    BRepClass3d_SolidClassifier classifier;
  };
  thread_local Workspace workspace;
  if (workspace.solid.IsNull() || !workspace.solid.IsEqual(solid)) {
    workspace.solid = solid;
    workspace.classifier.Load(solid);
  }
  workspace.classifier.Perform(p, tolerance);
  return workspace.classifier.State();
}
// Exact point keys, scoped to one immutable model and one worker. A centre
// known to be outside remains outside along a segment with positive distance
// to every model boundary. Radius/tolerance may change: only the outside fact
// is retained, and every ball/shaft sweep still gets its own boundary check.
struct OutsideCentres {
  TopoDS_Shape model;
  std::array<gp_Pnt, 128> points;
  std::size_t size = 0, next = 0;
  void select(const TopoDS_Shape& shape) {
    if (model.IsNull() || !model.IsEqual(shape)) {
      model = shape;
      size = next = 0;
    }
  }
  bool contains(const gp_Pnt& p) const {
    for (std::size_t i = 0; i < size; ++i)
      if (p.X() == points[i].X() && p.Y() == points[i].Y() &&
          p.Z() == points[i].Z())
        return true;
    return false;
  }
  void remember(const gp_Pnt& p) {
    points[next] = p;
    next = (next + 1) % points.size();
    size = std::min(size + 1, points.size());
  }
};
thread_local OutsideCentres outside_centres;
struct StartContainment {
  bool inside = false, outside = false;
};
StartContainment classify_start(const TopoDS_Shape& shape,
                                const std::vector<TopoDS_Solid>& solids,
                                const gp_Pnt& p, double tolerance) {
  if (solids.empty()) return {false, true};
  outside_centres.select(shape);
  if (outside_centres.contains(p)) {
    ++query_statistics.outside_reuses;
    return {false, true};
  }
  bool outside = true;
  for (const auto& solid : solids) {
    const auto state = containment(solid, p, tolerance);
    if (state == TopAbs_IN) return {true, false};
    outside &= state == TopAbs_OUT;
  }
  // ON/UNKNOWN preserve the previous conservative boundary-test behavior but
  // cannot seed an outside certificate or propagate it to another point.
  if (outside) outside_centres.remember(p);
  return {false, outside};
}
bool valid_cutter(double radius, double flute, double tolerance) {
  return std::isfinite(radius) && std::isfinite(flute) &&
         std::isfinite(tolerance) && radius > 0 && flute >= radius &&
         tolerance >= 0 && tolerance < radius;
}
bool finite_point(wave::Vec3d p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
}  // namespace
#endif
#ifdef TINGLECAM_HAVE_OCCT
void CadModel::Impl::prepare_collision_face(std::size_t index) const {
  const auto& collision = collision_faces[index];
  std::call_once(*collision.patches_once, [&] {
    BRepAdaptor_Surface surface(native_faces[index], true);
    if (surface.GetType() != GeomAbs_BSplineSurface) return;
    const auto spline = surface.BSpline();
    // Check the budget before conversion: its constructor materializes every
    // knot span, so checking Nb*Patches afterwards is already too late.
    if (std::int64_t(spline->NbUKnots()-1)*(spline->NbVKnots()-1) > 4096) return;
    const auto& bounds = domains[index];
    try {
      GeomConvert_BSplineSurfaceToBezierSurface converter(
          spline, bounds[0], bounds[1], bounds[2], bounds[3], 1e-10);
      if (std::int64_t(converter.NbUPatches())*converter.NbVPatches() > 4096) return;
      for (int u = 1; u <= converter.NbUPatches(); ++u)
        for (int v = 1; v <= converter.NbVPatches(); ++v) {
          auto piece = std::make_unique<Patch>();
          piece->initialize(converter.Patch(u,v));
          collision.patches.push_back(std::move(piece));
        }
    } catch (const Standard_Failure&) {
      collision.patches.clear();
    } catch (const std::runtime_error&) {
      collision.patches.clear();
    }
  });
}

bool CadModel::Impl::boundary_clear(const TopoDS_Shape& query,
                                    double clearance) const {
  Bnd_Box bounds;
  BRepBndLib::AddOptimal(query, bounds, false, false);
  std::vector<gp_Pnt> vertices;
  for (TopExp_Explorer vertex(query, TopAbs_VERTEX); vertex.More();
       vertex.Next())
    vertices.push_back(BRep_Tool::Pnt(TopoDS::Vertex(vertex.Current())));
  // Query shapes here are convex vertices, segments or planar shaft quads.
  // A separating plane against the Bezier control hull is a conservative
  // distance certificate. Refine only nearby hulls which cannot certify it.
  auto separates = [&](const Patch& patch) {
    // Long shaft quads and vertical patches can be almost parallel. Starting
    // along a corner-to-corner diagonal makes the hull iteration converge very
    // slowly. Surface normals supply useful separating directions immediately;
    // each direction is still certified against every control pole and query
    // vertex, so this does not approximate or relax the clearance test.
    if (patch.has_normal) {
      for (double sign : {-1., 1.}) {
        const auto direction = patch.normal * sign;
        double low = 1e100, high = -1e100;
        for (const auto& v : vertices)
          low = std::min(low, direction.Dot(v.XYZ()));
        for (const auto& v : patch.poles)
          high = std::max(high, direction.Dot(v.XYZ()));
        if (low - high >= clearance + 1e-7) return true;
      }
    }
    gp_XYZ axis = vertices.front().XYZ() - patch.poles.front().XYZ();
    std::vector<gp_XYZ> simplex{axis};
    for (int iteration = 0; iteration < 32; ++iteration) {
      const double length = axis.Modulus();
      if (length < 1e-12) return false;
      gp_XYZ direction = axis / length;
      auto q = vertices.front().XYZ(), p = patch.poles.front().XYZ();
      for (const auto& v : vertices)
        if (direction.Dot(v.XYZ()) < direction.Dot(q)) q = v.XYZ();
      for (const auto& v : patch.poles)
        if (direction.Dot(v.XYZ()) > direction.Dot(p)) p = v.XYZ();
      const gp_XYZ support = q - p;
      if (direction.Dot(support) >= clearance + 1e-7) return true;
      // GJK closest-simplex update. A segment-only update zigzags on long,
      // nearly parallel hulls and can miss an available separating plane.
      simplex.push_back(support);
      double best = 1e200;
      gp_XYZ closest;
      std::vector<gp_XYZ> active;
      auto consider = [&](gp_XYZ p,
                          std::initializer_list<std::size_t> indices) {
        if (p.SquareModulus() >= best) return;
        best = p.SquareModulus();
        closest = p;
        active.clear();
        for (auto i : indices) active.push_back(simplex[i]);
      };
      for (std::size_t i = 0; i < simplex.size(); ++i) {
        consider(simplex[i], {i});
        for (std::size_t j = i + 1; j < simplex.size(); ++j) {
          const auto edge = simplex[j] - simplex[i];
          if (edge.SquareModulus() > 1e-24) {
            const double t = std::clamp(
                -simplex[i].Dot(edge) / edge.SquareModulus(), 0., 1.);
            consider(simplex[i] + edge * t, {i, j});
          }
          for (std::size_t k = j + 1; k < simplex.size(); ++k) {
            const auto e = simplex[k] - simplex[i];
            const double aa = edge.SquareModulus(), bb = edge.Dot(e),
                         cc = e.SquareModulus();
            const double det = aa * cc - bb * bb;
            if (det <= 1e-20 * aa * cc) continue;
            const double d = -simplex[i].Dot(edge), f = -simplex[i].Dot(e);
            const double u = (d * cc - f * bb) / det,
                         v = (f * aa - d * bb) / det;
            if (u >= 0 && v >= 0 && u + v <= 1)
              consider(simplex[i] + edge * u + e * v, {i, j, k});
          }
        }
      }
      if (best < 1e-24) return false;
      // Tetrahedron containment prevents cycling when the hulls overlap.
      if (simplex.size() == 4) {
        const auto e = simplex[1] - simplex[0], f = simplex[2] - simplex[0],
                   g = simplex[3] - simplex[0], p = simplex[0] * -1.;
        const double volume = e.Dot(f.Crossed(g));
        if (std::abs(volume) > 1e-24) {
          const double u = p.Dot(f.Crossed(g)) / volume,
                       v = e.Dot(p.Crossed(g)) / volume,
                       w = e.Dot(f.Crossed(p)) / volume;
          if (u >= 0 && v >= 0 && w >= 0 && u + v + w <= 1) return false;
        }
      }
      if (axis.SquareModulus() - best < 1e-14 * std::max(1., best))
        return false;
      axis = closest;
      simplex = std::move(active);
    }
    return false;
  };
  std::function<bool(const Patch&, int)> patch_clear = [&](const Patch& patch,
                                                           int depth) {
    if (bounds.Distance(patch.bounds) >= clearance || separates(patch))
      return true;
    if (depth >= 12) return native_distance(patch.face(), query) >= clearance;
    std::call_once(patch.split_once, [&] {
      std::size_t count = cached_patch_children.load(std::memory_order_relaxed);
      while (count + 2 <= kCachedPatchChildren) {
        if (cached_patch_children.compare_exchange_weak(
                count, count + 2, std::memory_order_relaxed)) {
          const std::size_t charge = 2 * patch.poles.size();
          std::size_t poles =
              cached_patch_poles.load(std::memory_order_relaxed);
          bool reserved = false;
          while (poles + charge <= kCachedPatchPoles) {
            if (cached_patch_poles.compare_exchange_weak(
                    poles, poles + charge, std::memory_order_relaxed)) {
              reserved = true;
              break;
            }
          }
          if (!reserved) {
            cached_patch_children.fetch_sub(2, std::memory_order_relaxed);
            break;
          }
          std::array<std::unique_ptr<Patch>, 2> children;
          try {
            for (int half = 0; half < 2; ++half) {
              children[half] = std::make_unique<Patch>();
              children[half]->initialize_child(patch, half);
            }
            patch.children = std::move(children);
          } catch (...) {
            cached_patch_children.fetch_sub(2, std::memory_order_relaxed);
            cached_patch_poles.fetch_sub(charge, std::memory_order_relaxed);
            throw;
          }
          break;
        }
      }
    });
    for (int half = 0; half < 2; ++half) {
      if (patch.children[half]) {
        if (!patch_clear(*patch.children[half], depth + 1)) return false;
      } else {
        Patch child;
        child.initialize_child(patch, half);
        if (!patch_clear(child, depth + 1)) return false;
      }
    }
    return true;
  };
  for (std::size_t i = 0; i < collision_faces.size(); ++i) {
    const auto& face = collision_faces[i];
    if (bounds.Distance(face.bounds) >= clearance) continue;
    prepare_collision_face(i);
    bool proved = !face.patches.empty();
    for (const auto& patch : face.patches) {
      if (!patch_clear(*patch, 0)) {
        proved = false;
        break;
      }
    }
    if (!proved && native_distance(native_faces[i], query) < clearance)
      return false;
  }
  if (!loose_geometry.IsNull() && bounds.Distance(loose_bounds) < clearance &&
      native_distance(loose_geometry, query) < clearance)
    return false;
  return true;
}
#endif
bool CadModel::cutter_clear(wave::Vec3d tip, double radius, double flute_length,
                            double tolerance) const {
#ifndef TINGLECAM_HAVE_OCCT
  (void)tip;
  (void)radius;
  (void)flute_length;
  (void)tolerance;
  return false;
#else
  try {
    if (!valid_cutter(radius, flute_length, tolerance) || !finite_point(tip))
      return false;
    const gp_Pnt centre(tip.x, tip.y, tip.z + radius);
    if (classify_start(impl_->shape, impl_->solids, centre, tolerance).inside)
      return false;
    // The same-radius shaft capsule already includes the ball at its lower
    // endpoint. Checking that union once covers the complete cutter.
    if (flute_length > radius + 1e-6) {
      const auto axis = BRepBuilderAPI_MakeEdge(
                            centre, gp_Pnt(tip.x, tip.y, tip.z + flute_length))
                            .Edge();
      return impl_->boundary_clear(axis, radius - tolerance);
    }
    return impl_->boundary_clear(BRepBuilderAPI_MakeVertex(centre).Vertex(),
                                 radius - tolerance);
  } catch (const Standard_Failure& failure) {
    (void)failure;
    return false;
  }
#endif
}
bool CadModel::cutter_sweep_clear(wave::Vec3d a, wave::Vec3d b, double radius,
                                  double flute_length, double tolerance) const {
#ifndef TINGLECAM_HAVE_OCCT
  (void)a;
  (void)b;
  (void)radius;
  (void)flute_length;
  (void)tolerance;
  return false;
#else
  try {
    if (!valid_cutter(radius, flute_length, tolerance) || !finite_point(a) ||
        !finite_point(b))
      return false;
    if (wave::distance(a, b) < 1e-8)
      return cutter_clear(a, radius, flute_length, tolerance);
    // The sweep distance includes both endpoints. Only containment needs an
    // independent test: a sweep wholly inside a solid can miss its boundary.
    const gp_Pnt p(a.x, a.y, a.z + radius), q(b.x, b.y, b.z + radius);
    const auto start =
        classify_start(impl_->shape, impl_->solids, p, tolerance);
    if (start.inside) return false;
    const double xy = std::hypot(b.x - a.x, b.y - a.y);
    // The shaft-axis quad includes the ball-centre segment as its lower edge.
    // Its radius-offset therefore covers the union of ball and shaft sweeps.
    // Retain the separate ball query only for the near-vertical approximation,
    // where the shaft representation deliberately ignores a tiny XY drift.
    const bool ball_only = flute_length <= radius + 1e-6;
    const bool tiny_drift = xy > 0 && xy < 1e-8;
    if (ball_only || tiny_drift) {
      if (!impl_->boundary_clear(BRepBuilderAPI_MakeEdge(p, q).Edge(),
                                 radius - tolerance))
        return false;
      if (ball_only) {
        if (start.outside && radius - tolerance > 1e-7 &&
            !impl_->solids.empty())
          outside_centres.remember(q);
        return true;
      }
    }
    TopoDS_Shape shaft;
    if (std::hypot(b.x - a.x, b.y - a.y) < 1e-8) {
      shaft = BRepBuilderAPI_MakeEdge(
                  gp_Pnt(a.x, a.y, std::min(a.z, b.z) + radius),
                  gp_Pnt(a.x, a.y, std::max(a.z, b.z) + flute_length))
                  .Edge();
    } else {
      BRepBuilderAPI_MakePolygon wire;
      wire.Add(p);
      wire.Add(q);
      wire.Add(gp_Pnt(b.x, b.y, b.z + flute_length));
      wire.Add(gp_Pnt(a.x, a.y, a.z + flute_length));
      wire.Close();
      shaft = BRepBuilderAPI_MakeFace(wire.Wire()).Face();
    }
    const bool clear = impl_->boundary_clear(shaft, radius - tolerance);
    if (clear && start.outside && radius - tolerance > 1e-7 &&
        !impl_->solids.empty())
      outside_centres.remember(q);
    return clear;
  } catch (const Standard_Failure& failure) {
    (void)failure;
    return false;
  }
#endif
}
}  // namespace cam
