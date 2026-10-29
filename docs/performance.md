# Import, mesh and toolpath performance

The checkpoint before these changes is tagged
`checkpoint/performance-2026-10-29`. Build optimized binaries (`Release` is the
single-configuration default). CPU-only builds support all the changes below.

## Geometry correctness

OBJ polygons are triangulated in their dominant face plane, retaining their
original winding and 3D coordinates. Convex polygons take a linear fan path;
concave polygons use ear clipping. This keeps concave cutouts empty even on
vertical faces. Negative and slashed indices, repeated closing indices, and
inline comments are supported; index zero is rejected.

OBJ and both STL loaders retain vertical walls and their full Z bounds. The
heightfield slicer excludes zero-area XY projections from its acceleration
structure, rather than removing those faces from the display mesh. This fixes
missing preview walls and incorrect bounds after model rotation.

## STEP import

Spline extrema searches previously took seconds per face on the larger local
STEP model. Import now uses native control-hull bounds for collision rejection
and tessellation bounds including deflection for stock framing. Analytic faces
retain their exact bounds. Spline stock bounds may therefore extend slightly
past the surface, according to the meshing tolerance (normally 0.01 mm).

Face areas are approximate display metadata computed from the existing
triangulation. Native surfaces still determine finishing contacts and cutter
clearance. Bezier collision certificates are prepared once, on first use of a
face, and shared safely among planning workers. The knot-span budget is checked
before constructing the converter, avoiding a large allocation before the limit
can be applied. Unsupported certificates fall back to native clearance tests.

Containment pruning during mesh index construction is optional and now checks
at most 256 candidates per triangle. Unresolved triangles remain in the index;
their material still contributes to the surface envelope. Vertical-only meshes
produce empty height queries without constructing an invalid empty BVH.

## Interactive preview and planning

The result view retains world-space path lines in an OpenGL vertex buffer.
Orbiting and cutter playback reuse that buffer; paused, unchanged views can
reuse the completed frame. Path colours, visibility, selected passes, model
loads, tool edits and replanning invalidate the relevant cache. Bounds include
arc samples and per-level cutter sizes, avoiding clipping full circles whose
endpoints coincide.

Finalized feed schedules retain time and distance prefix indexes. Cutter-time
lookup and distance-to-time seeking use binary search; schedule durations and motion lengths are constant
time. Unindexed schedules keep the linear fallback. Indexes are rebuilt after
stock simulation or inter-layer linking changes commanded motion. Feed ranges
and mean engagement reuse finalized statistics instead of rescanning the pass
on each frame.

Wavefront construction reuses analysis for levels whose protected outline and
incoming-stock outline match exactly within one operation. Hash matches are
verified with full geometry equality. Each level receives independent mutable
wave/topology buffers and retains its own Z and axial depth; levels with
different stock remain separate calculations. `TINGLECAM_PROFILE_WORKERS=1`
reports requested and unique wavefront counts.

Routing indexes obstacle edges with an integer bounding-box tree. The final
intersection and containment predicates are unchanged, and proposed routes
still pass the existing Clipper validation. Ring bounds and winding signs are
cached. Large outlines use sparse visibility storage rather than eagerly
allocating a square table of all corner pairs.

## Reproduction

```sh
cmake --build build-cpu --target interactive_performance mesh_slicer_benchmark routing_benchmark
./build-cpu/interactive_performance
./build-cpu/routing_benchmark
./build-cpu/interactive_performance path/to/model.step
./build-cpu/mesh_slicer_benchmark tests/fixtures/meshes/LsJig1.obj 3
./build-cpu/heightmap_renderer_tests --benchmark
ctest --test-dir build-cpu --output-on-failure
```

`interactive_performance` reports import stages, mesh index construction,
a dense overlapping-triangle case, and linear/indexed timeline lookup with
matching checksums. The renderer benchmark includes 100,000 retained path
segments and checks that camera changes upload no path vertices.

For extracted SDKs, put their runtime library directories on `LD_LIBRARY_PATH`
when running tests. On a headless Mesa installation, use
`SDL_VIDEODRIVER=offscreen LIBGL_ALWAYS_SOFTWARE=1` for rendering checks.

## Measurements (2026-10-29)

Release CPU builds, with Open CASCADE enabled. These are example measurements
on this machine; the roughing and import comparisons are individual runs, and
routing uses the median of three warm paired runs. Import excludes mesh index
construction. The local STEP asset is not distributed with this repository.

| Workload | Before | After |
| --- | ---: | ---: |
| Native STEP import, 168 faces / 330,502 triangles | 51.68 s | 10.60 s |
| LsJig roughing, four subdivisions | 26.73 s | 24.22 s |
| 2,000 cursor lookups in 100,000 motion phases | 1,355 ms | 1.16 ms |
| 32 routes through 36 obstacles | 46.20 ms | 29.58 ms |

The roughing outputs have identical SHA256 hashes. Five unique coarse
wavefronts cover six levels on that fixture; jobs with changing outlines still
need independent wave analysis. The native STEP face/triangle counts are
unchanged. Its new CPU mesh index builds in 364 ms; the 16,000-triangle dense
overlap probe builds in 41 ms.

The 100,000-segment renderer probe uploads 5.6 MB once, then uploads zero path
vertices while orbiting. Idle views reuse the completed frame. Rendering was
validated with Mesa software OpenGL; these measurements do not establish
hardware GPU frame rates. CUDA was unavailable in this environment.

All 23 CTest checks pass, including independent native cutter-clearance
comparisons, geometry/topology regressions, retained OpenGL paths and motion
index boundaries. A complete result-view screenshot was also checked.
[Machine-readable results and checksums](../tools/benchmarks/interactive_performance.json)
record the input identity, command arguments and paired routing samples.
