# GPU mesh slicer and Clipper2 contour-wave demo

This desktop demo combines an NVIDIA cuBQL-accelerated downward-prism mesh
slicer with the existing Clipper2 wavefront and lineage analysis. It can load
independent OBJ or STL models for the subject and stock. A slice is converted
directly to Clipper2 `Paths64`, so it becomes the input to the plan view and
wave analysis without a rasterization step. `wavetest.svg` remains the default
2D subject when no subject mesh is loaded.

## Downward-prism mesh slicing

For a non-vertical triangle `T`, let `hT(x,y)` be the affine height of its plane
over its XY projection. The triangle represents the unbounded downward prism:

```text
P(T) = { (x,y,z) : (x,y) is in projectXY(T), z <= hT(x,y) }
```

At mesh-load time, all triangle AABBs are uploaded and cuBQL builds a GPU BVH.
The BVH is queried against itself in parallel. Prism `A` is completely contained
in prism `B` exactly when every projected vertex of `A` is inside the projected
triangle `B` and every vertex height of `A` is no higher than `hB` there. Such
triangles are flagged, their AABBs are invalidated, and the BVH is rebuilt so
subsequent slice queries never visit them. Equal duplicate prisms retain the
lowest triangle ID.

Each horizontal slice launches one cuBQL box traversal per XY grid tile. Tiles
query from the slice height through the model's maximum Z, and atomic flags
deduplicate triangles crossing tile boundaries. Only candidate IDs return to
the CPU. Each candidate triangle is clipped against `z >= slice_z`, projected
to XY, normalized to a common winding, and unioned with Clipper2.

This is deliberately a downward-envelope slicer, not a conventional closed-solid
surface intersection. Vertical triangles have zero projected area and are
discarded. The CPU-only fallback preserves the same semantics but uses a simple
self-comparison, so CUDA is strongly preferred for substantial meshes.

When a stock mesh is loaded, its sliced polygons define the clipping domain and
wave seed boundary. Without one, the original allowance rectangle around the
subject is used.

## Wave propagation

For a subject `S`, allowance rectangle `B`, stepover `e`, and a one-integer-unit
boundary seed `R0`, cumulative reached areas and visible wave bands are:

```text
R0       = (B - InflatePaths(B, -1)) - S
R(n+1)   = (InflatePaths(Rn, e) intersect B) - S
Wave 0   = R0
Wave n+1 = R(n+1) - Rn
```

This makes propagation originate exclusively at the bounding box. A front that
hits the subject is blocked there, while its unobstructed portions continue and
can advance around obstacle edges. The loop stops when the cumulative reached
area is canonically identical to the previous pass, becomes empty, or reaches
the 2000-generation safety limit. Every non-empty wave band is retained for
visualization.

## Build and run

SDL2 and OpenGL development packages are required. On Ubuntu/Debian they are
available as `libsdl2-dev` and `libgl-dev`. CUDA 12 or newer is required for the
GPU path. CMake fetches pinned versions of Clipper2, Dear ImGui, and cuBQL. It
prefers `/usr/local/cuda/bin/nvcc` when available and builds for the native GPU.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/clipper2_wave_demo
```

Build the functional CPU fallback with:

```bash
cmake -S . -B build -DWAVE_ENABLE_CUDA=OFF
```

An alternate SVG can be passed as the first argument or with `--svg`. The focused parser accepts
closed `M`, `L`, `H`, `V`, and `Z` path commands and applies `scale`, `translate`,
`matrix`, and `rotate` transforms. Curve commands are intentionally rejected
instead of silently approximated.

Models can be loaded from the sidebar or at startup:

```bash
./build/clipper2_wave_demo \
  --subject-mesh part.obj \
  --stock-mesh stock.stl \
  --subject-inflate 0.25 \
  --stock-inflate -0.1
```

For automated visual checks, `--screenshot output.ppm` renders one frame and
exits. Run `./build/clipper2_wave_demo --help` for the complete command-line
reference, including initial slice height, query grid, allowance, stepover,
and boundary offsets.

## Controls

- Enter an OBJ/STL path and load the subject or optional stock independently.
- Drag inside the 3D panel to orbit; use its mouse wheel to zoom.
- Adjust **Slice Z** to reslice both loaded models.
- Adjust **BVH query grid** to trade traversal count for GPU parallelism.
- Adjust **Subject inflation** and **Stock inflation** independently. Positive
  values expand boundaries and negative values contract them. Offsets are
  applied after slicing, so changing them does not rebuild or query the BVH.
- Drag **Allowance** (when no stock model is loaded) or **Stepover** to
  recompute immediately.
- Use **Generation** to inspect a pass, or animate through passes.
- Toggle the subject, allowance box, or complete generation overlay.
- Toggle **Show leading fronts** to display the previous front in violet and
  the current front in green. Open-front endpoints are drawn as dots.
- Toggle **Show parent links** to display the ancestry between consecutive
  leading-front components.
- Use **Detected events** to jump directly to splits, merges, and termination.
- Right-drag the plan canvas to pan; use its mouse wheel to zoom.
- Inspect BVH build/prune, candidate-query, and CPU polygon-union timings in
  the sidebar.

Geometry is converted to 64-bit integers at 1000 integer units per transformed
SVG unit before it is sent to Clipper2.

## Leading fronts and lineage

The analyzer retains every cumulative reached set `R[k]` in addition to the
visible generation band `W[k] = R[k] - R[k-1]`. It computes unreached free space
as:

```text
U[k] = (allowance_box - subject) - R[k]
```

Every boundary segment of `W[k]` is classified with small normal probes. A
segment is leading exactly when one side is in `U[k]`; segments coincident with
the subject or allowance box are excluded. Consecutive leading segments are
retained as maximal open or closed curves.

For parentage, each front from generation `k-1` is inflated by Stepover and
intersected with the actual `k` generation band. Current leading curves are
clipped against that contribution to establish graph edges. The resulting
bipartite transition graph classifies:

- one parent to several children as a split;
- several parents to one child as a merge;
- a parent with no child as a termination;
- a child with no parent as an unexpected birth.

This is an exact description of the discrete generations, subject only to a
small integer tolerance at coincident boundaries. Events that both begin and end
within one stepover step remain below the temporal resolution of the graph.

## Toolpath sequencing

The lineage is ordered into a single tool motion by `build_toolpath`. A front may
only be entered once every parent that opened the way to it has been cut, so
sequencing is a topological order of the lineage graph. Among the fronts that
are ready, the nearest to the current tool position is taken next; because a
front's children are the fronts closest to it, this walks each branch to its end
before backtracking without needing an explicit depth-first traversal.

Each front is entered at its nearest point: a closed front is rotated to start
there, an open one is reversed when its far endpoint is closer. Closed fronts are
normalised to one winding so the cutter stays on the same side of the material
throughout; which winding is climb is left to the post stage.

The move between two consecutive cuts is a straight segment, classified by what
it passes through. `cleared` accumulates band pieces as they are actually cut,
which is not the same as the cumulative reached set: with branch-first ordering,
generation 20 of one branch is cut long before generation 12 of another. The
classification is:

- **link** when the segment stays inside `cleared`, so the tool travels through
  air at depth;
- **step** when it also crosses the band piece its target front bounds, which is
  the same one-stepover bite the front itself takes;
- **retract** otherwise: lift, rapid, and plunge again.

The test is stated as avoiding the forbidden region -- everything not yet
cleared, which covers uncut material, the subject, and the space beyond the
stock at once -- rather than as staying inside the cleared region. Fronts are
offset output on the integer grid, so a move sliding along a wall sits tens of
units off the wall's true position and reads as a gouge under the direct test.

Band pieces are grouped as an outer contour with its holes. A hole ring taken on
its own describes the island it encloses rather than band material, which would
otherwise credit the subject as cleared and route links straight through it.
