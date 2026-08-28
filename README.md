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

### Leading in from the air

Seeded on the stock wall, the first contour is a full stepover inside it and the
cutter arrives at full width: whatever the lead-in, a pass whose every side is
solid takes a slot. So the wave is instead seeded a whole number of stepovers
out in the open air, and the first few generations produce contours that stand
clear of the stock. Following them, the tool takes up its bite over the run-in
rather than at first contact. `build_wavefront` takes the lead-in as a distance
and rounds it up to whole steps, so a front lands exactly on the wall rather
than a fraction of a step short of it; the demo asks for one tool radius, which
is what it takes a cutter to go from just touching the wall to fully across it.

The approach region is grown from the wall the wave actually starts on, not from
the stock solid:

```text
Wall     = ((K - InflatePaths(K, -1)) - S)
Approach = InflatePaths(Wall, n*e) - S
D        = K union Approach
R0       = (D - InflatePaths(D, -1)) - InflatePaths(S, 2)
R(n+1)   = (InflatePaths(Rn, e) intersect D) - S
```

Only boundary the wave starts from gets a run-in, so stock the subject occludes
contributes none; growing the solid instead would have manufactured a lead-in
running along the face of the part. `R0` is cut back by a hair more than the
seed ring is thick, because an approach region is carved out of open air and
leaves the part exposed alongside the outer edge that is meant to be the only
source -- inside the stock the subject covers that boundary and subtracting it
is enough, but in the air it is not, and the part would become a wave source in
its own right.

`WaveResult` carries the domain `D`, and the lineage bounds fronts with it
rather than with the stock. Bounding them with the stock would leave every
run-in band with nothing unreached ahead of it and no front to hand on, and
would call the stock wall a static boundary at the very point the wave is meant
to cross it. With no lead-in the two regions are the same and nothing changes.

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
  --stock-inflate -0.1 \
  --tool-radius 3.175 \
  --simulation-stock-inflate 0.05
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
- Toggle **Lead in from the air** to seed the wave one tool radius outside the
  stock, so the opening passes stand clear of it and take up their bite over the
  run-in. Turning it off starts the wave on the stock wall, where the first pass
  arrives at full width.
- Use **Generation** to inspect a pass, or animate through passes.
- Toggle the subject, allowance box, or complete generation overlay.
- Toggle **Show leading fronts** to display the previous front in violet and
  the current front in green. Open-front endpoints are drawn as dots.
- Toggle **Show parent links** to display the ancestry between consecutive
  leading-front components.
- Use **Detected events** to jump directly to splits, merges, and termination.
- Toggle **Show toolpath** to overlay the sequenced tool motion, and use
  **Play**, the step buttons, and **Progress** to drive a tool marker along it.
  The marker is coloured by what the tool is doing, cuts already made are drawn
  in white, and transitions stay visible ahead of the tool so a stepover, an
  in-material link, and a retract can be told apart at a glance. Cuts still to
  come are left out, since the generation contours already draw them.
- Adjust **Tool radius** to resize the cutter. It is a planning figure as well
  as a simulation one -- the lead-in and every plunge clearance are sized from
  it -- so moving it rebuilds the toolpath. **Simulation stock inflation** is
  the simulation's alone and only re-sweeps the existing path. Toggle **Show
  material simulation** to watch the raw stock slice be subtracted as the tool
  advances. The dashed cyan contour is the
  initial simulation stock and the solid cyan contour is what remains.
- Toggle **Color completed path by cutter load** to replace move colors with a
  blue/green/red engagement scale. Magenta sections have removed final-subject
  material. Red X markers identify retract destinations where the circular
  tool footprint still overlaps stock and a rapid plunge would be unsafe.
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

The stock stands in air, so the space beside it counts as already cleared: the
opening move plunges there and feeds in horizontally rather than ramping into
solid material, and later transitions may run round the outside of the stock
instead of lifting over the part. That space is held off the stock wall by the
same slack the clearance tests allow, or a move grazing along the wall would
read as free travel when it is in fact cutting.

What a descent sweeps is a disc of the tool's radius, so that radius is how far
inside cleared material a plunge has to land, and `build_toolpath` takes it for
that reason. Half a stepover is the floor rather than the figure -- it puts the
landing along the middle of the band the parent front opened rather than against
an edge -- and it is what this used to use outright, back when the radius was
known only to the simulation and every plan came out the same for every cutter.
A landing that already stands clear is used as it is, instead of being projected
onto the nearest edge of the clearance zone and then fed across to a place the
tool could have descended straight onto.


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
- **retract** when the straight segment is blocked and no route around it is
  worth taking: lift, rapid, and set the tool down in cleared material, from
  where a further move at depth reaches the front. The descent itself never
  cuts;
- **entry** where not even that is possible, because nothing has been cleared
  near the front yet. The tool has to ramp or helix into solid material, and the
  first move of any toolpath is one of these.

For a blocked segment, the planner constructs both an at-depth detour and a
retract/rapid/approach. It compares their complete represented XY distances and
takes the shorter candidate. This is deliberately a distance comparison rather
than a guessed detour multiplier. The slice does not provide a clearance height,
vertical motion, or separate feed and rapid rates; a postprocessor with those
values can replace the distance cost with elapsed time. The detour is walked in
both directions around travelable-region boundaries and then pulled straight
wherever a shortcut stays clear. Pulling straight is only an optimisation:
dropping vertices along a contour that hugs an obstacle leaves chords cutting
through it, so the shortened walk is kept only when it survives the clearance
test and the contour itself is used otherwise. Neither is trusted without
checking -- a walk taken from a region's own boundary can still carry a stretch
the tool may not follow.

A lift is planned the same way. The slice says nothing about how far the part
rises above it, so a rapid is routed through the free space around the subject
rather than straight over it. Where the free space is in two pieces there is no
such route, the rapid crosses the part, and `lifts_over_subject` counts it.

The clearance test erodes the forbidden region so that a move may graze along a
wall, and that has a cost worth stating: an obstacle thinner than twice the
tolerance vanishes from the test entirely, and a route may then be planned
straight through a rib of the part narrower than that. Measured over the sample
meshes this affects a small minority of lifts. Restoring the thin material by
morphological opening was tried and made matters worse, because it also blocks
the ordinary transitions that slide along a wall.

The test is stated as avoiding the forbidden region -- everything not yet
cleared, which covers uncut material, the subject, and the space beyond the
stock at once -- rather than as staying inside the cleared region. Fronts are
offset output on the integer grid, so a move sliding along a wall sits tens of
units off the wall's true position and reads as a gouge under the direct test.

Band pieces are grouped as an outer contour with its holes. A hole ring taken on
its own describes the island it encloses rather than band material, which would
otherwise credit the subject as cleared and route links straight through it.

## Material subtraction simulation

The simulator reuses the same raw stock and subject slices as the planner but
keeps their physical and planning roles separate:

- `--stock-inflate` and `--subject-inflate` affect wave planning only;
- `--simulation-stock-inflate` is the only offset applied to simulation stock;
- final-subject gouges are tested against the raw subject slice with no
  inflation; and
- `--tool-radius` controls the circular cutter footprint. It defaults to half
  the initial stepover, and unlike the offsets above it is not simulation-only:
  the planner sizes the lead-in and every plunge clearance from it, so changing
  it rebuilds the toolpath. `--no-lead-in` seeds the wave on the stock wall
  instead of out in the air.

`Cut`, `Step`, and `Link` polylines run at depth. Each is subdivided to roughly
one tool radius of travel per sample, inflated with round ends by the tool
radius, and subtracted from the stock left by earlier samples. Treating links
this way is intentional: a link expected to be clear by the planner can still
remove material when the independent simulation stock is larger.

The cutter-load proxy for one sample is the newly removed area divided by
`2 * tool_radius * sample_length`. A value of 100% corresponds approximately to
a full-width slot. Values can exceed 100% at an initial engagement or tight
corner because the circular end of the tool also enters new material. This is a
2.5D area-per-distance estimate; a later G-code stage can multiply by axial
depth and feed rate to estimate removal volume per unit time.

Gouges are the portion of actual removed stock intersecting the uninflated
final subject. For every `Retract`, the simulator also intersects the tool disk
at the rapid destination with stock remaining at that point. A non-empty
intersection is reported as an unsafe plunge and is not silently accepted as a
cut.

`Entry` moves remain unresolved. The current toolpath records only their XY
transfer, while a physically meaningful entry simulation needs the ramp or
helix and its Z motion. Retract XY motion is likewise treated as above the work;
only its destination plunge is checked.
