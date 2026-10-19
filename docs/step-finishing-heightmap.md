# STEP finishing and heightmap simulation

Checkpoint before this experiment: `8765523`.

## Models and operations

TingleCAM reads `.step` and `.stp` through native Open CASCADE. Imported models
retain trimmed BRep faces, derivatives, orientations and face identities. An
independent triangulation supplies the existing GPU mesh slicer and display.
OBJ/STL remain supported by every existing mesh operation.

Add **Surface finishing (STEP)**, choose a ball end mill, and select faces by
clicking the part in the setup view or checking the face list. Green marks the
selection; amber marks the face under the pointer. Showing just the part makes
selection easier. Saved selections include a fingerprint of the source file;
re-exporting a model requires reselection.

Surface finishing follows native U or V isoparametric curves. Stepover is in
millimetres along the surface, with adaptive subdivision of stretched parameter
domains. Along each contour the planner adaptively controls contact and tool-tip
chord error and limits segment length. Switching direction can substantially
change the usefulness of paths on a particular CAD patch. The current minimum
chord tolerance is 0.002 mm because commanded XY uses the existing 0.001 mm CAM
grid. This is a sampled tolerance, not a certified bound for arbitrary splines.

For radius `r`, contact `P`, oriented normal `n`, and allowance `a`, the ball
centre is `P + (r+a)n`; the commanded bottom tip is that centre minus `r` in Z.
Flat and vertical surfaces therefore receive different XYZ compensation.

Finishing currently generates geometric contours without collision avoidance.
It does not split cuts or insert retracts based on other faces, tool/shaft
interference, approach clearance or the sign of a face normal. Face trims and
singularities still bound the paths. Compatible contour ends
on adjacent selected faces join into continuous lanes. Select both faces in the
same operation and choose a U/V direction which runs across their shared edge.
Endpoints must be close and their tangents must agree; independent face grids
are retained, so arbitrary mismatched parameterizations are not remapped.

With **Bidirectional passes**, joined lanes can reverse to alternate direction.
Short neighboring passes use feed turnarounds on the native surfaces. Neighbours are
found within two stepovers in tool-tip or surface-contact space: ball
compensation can magnify the tool-tip spacing on convex faces. Turnarounds are
sampled from the compensated UV curve, with the same chord and segment-length
controls as cuts. Across adjacent faces, their shared native edge supplies a
contact point on both faces. Each side follows its own surface; a change in
normal is sampled around the ball's offset contour at the edge. These are
surface-following turns rather than straight shortcuts over convex profiles.

The `step_test` face selection forms one continuous path with one initial entry
and one final exit, without intermediate retract/plunge pairs. Disconnected
selections and one-way passes can still require clearance moves. The former
**Ignore collisions** toggle was removed; saved values are ignored when loading
older jobs. Normal stock allowance and remaining-stock feed assignment remain
active.
Reverse normals is available for open surfaces. Waterline, scallop-height,
rest-finishing and multi-axis paths are not implemented. Tool holders and larger
shanks are not represented by the tool library.

`--profile-planning` reports surface sampling work, linking wall time, sample
counts, seam joins, stay-down links and retracts; `operation_profile` separates
operation wall times. Work times sum across workers and must not be added to
wall times. The native CAD query helpers remain available for future work, but
finishing does not call them. See the
[track finishing benchmark](../tools/benchmarks/step_finishing_performance.md).

CLI examples (face numbers are 1-based; JSON stores 0-based numbers):

```sh
./build/tinglecam --part tracks_step_dxf/s.step --cad-info
./build/tinglecam --part tracks_step_dxf/s.step --box-stock 0 \
  --mode surface --face 8 --tool 6mm-ball --machine grbl-router --material hdpe \
  --surface-stepover 0.3 --surface-tolerance 0.01 --simulation-pitch 0.1 \
  --gcode /tmp/track-finish.nc
```

Face IDs belong to the particular STEP export; inspect your own file first.
Generated G-code requires the same machine/tool/workholding review as existing
TingleCAM operations. Use a previously roughed stock state for finishing rather
than expecting this strategy to rough a deep channel efficiently.

## Stock and planning

The simulation is an experimental XY heightmap with floating-point top/bottom Z
values. It deliberately ignores interior cavities. The first operation sets the
job grid (default 0.1 mm); later operations inherit the same grid and stock.
A 128-million-cell guard rejects excessively dense grids with an actionable error.
Snapshots share a paged directory of 32x32 tiles until written. Uniform heights
and repeated rows/columns are stored compactly without reducing resolution;
only changed pages, tiles and height arrays are copied.

In the GUI, edit **Stock grid pitch (mm)** on the first operation, then use
**Plan all** to rebuild it and the operations below it. Later operations show
the inherited setting and provide a shortcut to the first operation. **Last
planned grid** reports the actual pitch and dimensions of the retained result;
changing a field does not change that result until planning finishes.

The **Playback / Display** panel reports the actual simulated grid. The 3D
result uses native OpenGL raycasting at that full resolution, without converting
stock into a mesh or decimating its cells. Simulation pitch does not set
finishing pass spacing; use **Surface stepover (mm)** for that.

The renderer requires OpenGL 3.2 / GLSL 1.50, including texture buffers and a
floating-point depth attachment. It uses ordinary fragment shaders rather than
CUDA or compute shaders. A binary spatial hierarchy skips empty/occluded tile
bounds; rays intersect individual occupied top/bottom columns exactly. Uniform
and repeated-row/column tiles retain their compact representations on the GPU.
The renderer retains the displayed stock's page references plus compact CPU
staging and GPU buffers; copy-on-write page/tile identities identify changed uploads during
playback and backward seeks. Rotating an unchanged stock uploads no height data.
Stock, optional model meshes and the translucent cutter share a depth buffer;
paths and view controls remain explicit UI overlays.

This makes the stock renderer independent of CUDA hardware and CUDA/OpenGL
interop. The application still requires CUDA for the existing mesh slicer and
legacy polygon-prism renderer; this change does not replace those subsystems.
The full grid must fit the graphics driver's texture-buffer and memory limits.
Single pixel-centre rays preserve the stored column geometry; subpixel edges
can still alias, and grazing views can require more traversal. No surface
smoothing changes the simulation geometry.

Commanded XYZ phases are executed in order, including plunges, ramps, helices,
links and ball-nose motion. Straight flat/ball sweeps have an analytic continuous
lower envelope; arcs are chorded below the stock-grid resolution. Removed volume
assigns cutting feeds using the existing feed endpoints. Engagement is an
approximate volume/full-slot ratio, not a cutting-force model. Vertical entries
retain plunge feed. A touching descent whose clearance the grid cannot establish
is posted at plunge feed; an obstructed rapid traverse fails the operation.
Stock above usable flute length is rejected.

Playback replays the same XYZ phases from shared stock checkpoints, including
vertical moves while planar distance stands still. Each contour/layer keeps at
most ten checkpoints; forward playback executes only newly traversed motion.
Preliminary feed-estimation samples are discarded after the authoritative XYZ
schedule is built. The result view displays the
heightmap at its full simulation resolution through the native GPU renderer.
Stock queries for later operations derive polygons from the grid. Immutable
stock versions cache each requested Z polygon; planar subdivisions then maintain
their own Clipper geometry. The newest composed stock snapshot supersedes older
operations. Polygon extraction is deferred until a 2D stock section is displayed;
3D playback uses the native full-resolution raycaster. Such planning
slabs conservatively project their lower-height footprint; they are not an exact
volume representation of sloping or ball-machined stock.

Wavefront slicing, propagation, offsets, topology and cutting-path generation
remain the existing algorithms. Smooth planar cutter sweep evidence is retained
for their routing and subdivision boundaries so raster stair steps do not alter
their internal geometry. The planar simulation adapter uses a heightmap for
preliminary feed and landing estimates; the final serial XYZ pass supplies the
authoritative stock, feeds, timing and playback. Old simulation method keys load
as compatibility aliases for this backend.

Resolution affects narrow features, engagement estimates and clearance. This is
a heightfield experiment, not an exact solid-removal or machine collision
simulator. STEP finishing generates surface contours without collision avoidance.

## Building

Native STEP support defaults on and requires the C++ Open CASCADE development
SDK, independently of the Python `cadquery-ocp` used by the track exporter.
On Debian/Ubuntu install `libocct-data-exchange-dev` and its dependencies before
configuring. A custom SDK prefix can be supplied as:

```sh
cmake -S . -B build -DTINGLECAM_OCCT_ROOT=/path/to/occt/prefix
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

For the local experiment, Ubuntu OCCT 7.6.3 packages were extracted into
`build/occt-sdk` without changing system packages. The build cache points there.
Use `-DTINGLECAM_WITH_OCCT=OFF` for a mesh-only build; STEP import then reports
that the native dependency is unavailable.

## Validation of the track benchmark

Native OCCT 7.6.3 imported the generated DXF-profile track solids directly:

| Model | Faces | Display triangles | Finishing probe |
| --- | ---: | ---: | --- |
| `tracks_step_dxf/s.step` | 14 | 3,450 | Channel face 8, 6 mm ball, 2 mm stepover: 15 cutting contours |
| `tracks_step_dxf/L.step` | 14 | 27,676 | Channel face 8, 6 mm ball, 4 mm stepover: 7 cutting contours |

Both original probes used a 0.2 mm stock grid and produced G-code. Their former
collision-based skipped-region reports are historical; current finishing keeps
the selected contours. Native tests also cover planes, inclined normals, cylinders,
spherical poles, trimmed openings, source fingerprints,
XYZ replay and posting. Existing mesh CAM tests remain enabled. Interior-band
stock tests were replaced with top-down stock and volume-conservation tests.
The selected fourteen faces from `step_test` now form a regression fixture which
must have no intermediate retracts or plunges. Convex single-face and shared-edge
tests verify that turnarounds follow the compensated profile within chord tolerance.

## Performance diagnostics

`--profile-planning` reports geometric planning and final XYZ simulation times,
shared heightmap payload, motion schedules, wave/path geometry and saved
polygons. Linux also reports resident, peak resident and virtual process memory;
glibc builds report live/free allocator payload. Payload counts are estimates,
not a complete process-memory census.

See [the performance investigation](../tools/benchmarks/heightmap_performance.md)
for measured before/after results and a reproducible benchmark script.

Native renderer details, portability checks and frame-time probes are in
[the rendering report](../tools/benchmarks/heightmap_rendering.md).

## Graphics startup under WSL

On WSL with `/dev/dxg`, the app selects `GALLIUM_DRIVER=d3d12` before SDL loads
OpenGL. It uses the current mesh-processing device name as the missing default
for `MESA_D3D12_DEFAULT_ADAPTER_NAME`, so the tested local setup selects the
NVIDIA GeForce RTX 5060 instead of llvmpipe or the integrated adapter. The
heightmap renderer itself remains an OpenGL backend with no CUDA calls.

These defaults affect only the application process. An explicit `GALLIUM_DRIVER`,
`MESA_LOADER_DRIVER_OVERRIDE`, or enabled `LIBGL_ALWAYS_SOFTWARE` prevents the
automatic selection. An explicit adapter name is preserved. If automatic
hardware graphics initialization fails, the app restores the previous driver
settings and retries startup. Native Linux and Windows selection are unchanged.

The actual `GL_RENDERER` appears in stderr, Settings, and Playback / Display.
For comparison, explicit overrides still work:

```sh
LIBGL_ALWAYS_SOFTWARE=1 ./build/tinglecam
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=AMD ./build/tinglecam
```

The unfinished 64x64/16-bit block-renderer experiment is parked outside the
build; this startup change uses the previously tested full-resolution float
renderer so that driver performance can be assessed separately.
