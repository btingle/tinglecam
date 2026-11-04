# Native heightmap rendering

The stacked result raycasts reconstructed heightmap surfaces in OpenGL
3.2 / GLSL 1.50 fragment shaders. It does not tessellate or decimate the stock,
use CUDA, or copy rendered pixels through host memory. The ImGui canvas displays
the renderer's colour texture directly. The existing CUDA mesh slicer and
stock-removal simulation remain separate from this rendering path.

## Representation and updates

A binary spatial hierarchy encloses the 32x32 stock tiles. Its projected root
bounds limit the raycasting draw to a conservative screen rectangle, avoiding
shader invocations over the surrounding background. Rays traverse nearer
bounds first, shorten their interval after a hit, and skip empty or occluded
bounding boxes. Uniform tiles intersect as single boxes; other tiles use XY DDA to test
bilinear top/bottom patches. Local height ranges reject empty ray intervals
before constructing patches or solving intersections. Fully connected patches
skip discontinuity propagation. The renderer reads scalar, row,
column and dense float representations directly. Patches within a tile share
one descriptor fetch. It also handles bottom faces,
vertical walls, empty cells and partial tiles along the grid boundary.

Vertical walls intersect the bilinear 50% material boundary between samples,
rather than the original square column edges. Connected top/bottom sheets extend
to this same boundary, including at holes and height steps. Most wall intervals
need only one quadratic intersection; intervals crossing caps reuse cap equations
to validate candidate intersections. Empty neighbouring tiles carry only the
half-cell border halo needed by reconstruction. Uniform-box shortcuts are disabled
beside incompatible or empty neighbours. None of this constructs a wall mesh or
changes the stock-removal grid.

Immutable page and tile references identify changes between displayed stock
states. The renderer packs just the current scene into CPU staging and GPU
buffers; it does not retain a new GPU snapshot for each playback position.
Changed slots, descriptors and ancestor bounds upload as merged ranges. Growing
a GPU buffer requires uploading its packed payload again. Camera changes alone
send no height data. CPU staging mirrors and hierarchy bookkeeping also consume
memory; reported resident buffer bytes are not a complete process/driver census.

The stock shader writes a floating-point depth attachment. Loaded model meshes
and the translucent ball/flat cutter use ordinary GPU triangle rasterization in
the same framebuffer, with the same camera and depth convention. Stock occludes
buried geometry. Requested toolpaths and view controls remain UI overlays.
Setup uses framebuffer pixel density. Result starts at half that density on
each axis (one-quarter as many pixels), then linearly magnifies the colour
texture; its camera scale and pan use the same reduction to preserve framing.
Large Result canvases additionally cap the target at approximately 1,048,576
pixels (plus rounding each axis up). This is a pixel-count budget, so portrait,
ultrawide and high-DPI windows retain their aspect ratio. A full 4K canvas uses
roughly a 1365x768 target instead of 1920x1080. Smaller canvases retain the
half-density target. These defaults can be overridden under **Tools > Settings**:
**Stock / Result resolution** offers Automatic, 100%, 75%, 50% and 25%;
**Surface / Setup resolution** offers 100%, 75%, 50% and 25%. Explicit stock
percentages bypass the automatic pixel budget. Both settings persist across
sessions. UI text retains its original resolution.
Simulation pitch and the native-resolution UI captions/gizmos are unchanged.
Cap normals use screen derivatives where they agree with the surface;
close-up views and creases retain neighbourhood sampling. Wall normals filter
occupancy gradients over a dense neighbourhood with a radius of two to four
cells, chosen from the pixel footprint. This reduces vertical lighting bands
while preserving cap/wall creases; sparse sampling of a wider grid is avoided
because it introduces new aliasing. Unchanged
stock/camera/cutter frames reuse their completed colour/depth texture. Immutable
model handles also participate in frame identity. Mesh vertices and normals
are uploaded once and retained across camera, placement, colour, wireframe,
face-selection, and playback changes. Mesh buffers absent from the current
scene are released. Mutable raw-pointer callers retain an uncached compatibility
path; the application's setup and result views use owned immutable handles.

The setup preview uses the same GPU mesh pipeline, with CAD face ranges sharing
one vertex buffer. Native face picking uses a cached BVH, and contour visibility
uses a cached GPU depth render instead of CPU triangle rasterization. Headless
UI tests retain an independent CPU depth oracle. Origin gizmos, captions and
operation annotations remain UI overlays.

## Setup and mesh caching, 2026-11-03

A bounded end-to-end setup-view probe used the second setup of local
`setup_test1.json`, with 267,812 triangles across the part and fixture, at
960x720 on **D3D12 (NVIDIA GeForce RTX 5060)** in Release mode. Each measurement
averages 30 frames after five warm-up frames, including ImGui submission and
`glFinish`. Import, planning, vsync and stock simulation are excluded.

| Setup view | CPU triangle path | GPU mesh path |
| --- | ---: | ---: |
| Unchanged camera | 16.73 ms | 1.17 ms |
| Orbiting camera | 19.70 ms | 0.55 ms |

Both new measurements reported **zero mesh upload bytes** after warm-up.
The two mesh buffers used 19,282,464 bytes. Unchanged frames reused the rendered
image; orbiting frames redrew using the retained geometry. These are local setup
view timings, not a promise for full-resolution stock playback or the entire app.

Reproduce without planning the example:

```sh
cmake --build build --target scene_rendering_benchmark -j 4
timeout --kill-after=2s 20s ./build/scene_rendering_benchmark setup_test1.json
```

The renderer tests cover cache invalidation, CAD face ranges, colours,
transparency, offsets, wireframes, geometry replacement and GPU buffer release.
The GUI integration test exercises the setup view with fixtures and box stock,
native CAD selection and GPU contour depth against a CPU projection oracle.

## Half-resolution Result and shader optimisation, 2026-11-03

The shader now walks directly between sample-centre planes bounding bilinear
patches. Integer cell edges within a patch no longer require extra intervals
or repeated reconstruction. Missing-sample patches go straight to occupancy
reconstruction instead of calculating unused connected-cap coefficients.
Traversal descends immediately into the nearer child without repeating its
bounding-box test; flat leaves use one box intersection. Wall-normal filtering
combines its separable weights before accumulation, preserving the dense sample
filter, and cap-normal fallback gradients are computed only when needed.
These changes use the existing hierarchy and uploaded heights without new caches.

A bounded Release benchmark on **D3D12 (NVIDIA GeForce RTX 5060)** used a 40x40x8 mm
block with three ball-machined grooves. Each moving-camera measurement averages
eight frames with `glFinish`, excluding cold compilation, stock creation and GUI.
The half-resolution camera preserves the same framing.

| Stock pitch | Previous shader, 960x720 | Optimized shader, 960x720 | Optimized shader, 480x360 |
| --- | ---: | ---: | ---: |
| 0.1 mm | 2.56 ms | 1.36 ms | 1.12 ms |
| 0.025 mm | 5.94 ms | 3.42 ms | 2.14 ms |
| 0.01 mm | 8.46 ms | 4.24 ms | 2.12 ms |

The same-resolution shader improvement was approximately 42–50%; combined with
the smaller Result target, these component measurements improved by 2.3–4.0x.
They are synthetic render timings, not application-wide frame-rate guarantees.
Full-resolution before/after readbacks differed by at most 2.1e-7 in normalized
depth, with a handful of corner colour differences from floating-point ordering.
Independent geometric-reference tests additionally cover long rays through tile
seams, partial tiles, holes, stepped tops and undersides, and grazing views.
The GUI test verifies the quarter-pixel Result target at both normal and high DPI.

The test benchmark accepts an optional image prefix to save colour/depth pairs:

```sh
./build/heightmap_renderer_tests --benchmark 960 720 /tmp/heightmap-full
./build/heightmap_renderer_tests --benchmark 480 360 /tmp/heightmap-half
```

## 4K tuning on setup_test1.json, 2026-11-04

The target is setup **Op 1** (the second setup), displaying its final
**Surface finish 2** result, with the fixture and cutter visible. The stock is
1600x3600 samples at 0.02 mm pitch, including the inherited underside. This
probe uses the real job and configured tool/material libraries rather than a
synthetic stock. It plans the job once, then measures the application Result
view with ImGui submission and `glFinish` on **D3D12 (NVIDIA GeForce RTX 5060)**.
Planning, cold shader compilation, vsync and live stock advancement are excluded.
Each orbit measurement averages 30 changing camera orientations after warm-up.

At a 3840x2160 window, the previous half-density target took **17.65 ms** per
orbit frame. Sharing compressed-tile descriptors across patch corners and
clipping the stock draw to its projected bounds took **16.40 ms** at the same
resolution; the before/after 4K screenshots were byte-identical. Adding the
approximately one-million-pixel Result budget took **14.51 ms**, about **18%**
less frame time overall. The capped target contains 1,049,420 pixels after
rounding. Idle frames remained approximately 1.3 ms with no recurring mesh
uploads. These are local Result-view timings, not whole-application FPS or a
promise of the same improvement at every camera angle.

Neither change adds a cache or alters simulation pitch, intersections or normal
filtering. Candidate traversal and cap-equation changes that did not improve
this job were discarded. Regression checks cover screen-edge clipping,
completely offscreen stock, caller scissor restoration, portrait targets and
high-DPI windows, in addition to the existing independent intersection oracle.

Reproduce the real-job probe (it requires the source models referenced by the
local file and the configured tool/material libraries):

```sh
cmake --build build --target scene_rendering_benchmark -j 4
timeout --kill-after=2s 60s ./build/scene_rendering_benchmark setup_test1.json \
  --result --size 3840 2160 /tmp/op1-result.ppm
```

`--result` displays the last operation's completed stock; without it the tool
benchmarks Setup. `--size` sets the view dimensions. The progress callback
cancels planning after 45 seconds; the outer timeout also bounds non-cancellable
work. No job files or user configuration are written.

## Reconstructed-surface optimisation, 2026-10-20

Local Release-build probe of `allowance_remove_test.json` at its saved **0.02 mm**
pitch, using **D3D12 (NVIDIA GeForce RTX 5060)**. Each orbit measurement averages
eight changing camera orientations and includes `glFinish`. It excludes planning,
cold shader compilation, stock removal and the surrounding GUI. The cache cannot
serve these moving-camera frames. Pixel resolution and simulation data are the
same before and after the changes.

| View | Before | After |
| --- | ---: | ---: |
| 960x720, elevation 0.62 rad | 18.29 ms | 9.70 ms |
| 1440x1080, elevation 0.62 rad | 30.01 ms | 13.26 ms |
| 960x720, elevation 0.18 rad | 15.67 ms | 8.68 ms |

Unchanged renderer calls reuse the completed frame without raycasting or querying
OpenGL state. A short forward-playback probe on pass 31 measured about **0.51 ms**
for CPU stock advancement and **9.70 ms** for stock upload/rendering at 960x720.
These are local component timings, not end-to-end GUI frame-rate guarantees.
The application prints its actual OpenGL renderer at startup and under
Tools > Settings; `llvmpipe` indicates software/CPU rendering.

## Shader wall reconstruction, 2026-10-20

Same Release-build job/device probe as above, with a fresh baseline using the
previous square wall intersections. Both measurements move the camera and wait
for GPU completion; neither can use the idle-frame cache.

| View | Previous walls | Reconstructed walls + filtered normals |
| --- | ---: | ---: |
| 960x720, elevation 0.62 rad | 8.93 ms | 11.88 ms |
| 1440x1080, elevation 0.62 rad | 12.35 ms | 18.29 ms |
| 960x720, elevation 0.18 rad | 7.68 ms | 11.20 ms |

The more accurate wall intersections have a measurable rendering cost. A short
pass-31 playback probe measured 0.49 ms CPU stock advancement and 11.26 ms stock
upload/rendering at 960x720. Unchanged frames still reuse their colour/depth
texture. No additional dense stock buffer or wall mesh is allocated. Synthetic
coarse-grid curved walls and the actual allowance-removal result were inspected
for silhouette stair steps and vertical lighting bands.

## Measurements of the original column renderer

Single local probe on 2026-10-16, Release build, **Mesa llvmpipe**, LLVM 20.1.2,
Mesa 25.2.8, with the advertised context restricted to OpenGL 3.2 / GLSL 1.50.
This is a software OpenGL implementation, not a discrete GPU performance claim.

The scene is a 40x40x8 mm block with three ball-machined grooves. The viewport is
640x480, oblique orthographic, with stock only. Frame time is the mean of eight
slightly panned frames, including `glFinish`; it excludes planning, playback
cuts, the rest of the application UI, and cold shader compilation.

| Pitch | Simulation cells | Mean frame | Renderer buffer estimate |
| --- | ---: | ---: | ---: |
| 0.1 mm | 160,000 | 4.00 ms | 2.4 MiB |
| 0.025 mm | 2,560,000 | 4.91 ms | 2.7 MiB |
| 0.01 mm | 16,000,000 | 8.05 ms | 4.7 MiB |

Every unchanged-scene frame uploaded **zero height-data bytes**. A separate
100-million-cell uniform-stock check used **11.2 MiB** of stock/hierarchy/image
buffers at a 32x32 test viewport. These low payloads depend on uniform/repeated
heights; an arbitrary densely machined grid requires more memory.

## Validation and reproduction

```sh
cmake --build build --target heightmap_renderer_tests
ctest --test-dir build -R heightmap_renderer_tests --output-on-failure
SDL_VIDEODRIVER=offscreen LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_GL_VERSION_OVERRIDE=3.2 MESA_GLSL_VERSION_OVERRIDE=150 \
  ./build/heightmap_renderer_tests --benchmark
```

The renderer test compares pixel depths with an independent exhaustive CPU
reference for bilinear material boundaries between horizontal cap levels, and
with analytic plane/saddle intersections for smooth caps. Cases cover top,
underside, horizontal, nearly vertical and oblique views; negative origins,
empty cells, varying floors, both pattern orientations and partial edge tiles.
Explicit wall checks cover curved isolines, height steps, holes, raised
undersides, sloping cap joins and reconstruction inside empty neighbouring
tiles. Incremental edits and backward seeks must update those neighbours too.
Other checks cover zero uploads for unchanged stock, frame caching, compact
large grids, stock/model/cutter depth occlusion and OpenGL state restoration.
All sixteen CTest targets pass.

The graphics device must support the existing OpenGL 3.2 context and have enough
texture-buffer capacity and memory for the scene. Subpixel boundaries can still alias, and grazing angles can require more ray
traversal. Reconstructed boundaries remain limited by the simulation samples;
coarse grids cannot recover an exact original curve. The display interpolation
uses the original simulation pitch without silently reducing its resolution.

The API requirements are standard desktop OpenGL:
[Khronos buffer textures](https://wikis.khronos.org/opengl/Buffer_Texture),
[fragment shaders](https://wikis.khronos.org/opengl/Fragment_Shader), and
[depth testing](https://wikis.khronos.org/opengl/Depth_Test).
