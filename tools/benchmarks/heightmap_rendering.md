# Native heightmap rendering

The stacked result raycasts reconstructed heightmap surfaces in OpenGL
3.2 / GLSL 1.50 fragment shaders. It does not tessellate or decimate the stock,
use CUDA, or copy rendered pixels through host memory. The ImGui canvas displays
the renderer's colour texture directly. The existing CUDA mesh slicer and
stock-removal simulation remain separate from this rendering path.

## Representation and updates

A binary spatial hierarchy encloses the 32x32 stock tiles. Rays traverse nearer
bounds first, shorten their interval after a hit, and skip empty or occluded
bounds. Uniform tiles intersect as single boxes; other tiles use XY DDA to test
bilinear top/bottom patches. Local height ranges reject empty ray intervals
before constructing patches or solving intersections. Fully connected patches
skip discontinuity propagation. The renderer reads scalar, row,
column and dense float representations directly. It also handles bottom faces,
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
High-DPI canvases allocate the raycast target at framebuffer pixel density.
Cap normals use screen derivatives where they agree with the surface;
close-up views and creases retain neighbourhood sampling. Wall normals filter
occupancy gradients over a dense neighbourhood with a radius of two to four
cells, chosen from the pixel footprint. This reduces vertical lighting bands
while preserving cap/wall creases; sparse sampling of a wider grid is avoided
because it introduces new aliasing. Unchanged
stock/camera/cutter frames reuse their completed colour/depth texture. Mutable
mesh overlays force a redraw, because they have no geometry revision identifier.

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
