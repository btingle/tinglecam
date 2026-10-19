# Native heightmap rendering

The stacked result now raycasts full-resolution heightmap columns in OpenGL
3.2 / GLSL 1.50 fragment shaders. It does not tessellate or decimate the stock,
use CUDA, or copy rendered pixels through host memory. The ImGui canvas displays
the renderer's colour texture directly. The existing CUDA mesh slicer and
legacy polygon-prism renderer remain separate dependencies of the application.

## Representation and updates

A binary spatial hierarchy encloses the 32x32 stock tiles. Rays traverse nearer
bounds first, shorten their interval after a hit, and skip empty or occluded
bounds. Uniform tiles intersect as single boxes; other tiles use XY DDA to test
the occupied top/bottom column intervals. The renderer reads scalar, row,
column and dense float representations directly. It also handles bottom faces,
vertical walls, empty cells and partial tiles along the grid boundary.

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

## Measurements

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

The renderer test compares pixel depths with an independent CPU traversal of
every occupied column. Cases cover top, underside, horizontal, nearly vertical
and oblique views; negative origins, empty cells, varying floors, both pattern
orientations, dense diagonal ball cuts and partial edge tiles. It verifies
single-tile updates, backward snapshot restores, zero uploads for unchanged
stock, compact large grids, stock/model/cutter depth occlusion, empty scenes and
OpenGL state restoration. All fifteen CTest targets pass. A finishing playback
screenshot of `tracks_step_dxf/s.step` at 0.025 mm pitch was also inspected.

The graphics device must support the existing OpenGL 3.2 context and have enough
texture-buffer capacity and memory for the scene. Subpixel column boundaries
can still alias; grazing angles can require more ray traversal. Stock geometry
is not smoothed for display, and there is no silent resolution reduction.

The API requirements are standard desktop OpenGL:
[Khronos buffer textures](https://wikis.khronos.org/opengl/Buffer_Texture),
[fragment shaders](https://wikis.khronos.org/opengl/Fragment_Shader), and
[depth testing](https://wikis.khronos.org/opengl/Depth_Test).
