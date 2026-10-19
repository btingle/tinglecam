# TingleCAM

A desktop CAM application for wavefront roughing, contour finishing, pockets,
plane clearing, and native STEP surface finishing. Load OBJ/STL/STEP part and stock models, configure operations,
preview toolpaths and material removal, and export G-code. Jobs save as JSON;
pockets and planes can also be defined without a mesh.

Requires CMake 3.20+, a C++20 compiler, the CUDA toolkit and a compatible NVIDIA
GPU, SDL2 development files, OpenGL, and the Open CASCADE C++ SDK
(`libocct-data-exchange-dev` on Debian/Ubuntu). CMake fetches the remaining dependencies
(Git and internet access required).

Build and launch from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/tinglecam
```

In the GUI, load models or a saved job, choose tools, machine, and material,
add operations, plan, then generate and save G-code. Machine/material profiles
and starter tools live in `data/`; local tools and settings live in `config/`.

Try the included example or generate G-code without opening a window:

```sh
./build/tinglecam --job data/examples/pockets-and-planes.json
./build/tinglecam --job data/examples/pockets-and-planes.json --gcode example.nc
./build/tinglecam --help
```

For face selection, ball-nose compensation, the experimental heightmap stock
simulator, SDK options and CLI examples, see
[STEP finishing and heightmap simulation](docs/step-finishing-heightmap.md).
The 3D heightmap result is raycast directly at full resolution using OpenGL 3.2
fragment shaders; this renderer does not require CUDA or CUDA/OpenGL interop.
The existing mesh slicer still requires CUDA.
Under WSL, startup automatically selects Mesa's D3D12 driver and the mesh GPU.
The active renderer is shown under Settings and Playback / Display; explicit
Mesa/driver environment overrides take precedence.
Timing, memory measurements and the repeatable benchmark are documented in the
[performance investigation](tools/benchmarks/heightmap_performance.md).

Run the tests with `ctest --test-dir build --output-on-failure`.

The standalone [marble track generator](tools/README-tracks.md) in
`tools/generate_tracks.py` exports track contours as OBJ, G-code, or STEP surfaces
for CAD applications such as SolidWorks. It accepts continuous DXF profiles and
can cap closed profiles to produce STEP solids.
