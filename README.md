# TingleCAM

A desktop CAM application for wavefront roughing, contour finishing, pockets,
and plane clearing. Load OBJ/STL part and stock models, configure operations,
preview toolpaths and material removal, and export G-code. Jobs save as JSON;
pockets and planes can also be defined without a mesh.

Requires CMake 3.20+, a C++20 compiler, the CUDA toolkit and a compatible NVIDIA
GPU, SDL2 development files, and OpenGL. CMake fetches the remaining dependencies
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

Run the tests with `ctest --test-dir build --output-on-failure`.
