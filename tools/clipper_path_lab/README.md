# Clipper Path Lab

Small, standalone 2D path playground powered by Clipper2. It deliberately
lives outside the main application and has its own CMake project.

## Build

From this directory:

```sh
cmake -S . -B build
cmake --build build -j
./build/clipper_path_lab
```

For an offline build using the dependency trees already present in the parent
repository:

```sh
cmake -S . -B build \
  -DCLIPPER2_CPP_DIR=../../build/_deps/clipper2-src/CPP \
  -DIMGUI_SOURCE_DIR=../../build/_deps/imgui-src
cmake --build build -j
```

## Use

- Add a rectangle, circle, or open line from the Primitives panel. Rectangle
  width and height are independent.
- Choose `Draw path`, then click points on the canvas. Press Enter to finish;
  toggle `Closed` before drawing for a closed path.
- Each path has explicit `Set A`, `Set B`, and `Delete` buttons. `A` is the
  subject and `B` is the clip, so Difference means `A - B`. The highlighted
  `A*` and `B*` labels show the current assignments.
- Positive offset expands a path and negative offset contracts a closed path.
  Open-path offsets become closed outlines, using Clipper2's rounded end type.
  `Arc tolerance` controls rounded-corner detail: lower values create more
  segments and smoother corners, while higher values are lighter/faster.
- Translate or rotate operand A from Transform operations. Rotation is
  counter-clockwise and uses the editable pivot (the origin by default).
- Copy/Paste uses the system clipboard. Duplicate and closed/open conversion
  are available for the A path.
- `Convert A to open (preserve)` keeps the former closing edge by appending a
  duplicate of the starting vertex; the regular conversion removes that edge
  visually.
- `Pick vertices on A` enters vertex-pick mode. Select two vertices on the
  canvas and choose `Keep between selected`; open paths create one result,
  while closed paths create both possible open arcs.
- OBJ import/export uses raw `v` and `l` curve records; each `l` record becomes
  one path and a line that repeats its first vertex is treated as closed.
- Adjust GUI scale in the Interface section.

Open paths can be unioned and can be clipped by closed paths. Clipper2 does
not accept an open path as a clipping region, so the UI reports that constraint
for invalid open/open or `closed - open` boolean requests.
