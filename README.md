# TingleCAM

A desktop CAM application for wavefront roughing, contour finishing, pockets,
holes and counterbores, plane clearing, and native STEP surface finishing. Load OBJ/STL/STEP part and stock models, configure operations,
preview toolpaths and material removal, and export G-code. Jobs save as JSON;
pockets, planes and manual holes can also be defined without a mesh.

Requires CMake 3.20+, a C++20 compiler, SDL2 development files, OpenGL 3.2+, and the Open CASCADE C++ SDK
(`libocct-data-exchange-dev` on Debian/Ubuntu). CMake fetches the remaining dependencies
(Git and internet access required).

Build and launch from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/tinglecam
```

CUDA mesh acceleration is built when a CUDA toolkit is detected. Without one,
the build uses the cuBQL CPU slicer. To explicitly disable CUDA, configure with
`-DTINGLECAM_WITH_CUDA=OFF`. At runtime, compatible CUDA devices are preferred;
missing/incompatible devices or drivers use the CPU slicer automatically.
`TINGLECAM_SLICER=cpu` forces CPU queries, while `TINGLECAM_SLICER=cuda` requires
CUDA and reports an error if it cannot be used. The default is `auto`.

In the GUI, open a job or start one: choose the part model, the stock, the
machine and the material in the setup panel, add operations, press **Plan**
(F5), watch the result, then **Export G-code** (Ctrl+E). The Help menu lists the
keyboard shortcuts.

Feeds and speeds come from the tools. Each tool in the library (Tools > Tool
library) carries cutting data per material: spindle speed, chip load and plunge
feed. An operation uses its tool's data for the job material, and can override
any of the three. The full-engagement feed is rpm x flutes x chip load; lighter
radial cuts run faster to keep the same chip, up to the machine's maximum feed.
Machines only set limits (spindle range, rapid rates) and post settings.

Machine and material profiles and starter tools live in `data/`. Your own tools
and settings live in `~/.config/tinglecam/` (`%APPDATA%\TingleCAM` on Windows);
on first run the starter tools are copied there.

Try the included example or generate G-code without opening a window:

```sh
./build/tinglecam --job data/examples/pockets-and-planes.json
./build/tinglecam --job data/examples/pockets-and-planes.json --gcode example.nc
./build/tinglecam --help
```

The **Hole** mode supports independent circular-hole/counterbore identification,
per-level drilling or milling strategies, and linked pre-drilling operations.
See [holes and counterbores](docs/hole-mode.md) or open
`data/examples/holes-and-counterbores.json`.

For face selection, ball-nose compensation, the experimental heightmap stock
simulator, SDK options and CLI examples, see
[STEP finishing and heightmap simulation](docs/step-finishing-heightmap.md).
The 3D heightmap result raycasts the full simulation grid using OpenGL 3.2
fragment shaders; this renderer does not require CUDA or CUDA/OpenGL interop.
Tools > Settings provides independent render-resolution controls for Stock/Result
and Surface/Setup; these affect display quality without changing simulation pitch.
The mesh slicer uses CUDA when available, with a cuBQL CPU fallback.
Under WSL, startup automatically selects Mesa's D3D12 driver and the mesh GPU.
The active renderer is shown under Tools > Settings; explicit
Mesa/driver environment overrides take precedence.
Timing, memory measurements and the repeatable benchmark are documented in the
[performance investigation](tools/benchmarks/heightmap_performance.md).
For STEP import, mesh triangulation, retained toolpath previews and routing,
see [geometry and interactive performance](docs/performance.md).

Run the tests with `ctest --test-dir build --output-on-failure`.

Choose **Roland MDX-40A** in the **Export flavor** dropdown in the export file
dialog. The selection applies to whole-job and checked-operation exports,
including Ctrl+E. Fit the tool before starting: Roland exports use one tool per
file because the MDX-40A does not support the generic tool-change pause.
For jobs with multiple tools, export checked operations using one tool at a time.
This flavor controls NC syntax; feeds, spindle speed, and clearance still come
from the planned job and its machine setup. Custom machine header/footer,
path-blending, coolant, and tool-length-offset commands are omitted.
Headless export accepts `--export-flavor roland-mdx40a`.

The standalone [marble track generator](tools/README-tracks.md) in
`tools/generate_tracks.py` exports track contours as OBJ, G-code, or STEP surfaces
for CAD applications such as SolidWorks. It accepts continuous DXF profiles and
can cap closed profiles to produce STEP solids.

Jobs support multiple setups with independent machining contexts, shared model
placements, fixture/keep-away geometry, and direct inheritance of remaining
heightmap stock. See [setups and stock inheritance](docs/setups.md), or open
`data/examples/two-setups.json`.

In Setup, **Model orientation** provides ±90° turns around world X, Y, and Z
through the input coordinate origin. Rotate part and stock together (the default),
or select either model separately; Reset orientation restores the selected inputs.
Orientations are saved in the job and apply to OBJ/STL meshes and native STEP
surfaces. Rotation rebuilds box stock, resets work depths and the program origin
from the new bounds, and invalidates toolpaths.

Roughing operations can use ordered XY bounding boxes to finish one part's
roughing and terrace cleanup before proceeding to the next. Add them under
**Cutting > Roughing areas**; see [ordered roughing areas](docs/roughing-areas.md)
or open `data/examples/four-part-roughing.json`.

For roughing, **Advanced > Stock boundary dilation** adds an outward XY planning
margin to the stock outline at each Z level. A value of **1 mm** expands a
20×20 mm square to **22×22 mm**, retaining square corners. It defaults to zero,
is saved with the operation and operation defaults, and also applies to cleanup
subdivisions. The existing **Stock oversize** control adjusts entry headroom
separately. The material-removal simulation uses the loaded stock geometry;
use a stock model or box-stock margins to represent a physically larger blank.
Headless planning accepts `--stock-dilation MM`.
