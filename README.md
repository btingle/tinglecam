# TingleCAM

TingleCAM is an experimental 2.5D adaptive-roughing CAM application built on
Clipper2. It slices part and stock meshes, generates wavefront passes, simulates
material removal, assigns feeds from measured engagement, previews the result,
and emits linear G-code.

## Build

Requirements include CMake 3.20+, a C++20 compiler, SDL2, and OpenGL.

```sh
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CUDA mesh slicing is enabled when a suitable toolkit is available. Disable it
explicitly with `-DWAVE_ENABLE_CUDA=OFF`.

The build produces:

- `tinglecam`: the CAM application and headless G-code command.
- `clipper2_wave_demo`: the wavefront geometry diagnostic viewer.

## Planning flow

For every depth, TingleCAM:

1. Slices the part and stock meshes.
2. Offsets the protected part by the cutter radius and stock allowance.
3. Builds and sequences the wavefront passes.
4. Resamples each uninterrupted at-depth path at a uniform traveled-distance
   pitch.
5. Simulates the normalized intervals against the remaining stock.
6. Maps each interval's engagement directly to its configured feed range.
7. Merges adjacent collinear intervals only when their assigned feeds and
   motion types are compatible.
8. Posts the resulting linear motion.

The path pitch defaults to 0.2 mm and is configurable per operation. Spacing is
carried across original Cut, Step, and Link boundaries. Original microscopic
vertices are not retained merely because they were present in offset output.
Retracts, entries, and disconnected paths restart the spacing.

## Engagement feeds

A machine profile supplies a range for each material:

```json
{
  "material": "hdpe",
  "full_engagement": 500.0,
  "no_engagement": 2000.0
}
```

For a simulation engagement `e`, clamped to `[0, 1]`, the commanded feed is:

```text
feed = full_engagement + (no_engagement - full_engagement) * (1 - e)
```

Thus full engagement uses the slower endpoint and zero engagement uses the
faster endpoint. An operation can replace the library range with manually
entered endpoints. Retracts use the machine rapid feed, while vertical entries
use the tool's material-specific plunge feed.

## Configuration

Machines and materials are individual JSON files under `data/`. User tools
are stored under the platform configuration directory and are initialized from
`data/starter-tools.json` on first run. Unknown JSON keys are ignored.

A machine profile contains only values used for feed lookup, spindle clamping,
timing, and G-code emission:

```json
{
  "id": "makera-z1",
  "name": "Makera Z1",
  "controller": "smoothieware",
  "rapid_feed": 2000.0,
  "min_rpm": 6000.0,
  "max_rpm": 15000.0,
  "rpm_resolution": 100.0,
  "spindle_spinup_seconds": 3.0,
  "feed_ranges": [],
  "tool_change": "automatic",
  "tool_change_x": 0.0,
  "tool_change_y": 0.0,
  "length_offset_mode": "none",
  "coolant_flood": false,
  "coolant_mist": true,
  "output_inches": false,
  "decimals": 3,
  "line_numbers": false,
  "line_number_increment": 10,
  "program_header": "",
  "program_footer": "",
  "path_blending": ""
}
```

A material contains only the identity used by the library and UI:

```json
{
  "id": "hdpe",
  "name": "HDPE"
}
```

A tool contains its used geometry, post identifiers, and per-material cutting
data:

```json
{
  "id": "6mm-3f-alu",
  "name": "6 mm aluminium end mill",
  "diameter": 6.0,
  "flute_length": 16.0,
  "center_cutting": true,
  "tool_number": 1,
  "length_offset": 1,
  "cutting_data": [
    {
      "material": "hdpe",
      "spindle_rpm": 15000.0,
      "max_ae_over_d": 0.5,
      "max_ap_over_d": 2.0,
      "plunge_feed": 250.0,
      "coolant": "air-blast"
    }
  ]
}
```

## Command line

```text
--part PATH
--stock PATH
--tool ID
--machine ID
--material ID
--stepover RATIO
--stepdown MM
--path-pitch MM
--full-feed RATE
--no-feed RATE
--gcode PATH
--screenshot PATH
--view pass|result
--at FRACTION
```

Supplying either manual feed endpoint selects manual feeds for the operation.
The graphical interface shows the recalled machine/material range and offers
the same override.

Example:

```sh
./build/tinglecam \
  --part objs/final_obj/L.obj \
  --stock objs/stock_obj/L.obj \
  --machine makera-z1 \
  --material hdpe \
  --path-pitch 0.2 \
  --gcode L.nc
```

## Simulation and safety

The material simulator sweeps the cutter through every normalized Cut, Step,
and Link interval. It records engagement, removed area, remaining stock, gouges,
and animated stock snapshots. Retract destinations are checked against
remaining stock, and unresolved entries are reported.

The headless output always prints all safety counters, including zeros:

- unsafe plunges;
- entries into solid material;
- rapid paths that cross the protected subject;
- unreachable fronts;
- gouged area;
- excess stock left standing.

Machine profiles marked `specifications_unverified` produce both an on-screen
warning and a warning in generated G-code. Generated programs still require
operator review before they are run on a machine.
