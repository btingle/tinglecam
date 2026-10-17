# Marble track generator

`generate_tracks.py` generates the existing nine track templates as OBJ meshes,
G-code with OBJ previews, or mathematically parametric B-spline surfaces in STEP.
Coordinates and exported CAD units are millimetres.

## Installation

Use Python 3.10 or newer. From the repository root:

```sh
python3 -m venv .venv-tracks
. .venv-tracks/bin/activate
python3 -m pip install -r tools/requirements-tracks.txt
```

OBJ input with OBJ/G-code output needs only `numpy` and `numba`. STEP additionally needs `scipy` and
`cadquery-ocp` (the Open CASCADE bindings, without the full CadQuery application).
DXF input needs `ezdxf`, and DXF splines additionally need `scipy`. These optional
dependencies are imported only when used.

## Continuous DXF profiles and the OBJ benchmark

Use `--profile-path` to load a DXF without converting its curves into profile
edges. The older `--profile-obj-path` spelling is still accepted. Supported DXF
geometry includes LINE, ARC, CIRCLE, ELLIPSE, SPLINE, LWPOLYLINE (including bulges),
2D POLYLINE, and block INSERTs containing these entities. Geometry must lie in the
XY plane with +Z extrusion. X is profile width and Y is profile height. Declared
DXF units are converted to millimetres; unitless files are interpreted as mm.

Entities may be unordered or reversed. They must form one unbranched contour;
use `--profile-layer PROFILE` to select it from a larger drawing. TEXT, MTEXT,
DIMENSION, and POINT annotations are ignored. Multiple loops, holes, branches,
unsupported geometry, and out-of-plane entities are rejected. Endpoint gaps up
to `--profile-join-tolerance` (default 0.00001 mm) are snapped with tiny linear
endpoint corrections. These joins do not bridge larger gaps. As with OBJ input,
an open contour gets a closing line unless `--open-path` is supplied; an explicitly
closed contour stays closed even with that flag.

The converted benchmark is [`data/track_profiles/trackprofile.dxf`](../data/track_profiles/trackprofile.dxf).
Its 389 original OBJ vertices reconstruct as **five lines, six circular arcs, and
one ellipse**, with a maximum fitted source-vertex residual of about **0.000002 mm**.
The main channel has ellipse radii of approximately 9 and 8 mm; it is preserved
as one ellipse rather than approximated by several circles. See the adjacent
[`trackprofile.fit.json`](../data/track_profiles/trackprofile.fit.json) report.

```sh
python3 tools/generate_tracks.py --step \
  --profile-path data/track_profiles/trackprofile.dxf \
  --cap-ends --save-dir tracks_step
```

This produces 12 side faces on ordinary tracks or 24 on transition tracks, plus
two cap faces. The fitter evaluates each DXF curve continuously: increasing fit
accuracy adds control points without introducing more faces within that span.
Spline knots that may represent sharp corners are retained as span boundaries.

To reconstruct another ordered OBJ as a DXF:

```sh
python3 tools/convert_track_profile.py objs/trackprofile.obj \
  data/track_profiles/trackprofile.dxf \
  --fit-tolerance 0.00001 --report data/track_profiles/trackprofile.fit.json
```

Reconstruction identifies contiguous lines, circular arcs, and elliptical arcs
without assuming the benchmark's dimensions or symmetry. `--fit-tolerance` limits
source-vertex residuals, not deviation from the short OBJ chords between vertices:
a recovered arc restores the smooth curve represented by those chords. The
benchmark's vertex coordinates are also stored in `tests/fixtures/track_profiles/`
so regression checks do not depend on the ignored `objs/` directory.

## Generate STEP surfaces

```sh
python3 tools/generate_tracks.py --step --save-dir tracks_step
```

For your own profile and a finer fit:

```sh
python3 tools/generate_tracks.py --step \
  --profile-obj-path jig_profile_full_lo.obj --open-path \
  --grid-size 40 --elevation-gain 6 \
  --step-tolerance 0.005 --step-max-samples 257 \
  --save-dir tracks_step
```

Each track gets a separate `.step` file. Uncapped exports open in SolidWorks as surface geometry.
The exported AP214 files contain sewn B-spline faces and explicit millimetre units.
They contain the surface geometry, not native SolidWorks equations, dimensions,
or a feature history. Track ends remain uncapped by default. Add **`--cap-ends`**
(or `--cap`) for a closed profile to produce a STEP solid. Caps reuse the exact
fitted boundary curves, then the closed shell is oriented and checked as a solid.
The same flag produces a welded, triangulated, capped OBJ mesh, including concave
profiles. Caps are rejected for open profiles and G-code output.

The STEP fitter samples the track equations directly, before mesh resampling.
Each OBJ edge or continuous DXF span gets its own patch, preserving corners. Transitions
are split at their straight/curve joins, accounting for the join's dependence on
profile width. The original generator's Z clamps are applied, including their
small creases on some descending tracks.

The fit starts with five samples along each patch axis and bisects intervals
that fail the deviation check. Adjacent patches use matching knot vectors along
their common edges so those edges coincide. The check compares Euclidean distances
against the equations at sample nodes, boundaries, and three interior positions per
interval in both axes. `--step-tolerance` defaults to **0.01 mm**. The printed
deviation is a sampled measurement, not a proven continuous maximum. Smooth
formulas fit compactly; clamped regions may need many more samples.

`--step-max-samples` caps each axis at 129 samples by default. If the fit cannot
meet the sampled tolerance, export stops with an error; increase the limit or
relax the tolerance. The exporter also checks face construction, shell validity,
and that the patches sew into one connected surface shell. Profiles that reach
or cross a bend axis are rejected: increase grid size or reduce profile width.
The generator retains its existing template names and geometry, including `L`
being flat and `R` being elevated.

## OBJ and G-code

Existing commands still work:

```sh
python3 tools/generate_tracks.py --save-dir tracks_obj --resolution 256
python3 tools/generate_tracks.py --gcode --save-dir tracks_gcode --translate=0,0,12
```

`--resolution` controls longitudinal sampling for OBJ/G-code only. Their profile
sampling uses 0.25 mm arc-length spacing and retains all original corners and the
final endpoint. `--translate` applies to G-code and its preview only. `--gcode`
and `--step` are mutually exclusive; omit both to produce OBJ meshes.

OBJ profile files must list vertices in contour order. The first two coordinates
of each `v` record become profile width and height; OBJ line/face connectivity is
ignored. Texture and normal records are ignored. Consecutive duplicate vertices
are removed. By default, the profile closes once; `--open-path` leaves it open
through both resampling and export. The built-in profile is a 24 by 12 mm rectangle.

## Code layout and validation

- `generate_tracks.py`: track equations, template roster, contour sampling, CLI.
- `track_profiles.py`: profile loading, closure, and arc-length resampling.
- `track_curves.py`: continuous line/ellipse/spline spans and curve sampling.
- `track_dxf.py`: DXF import, contour ordering, units, and reconstruction output.
- `convert_track_profile.py`: automatic OBJ line/arc/ellipse reconstruction.
- `track_exports.py`: OBJ meshes, G-code, and OBJ toolpath previews.
- `track_step.py`: adaptive B-spline fitting, sewing, validity checks, STEP export.

Run the Python checks independently of the CAM application's C++ tests:

```sh
python3 -m unittest discover -s tests -p 'test_generate_tracks.py'
python3 -m unittest discover -s tests -p 'test_track_dxf.py'
```

With STEP dependencies installed, these checks write and reimport all nine track
types with open and closed profiles, check units, face counts, free edges and
validity, and compare reimported surfaces to independent equation samples.
The DXF checks reconstruct and reimport the benchmark, check curve residuals and
entity counts, and verify all nine capped STEP solids have positive volume and
no free edges. They also exercise DXF units, polyline bulges, splines, and concave
OBJ cap triangulation.
