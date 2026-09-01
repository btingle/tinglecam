# TingleCAM

TingleCAM is an experimental 2.5D CAM application built on Clipper2. It supports
wavefront roughing and offset-contour finishing, simulates material removal,
assigns feeds from measured engagement, previews the result, and emits linear
G-code.

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

## Workflow

Which entry fronts the wave may start from is an input to planning, not a stage
of it. Finding them costs a seeding pass and no propagation, so a choice can be
made and unmade without planning anything.

1. **Plan** shows the part and stock models so the depth range can be set
   against them.
2. **Select fronts** draws each entry front where it actually is and lets any of
   them be declined. Nothing is planned here: the fronts are generation zero of
   the wave, which is built before the first propagation step, so this asks the
   models for their slices and stops. Slices are cached, so it is quick after
   the first look.
3. Planning the job runs the wave without the declined entries, then builds the
   toolpaths, simulation and feeds. **Pass** and **Stacked result** show what
   came out.

The list does not move when the choice does. Declining an entry drops its own
seed piece before propagation begins and leaves every other piece exactly where
it was, so the fronts on offer are the same fronts whatever has been declined --
which is what lets a front be declined and reinstated for the price of a click,
and lets a front be declined that a plan would have failed on.

The choice is recorded on the operation as one group of anchor points per
declined entry, a point on each layer it appeared at. Anchors are matched
geometrically rather than by index, so a selection survives a change of stepdown
or stepover that renumbers everything, and it is saved with the job.

Material only a declined entry could have reached is left standing, and the plan
says how many entries were declined rather than hiding it.

## Stock

With no stock model, the blank is the part's bounding box grown in X and Y by
`box_stock_margin` (5 mm by default). Z is left at the part's own range: a blank
oversize in Z means facing the top off, which is a pass in its own right rather
than something a fallback should decide to add. The blank is one rectangle at
every depth.

A named stock model always wins. Clearing the model field and reloading drops
it and hands the job back to the box, so a blank the job no longer names cannot
go on standing in for one.

## Planning flow

For every roughing depth, TingleCAM:

1. Slices the part and stock meshes.
2. Offsets the protected part by the cutter radius and stock allowance.
3. Builds the wavefront passes, endpoint boundary provenance, per-layer nodes,
   and inter-layer associations.
4. Sequences the wavefront passes and adds a boundary cleanup contour, clipped
   to what the wave reached.
5. Resamples each uninterrupted at-depth path at a uniform traveled-distance
   pitch.
6. Simulates the normalized intervals against the remaining stock.
7. Maps each interval's engagement directly to its configured feed range.
8. Merges adjacent collinear intervals only when their assigned feeds and
   motion types are compatible.
9. Posts the resulting linear motion.

The cleanup contour is derived from the part slice rather than from the wave,
so on its own it is emitted wherever the part has a boundary -- including
depths the wave never reached, whether because the only entry into them was
declined or because nothing can get in at all. Cutting there is a finishing
pass through solid stock that nothing has roughed, so the contour is clipped to
the region the wave actually swept. A depth the wave could not start at emits
nothing. The contour lies exactly on that region's inner boundary, both being
the same slice offset by the same distance, so the clip allows `boundary_slack`
-- the same rounding allowance a front's boundary contact is accepted with.

Lead-in wavefronts that remain wholly in air are retained in the lineage graph
to establish entry ordering, but are not emitted as machining contours. On
each lead-in branch, the first front whose cutter sweep touches stock is also
used only as entry geometry; cutting begins at its child generation (+1).

Planning failures retain their phase, depth, operation name, and original
exception text. The full diagnostic is shown in the application banner and
written to the launching console; a partially usable multi-operation job keeps
failed operations as warnings instead of hiding the successful ones.

Contour finishing is deliberately split into two independent pieces. Geometry
derivation offsets the existing slice contours and optionally clips them to the
stock, producing unordered open or closed curves. The general traversal planner
then selects contour starts/directions and orders them. The same traversal API
accepts node endpoints and other path-like tasks; it is not contour-specific.
Each finishing variant includes a checked straight approach from a landing at
least one cutter radius outside the stock. A variant with no approach that stays
outside the protected cutter-centre region is rejected.

Disconnected routes use one safe plane at program `Z0 + rapid_clearance_height`
(10 mm by default). Retract, XY rapid, and descent distance all contribute to
the traversal cost. A legal already-cleared stay-down connection may be used
instead. Cutter-clear stock approaches are stored separately from contour
geometry and are used only for a safe-plane entry, so stay-down cleanup does
not revisit a shared landing point between contours.

## Inter-layer analysis

Every wavefront endpoint retains its connected machinable-boundary component,
arclength position, and classification error. Across a `1 -> 1` lineage edge,
the fronts belong to the same node. A split front remains upstream, a merge
front remains downstream, and a many-parent/many-child front is a singleton.
Known endpoint contacts changing boundary components without a split or merge
are treated as an internal topology error.

Node footprints are matched between adjacent Z layers by exact containment:
the lower footprint minus the upper footprint must be empty, and the nodes must
have the same evaluation order. No machining tolerance is applied. Entry fronts
are the connected generation-zero propagation sources, rather than fragments
obtained by subtracting the subject from the stock wall. They are matched
separately: the complete lower curve must fit inside the configurable
`entry_front_match_tolerance` envelope around the upper curve (3 mm by default).
Both matches must be uniquely one-to-one or the corresponding Z-chain stops.

A stable open node is bounded explicitly by its first and last fronts and the
two endpoint rails between them. If a run attaches to or detaches from a static
boundary (open and closed fronts mix), its connected swept-band boundary is the
closure proof instead; no endpoints are invented across that transition. A
non-terminal singleton receives a local half-pitch footprint. Empty or
disconnected node boundaries are topology errors.

The plan view's **Debug view** section can draw node boundaries either with a
distinct colour per node or with stable Z-group colours. Entry-front Z-groups
and classified boundary contacts are available there as separate overlays.
Node areas use the same fill path as simulated stock, with an opaque darkened
version of the node colour beneath a bright boundary outline.

The stacked result follows playback by default and draws the active cutter as
a translucent flute-length cylinder at its commanded XYZ position. Retracts,
clearance-plane traversals, verified clear descents, unresolved plunges, and
the final lift are animated at their configured machine rates while planar
stock progress pauses. A descent following a collision-free `Retract` uses
machine rapid; only an unresolved entry retains the tool's plunge feed.
Turning **Stock follows the cutter** off shows the completed result instead.

Each pass but the first opens with the rapid that carries the cutter from where
the previous pass lifted off to where this one starts, on the clearance plane.
It is a phase of the pass's own commanded-motion schedule, so it is timed at the
machine rapid, counted in the travel totals, and animated like any other move
rather than being a jump between passes. It covers no planar path distance, in
the same way the descent that follows it does. G-code was always emitting this
move; what it did not have was a plan that owned it.

The **Select fronts** view draws the stock as one plain solid and each entry
front as a coloured ribbon along the curve it actually occupies, one per pass,
stacking down its Z chain. A front is a stretch of wall the wave starts from,
not a volume, so it is drawn on the surface: giving it thickness would put faces
inside the stock for the depth sort to fight over and would claim something
about the material that a front does not say. Where a lead-in stands a front off
the stock, its ribbon stands off with it -- the same place the pass view's
entry-front overlay draws that curve. Nothing quantifies what an entry reaches:
reaching is propagation, and this view deliberately does not propagate.

The **Plan** view shows the loaded meshes themselves, cycling between stock,
part, and both with the button in its corner. Nothing here is a plan, so nothing
here looks at the models through a slicer: sampling them at a fixed set of
depths put a staircase on every curved face and littered the flat ones with the
hairline slivers two independently decimated slices differ by. With both shown
the stock is drawn see-through over the part. The two solids simply overlap;
cutting a part-shaped hole in the stock instead gave every wall of the part an
exact coincident twin on the hole's inside for the depth sort to pick between.

The stacked view is framed on the whole operation -- every pass's stock, every
pass's commanded position, and room for the flute -- taken once from the plan.
Nothing that moves contributes, so the model stays put while the cutter runs
through it.

The analysis also records a section-first node schedule. Execution remains
layer-based until node-sized path fragments can preserve their simulation and
safe-entry state when interleaved; the planner does not silently reorder those
moves using stale engagement data.

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
--mode rough|finish
--box-stock MM
--models stock|part|both
--contour-direction either|climb|conventional
--entry-front-tolerance MM
--rapid-clearance MM
--full-feed RATE
--no-feed RATE
--gcode PATH
--screenshot PATH
--view models|fronts|pass|result
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
