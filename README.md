# TingleCAM

TingleCAM is an experimental 2.5D CAM application built on Clipper2. It supports
wavefront roughing, offset-contour finishing, parametric pockets and plane clearing,
simulates material removal,
assigns feeds from measured engagement, previews the result, and emits linear
G-code.

## Build

Requirements include CMake 3.20+, a C++20 compiler, SDL2, OpenGL, a CUDA-capable
GPU, and the CUDA toolkit. Mesh preprocessing always uses the cuBQL CUDA BVH;
there is no CPU fallback.

```sh
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The build produces:

- `tinglecam`: the CAM application and headless G-code command.
- `clipper2_wave_demo`: the wavefront geometry diagnostic viewer.

## Wavefront analysis viewer

`clipper2_wave_demo` is intentionally limited to wave propagation and its
topology; it does not build toolpaths or run material-removal simulation. It
records the recursive entry-selection analysis and lets the result be replayed
one decision frame at a time:

- the simultaneous baseline for a node layer;
- the trial retaining only the longest entry in each graph-connected remaining
  volume; and
- the accepted strategy. Nodes below 0.5% of the incumbent strategy's largest
  approximate node area remain in the result but do not affect selection;
  strictly fewer significant nodes wins, and ties retain the baseline.

The canvas can overlay generations, leading-front parentage, node boundaries,
and split, merge, termination, or merge-termination events. During decision
playback, entries in the same graph-derived volume share a colour; retained
entries are thick, while muted entries are grey and crossed out. A dashed blue
outline shows the cached remaining domain used by that layer.

Before nodes are built, a split is rejected when every extra branch terminates
without another split or merge and covers less than half a square stepover.
The fronts and swept bands remain in the result; only the below-resolution
split event and resulting node boundary are omitted. The viewer exposes this
as **Tiny split area (stepover^2)**, and zero disables the filter.

Run it interactively with `./build/clipper2_wave_demo`, or inspect the same
recorded decisions without a window:

```sh
./build/clipper2_wave_demo --dump-analysis
./build/clipper2_wave_demo --analysis-step 4
```

The viewer uses rounded wave offsets by default. Disable **Rounded waves** in
the UI, or pass `--mitered-waves`, to use the faster sharp-mitered geometry for
topology diagnostics. `--round-waves` selects the normal rounded mode
explicitly.

Interactive startup prepares only the subject/stock preview. Geometry options
are collected without triggering analysis on every slider tick; press **Build
analysis** (or **Rebuild analysis**) when the inputs are ready. Analysis dumps,
requested decision frames, and screenshots still build automatically because
their output requires a completed analysis.

When no stock mesh is loaded, the diagnostic rectangle is derived from the
full subject model's XY bounds plus the configured allowance. It remains fixed
while Z changes; it is never refitted to an individual slice.

## Workflow

The workspace has three columns. The left column keeps program-wide Job / Origin
and Libraries controls above an always-visible ordered operation list. The
centre contains the model and result previews. The right column contains the
selected operation's settings and playback/display controls. Selecting an
operation on the left selects its settings, planned passes, and preview together.
Roughing and contour-finishing operations can be added explicitly, duplicated,
removed, and moved up or down.

1. **Plan** shows the part and stock models so the depth range can be set
   against them. The range is stated as depths below the top of the work -- the
   taller of the part and the blank -- and defaults to the whole of it. Measured
   that way rather than as a pair of model Z heights, a blank standing proud of
   its part is faced off instead of left standing, and where the origin sits
   never enters into what gets cut. It also carries the origin picker: a place
   on the blank per axis, applied on a button so a stray click cannot silently
   move where the program thinks the work is, and applied once as a model loads
   so the origin starts where the picks say rather than wherever the mesh
   carries its own zero. The three figures beside it are the origin measured from
   the bottom-back-left corner of the blank, in the model's own millimetres,
   because that is the corner an operator can find with an edge finder. They can
   be typed into directly. What the job carries is the other half of the same
   statement -- the offset added to every posted coordinate, which is the
   negative of the point picked -- and no part of planning ever sees this side
   of it, so moving the origin does not invalidate a plan. Posted coordinates
   move with it, comments included.
2. Planning the job builds each operation in list order, then produces its
   toolpaths, simulation, and feeds. **Pass** and **Stacked result** show what
   came out.

Operations run in list order. Each simulated level begins with the nominal stock
minus the swept material from every successful earlier operation whose axial
slab crosses that level. Consequently engagement, feeds, plunge checks, and
playback for a finishing operation see the stock left by roughing instead of a
fresh copy of the blank. Toolpath geometry is still derived from nominal slices;
clipping finishing paths to the inherited stock is a separate future step.

A roughing operation's **Finishing subdivisions** setting repeats its complete
depth plan at evenly interleaved offsets, handing each repetition the stock left
by the one before it. For example, a 1 mm stepdown with four subdivisions runs
the base floors first, followed by groups beginning 0.75, 0.5, and 0.25 mm below
the work top. Each group retains its own base-layer stock record. The stacked
preview interleaves all of those records into physical Z order; layers belonging
to a group that has not started yet temporarily mirror the active group's stock
at the corresponding depth, so playback updates only one simulated layer.

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
3. Builds the simultaneous wavefront baseline and recursively tests each
   graph-connected entry group. A longest-entry trial is committed only when
   it produces fewer topology nodes.
4. Extracts only the root nodes committed at each recursive layer and sequences
   those stages with one continuous cutter position and cleared-state model.
   The committed stages are also composed into the final per-depth lineage and
   node topology used by node highlighting, entry-front display, cleanup
   clipping, and inter-layer Z association; the discarded simultaneous suffix
   is not exposed as the planned result.
5. Adds a boundary cleanup contour, clipped
   to what the wave reached.
6. Stitches the seams between depths, once every depth has a toolpath, so a
   transition can be compared against the lift it replaces.
7. Resamples each uninterrupted at-depth path at a uniform traveled-distance
   pitch.
8. Simulates the normalized intervals against the remaining stock.
9. Maps each interval's engagement directly to its configured feed range.
10. Merges adjacent collinear intervals only when their assigned feeds and
   motion types are compatible.
11. Posts the resulting linear motion.

A depth the part does not reach is cleared rather than skipped, in either mode.
There is no surface to finish there because there is no part there, but there is
stock, and all of it is waste: leaving it standing would leave every pass below
it cutting into a blank that is still the wrong shape. Such a depth gets the
same wave a roughing pass would use, propagated against a subject parked outside
the work so the whole level clears away.

The cleanup contour is derived from the part slice rather than from the wave.
Portions whose cutter centre lies inside stock are accepted only where the wave
actually reached, with `boundary_slack` allowing for independently rounded
geometry. On later finishing subdivisions the planner also allows the cutter
centre into the exterior air beside remaining stock by the cutter radius minus
the roughing operation's cleanup tolerance. The tolerance defaults to 0.15 mm
to exclude small numerical remnants while still admitting meaningful edge
stock. This exterior region is part of the roughing wave's centreline domain as
well as the cleanup contour's eligibility region, while the simulation retains
the uninflated physical stock. Setting the tolerance to the cutter radius
retains the strict stock-only wave rule. This lets a subdivision rough and clean
accessible edge remnants without authorizing motion through the protected part
or treating nearby air as material.

Lead-in wavefronts that remain wholly in air are retained in the lineage graph
to establish entry ordering, but are not emitted as machining contours. Behind
them, cutting begins at the first front whose cutter sweep reaches the stock:
the run-in is sized at a radius plus the stock allowance precisely so that front
is the one taking the designed first bite of a stepover less the allowance.

The seed ring is the exception. It is a one-unit ring on the edge of the
approach region rather than a pass, and where the subject notches that region it
runs along the stock wall itself, so it would cut at very nearly a full slot.
With no lead-in there is no approach and the seed is the stock wall, which is
the pass the operator asked for by turning the run-in off.

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

Disconnected routes use one safe plane, resolved once by `job_rapid_plane`,
recorded on the plan, and used by the planner, the feed schedule and the post
alike -- a planner costing lifts against a height the post then disagrees with
would undercharge every one of them.

It is `rapid_clearance_height` above the tallest thing in the way, part or
blank, and not above the origin. Where the origin sits is a matter of where it
is convenient to measure from, and it moves; what a traverse has to clear does
not. Measured from the origin, moving the origin moved the plane, which moved
what every lift cost, which meant re-planning a job to say where its zero was.
The height is over the work, so the plan is the same motion whichever corner
the numbers are measured from, and only the post ever sees the difference.

Retract, XY rapid, and descent distance all contribute to the traversal cost,
each at the rate of the axis that moves it. `rapid_feed_z` is the Z rate;
omitted or zero, the lateral `rapid_feed` covers both. No resolution between
the two is needed, because no diagonal rapid is ever written: a lift rises
where it stands, crosses at the plane, and descends, so every rapid moves one
axis or the other. A slower Z makes lifting dearer and tips more transitions
into staying down.

A legal already-cleared stay-down connection may be used instead. Cutter-clear
stock approaches are stored separately from contour geometry and are used only
for a safe-plane entry, so stay-down cleanup does not revisit a shared landing
point between contours.

What a connector may travel through is everywhere the cutter may be at that
depth: the space beside the stock, plus whatever the layer has had removed from
it by the time the connector is planned. For roughing cleanup that is the wave's
own cleared region, which includes that space -- the reached region alone stops
at the stock wall, and a cleanup contour ending on that wall could not then be
joined to the next one round the outside. For contour finishing nothing has been
removed yet, and the space beside the stock is the whole of it, which is enough:
contours clipped to the stock begin and end on its wall, and two of them can
finish and start a few millimetres apart along it.

Where the straight segment is blocked, the route around it is the Euclidean
shortest path through the region the tool may travel in, found by Dijkstra over
a visibility graph of that region's boundary (`src/routing.h`). Following the
boundary instead answers a different question: getting around an obstacle and
getting past it are not the same, and a rib standing between two points 40 mm
apart cost a walk of the whole part outline, which lost to lifting every time.
The same router serves the wavefront sequencer, the generic traversal planner
and the seam between depths.

Everywhere a route is asked for, the region is eroded first by the slack the
router may spend. That slack exists to forgive two separately rounded
descriptions of one wall disagreeing, which is what lets a route run along that
wall; spent instead against the part, or against the radial allowance standing
in front of it, it is a cut into the finished surface. The straight-line test is
not eroded -- it demands the whole segment inside the region as given.

Between one depth and the next, the cutter lifts only when lifting is quicker.
Where the pass that just finished cleared a way across at its own floor, the
transition is routed through it and the tool steps down a single pass depth
instead of making the full trip to the plane and back. That way across belongs
to the earlier pass -- it runs at that pass's depth, through what that pass
removed -- so it extends that pass's toolpath and is simulated with it rather
than being motion between passes that nothing checks. The region is eroded by
the router's own allowance first: that slack exists to forgive two roundings of
one wall disagreeing, and spent against the part it would be a cut into the
finished surface.

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

Every view takes the same three gestures: left-drag orbits (the flat pass view,
having nothing to orbit, ignores it), right-drag pans, and the wheel zooms. Pan
is in pixels and is not divided by the zoom, so a drag moves what is under the
pointer by the distance the pointer moved; the wheel scales the pan with it, so
zooming after panning scales the picture about the middle of the canvas instead
of about a centre that panning has pushed off the edge. **Fit view** also
recentres the 3D views, which frame themselves but can still be panned into a
corner.

The stacked result follows playback by default and draws the active cutter as
a translucent flute-length cylinder at its commanded XYZ position. Retracts,
clearance-plane traversals, verified clear descents, unresolved plunges, and
the final lift are animated at their configured machine rates while planar
stock progress pauses. A descent following a collision-free `Retract` uses
machine rapid; only an unresolved entry retains the tool's plunge feed.
Turning **Stock follows the cutter** off shows the completed result instead.

A button in its corner cycles the loaded models over the stack -- none, part,
blank, both -- so where the passes still stand proud of the finished shape can
be read directly rather than inferred. It is an overlay: the models are sorted
among themselves and drawn after the stack, not sorted into it. A simulated
pass wall and the part surface it was cut to occupy the same place to within a
rounding, so a shared sort has no real answer for which is in front and decides
per facet on centroid depth; the two then interleave and the overlay speckles
and crawls as the view turns. Nothing the overlay is for needs the model
occluded by the stack, so it goes on top, and being see-through is what lets
the stack be read through it. The part is drawn warm where every pass is cool:
the plan view's blue would be the consistent choice and is the wrong one, being
within a shade of the colour the passes already carry, and telling the two
apart is the entire question the overlay answers.

Each pass but the first opens with the rapid that carries the cutter from where
the previous pass lifted off to where this one starts, on the clearance plane.
It is a phase of the pass's own commanded-motion schedule, so it is timed at the
machine rapid, counted in the travel totals, and animated like any other move
rather than being a jump between passes. It covers no planar path distance, in
the same way the descent that follows it does. G-code was always emitting this
move; what it did not have was a plan that owned it.

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

The path pitch defaults to 1 mm and is configurable per operation. Spacing is
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
use the operation's plunge/ramp feed.

## Configuration

Machines and materials are individual JSON files under `data/`. User tools
are stored under the platform configuration directory and are initialized from
`data/starter-tools.json` on first run. Unknown JSON keys are ignored.

The **Libraries** tab shows the exact active `tools.json` path (on Linux,
`~/.config/tinglecam/tools.json`, or `$XDG_CONFIG_HOME/tinglecam/tools.json`).
Use **New tool**, enter diameter, flute length, flute count and type, then
**Save tool**. The identifier is generated automatically; **Edit**
keeps it and **Copy** creates a new one. Changes invalidate the existing plan.

For JSON edits, edit the active file and click **Reload from disk**, or restart
TingleCAM. Editing `data/starter-tools.json` does not change an existing user
library automatically: use **Import tools** in the Libraries tab to apply it.
The import path defaults to that starter file and can point to another library
JSON. Imports add new IDs and replace matching IDs while keeping other tools.
Invalid imports leave the current library intact. Close the tool builder and
wait for planning to finish before reloading or importing.

Tools contain geometry only. Stepover, stepdown, spindle RPM, plunge/ramp feed,
and coolant belong to each operation. Tool/material engagement caps have been
removed. Machine RPM limits still apply. Pocket and plane operations reject a
requested total depth beyond flute length instead of silently changing it.
Old tool JSON files still load: retired fields (including `cutting_data`) are
ignored and disappear on the next save. Missing flute counts default to two;
missing type defaults to flat (the old ball starter is recognized by its ID).
Tool numbers and length offsets can be set per operation; zero uses the tool's
1-based library position and matching H number.
Warnings wrap in the GUI, can be copied with **Copy warnings**, and are also
printed to the launching terminal when planning or G-code generation completes.

A machine profile contains only values used for feed lookup, spindle clamping,
timing, and G-code emission:

```json
{
  "id": "makera-z1",
  "name": "Makera Z1",
  "controller": "smoothieware",
  "rapid_feed": 2000.0,
  "rapid_feed_z": 800.0,
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

A tool has a name/ID and four physical attributes:

```json
{
  "id": "6mm-3f-alu",
  "name": "6 mm aluminium end mill",
  "diameter": 6.0,
  "flute_length": 16.0,
  "flutes": 3,
  "type": "flat"
}
```

Supported type values are `flat`, `ball`, `drill`, and `v-bit`. The new flat-floor
pocket and plane planners require `flat`; the other types remain available in
the library. Flute count is stored as physical metadata and does not impose a
feed formula.

## Pockets and plane clearing

Use **Add pocket** or **Add plane clearing** in the operation list, or change an
existing operation's **Mode**. Remove the initial roughing operation if the job
contains only dimension-driven operations: these need no part or stock mesh.
Both accept operation stepover (fraction of diameter), stepdown in mm, and the
same spindle/feed controls as the mesh operations. All coordinates are in the
job frame, with the job origin offset applied once when posting.

A pocket can be circular, rectangular (equal width/height makes a square), or a
rounded rectangle. Enter center X/Y, top Z, depth, and width/height or radius.
For circles, radius defines the opening; for rounded rectangles it defines the
corner radius. Z is the top surface, and the floor is `top_z - depth`.
Each stepdown starts with a gradual helix, followed by a full turn at the floor
and outward clearing rings. Helix angle defaults to 3 degrees and ramp speed
comes from the operation. The post emits simultaneous XYZ linear segments for
the helix, and playback follows that same 3D timeline. Pockets too small for a
helix, or rounded corners smaller than the cutter radius, are rejected. Square
internal corners retain the cutter radius.

Plane clearing takes an editable list of rectangles (center X/Y, width, height)
and shared top Z and depth. Add/remove as many areas as needed. At each stepdown
it clears each rectangle using alternating straight rows and an edge cleanup,
then retracts to the job clearance plane before crossing to another area. Entry
uses the operation's plunge feed. The cutter stays inside each rectangle, so
corners retain its radius; areas must accommodate the cutter. Overlapping areas
share simulated stock. With loaded or box stock, pocket and plane passes cut
only the active stock remaining after earlier operations. An area outside that
stock produces no cutting motion. Without stock, standalone dimension-only
jobs continue to use their specified regions as the starting material. These
operations do not protect loaded part surfaces automatically.

The stacked pocket/plane preview is cumulative. Earlier stock remains visible
outside the footprint and above/below the operation's depth range. Local slabs
retain all crossed layer boundaries, even when one new stepdown spans several
roughing layers. Boundary splits share the original polygon data. Prior-stock
clipping is cached; playback removes material locally and leaves future layers
intact. Overlapping areas apply their removals in execution order.

Simulation remains a 2D slab approximation: the floor stock during a helical
entry is conservatively left intact until the full turn at the floor. The
motion preview, posted Z coordinates, and time estimate use the actual helix.

**Job / Origin → Save job / Load job** preserves the complete ordered operation
list and all area dimensions. The same file can be used with `--job`:

```sh
./build/tinglecam --job data/examples/pockets-and-planes.json
./build/tinglecam --job data/examples/pockets-and-planes.json --gcode /tmp/pockets.nc
```

## Command line

```text
--job PATH
--part PATH
--stock PATH
--tool ID
--machine ID
--material ID
--stepover RATIO
--stepdown MM
--finishing-subdivisions N
--path-pitch MM
--mode rough|finish|pocket|plane
--box-stock MM
--models stock|part|both
--contour-direction either|climb|conventional
--entry-front-tolerance MM
--rapid-clearance MM
--full-feed RATE
--no-feed RATE
--gcode PATH
--profile-planning
--screenshot PATH
--view models|pass|result
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

`--profile-planning` prints machine-readable per-phase wall times and scale
counters to stderr during a headless G-code run. The repository benchmark runs
the regression suite and a representative planning corpus:

```sh
./benchmark_planning.sh build 3
```

The optional third argument names an output directory for profiles, posted
G-code, hashes, and resource measurements. Otherwise a new directory is made
under `/tmp` and reported at startup.

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
