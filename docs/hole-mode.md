# Holes and counterbores

Add a **Hole** operation from the operations menu. The existing **Pocket** mode
and its general convex-pocket identification remain available.

## Manual entry and discovery

Manual holes have an XY centre and one or more levels, each with a radius,
top Z, bottom Z and through-hole flag. Add deeper levels for nested counterbores.
Coordinates appear in the job's program frame, like other operation settings.

For discovery, load the part, enable **Identify automatically**, and press
**Identify holes**. This runs independently of planning and needs no tool or
stock selection. It reports progress and can be cancelled.

**Inspection depth** is measured down from the part's top. The default, zero,
inspects just inside the top surface. **Circle tolerance** is the maximum radial
error over the complete fitted boundary, including the interiors of polygon
edges. Discovery slices the imported source geometry, independently of the
simulation grid. STEP discovery currently uses its imported triangulation, so
mesh tessellation and the slicer's 0.001 mm coordinate resolution affect the
smallest useful tolerance.

When **Find counterbores** is enabled, twelve probes just inside the current rim
look for a shoulder. A localized slice below that shoulder finds a smaller,
concentric circular opening, then repeats for further levels. Probe disagreement,
tapers, chamfers and ambiguous branches are reported rather than silently
inventing levels. All probe misses terminate a through hole. Its nominal bottom
is the part's lower Z bound; this extent can be edited before planning. Disable
counterbore discovery to retain just the initial opening and its depth.

Use the system/level checkboxes or click a rim in the Setup view to choose cuts.
Each level can override the default strategy and tool. Re-identification preserves
matching selections and overrides by geometry. New unmatched features are left
unselected after an earlier identification. Editing source geometry or inspection
settings requires identification again; editing machining settings does not.

## Cutting strategies

- **Drilling:** use a drill matching the selected bore diameter. Supports a
  single feed descent or peck drilling, retract clearance, through-hole
  breakthrough, and an independent depth adjustment.
- **Helical finishing:** follows the finished wall downwards, then completes a
  circle at the floor. Helix angle and milling stepdown limit the descent per
  turn. This is a wall-finishing path; it intentionally imposes no constraint on
  a central core or on previously cleared stock.
- **Pocket clearing:** clears each depth layer with a helical entry and offset
  contours, retaining the existing pocket planner's stay-down layer transitions.
- **Adaptive by radius:** chooses pocket clearing above the cutoff radius and
  helical finishing at or below it, independently for every counterbore level.

The tool comes from the operation unless a level overrides it. Ready levels
using the same tool are batched across systems while retaining each system's
shallow-to-deep dependencies. Simulation, playback, feeds, tool numbering and
G-code use the actual tool at each pass. Arcs remain G2/G3; drilling expands into
ordinary G0/G1 moves, including pecks, without requiring controller canned cycles.

## Drill geometry and pre-drilling

The tool editor's **Drill** type has an included **Point angle** (118 degrees by
default) and optional **Tip flat diameter**. A 180-degree point is flat. The
conical/frustum tip is represented in stock simulation and cutter previews.

**Drilling depth** chooses between:

- **Tip reaches bottom Z:** the tip stops at the requested floor.
- **Full diameter reaches bottom Z:** the tip extends below the requested floor
  by its point length, `(diameter - tip_flat_diameter) / (2*tan(point_angle/2))`.

A positive **Depth adjustment** extends deeper; a negative value ends shallower.
Through-hole breakthrough adds to that depth only for levels marked through.

**Insert pre-drilling operation** creates an editable drilling operation immediately
before the milling operation. It inherits the selected milling systems and drills
once per system to the deepest selected milling level. Its tool, feeds, depth
reference/adjustment, pecking and breakthrough are independent. Selection changes
in the source update the linked operation and invalidate its previous plan. All
pilot holes execute before changing tools for milling. Keep this operation above
its source; a missing or reversed link is reported during planning.

Open `data/examples/holes-and-counterbores.json` for a manual example with two
nested counterbores and an adaptive strategy. No model is required for that example.
