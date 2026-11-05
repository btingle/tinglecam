# Setups, shared geometry, and remaining stock

A setup is a machining context, with its own model placements, stock input,
fixtures, machine/material selection, origin, and work offset. Operations appear
under their setup in the operation list. Selecting an operation selects its
setup's geometry. **Add operation** inserts into the selected setup.

Use **Add setup** to create another context. It starts with the selected setup's
part resource and references that setup's last operation for remaining stock,
when one exists. Fixtures are not automatically carried forward. The
**Inheritance** section lets you change each choice:

- **Part resource from** shares an earlier setup's imported part. The local
  orientation and translation remain independent.
- **Fixtures from** adds an earlier setup's fixtures to the local fixtures. Use
  **Make inherited fixtures local** to edit their placements independently.
- **Properties from** links machine/material, clearances,
  origin/reference, and work offset as a group. Choosing **Local** retains the
  effective values and makes them editable. Empty machine/material selections
  in a setup resolve to the document defaults.
- **Stock from** chooses new stock or the remaining stock after a named operation
  in an earlier setup. Renaming or reordering operations never retargets this
  reference. A missing or failed source is an error, rather than permission to
  restart from the original blank.

Within a setup, stock passes through operations in execution order. Different
setups have independent histories unless linked explicitly. Upstream edits mark
stock consumers out of date; changing a downstream context does not invalidate
its source. Setup movement preserves operation/result identities and rejects
orders that would make inheritance point forward. Operations can be moved
between setups through their context menu.

## Placement and imported resources

Source imports are immutable and shared by canonical file path and file
revision. Rotation and translation create placed query views. They do not read
the source file again. Mesh acceleration for a new cube orientation is a derived
cache, shared by views with that orientation. Translation queries reuse the
existing acceleration structure. STEP placements share native topology and
retain face identities, so finishing selection is still bound to the source.

The existing quarter-turn controls apply to part, stock, or both. **Part and
stock** also rotates an inherited stock result. Positions are applied in setup
axes after orientation. Work origin remains a separate posting offset. As
before, the rotation controls reset work depths and apply the origin picks;
part-position fields preserve the chosen origin. Stock positioning reapplies
stock-relative origin picks and preserves a part-relative origin.

## Direct heightmap transfer

Stock results retain the double-sided simulation grid. A transfer remaps cells
and upper/lower heights directly; it never converts remaining stock into a mesh
and rasterizes it again. XY-only translations share the existing copy-on-write
storage. Flips and grid-axis swaps remap samples exactly without interpolation.
Downstream cutting cannot modify an upstream snapshot.

The supported transfer rotations preserve or reverse the heightmap's Z column
axis: 180-degree X/Y flips and quarter turns around Z. A 90-degree X/Y transfer
requires conversion and is reported as unsupported. It does not prevent a setup
from using that model orientation with newly defined stock. A transferred grid
keeps its source pitch. The receiving setup's stock pitch control shows the
inherited spacing and is disabled; change the source setup to choose a new pitch.

Roughing projects all remaining stock above each cutting floor, including
material whose underside is above that floor. For example, a ball-mill lip at
the original bottom remains visible to roughing after the stock is flipped.
This also applies when consuming a previous operation's simulated result.
Cross-section displays still show only material crossing their selected plane.

## Fixtures and keep-away geometry

Add mesh or STEP models under **Fixtures / keep-away**, choose the role, and set
placement and clearance. Both roles protect regions from roughing. They are
separate from part geometry and never become finishing propagation surfaces.

Boundary finishing derives its target contours from the part only, clips them
against fixture clearance, and uses the combined protected region for approaches
and links. The safe plane clears fixtures as well as part and stock. XYZ
simulation records commanded motion entering fixture/keep-away clearance without
aborting the operation. The operation warns with the first contact location;
playback turns the tool red during the recorded contact intervals in both the
result and pass views. Contact positions are in setup coordinates and include
the configured clearance. Contact boundaries are localized to within 0.05 mm of
tool travel. Export remains available for the operator to judge the contact.
These checks conservatively protect the cutter's cylindrical footprint; they do
not introduce a separately modeled tool holder.

Keep-away geometry is currently scoped to its setup. It restricts cutting and
does not add material to the stock model. Material actually present beneath a
fixture remains in the resulting heightmap.

Empty roughing terraces are normal. Their "Nothing was left to cut" diagnostics
are available under **Debug** in the messages drawer, or with the command-line
`--profile-planning` option, and are hidden from normal warnings and the status bar.

## Files and export

Legacy job files open as one setup. New files store stable setup and operation
IDs, local contexts, placements, and inheritance references. Source models stay
external; simulation results are recomputed when planning after reopening.

Each operation posts in its setup's origin and work offset. At a setup change,
export retracts in the outgoing context, stops spindle/coolant, optionally issues
M0 (**Pause before this setup**), and selects the incoming work offset. Operations
using different machine profiles must be exported separately. Exporting checked
operations can produce a later setup's program without including its stock-source
operations.

Try the included two-sided example:

```sh
./build/tinglecam --job data/examples/two-setups.json
./build/tinglecam --job data/examples/two-setups.json --gcode /tmp/two-setups.nc
```

The second setup flips the first roughing result and introduces a clamp on the
opposite side. Both operations keep roughing boundary finishing enabled.
