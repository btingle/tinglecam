# FreeCAD adaptive roughing

Roughing uses the vendored FreeCAD Adaptive2d solver for each XY layer. No
FreeCAD installation is needed. The existing coarse descent, depth breaks,
upward terrace subdivisions, stock handoff, simulation, feed assignment, and
simulation-based inter-layer router remain in place.

The engine receives each layer's physical stock and protected model outline,
including allowances and fixtures. Previously removed material and air are
supplied as cleared regions. Roughing boxes bound the cutter centre, as before.
Identical layer inputs reuse a solve; each level owns its own mutable path.

FreeCAD cutting moves become cuts, clear links stay at depth, and uncleared
links retract to the existing safe plane. Landing approaches reserve the
heightmap's cell-diagonal clearance; a short cleared link connects a displaced
landing to FreeCAD's start point. If no conservative rapid landing is available,
the adapter retains FreeCAD's feed-descent convention. Helix entries retain the centre and
radius chosen by FreeCAD. TingleCAM generates counterclockwise XYZ helices with
a 3 degree maximum ramp angle and pitch no greater than the layer's axial depth,
followed by a full circle at the floor. Simulation checks the actual helix and
continues to assign feeds. Resampling retains the adaptive solver's vertices
so it cannot shortcut curved cuts across a protected boundary. Unreachable or
incomplete FreeCAD regions are
reported. The existing boundary-cleanup toggle controls FreeCAD's finishing
profile; TingleCAM does not append its old roughing contours.

FreeCAD's face-selection UI and its CAD-to-polygon preparation are separate
from Adaptive2d and are not imported. This integration uses TingleCAM's existing
slice and area selection. FreeCAD's profiling modes, conical helices and extra
helix controls are not exposed here. Legacy wavefront visualization data is
empty for adaptive roughing; the standalone wavefront demo and contour-finishing
code still use the wave engine.

See [upstream provenance and license](../third_party/freecad_adaptive/README.md).

Legacy wave-specific entry, split-area and cleanup-tolerance fields remain
readable in saved jobs but do not control adaptive roughing. Their UI controls
are hidden. Stock dilation, allowances, stepovers and depth controls still apply.
