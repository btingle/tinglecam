# Independent stock islands

At each roughing depth, disconnected regions of incoming stock propagate
independently. The planner separates the physical stock before adding the
lead-in and exterior cutter-reach regions. Overlapping approach regions in air
do not join two stock islands or remove their inward-facing wave sources.
Each island completes its recursive wavefront stages before the next island
is routed. The existing lead-in distance, stepover and cutter reach still apply.

Every island sees all protected part and fixture geometry. Routing receives
the complete incoming stock and carries the cleared cutter-centre region and
tool position between stages, including between islands. Separating propagation
does not turn neighbouring stock into air or change the entry strategy. Boundary
finishing and material-removal simulation still operate on the whole depth pass.
Holes remain attached to their stock island; separate solids inside a hole
propagate independently.

Several parts on one connected plate still share a propagation region. Use
[ordered roughing areas](roughing-areas.md) to choose independent regions and
part order for that case. Area edges do not create wave sources.

The implementation keeps per-island analyses and committed stages separate.
Only the combined geometry used for display and depth association is unioned.
Boundary-contact indices are remapped when combining lineages so endpoint
provenance remains valid across different island boundaries.

`stock_island_tests` covers overlapping approach regions, neighbouring stock
visibility, global protected geometry, contour separation, removal coverage,
boundary-contact remapping, nested islands and unchanged connected-plate
propagation. The coverage test allows the terminal radial stepover handled by
boundary finishing; it checks for gaps beyond that band.
