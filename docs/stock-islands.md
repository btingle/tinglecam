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

Roughing landings apply cutter clearance to physical stock separately from
fixture keep-outs and roughing-area limits, which already constrain the tool
centre. Rejected initial approaches leave no rapid waypoint behind.

Boundary cleanup runs only after roughing emitted cuts without skipped entries
or unreachable fronts at that depth. A theoretical propagated region alone does
not authorize cleanup. This also leaves a thin rim untouched when no roughing
contour was emitted, rather than generating a standalone boundary cut.

Stock-aware inter-level linking also runs with fixtures present. Candidate
vertical and horizontal sweeps are checked against fixture geometry, and routed
links include fixture keep-outs. A blocked connection retains its safe-plane
retract.

Wave simplification applies to the cumulative reached region, while the active
moving band retains its exact boundaries. Independently simplifying that thin
band can collapse its tips or shift its trailing edge away from the preceding
wave, producing detached fronts and blocking access to their descendants.
Keeping the band exact prevents those seams during the original propagation;
there is no retry or second planning pass. `stock_island_tests` includes the
Area 1, Z=13 slice from `setup_test3.json` at 0.01 mm simplification and checks
front connectivity and coverage against unsimplified propagation.
