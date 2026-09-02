# Toolpath Planning Optimization Plan

## Objective

Reduce end-to-end CAM planning time without weakening geometric correctness, toolpath safety, deterministic output, or final stock-removal validation.

The current representative profiles divide planning time roughly as follows:

- Simulation and feed preparation: 47–53%
- Wavefront construction and topology: about 28–29%
- Toolpath construction and stitching: about 18–25%

Clipper2 scanline and intersection work dominates sampled CPU time. The plan therefore focuses first on eliminating unnecessary exact polygon operations, then on making the remaining operations cheaper, and only afterward on parallel execution.

## Phase 0: Measurement and Correctness Gates

Establish a repeatable benchmark and correctness harness before changing algorithms.

### Benchmark corpus

Include workloads that exercise different scaling dimensions:

- `L` and `Rtwist` as representative existing jobs
- Fine, normal, and coarse path pitch
- Small stepover and many wavefront generations
- Multiple pockets and complex contour boundaries
- Roughing cleanup and finishing paths

### Instrumentation

Record total and per-level time for:

- Slicing and protected-part preparation
- Wavefront propagation
- Front lineage construction
- Interlayer topology
- Main toolpath construction
- Cleanup and safe approaches
- Routing and stitching
- Resampling
- Stock simulation
- Feed assignment and final linking

Add counts that explain changes in runtime:

- Clipper union, intersection, difference, inflate, and simplify calls
- Simulated path intervals
- Wavefront generations and fronts
- Lineage and interlayer candidate comparisons
- Router constructions, corner counts, visibility tests, and expanded nodes
- Safe-approach candidates and exact intersection tests

### Correctness gates

Retain golden or differential checks for:

- Toolpath geometry and ordering where ordering is semantically important
- Collision and containment safety
- Feed values and engagement measurements
- Final remaining stock and removed material
- Posted machine motion
- Deterministic results across repeated and parallel runs

Performance changes should report both elapsed time and the operation counts responsible for it.

## Track 1: Simulation

Simulation is the largest current phase and scales superlinearly with path pitch. Every fine path interval constructs a cutter sweep and repeatedly applies exact Boolean operations to an increasingly detailed remaining-stock polygon.

### 1A. Remove redundant geometric work

Start with exact, low-risk simplifications:

- Avoid re-unioning output that Clipper has already returned in normalized, disjoint form when the relevant invariants hold.
- Compute area directly from disjoint Boolean results where a further union is unnecessary.
- Use bounding-box rejection before gouge and stock intersection operations.
- Avoid constructing or retaining geometry needed only for playback when playback data is not requested.
- Reuse normalized representations within an interval rather than converting or rebuilding them multiple times.

Each removal should be guarded by equivalence tests against the current simulator.

### 1B. Reduce full-stock Boolean operations

Prototype ways to avoid intersecting and subtracting every cutter capsule from the entire accumulated stock boundary:

- Spatially partition remaining stock and operate only on affected components or tiles.
- Process short path chunks while preserving exact per-interval engagement values.
- Separate the data needed for engagement calculation from the representation used to produce final remaining stock.
- Reuse shared geometry between consecutive cutter-sweep capsules where practical.

These are experiments rather than immediate replacements. The acceptance criterion is identical engagement and final-stock results on the correctness corpus.

### 1C. Separate planning data from playback data

The planner and feed calculation need engagement measurements, while the UI may need per-move sweep and remaining-stock snapshots. Make those storage requirements explicit:

- Keep a minimal simulation result for planning and feeds.
- Build or retain detailed playback snapshots only when requested.
- Measure peak memory as well as runtime.

### Simulation target

Move runtime toward linear scaling with the number of simulated intervals, with substantially less sensitivity to accumulated stock-boundary complexity.

## Track 2: Toolpath

Toolpath time is spread across repeated topology work, exact connection-safety tests, construction of growing cleared/forbidden regions, cleanup approaches, routing, and level stitching.

### 2A. Eliminate duplicate topology construction

`build_inter_layer_topology()` already stores each layer's topology, but `build_toolpath()` constructs `LayerTopology` again. Pass the existing topology into toolpath construction and remove the duplicate work.

### 2B. Order connection tests from cheap to expensive

For every candidate connection:

1. Test the direct segment against the already available cleared-region constraint.
2. Construct combined travel geometry only if that test fails.
3. Evaluate plunge/lift alternatives only if still necessary.
4. Invoke the router last.

Build derived forbidden regions lazily and at most once for each version of the cleared region.

### 2C. Make routing demand-driven

Preserve the current router's geometric safety model, including all boundary corners and its final exact path validation, while avoiding eager construction of a dense visibility graph.

- Perform the direct legality test before constructing a `Router`, especially during stitching.
- Compute the lift alternative first so its cost becomes an upper bound for routing.
- Search the same all-corner visibility graph lazily with cost-bounded A* rather than precomputing every corner-to-corner edge.
- Memoize visibility results only within a single router invocation.
- Add a spatial index over boundary edges so visibility tests inspect nearby candidate edges instead of scanning the full boundary.
- Retain neighboring boundary edges and the final exact Clipper validation as safety backstops.

Do **not** introduce cross-level or cross-region routing-region caches. The expected reuse is uncertain and invalidation/identity costs make that a poor first optimization. Only ephemeral memoization inside one route computation is in scope.

Also avoid initially replacing the graph with reflex corners only or a boundary-walk route: offset zigzags make the former unsafe, while the latter can produce severely suboptimal paths.

### 2D. Accelerate safe-approach selection

`add_safe_stock_approaches()` currently compares contour variants with many stock-boundary edges using exact intersections.

- Build a spatial index over outside boundary edges.
- Consider nearby candidates in distance order.
- Reject candidates with bounding-box and inexpensive distance checks.
- Use the existing exact intersection proof only for the surviving candidates.

### 2E. Make stitching demand-driven

For connections between levels:

1. Try the direct connection.
2. Establish the lift path and its cost.
3. Run bounded routing only when a cheaper safe route may exist.

This avoids constructing a full router for connections that are already straight-line legal or cannot beat the lift alternative.

## Track 3: Wavefront and Topology

Wavefront work repeatedly performs polygon morphology on the cumulative reached region. Afterward, lineage code reconstructs band and front relationships with more Boolean operations and broad pairwise geometric searches. The core issue is repeated processing of an ever-growing cumulative region rather than work local to the advancing frontier.

### 3A. Remove redundant full-region operations

Audit every wavefront generation for:

- Unions whose inputs are already normalized or disjoint
- Repeated canonicalization used only for equality checks
- Full copies of cumulative regions
- Differences and intersections whose bounding boxes cannot overlap
- Derived regions that can be carried forward safely instead of reconstructed

Apply the same operation-count and differential-correctness gates used for simulation.

### 3B. Prototype frontier-local propagation

Investigate advancing only the current frontier and merging newly reached material into the cumulative set, rather than inflating the entire cumulative reached region every generation.

This is the highest-risk serial optimization because holes, merging fronts, narrow channels, and topology changes can make local propagation subtle. Validate generation by generation against the current implementation, comparing:

- Reached area
- Connected components and holes
- Extracted leading fronts
- Termination generation
- Downstream lineage and toolpaths

Keep the existing implementation available as the reference until the new formulation is proven across the corpus.

### 3C. Reduce lineage search and reconstruction

- Maintain or update the unreached region incrementally where exact invariants allow it.
- Use a spatial index for static boundary edges during leading-front endpoint classification.
- Apply bounding-box filters before point-in-polygon and exact containment tests.
- Index child components spatially before parent-to-child matching instead of inflating every parent and testing every child.
- Reuse the resulting layer topology in toolpath construction.

### 3D. Prune interlayer topology comparisons

Interlayer construction currently compares many nodes and entry fronts between adjacent layers.

- Reject impossible containment pairs by bounding box.
- Use area ordering to eliminate impossible parents.
- Group candidates by evaluation level and spatial overlap.
- Query a spatial index for plausible containers.
- Retain exact containment as the final decision.

## Parallelization

Parallelize only after the serial kernels have been simplified, so concurrency does not hide avoidable work or multiply peak memory.

The intended execution order is:

1. Pre-slice and populate shared caches serially.
2. Build wavefront and lineage for independent levels in parallel.
3. Build interlayer topology in its required ordered phase.
4. Build per-level toolpaths and cleanup work in parallel.
5. Stitch levels in order.
6. Resample and simulate independent levels in parallel.
7. Perform feed linking and final rollup in order.

Parallel execution requirements:

- Bounded worker pool rather than one thread per level
- Explicit memory cap, particularly for simulation results
- Shared caches populated before workers start or made demonstrably thread-safe
- Results collected by stable level index
- Deterministic output and error reporting
- Thread-safety checks around Clipper wrappers and other shared state

## Delivery Order

1. Add the benchmark corpus, phase timers, operation counters, and correctness gates.
2. Optimize the exact simulation hot loop and separate playback storage.
3. Remove duplicate toolpath topology work; reorder connection tests; improve routing, safe approaches, and stitching.
4. Optimize wavefront and lineage operations, beginning with low-risk redundancy and candidate pruning.
5. Add bounded per-level parallelism after serial behavior is stable.
6. Reprofile the complete corpus and decide whether frontier-local wavefront propagation and deeper simulation representation changes justify their complexity.

At the end of each delivery step, regroup using measured speedup, operation-count changes, peak memory, and correctness results before committing to the next higher-risk change.
