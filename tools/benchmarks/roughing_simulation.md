# Roughing simulation experiments

The default is **Area / local stock**. It retains the existing simulation/feed
pitch, processes intervals in chronological order inside a bounded local stock
window, and commits capsule sweeps to the complete stock once per batch. Batches
contain at most 64 intervals and normally span at most four cutter radii of
travel (an individual longer input interval remains intact).

**Contact boundary (experimental)** estimates projected cutting width from the
stock-contacting leading circumference of the cutter. Three fixed Gauss
quadrature locations average that estimate over each original interval. This is
not adaptive sampling. Entry intervals and intervals whose contact estimate
misses positive material removal fall back to area engagement. Stock updates,
gouge checks, and plunge checks use swept geometry, not the contact estimate.
Finite contact sampling can still underestimate or overestimate local engagement
peaks. Both methods are geometric load estimates, not cutting-force models.

## Comparing in the app

1. Under the operation's **Engagement method**, select **Compare both (area feeds)**.
2. Plan the operation, then choose **Colour by → Engagement** in Playback / Display.
3. Switch **Engagement display** between **Area**, **Contact boundary**, and
   **Absolute difference**. Switching the display does not require replanning and
   works in the pass and 3D result views. Difference colours run from blue at zero
   to red at ten percentage points; the panel also reports mean and maximum error.
4. Comparison mode retains area-based feeds. To try contact-based feeds and their
   estimated machining time, select **Contact boundary (experimental)** in the
   operation and replan. **Area / original reference** retains the previous global
   per-interval algorithm for comparison.

## Reproducing benchmarks

`benchmark_planning.sh` now runs all four modes for its small-cutter LsJig case
and checks deterministic G-code within each mode. It uses isolated GUI defaults
and the shipped tool library. For the user's existing tool/defaults, run:

```sh
build/tinglecam --part tests/fixtures/meshes/LsJig1.obj --box-stock 5 \
  --mode rough --tool 3.175mm-1f --stepdown 3 --stepover 0.2 \
  --finishing-subdivisions 4 --path-pitch 0.2 \
  --simulation-method compare --profile-planning --gcode /tmp/lsjig-compare.nc
```

Replace `compare` with `reference_area`, `local_area`, or `contact` to measure
those modes. The custom tool ID above belongs to the user's library; the shipped
3.175 mm tool is `3.175mm-square`. The measurements below used a 3.175 mm flat
cutter, 5 mm XY stock margin, 0.2 mm radial allowance, zero axial allowance,
0.2 mm cleanup tolerance, 500/1200 mm/min manual engagement feeds, and the
`makera-z1` machine profile with HDPE. Full depth is 17 mm.

## Results

Median of three separate CLI runs per method, Release (`-O3`), Ryzen 7700X,
with planner concurrency capped at eight workers. Times are seconds; planning
includes simulation, while process wall time also includes startup and export.
Raw timings and G-code hashes are in [roughing_simulation.csv](roughing_simulation.csv).

| Method | Simulation | Planning | Process wall time |
| --- | ---: | ---: | ---: |
| Original reference area | 12.442 | 20.747 | 21.178 |
| Local area (default) | 2.617 | 11.110 | 11.512 |
| Contact boundary | 2.591 | 11.141 | 11.578 |
| Compare both | 2.693 | 10.974 | 11.385 |

Local batching makes simulation 4.75 times faster and reduces total planning time
by about 46%. Contact estimation provides no meaningful additional speedup on
this job; small timing differences between the three local modes are noise.
The original 0.2 mm sampling pitch is unchanged.

Contact versus area engagement in comparison mode differs by a distance-weighted
mean of 0.00746 percentage points and a maximum of 3.85221 percentage points.
There are 188 area fallbacks across 10,920 stock batches. All four methods report
zero gouge area and zero unsafe plunges on this job. Each method produces
deterministic G-code across its three runs; comparison mode matches local area
G-code exactly. Contact mode changes the feed estimates.

## Geometry and engagement validation

On identical LsJig paths, local batching versus the original simulator produced:

- Mean absolute engagement difference: 0.000275 percentage points, weighted by distance.
- Maximum engagement difference: 0.0788 percentage points.
- Maximum per-layer symmetric stock difference: 0.00986 mm².
- Total symmetric stock difference across the cutting layers: 0.0601 mm².
- No unsafe-plunge count differences; zero reported gouge area.

Batching changes Boolean-operation grouping on the 0.001 mm integer grid.
It is not bitwise equivalent to sequential subtraction. In the end-to-end LsJig
plan this resulted in one additional conservative clearance retract (11 stay-down
links versus 12), increasing estimated machining time by about five seconds.
Local area, contact, and comparison modes share the same global stock updates.
Comparison mode must produce identical G-code to local area mode.

Regression coverage includes steady side cuts, slots, reversals, corners,
holes/islands, a thin gouged feature, fractional tool diameters, stock handoff,
comparison feed selection, and completed preview stock.
