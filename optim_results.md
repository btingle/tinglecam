# Planning Optimization Results

This file records the implementation and benchmark outcome of
[`optim_plan.md`](optim_plan.md). All timings are wall-clock measurements from a
Release build while another interactive TingleCAM process remained running, so
individual absolute values contain system-load noise. Decisions were made from
paired runs, repeated phase measurements, deterministic output hashes, and the
full regression suite.

## Benchmark and correctness gates

Planning now records these phases in `cam::PlanningMetrics` and exposes them
with `tinglecam --profile-planning`:

- slicing and protected-part preparation;
- wavefront, lineage, and interlayer topology;
- toolpath, cleanup, and stitching;
- resampling, simulation, and feeds;
- total planning time;
- wave generations, fronts, toolpath moves, simulation intervals, empty
  intervals, and bounds-rejected gouge tests.

Run the complete repeatable corpus with:

```sh
./benchmark_planning.sh build 3
```

The runner first executes all CTest tests, then covers representative roughing,
fine simulation pitch, small stepover/many wave generations, complex
boundaries, and contour finishing. Every run emits a G-code SHA-256 hash and
retains its profile, posted G-code, stdout, stderr, time, and peak RSS.

## Retained optimizations

### Simulation

- Sum the signed area of normalized Clipper Boolean results directly instead
  of unioning the same result again.
- When sweep/remaining-stock intersection is empty, skip the provably no-op
  stock difference.
- Reject gouge intersections by non-overlapping bounds before invoking
  Clipper.
- Simulate independent depth levels with a deterministic, bounded four-worker
  pool; resampling and feed linking remain ordered.

On the `L` workload, 6,200 of 26,693 intervals remove no material and 11,562
gouge tests are rejected by bounds. On `Rtwist`, the corresponding counts are
2,502 of 16,262 and 3,197.

### Toolpath and routing

- Reuse the `LayerTopology` already built by interlayer association instead of
  constructing it again inside `build_toolpath()`.
- Sort safe stock-approach projections by cheap distance, then stop at the
  first candidate that passes the existing exact collision test. Stable sorting
  preserves the previous equal-distance tie order and selects the same nearest
  legal approach.
- Keep the router's same all-corner graph and Dijkstra search, but memoize
  corner visibility lazily as search expansion requests it. Boundary-neighbor
  edges and final exact validation are unchanged.
- Build independent per-depth main toolpaths and rough cleanup paths with the
  same bounded worker pool; stitching remains ordered.

No cross-level or cross-region routing-region cache was added.

### Wavefront

- Remove redundant unions immediately after Clipper Difference operations in
  wave seeding and generation. Difference already returns normalized,
  non-overlapping output.
- Build wavefront and lineage phases independently across depths after all
  slice-cache population is complete. Interlayer association remains ordered.

## Reverted experiments

The following experiments did not improve the paired representative workloads
and were completely removed:

- Lazily constructing the target-band travel region only after a direct-cleared
  segment failed: flat on `Rtwist`, about 1% slower on `L`.
- Rejecting stitching routes with an optimistic lift-cost lower bound: rejected
  0 of 13 candidates on both jobs.
- Bounding-box pruning before interlayer node/front containment: interlayer
  time was unchanged on the representative topology sizes.

## Before and after

The baseline is the instrumented, otherwise unoptimized checkpoint. The final
figures are from the same command shapes after all retained changes.

| Workload | Baseline planning | Final planning | Baseline process wall | Final process wall | Baseline RSS | Final RSS |
|---|---:|---:|---:|---:|---:|---:|
| `L`, 0.2 mm pitch | 6,084 ms | 2,137 ms | 8.51 s | 4.34 s | 63,052 KiB | 74,632 KiB |
| `Rtwist`, 0.2 mm pitch | 3,613 ms | 1,444 ms | 4.21 s | 2.02 s | 43,856 KiB | 51,040 KiB |

That is a 2.85x internal planning speedup on `L` and 2.50x on `Rtwist`, with an
approximately 50% reduction in full headless process wall time and a 16–18%
increase in peak resident memory.

Representative final phase profiles:

| Phase | `L` | `Rtwist` |
|---|---:|---:|
| Slicing | 913 ms | 711 ms |
| Wavefront | 67 ms | 27 ms |
| Lineage | 130 ms | 50 ms |
| Interlayer | 104 ms | 85 ms |
| Toolpath | 184 ms | 63 ms |
| Cleanup | 19 ms | 11 ms |
| Stitching | 25 ms | 16 ms |
| Simulation | 664 ms | 468 ms |

The final three-repeat extended corpus was deterministic in every case. Median
planning time was 3,260 ms for fine-pitch `L`, 2,307 ms for the small-stepover
`sSs` case (267 wave generations and 38,836 simulation intervals), and 1,603 ms
for `L` contour finishing.

The direct-area optimization changes a small number of posted integer feed
commands by 1 mm/min because it removes the tiny geometric rounding introduced
by re-unioning a normalized Boolean result. Path coordinates, move counts,
safety totals, material totals at displayed precision, and posted line counts
are unchanged. Every later optimization is byte-identical relative to that
accepted result, and repeated fine-pitch parallel runs produced identical hashes
and counters.

## Final decision gate

After the retained serial reductions and bounded parallelism, slicing is the
largest serial phase on both representative jobs. The more invasive ideas in
the original plan—frontier-local wave propagation, a partitioned simulation
stock representation, playback/planning representation separation, router edge
spatial indexing, and deeper topology spatial indices—now target comparatively
small residual phases or require broad representation changes. They are not
implemented without a workload that demonstrates enough remaining cost to
justify their correctness and maintenance risk.
