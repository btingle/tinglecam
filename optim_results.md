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
- Keep Difference output directly after engaged moves instead of normalizing
  it with another union.
- Retain only step endpoints and periodic remaining-stock checkpoints for
  playback. Cutter sweeps are reconstructed for the current move, and at most
  three completed moves are replayed from the nearest checkpoint.
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
- Bounding-box pruning for lineage parent/child front tests: flat-to-worse even
  on the 267-generation `sSs` workload.
- Deriving engagement from the area delta of the remaining-stock Difference:
  faster, but it changed 1,296 posted feed lines on `L` and shifted empty-step
  classification through subtraction of large areas.
- A conservative exact straight-stitch check before Router construction:
  faster, but it selected direct links where the existing router selected a
  detour and therefore changed transition geometry and simulation counts.

## Before and after

The baseline is the instrumented, otherwise unoptimized checkpoint. The final
figures are from the same command shapes after all retained changes.

| Workload | Baseline planning | Final planning | Baseline process wall | Final process wall | Baseline RSS | Final RSS |
|---|---:|---:|---:|---:|---:|---:|
| `L`, 0.2 mm pitch | 6,084 ms | 2,082 ms | 8.51 s | 4.25 s | 63,052 KiB | 42,408 KiB |
| `Rtwist`, 0.2 mm pitch | 3,613 ms | 1,409 ms | 4.21 s | 1.97 s | 43,856 KiB | 30,888 KiB |

That is a 2.92x internal planning speedup on `L` and 2.56x on `Rtwist`, with an
approximately 50% reduction in full headless process wall time and a 30–33%
reduction in peak resident memory.

Representative final phase profiles:

| Phase | `L` | `Rtwist` |
|---|---:|---:|
| Slicing | 903 ms | 702 ms |
| Wavefront | 66 ms | 27 ms |
| Lineage | 126 ms | 49 ms |
| Interlayer | 103 ms | 83 ms |
| Toolpath | 186 ms | 62 ms |
| Cleanup | 19 ms | 11 ms |
| Stitching | 24 ms | 15 ms |
| Simulation | 628 ms | 447 ms |

The final hash-enforcing extended corpus was deterministic in every case.
Planning time was 3,167–3,185 ms for fine-pitch `L`, 2,246–2,263 ms for the
small-stepover `sSs` case (267 wave generations and 38,836 simulation
intervals), and 1,593–1,600 ms for `L` contour finishing.

The direct-area optimization changes a small number of posted integer feed
commands by 1 mm/min because it removes the tiny geometric rounding introduced
by re-unioning a normalized Boolean result. Path coordinates, move counts,
safety totals, material totals at displayed precision, and posted line counts
are unchanged. Every later optimization is byte-identical relative to that
accepted result, and repeated fine-pitch parallel runs produced identical hashes
and counters.

## Final decision gate

After the retained serial reductions, playback-storage changes, and bounded
parallelism, slicing is the largest serial phase on both representative jobs.
The more invasive ideas in the original plan—frontier-local wave propagation,
a partitioned simulation stock representation, router edge spatial indexing,
and deeper topology spatial indices—now target comparatively small residual
phases or require broad representation changes. They are not implemented
without a workload that demonstrates enough remaining cost to justify their
correctness and maintenance risk.
