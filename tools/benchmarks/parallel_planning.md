# Parallel planning and progress

This records the initial worker comparison. The [cleanup follow-up](cleanup_parallel.md)
adds parallel subdivision planning and fixes upward transition checks.

Local area remains the default simulation method. Planning progress now tracks
actual worker completion instead of advancing while layers are merely queued.
The blue portion shows completed work (including completed simulation intervals);
amber covers the unfinished portion of assigned work. Unassigned work remains
unfilled. The panel shows completed layers and active workers for the current
stage, and remains visible above the scrolling operation list.

Workers publish simulation interval progress every 64 intervals and at move
boundaries. The coordinator forwards progress roughly every 50 ms and at stage
boundaries; GUI callbacks never execute concurrently on geometry workers.
Wavefront, toolpath, and boundary cleanup stages report individual layer
completion. Progress fractions are work estimates, not time estimates: different
layers and stages have different costs.

Cancellation stops assigning new layers, interrupts material simulation between
interval batches, and joins all workers before returning. Other geometry tasks
finish their current layer before stopping. A one-worker run uses the same
algorithms directly on the coordinator, without launching geometry threads.

## Reproduction

```sh
python3 tools/benchmarks/parallel_planning.py --runs 3 --output /tmp/worker-benchmark
```

The script benchmarks worker limits of 1, 2, 4, and 8, alternates their order,
records per-stage times and process CPU/wall time, and requires byte-identical
G-code across every run and worker count. By default it uses isolated settings
and the shipped `3.175mm-square` cutter. Use `--config PATH --tool ID` to compare
an existing XDG configuration and tool library instead.

The environment variable `TINGLECAM_PLANNING_WORKERS=1` selects serial planning;
values 1–64 override the worker limit. Without an override, the default is the
smaller of eight and hardware concurrency, further limited by available layers.
Only independent layer work is parallelized; stock dependencies are preserved.

## Measured workload

Release (`-O3`), Ryzen 7700X, LsJig1.obj, 3.175 mm flat cutter, 3 mm stepdown,
0.2 stepover ratio, four subdivisions, 0.2 mm simulation pitch, local area,
5 mm XY stock margins, 0.2 mm radial allowance, zero axial allowance,
0.2 mm cleanup tolerance, 500/1200 mm/min manual engagement feeds, HDPE,
and the Makera Z1 profile. The user's settings and tool library were copied to
an isolated directory before benchmarking. No builds or other test workloads
ran concurrently with the benchmark.

There were six coarse layers available together in this version, so the
eight-worker limit used at most six workers. Upward cleanup layers were still
planned sequentially; the follow-up removes that scheduling restriction. Slicing, feed scheduling, and
cross-layer linking also contain ordered work. Consequently whole-job speedup
cannot scale directly with the worker limit.

## Results

Median of three fresh-process runs per worker limit; all times are seconds.

| Worker limit | Total planning | Wavefronts | Toolpaths | Simulation | Speedup |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 27.757 | 13.357 | 4.581 | 8.787 | 1.00× |
| 2 | 17.326 | 8.903 | 2.711 | 4.911 | 1.60× |
| 4 | 11.437 | 5.396 | 1.870 | 3.438 | 2.43× |
| 8 | 10.498 | 5.460 | 1.856 | 2.427 | 2.64× |

The default worker limit delivers a **2.64× whole-job speedup**, or about a 62%
reduction in planning time. Simulation improves by 3.62×. Four workers capture
most of the benefit; raising the limit to eight saves another 0.94 seconds here,
primarily in simulation. Wavefront construction is slightly slower at eight
than four on this fixture, so speedup is not uniform across stages.

Process CPU time rises only from 28.04 to 29.37 seconds, while wall time falls
from 28.13 to 10.91 seconds. This demonstrates concurrent useful work with a
modest increase in CPU cost, rather than a change in algorithms or output.
All twelve runs produce the same G-code hash, also identical to the preceding
local-area benchmark. The default limit remains eight.

[Raw timings and hashes](parallel_planning.csv) include every stage and run.
These measurements apply to this fixture and machine; jobs with fewer independent
layers will obtain less benefit.

## Validation

Regression tests exercise serial and parallel progress, coordinator-only
callbacks, monotone fractions, active/completed counts, intermediate simulation
updates, cancellation with queued work, worker joining, and worker exceptions.
Existing geometry and CAM regressions continue to check the actual planned
paths. Benchmark hashes additionally compare the complete posted output across
worker counts.
