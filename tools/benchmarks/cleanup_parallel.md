# Parallel cleanup and upward transitions

## Scheduling and output order

After coarse descent is planned, the planner runs all first subdivisions in
parallel, then all second subdivisions, and so on. At each round boundary,
workers share the completed stock history read-only. A cleanup layer lies inside
one coarse terrace's Z interval: other terraces' cleanups cannot affect its
incoming stock. Only earlier subdivisions of its own terrace are dependencies.

Planning order is separate from execution order. Results are moved back into the
original order: all coarse descent layers, then all upward subdivisions of the
deepest terrace, then the next terrace. Topology indices, preview groups, and
warnings are assembled in that order. Arrival/feed schedules are rebuilt against
the execution predecessor before the final stock-validated linker runs.

Before the linker changes, this scheduler produced byte-identical G-code to the
previous sequential-cleanup scheduler on LsJig. No full stock or plan is cloned
for each worker. Large path/result buffers are moved when reordering them.

Concurrent slicing exposed the CUDA slicer's shared query scratch buffers. A
per-mesh lock now protects flags/count/output until their host download finishes.
The shared BVH stays resident; CPU polygon construction and the expensive CAM
stages run outside that lock. Concurrent slice tests compare geometry against
serial queries.

## Why upward links were retracting

The linker already lifts first for an upward connection and traverses first for
a downward connection. Two geometric consistency problems caused false failures:

- It added radial allowance around the nominal part even where prior cuts had
  already removed stock along that boundary. It now checks actual remaining
  stock, including all uncut allowance, and independently protects the finished
  part. Cleared space is not filled back in by an idealized allowance outline.
- Round cutter sweeps are approximated by polygons. A sweep at another angle
  can encounter tiny numerical remnants inside an already-executed circular
  cutter sweep. For a remaining-stock collision, the linker can prove that each
  entire collision polygon lies inside one previous cutter capsule covering
  that stock slab's complete height. Capsules are convex, so containment of all
  polygon vertices in the same capsule suffices. Small area by itself never
  permits a move. Finished-part collisions cannot use this exception.

On the benchmark LsJig job, all 18 connections between the 19 nonempty cutting
layers now stay below clearance, versus 11 previously. Genuine obstacles still
cause a detour or retract. The five empty cleanup layers remain empty, and the
reported gouge area, unsafe plunges, and unresolved entries remain zero.

## Reproduction and timing interpretation

Use the same workload/settings described in [the preceding benchmark](parallel_planning.md):

```sh
python3 tools/benchmarks/parallel_planning.py --profile-workers --runs 3 \
  --output /tmp/cleanup-benchmark
```

Use `--config PATH --tool ID` for an existing XDG configuration/tool library.
`TINGLECAM_PROFILE_WORKERS=1` also enables individual task/batch timings directly
in the CLI. `TINGLECAM_TRACE_LINKS=1` reports collision proofs and link costs.
The benchmark now records per-process peak RSS and page faults as well as CPU
and wall time. Each run must produce the same G-code hash across worker counts.

`total` and `parallel_cleanup` are wall-clock times. `cleanup_work` sums worker
elapsed time for all cleanup passes. The other phase metrics now include summed
cleanup-worker durations, which can overlap; do not add them to infer total wall
time or directly compare them with the old parallel phase wall times. The
per-batch worker profile reports both elapsed work and actual batch wall time.

## Scaling diagnosis

The coarse layers have very unequal workloads. In the profiled serial run,
wavefront tasks took approximately 3.47, 3.48, 4.39, 0.42, 0.24, and 0.24 seconds.
Four workers already accommodate the three expensive tasks. Additional workers
finish the small tasks early and cannot help the thread still processing the
longest layer. The same imbalance occurs in toolpath construction; simulation
is more evenly distributed and benefits more from the additional workers.

This is the main measured limit, rather than a whole-job copy on worker startup.
Each layer still needs its own mutable Clipper/simulation workspace, just as in
serial planning, with more of those transient allocations alive concurrently.
Memory use and CPU work measurements below quantify the resulting overhead;
they do not isolate cache, allocator, memory bandwidth, or CPU frequency effects.
No hardware-counter attribution is claimed.

## Results

Median of three fresh-process runs per worker limit on the Ryzen 7700X, with
local area, the same LsJig settings, and no concurrent build/test workload.
Eight is the limit; this fixture supplies at most six independent layers.

| Worker limit | Total planning (s) | Cleanup rounds (s) | Process CPU (s) | Peak RSS (MiB) |
| --- | ---: | ---: | ---: | ---: |
| 1 | 27.790 | 2.006 | 28.086 | 359.8 |
| 2 | 16.753 | 1.235 | 28.527 | 393.0 |
| 4 | 10.378 | 0.987 | 28.869 | 466.3 |
| 8 | 9.560 | 0.965 | 29.422 | 482.4 |

The default limit is **2.91× faster than serial planning**.
The preceding default-worker benchmark measured 10.498 s; this version measures
9.560 s, approximately 8.9% less. That is a comparison with the preceding
measurement, not a fresh interleaved old/new benchmark. Cleanup round wall time
falls from 2.006 to 0.965 s. Peak memory is about
1.34× the serial peak, and aggregate process CPU time rises about
4.8%, while wall time drops substantially.

All twelve runs produce identical G-code, with all 18 layer connections accepted
below clearance. Cutting order, layer count, simulation interval count (311,472),
and removal remain unchanged. The intended G-code changes are the improved
between-layer transitions.

[Raw run timings and hashes](cleanup_parallel.csv) and
[per-batch/per-task timings](cleanup_workers.csv) record the measurements.

## Regression coverage

All 12 test suites pass. Tests cover parallel/serial posted equivalence, original
interleaved depth order, irregular depth breaks, stock inherited at each executed
layer, preview/final stock, topology, concurrency and cancellation, concurrent
GPU queries, upward/downward links, overhead stock, genuine islands, cleared
versus uncut allowance, and rejection of unproven tiny collisions. A separate
check ensures earlier removal evidence never waives finished-part protection.
