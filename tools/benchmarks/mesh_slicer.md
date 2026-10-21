# cuBQL CPU slicer experiment

Measured 2026-10-20 in the local Release build on a Ryzen 7 7700X (8 cores,
16 hardware threads), NVIDIA RTX 5060, under WSL. CUDA remains the preferred
backend. The measurements below describe the initial CPU experiment;
the CPU backend is now also the automatic build/runtime fallback when CUDA
is unavailable. The recorded `test2` compatibility difference remains open.

## Implementation and reproduction

The CPU backend uses the pinned cuBQL dependency's `cpuBuilder` (spatial median),
box queries and shrinking-ray traversal. It retains the existing triangle
predicates, containment tolerances and CPU polygon reconstruction. Containment
pruning and large ray batches use up to eight CPU workers; each slice does one
BVH traversal, with private candidate storage. Independent slices can overlap.
It does not restore the old all-pairs CPU containment algorithm.

```sh
cmake --build build --target mesh_slicer_benchmark --parallel 4
./build/mesh_slicer_benchmark tests/fixtures/meshes/LsJig1.obj 3
./build/mesh_slicer_benchmark tests/fixtures/meshes/final/L.obj 3
./build/mesh_slicer_benchmark models/meshes/holejig_upright.obj 3
./build/mesh_slicer_benchmark models/parametric/jig_final/Ls.step 3

# Automatic selection (CUDA when usable, CPU otherwise):
./build/tinglecam
# Force the CPU backend throughout a regular application/CLI run:
TINGLECAM_SLICER=cpu ./build/tinglecam
# Explicit GPU baseline:
TINGLECAM_SLICER=cuda ./build/tinglecam
# Optional CPU worker limit for further comparisons:
TINGLECAM_SLICER=cpu TINGLECAM_SLICER_THREADS=1 ./build/tinglecam
```

The last two input models are local user assets. CMake builds CUDA acceleration
when a toolkit is detected; `-DTINGLECAM_WITH_CUDA=OFF` explicitly builds without
CUDA, and no toolkit is required for the CPU build. A CUDA-enabled executable
also falls back to CPU when its kernels cannot load or initialize on the current
device/driver. Explicit `TINGLECAM_SLICER=cuda` requests fail instead of falling
back. No saved-job schema or GUI setting changes. On a machine/build without
usable CUDA, the microbenchmark reports CPU results and skips GPU comparisons.

The benchmark imports a mesh once, alternates CPU/GPU measurement order across
three runs, and compares 32 evenly spaced slices, eight-worker concurrent
slicing, 100 batches of 12 downward rays, and a batch of 65,536 rays. Query
timings are host wall time, including CUDA allocations, transfers, sorting and
synchronization, rather than CUDA kernel-event timings. Build figures are the
existing `MeshBuildStats::build_milliseconds`, excluding mesh import/copy.
Comparisons run after the measured work. CPU and GPU use the same imported
triangles, so STEP import is excluded from the microbenchmark timings.

## Median results

All times below are milliseconds. Each pair is **CPU / CUDA**.

| Model | Triangles | Build/prune | 32 serial slices | 32 slices, 8 workers | 100 x 12 rays | 65,536 rays |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Roughing jig | 624 | 0.53 / 3.77 | 8.87 / 18.96 | 3.48 / 8.04 | 0.94 / 13.35 | 2.93 / 0.37 |
| Dense track | 24,638 | 8.15 / 8.84 | 1653 / 1708 | 448 / 457 | 1.12 / 10.69 | 3.90 / 1.09 |
| Upright hole jig | 1,691 | 1.21 / 2.77 | 30.75 / 39.36 | 5.91 / 11.05 | 0.91 / 9.75 | 2.01 / 0.36 |
| Native STEP jig | 89,812 | 27.10 / 10.76 | 9734 / 9747 | 1657 / 1663 | 1.11 / 10.48 | 2.94 / 1.20 |

The first CUDA build in each fresh benchmark process includes context startup
and took 157–225 ms; the median build numbers primarily reflect a warm context.
STEP reading/triangulation took about 4.3 seconds once, independently of the
slicer backend.

The native STEP case spends about 9.55–9.67 seconds of its serial slice time in
the existing CPU clipping/polygon union. Changing candidate-query backends has
little effect on that workload. CUDA wins on bulk ray throughput; CPU avoids
allocation/transfer/launch costs for tiny batches. The existing GPU backend
serializes its candidate-buffer transaction, while CPU slice queries have no
shared scratch buffers.

## Correctness and limits

All four microbenchmark inputs produced matching slice regions on the 0.001 mm
integer grid and matching sampled ray hit masks/depths (depth tolerance 0.001 mm).
The hole jig retained 896 triangles on CPU versus 912 on CUDA; the STEP jig
retained 80,342 versus 80,343. Redundant-triangle pruning is sensitive to floating
point arithmetic and BVH candidate grouping. These count differences did not
change the sampled slice regions or rays. Candidate/pruning counts alone are
therefore reported rather than required to be identical on arbitrary models.

The regression test also checks known analytic ray depths/misses, hidden-prism
pruning, duplicate tie-breaking, randomized slopes and 10,000 rays, grid-size
independence, concurrent queries, clearing and moving a slicer.
All 18 CTest tests pass with each backend selected (CUDA 35.55 s, CPU 33.07 s).
The small roughing CLI run also succeeds with `CUDA_VISIBLE_DEVICES=''` and
the CPU backend, producing the same G-code as the other small-jig runs.

Full saved-job comparisons use snapshots of the current tool/material libraries,
and copies of `test2.json` and `allowance_remove_test.json` with only their moved
model paths repaired. Originals are untouched. Raw runs and copied inputs are
under `/tmp/tinglecam-slicer-benchmark/` in this session.

Median complete CLI runtimes, including model loading, planning and export:

| Job | CPU | CUDA | G-code comparison |
| --- | ---: | ---: | --- |
| Small roughing jig | 3.82 s | 4.16 s | Byte-identical across all six runs |
| `test2` | 15.51 s | 15.79 s | Stable per backend; cleanup paths differ |
| `allowance_remove_test` | 10.29 s | 10.51 s | Byte-identical across all six runs |

Planning alone was almost unchanged: CPU/CUDA 3.750/3.785 s for the small jig,
10.440/10.454 s for `test2`, and 5.656/5.648 s for finishing only. Complete-process
totals include import and backend initialization outside the planner.

`test2` is **not byte-identical between backends**: each backend repeats its own
G-code consistently, but some roughing cleanup paths differ. Both cut 160 of
169 passes, report 24.4 minutes and the same zero gouge/plunge/entry diagnostics;
CPU reports 28,777 mm of cutting versus CUDA's 28,766 mm. This requires further
geometric investigation before treating the CPU path as an interchangeable
production replacement. The benchmark does not alter the CUDA predicates to
force agreement.

The CPU backend now serves as the portability fallback, with the above output
compatibility difference still documented. GPU acceleration remains useful for
large ray batches. A portable GPU query backend alone would leave the dominant
clipping/union cost of dense slicing on the CPU.

## Automatic fallback validation

The CUDA-enabled build and a fresh `TINGLECAM_WITH_CUDA=OFF` build each pass all
19 CTest tests, including automatic selection with `CUDA_VISIBLE_DEVICES=''` and
failure of an explicit CUDA request when unavailable. The CPU executable has no
CUDA link libraries or CUDA shared-library dependencies. The benchmark also
builds and runs in CPU-only mode.

Configuration succeeds with an absent CUDA compiler (simulated with
`CMAKE_CUDA_COMPILER=NOTFOUND`) and with a toolkit but no visible CUDA device.
The latter uses the compiler's default architecture rather than requiring a
native GPU on the build host. A full roughing/export run from the CUDA build
with its GPU hidden produces byte-identical G-code to the CPU-only executable.
