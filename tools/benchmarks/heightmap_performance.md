# Finishing and heightmap performance investigation

Measured on 2026-10-16, Release build, native OCCT 7.6.3, default planning worker
limit (up to eight). These are single wall-time/RSS probes, not statistical
medians. Measurements include model loading and posting. The baseline was the
previous STEP/heightmap implementation saved as `build/tinglecam-before-performance`.

## Results

| Case | Before | After | Before peak RSS | After peak RSS |
| --- | ---: | ---: | ---: | ---: |
| Straight STEP track, face 8, 2 mm stepover, 0.1 mm grid | 45.40 s | 0.61 s | 189 MiB | 199 MiB |
| Curved STEP track, face 8, 4 mm stepover, 0.1 mm grid | 53.21 s | 5.22 s | 191 MiB | 204 MiB |
| LsJig roughing, 0.1 mm grid | 18.27 s | 12.73 s | 983 MiB | 540 MiB |
| LsJig roughing, 0.025 mm grid | — | 13.45 s | — | 545 MiB |
| Straight track, 0.3 mm stepover, 0.025 mm grid | — | 5.76 s | — | 225 MiB |

All three matching before/after cases produced **byte-for-byte identical G-code**.
The finer grid has sixteen times as many XY cells. Grid pitch can change
engagement/feed estimates, so fine-grid G-code is not expected to match coarse.
STEP finishing peak RSS increased slightly with independent worker workspaces.

## Causes and changes

Finishing spent most of its time reconstructing native CAD distance-query
workspaces and checking every short sampled segment separately. Each worker now
reuses its model-distance workspace. Groups of sufficiently straight segments
share a continuous native sweep query, tightened by their maximum deviation
from the chord; failed groups split recursively. The complete CAD model still
protects the ball and shaft. Independent contours run in parallel and gather in
the original deterministic order. Progress callbacks remain on the coordinator.

Stock resolution previously amplified snapshot directory copies, dense arrays
for untouched flat stock, duplicate preliminary simulation histories, repeated
full sweep bounding-box scans and full-volume queries. Stock now uses paged
copy-on-write directories, scalar/repeated-row/repeated-column height storage,
tile maxima, restricted diagonal sweep traversal, precomputed analytic cutter
kernels and an incrementally maintained volume. Compression preserves every
stored float height. Playback history is bounded to at most ten snapshots per
layer/contour; preliminary histories are disabled and feed samples released
once authoritative XYZ phases are available. Motion/feed vectors reserve their
required capacities.

Stock polygon queries previously revisited older operation snapshots even when
the latest snapshot already included those cuts. Queries now start from the
newest composed snapshot. Each immutable grid caches its polygon per exact Z;
planar subdivision work continues against its own Clipper polygons. Wavefront
slicing, propagation and path topology are unchanged. The GUI no longer extracts
2D stock polygons on every 3D playback frame, advances only new motion during
forward playback, and reuses unchanged display meshes.

The subsequent native OpenGL heightmap renderer supersedes that display-mesh
path: stock now renders at full simulation resolution through GLSL raycasting,
with changed-tile uploads and a shared depth buffer for models and the cutter.
The planning measurements in this report are unaffected by that display change.

## Where memory goes

For the default roughing probe after planning:

| Retained payload estimate | Size |
| --- | ---: |
| Shared heightmap history, 183 checkpoints | 8.1 MiB |
| Motion and feed vectors | 70.3 MiB |
| Wavefront, lineage and path geometry | 66.6 MiB |
| Saved planning polygons | 5.2 MiB |
| Preliminary sampled feed history | 0 |

At 0.025 mm the grid history rises to 36.5 MiB; the fine finishing probe retains
17.9 MiB. The counters deduplicate shared grid pages/tiles/arrays but exclude
allocator overhead and cached polygon payload. Wave/path counts omit some
container/topology metadata; these are useful attribution estimates, not a full
heap census. CPU model meshes, CUDA buffers/BVH, OCCT, graphics, libraries and
thread stacks also contribute. The GUI deliberately preserves an old preview
while a replacement plan runs, temporarily retaining both plans.

The roughing process ended near **338 MiB resident** after a 540 MiB peak.
Clipper/worker scratch left substantial freed allocator arenas; glibc builds now
return idle pages at the completed operation-range boundary when free heap
exceeds 64 MiB. This happens after workers join, preserving live plans.

Virtual address reservations are much larger than physical use: this imported
model run reported roughly 57 GiB `VmSize`, versus 338 MiB `VmRSS`. A tiny
model-free pocket run reported about 60 MiB virtual / 23 MiB resident. The large
reservation appears with model/GPU initialization; it should not be interpreted
as that amount of heightmap RAM. Check resident memory and its peak when
assessing actual pressure.

The remaining roughing cost is principally existing wavefront/Clipper work and
routing; the final XYZ grid simulation took about 0.57 s at 0.1 mm and 3.14 s at
0.025 mm. Curved finishing still spends most of its geometry time in native
CAD interference checks. Arbitrarily high resolution still increases cut work
and changed-tile storage; the grid guard is now 128 million cells.

## Reproduction and validation

The script isolates application configuration by default, using shipped tool
IDs. Generated track STEP files must exist or finishing cases are skipped:

```sh
python3 tools/benchmarks/heightmap_performance.py \
  --baseline build/tinglecam-before-performance \
  --output /tmp/tinglecam-heightmap-benchmark --runs 3
```

For the exact local tool configuration used above:

```sh
python3 tools/benchmarks/heightmap_performance.py \
  --use-user-config --ball-tool 6mm-ball --rough-tool 3.175mm-1f \
  --output /tmp/tinglecam-heightmap-benchmark
```

Each case writes G-code, its SHA256, process timing/RSS, planner/memory counters
and logs, plus `results.csv`. Roughing uses `tests/fixtures/meshes/LsJig1.obj`,
5 mm box-stock margin, 3 mm stepdown, 0.2 stepover, four finishing subdivisions
and 0.2 mm path pitch. Finishing uses the generated `tracks_step_dxf/s.step`
and `L.step`, face 8, a 6 mm ball, and zero box margin.

Validation includes analytic cutter sweeps against independent dense pose
sampling, optimized tile traversal against full-cell traversal, immutable
snapshots, a 100-million-cell/100-snapshot memory check, lossless pattern
compaction, one polygon conversion per stock version/Z, composed-operation
stock queries, canonical arc playback and forward-playback/checkpoint agreement.
Native finishing tests exercise trims, obstruction checks, normals, progress
and XYZ posting. All fourteen CTest targets pass. The mesh-only core also builds;
a fine-grid finishing result screenshot was rendered and inspected.
