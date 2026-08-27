# Clipper2 contour-wave demo

This desktop demo loads the closed polygonal paths from `wavetest.svg`, creates
an allowance rectangle around their bounding box, and sends waves inward from
that rectangle using Clipper2. Subject boundaries only block waves; they never
become wave sources.

For a subject `S`, allowance rectangle `B`, engagement `e`, and a one-integer-unit
boundary seed `R0`, cumulative reached areas and visible wave bands are:

```text
R0       = (B - InflatePaths(B, -1)) - S
R(n+1)   = (InflatePaths(Rn, e) intersect B) - S
Wave 0   = R0
Wave n+1 = R(n+1) - Rn
```

This makes propagation originate exclusively at the bounding box. A front that
hits the subject is blocked there, while its unobstructed portions continue and
can advance around obstacle edges. The loop stops when the cumulative reached
area is canonically identical to the previous pass, becomes empty, or reaches
the 2000-generation safety limit. Every non-empty wave band is retained for
visualization.

## Build and run

SDL2 and OpenGL development packages are required. On Ubuntu/Debian they are
available as `libsdl2-dev` and `libgl-dev`. CMake fetches pinned versions of
Clipper2 and Dear ImGui.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/clipper2_wave_demo
```

An alternate SVG can be passed as the first argument. The focused parser accepts
closed `M`, `L`, `H`, `V`, and `Z` path commands and applies `scale`, `translate`,
`matrix`, and `rotate` transforms. Curve commands are intentionally rejected
instead of silently approximated.

## Controls

- Drag **Allowance** or **Engagement** to recompute immediately.
- Use **Generation** to inspect a pass, or animate through passes.
- Toggle the subject, allowance box, or complete generation overlay.
- Toggle **Show leading fronts** to display the previous front in violet and
  the current front in green. Open-front endpoints are drawn as dots.
- Toggle **Show parent links** to display the ancestry between consecutive
  leading-front components.
- Use **Detected events** to jump directly to splits, merges, and termination.
- Right-drag the canvas to pan; use the mouse wheel to zoom.

Geometry is converted to 64-bit integers at 1000 integer units per transformed
SVG unit before it is sent to Clipper2.

## Leading fronts and lineage

The analyzer retains every cumulative reached set `R[k]` in addition to the
visible generation band `W[k] = R[k] - R[k-1]`. It computes unreached free space
as:

```text
U[k] = (allowance_box - subject) - R[k]
```

Every boundary segment of `W[k]` is classified with small normal probes. A
segment is leading exactly when one side is in `U[k]`; segments coincident with
the subject or allowance box are excluded. Consecutive leading segments are
retained as maximal open or closed curves.

For parentage, each front from generation `k-1` is inflated by Engagement and
intersected with the actual `k` generation band. Current leading curves are
clipped against that contribution to establish graph edges. The resulting
bipartite transition graph classifies:

- one parent to several children as a split;
- several parents to one child as a merge;
- a parent with no child as a termination;
- a child with no parent as an unexpected birth.

This is an exact description of the discrete generations, subject only to a
small integer tolerance at coincident boundaries. Events that both begin and end
within one engagement step remain below the temporal resolution of the graph.
