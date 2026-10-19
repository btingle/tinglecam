# STEP finishing: Ls track, contour links and CAD clearance

Current status (2026-10-17): finishing now generates continuous surface contours
without collision avoidance. The collision timings and intermediate retract
reductions below document earlier experiments. See **Geometric finishing** at
the end for the current result.

Measured 2026-10-16 on the local Release build with OCCT 7.6.3 and the default
limit of eight planning workers. Before/after probes and a confirmation run; local tool `6mm-ball`,
machine `grbl-router`, material `hdpe`, 0.3 mm surface stepover, default 0.01 mm
chord tolerance, 0.1 mm stock grid, zero box margin. STEP fingerprint:
`a224964828271c4b`. Face numbers below are 1-based.

| Selection | CAD contours/linking | XYZ stock simulation | Full retracts | Seam joins |
| --- | ---: | ---: | ---: | ---: |
| Face 16, before | 91.472 s | 0.267 s | 127 | 0 |
| Face 16, after (two probes) | 4.326–5.630 s | 0.269–0.293 s | 1 | 0 |
| Faces 15 + 16, after (two probes) | 10.741–16.903 s | 0.552–0.832 s | 1 | 127 |

Face-16 contour generation improved by about 16–21 times. Loading and posting
are outside these stage times; complete process wall times were 92.71 s before,
5.77–7.77 s for face 16 after, and 12.40–19.61 s for both faces after. Another
TingleCAM process was active during the confirmation probes; these are not
isolated statistical benchmarks. Both after probes produced byte-identical
G-code for each selection. Peak RSS for face 16 was approximately 239 MiB before
and 241–246 MiB after. Small allocation differences vary by run.

## Findings

The slow stage was native CAD interference checking, before stock simulation.
A probe took approximately 0.12 s for one original full-model cutter sweep,
compared with 0.086 s for 10,000 surface evaluations. B-spline ball/shaft
distances dominated; repeatedly constructing the solid classifier also added
work. The faster face-16 run reported 78,910 surface samples, 8,508 sweep calls,
0.660 s sampling work and 28.234 s collision work. These work times sum across
workers; linking took 0.584 s wall time, overlapping its own collision work.

Each imported B-spline is decomposed exactly at knot spans into rational Bezier
patches with positive weights. Control hulls and local subdivision provide
conservative clearance certificates for the ball-centre segment and swept shaft
axis quad. Uncertain cases use native distance queries; possible collisions
against untrimmed patches fall back to the original trimmed face. Analytic
faces, free edges/vertices, unselected faces and solid containment are retained.
Solid classification reuses a thread-local workspace. No display mesh is used
as the clearance authority.

Retracts were unconditional per contour, rather than caused by rejected short
links. The planner now matches compatible contour endpoints on native adjacent
faces, assembles continuous lanes, and chooses short collision-checked feed
connectors between them. A blocked or long connector keeps the clearance-plane
route. One-way machining retains the requested contour direction. New links
also protect the normal stock allowance.

For face 16, **all 8,128 original nonvertical cutting segments remain**, allowing
reversed direction, with exactly 126 additional feed connectors. The existing
130 inaccessible/colliding sample diagnostic is unchanged. Faces 15 and 16
produce 127 continuous lanes, each spanning both faces, with 126 stay-down
links. The example's estimated machining time drops from about 30.1 to 2.7
minutes for face 16, largely by eliminating repeated slow plunges. That estimate
depends on the configured feeds and incoming stock.

## Reproduction

The STEP fixture is generated user geometry and must be present locally.
Tool IDs below refer to the local configuration.

```sh
./build/tinglecam --part tracks_step_dxf/Ls.step --box-stock 0 \
  --mode surface --face 16 --tool 6mm-ball --machine grbl-router --material hdpe \
  --surface-stepover 0.3 --simulation-pitch 0.1 --profile-planning \
  --gcode /tmp/ls-face16.nc
```

Repeat with `--face 15 --face 16` to exercise the common edge. In the GUI select
both faces in one surface-finishing operation with **Follow V direction** and
**Bidirectional passes** enabled, then replan.

Regression coverage includes independent full-model OCCT comparisons against
72 rational NURBS cutter sweeps with imported transformations; sewn face joins;
one-way cutting; motion continuity; allowance-preserving links; rejected links
around an unselected obstruction; free edge/vertex obstacles; trims; cancellation;
and XYZ stock conservation, playback and posting.

All 16 CTest targets pass. The mesh-only core also builds successfully.

## Vertical-wall stall regression

Selecting the outer curved vertical wall (face 8) exposed a failure in the hull
clearance shortcut. Its segment-only closest-point update zigzagged on tall,
nearly parallel shaft/surface hulls, then entered pathological OCCT extrema
searches on tiny B-spline patches. A debugger showed the workers in native
curve/curve and curve/surface global optimization. The isolated faces-7-and-8
reproduction did not finish within a 90-second limit.

The shortcut now tries certified surface-normal separating directions and uses
a bounded GJK closest-simplex update, including segment, triangle and tetrahedron
cases. Acceptance still requires the same explicit support-plane inequality
against every query vertex and Bezier control pole. Native checks remain for
ambiguous cases; the geometric clearance tolerance is unchanged.

The original face 8 extracted into a 16 KiB standalone STEP fixture reproduces
the problem without the rest of the model. The old binary failed to finish
within 12 seconds; the fixed binary generated its contours in 0.333 s and
simulated stock in 0.024 s. All 33 contours were retained with 32 stay-down links,
one final retract and no inaccessible-area diagnostic. The automated regression
also checks that ball penetration is rejected and has a 30-second CTest timeout.

Using the user's `step_test.json`, with faces 7, 8, 23 and 24 added to its original
finishing selection, roughing plus finishing planning completed in 29.920 s.
The save itself was left unchanged. CLI posting then refused the existing mix of
`makera-z1` and `grbl-router` operations; that is separate from the successful
planning result. Reproduction logs are `/tmp/step-test-all-vertical.log` and
`/tmp/vertical-wall-actual-after.log` for this session.

## Reusing clearance work

The next profile separated containment, boundary proofs and exact distances.
Of measured clearance worker time, approximately 73% was repeated solid-centre
classification, 21% native distance queries and 6% bounds/hull work. Counters for
the expanded finishing selection are:

| Quantity | Before | After |
| --- | ---: | ---: |
| Containment tests | 22,180 | 1,050 |
| Reused outside classifications | 0 | 20,258 |
| Exact native distance queries | 7,347 | 2,323 |
| Vertical approach/departure tests | 950 | 78 |
| Rejected cutting segments | 66 | 66 |
| Downward-normal samples skipped | 812 | 812 |
| Seam joins / stay-down links / retracts | 235 / 201 / 39 | 235 / 201 / 39 |

The outside-centre cache uses exact coordinates and immutable model identity.
An outside start plus a positively clear ball-centre sweep certifies its end as
outside. Every radius/flute/allowance-specific boundary sweep is still checked.
Failed sweeps and ON/UNKNOWN classifications cannot certify their far endpoints.
Tests cover connected reuse, boundary crossings, buried endpoints after failed
sweeps, changed tool size and model changes at identical coordinates.

Bezier refinements cache control hulls, normals and native patch faces using
thread-safe one-time initialization, with per-model node/pole budgets. Further
hull refinement before native fallback reduces exact-query work. Stepover
subdivision reuses shared physical sections, and contour trimming reuses the
midpoint classifications already computed by adaptive sampling.
The shaft sweep includes the ball-centre sweep at its lower edge, so a single
same-radius body query covers both. A separate ball query remains for the
near-vertical approximation with tiny nonzero XY drift; regression cases compare
both clear and colliding moves against the original independent predicate.

Approaches are checked after stitching, only at actual entry/exit points. A
blocked exit can feed back along the verified path to the last clear entry.
Regressions check this against independent native distances, including one-way
cuts, reversed surface parameters and completely inaccessible one-way entries.
Diagnostics now separate the 66 rejected segments from 812 downward-normal
samples, and report approach checks, skipped chains and escape retraces.

For a full posted comparison, a temporary copy of the supplied job selects all
four vertical faces and uses `grbl-router` for both operations so one file can
be posted. The user's save is not modified. The matched run produced identical
129,039-line G-code (SHA256
`dedc38308e90ca9f3ac3bff546b2ef7ea27e54d965c623f3bdc579a0e5124366`),
with process time improving from 29.14 to 21.45 s. Planning time improved from
27.81 to 20.27 s; peak RSS was approximately 578 MiB before and 583 MiB after.
These are individual probes, subject to system load. The new per-operation
profile attributes 16.27 s to roughing and 4.01 s to finishing (3.48 s contours,
0.53 s stock simulation). Surface evaluations dropped from 299,734 to 209,464.
A confirmation run produced the same G-code and query counts in 25.40 s, with
18.47 s roughing and 4.55 s finishing, illustrating the wall-time variability.

An isolated faces-15-and-16 comparison against the binary immediately before
these changes reduced contour generation from 6.361 to 2.183 s. Total planning
fell from 6.909 to 2.780 s (stock simulation 0.548 to 0.597 s); process wall time
was 7.99 s before and 4.35 s after. Both produced identical 16,660-line G-code
(SHA256 `82d0a2bd5732489eb48c391f271627ed1195637cb20b1548c7fe05c02862c201`),
127 seam joins, 126 stay-down links and one retract. These measurements use the
same arguments as the reproduction above, with both faces selected.

## Convex-edge turnarounds

On 2026-10-17 the saved `step_test.json` finishing selection (14 faces, without
the outer vertical walls) still produced 37 retracts with checks on and 31 with
**Ignore collisions** enabled. Its 0.3 mm surface stepover gave a 0.6 mm tool-tip
link limit, while ball compensation over convex profile edges produced adjacent
tool-tip gaps around 0.78 mm. The unchecked endpoint graph had 31 disconnected
components at that limit; ordering alone could not join them.

Endpoint searches now also use contact spacing on the same or topologically
adjacent face. This reduces the unchecked case to one final retract. With CAD
checks enabled, straight chords across the convex edges genuinely intersect the
face. The linker now tries an adaptive UV connector on the same native face
when its straight connector fails. Each resulting feed segment still protects
the complete model and the normal stock allowance; trims and downward-facing
samples reject the alternative, and unavailable alternatives retain retracts.

| Saved finishing selection | Before | After |
| --- | ---: | ---: |
| Retracts, checks enabled | 37 | 11 |
| Stay-down links, checks enabled | 137 | 163 |
| Retracts, collisions ignored | 31 | 1 |
| Stay-down links, collisions ignored | 138 | 168 |
| Rejected cutting segments, checks enabled | 66 | 66 |
| Cutting segments, checks enabled | 21,997 | 21,997 |
| Cutting segments, collisions ignored | 22,063 | 22,063 |

Both cutting-segment multisets are unchanged, allowing reversed direction.
The remaining checked retracts include transitions between different faces and
collision-split runs where a checked connector was not found; the last retract
is the final exit. The user's save was not modified. Probe sources, before/after
motion dumps and logs are in `/tmp/ls-retract-investigation/` for this session.

A standalone convex-cylinder regression uses a 3 mm-radius ball on a 1 mm fillet:
nine passes now have eight curved turnarounds and one final retract. Independent
full-model OCCT distances verify every turnaround, including an expanded cutter
to protect a 0.2 mm stock allowance. One-way passes retain their direction and
the ignored-collision case issues no native sweep checks.

## Geometric finishing

At the user's request, collision-based finishing was removed rather than kept
behind a toggle. Cutting sweeps, approach checks, rejected-segment splitting,
escape retraces and normal-Z accessibility filtering no longer affect finishing.
The obsolete toggle and clearance profile counters were removed; old job fields
load without changing the new geometric paths. Face trims still bound contours.

Every turnaround now follows the compensated native surface. On the same face,
it samples the UV curve between neighbouring passes. Across adjacent faces, the
closest shared-edge point supplies UV coordinates on both surfaces, and each
side follows its own native patch. Changes in contact normal are sampled around
the offset contour at that edge. Chord tolerance and maximum segment length
apply throughout; no straight shortcut replaces a curved profile.

The original `step_test.json` selection now produces 175 connected contours,
174 surface turnarounds, one initial entry and one final exit: **zero intermediate
retract/plunge pairs**. The original save is unchanged. Its fourteen native faces
are extracted into `tests/fixtures/cad/track_finish_faces.step` to enforce this
result automatically. Separate convex-fillet tests measure the actual offset
curve, including a shared-edge transition and normal stock allowance.

A finishing-only CLI copy of the save completed contour generation in 0.421 s
and stock simulation in 0.719 s (1.139 s operation total, 0.1 mm grid). Posting
produced 20,359 lines with no G0 moves between the initial entry and final exit.
This probe excludes roughing; the original save and its first-operation grid
pitch were not changed. The native regression and CLI output both confirm one
entry and one final retract across all 175 contours.
