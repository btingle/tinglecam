# Continuously varying finishing heights and fixture checks

The local `setup_test1.json` case exposed a stock-simulation bottleneck after
flipping stock into a second setup. The first setup has no fixture; the second
has a roughly 100,000-vertex fixture mesh. Its last surface-finishing operation
uses a 3.175 mm ball mill. Transferred stock retains the original 0.02 mm grid,
even though the second setup requests 0.1 mm.

A bounded debugger run interrupted the final simulation after four seconds.
The active stack was:

```text
ClipperBase::BuildIntersectList
MeshSlicer::slice
JobGeometry::fixture_keep_out
JobGeometry::fixture_clear
simulate_operation_xyz
```

Every distinct finishing Z value missed the fixture-outline cache, requiring a
whole-mesh downward-envelope slice, polygon union, radius offset, and union of
protected regions. Roughing amortized those outlines across fixed-height moves;
continuous finishing did not. This work also retained a new outline for every
height. The flipped heightmap's bottom samples were not the identified bottleneck.

Motion validation now uses a shared, lazy BVH over fixture triangles. Each sweep
visits only nearby geometry, clips candidate triangles at the requested height,
and tests their convex XY projections against the swept cutter footprint. It
keeps the downward-envelope constraint and a small conservative numerical guard.
There is no Z quantization, heightmap resampling, or global polygon construction
in the motion check. Roughing still uses cached global outlines for planning.

Before contact reporting replaced the abort, one Release run of the last operation reached an existing fixture-clearance
violation after 2.81 seconds of simulation. Its total operation time was 4.10
seconds. The earlier surface operation's simulation took 7.09 seconds. These are
not timings for a successfully completed last operation: the last one is rejected
when it encounters the fixture. The old polygon check independently confirmed
the same violation for this move, in setup coordinates:

```text
(-20.087, 6.437, 12.751432) -> (-19.991, 6.413, 12.396686)
```

A focused comparison on that fixture and move, with small changes in height,
took 560.9 ms for 12 old polygon checks and 13.0 ms for 12,000 warm indexed checks.
Both methods classified every tested move as blocked. The whole-job run took
about 34.5 seconds and exported the valid earlier operations while reporting
that the final operation was skipped.

For a bounded local rerun with the job's matching tool and material libraries:

```sh
timeout --signal=INT --kill-after=5s 45s ./build/tinglecam \
  --job setup_test1.json --profile-planning --gcode /tmp/setup-test1.nc
```

Regression coverage checks sloped and vertical fixtures, tangency, crossing
sweeps with clear endpoints, all 24 cube orientations with translations,
concurrent queries, and agreement with an independent polygon oracle away from
the integer-rounding boundary band. Another test verifies that 1,000 varying-Z
motion checks leave the global fixture-outline cache unchanged.

With contact reporting enabled, a subsequent bounded Release run completed all
four operations in about 39.4 seconds, with no skipped operations. The last
operation's simulation took 7.24 seconds (the earlier surface operation took
7.44 seconds) and recorded 226 contacting motions. Contacts are localized to
intervals within 0.05 mm of tool travel, with positions in setup coordinates;
playback changes the tool colour only within those intervals. No fixture outline
is rebuilt by this refinement. The run used a 15-second limit on the final
simulation stage and exited normally.
