`vertical_track_wall.step` is face 8 (1-based) extracted from the generated
`tracks_step_dxf/Ls.step` track. It retains the original cubic B-spline geometry,
knots and orientation, including the almost parallel rulings which caused
finishing clearance queries to spend unbounded time in OCCT extrema searches.
The standalone face is sufficient to reproduce that failure; no stock or
interior geometry is required.

`track_finish_faces.step` contains the fourteen selected finishing faces from
`step_test.json` (faces 9–22 of the same generated track). Shared native edges,
original B-splines and face orientations are preserved. The finishing regression
requires one continuous path across all contours: one initial entry, no
intermediate retract/plunge pairs, and one final exit. This catches tool-tip
distance limits, clipped transition samples and straight shortcuts across
convex profile curves.
