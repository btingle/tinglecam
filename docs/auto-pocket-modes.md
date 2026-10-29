# Automatic pocket detection and strategies

## Diagnosis: jig_lr_op1_rev2.obj

The supplied export contains 24 openings at both the inspection plane (Z 11)
and the top surface (Z 12), over a part spanning Z 6–12. The old convexity test
accepted only six contours. Small reverse turns where triangulated facets meet
were large relative to the very small turns of a 128-segment circle. Eighteen
otherwise circular openings were rejected as concave. Top-opening matching was
not responsible for those losses.

Convexity is now tested on a contour simplified at 0.002 mm precision. The
original, unsimplified contour is retained for tool-fit checks and machining.
Genuinely concave pockets and pockets containing islands remain unsupported.

Direct measurements of the OBJ vertices show eight approximately 4 mm holes
and sixteen approximately 3.175 mm holes, rather than 24 holes of diameter 4 mm.
Examples:

| Centre XY (mm) | Exported diameter (mm) |
| --- | --- |
| (-30, 45) | 4.000 |
| (-52.84, 22.794) | 3.175 |
| (-64.794, 10.84) | 3.175 |

The two previously detected holes rejected by a 3.175 mm cutter are from the
3.175 mm group. They have no orbital clearance for a same-size end mill.
After the detection correction, a 3.175 mm tool with sufficient flute reach
accepts eight holes in layered/continuous-helix modes, or all 24 in centre
drilling mode. The latter uses a 0.002 mm fit tolerance for mesh faceting and
integer-grid rounding; it does not permit an orbital entry in a same-size hole.

## Choosing a strategy

Enable **Auto-identify pockets**, then choose **Auto-pocket strategy** in the
operation panel or in operation defaults. Existing jobs default to **Layered
pocket**, retaining the previous behaviour.

- **Layered pocket** enters helically at the configured helix angle and clears
  each depth layer before descending further.
- **Centre drilling** feeds vertically at the opening's centroid. It excavates
  only the cutter-sized centre; a larger pocket retains material at its walls.
  Use a centre-cutting end mill or a drill. Choose straight feed, full-retract
  peck, or short-retract chip breaking. Set peck depth, short-retract distance,
  clearance above the opening, and optional bottom dwell. Peck returns approach
  the previous cut depth from above; they never rapid into the next uncut peck.
- **Continuous helix** uses one stepdown per full revolution. A partial final
  revolution reaches the exact floor without changing pitch, followed by one
  complete revolution at the floor. It requires a circular opening and enough
  orbital clearance. Openings too wide for the cutter's orbit to excavate the
  centre are excluded; use layered pocketing for those.

Drilling and continuous helix each appear as one full-depth pass per hole.
Drilling has a point marker; its final tool-sized disk is removed from the
stock model at pass completion. During-pass stock rendering is approximate.
The cutter playback and posted code carry the full drilling/helix sequence,
including peck retracts and bottom dwells. The Along pass slider seeks commanded
machine time in seconds, so vertical motion remains seekable even without XY
travel. Turning Machine time off uses a fixed speed along the complete XYZ
motion, while retaining dwell holds.

Drilling cycles expand into explicit moves rather than G81/G83 canned cycles,
so they also work with controllers without canned-cycle support. Dwell is
posted as G4 and contributes to estimated machining time. Mach3 dwell output
assumes its G04 time unit is configured as seconds.

Strategy settings are saved under `pocket_auto` in the job JSON:

```json
{
  "strategy": "drill",
  "drilling_cycle": "peck",
  "peck_depth": 1.0,
  "chip_break_retract": 0.5,
  "drilling_clearance": 1.0,
  "bottom_dwell_seconds": 0.0
}
```

Other strategies are `layered` and `continuous_helix`; other drilling cycles
are `feed` and `chip_break`. The operation's existing `stepdown` sets continuous
helix pitch. The `helix_angle_degrees` setting applies only to layered pocketing.
Exclusion counts and reasons continue to appear in planning logs.

Run `auto_pocket_modes_tests` for synthetic mode regressions, or pass the jig's
OBJ path to include the model-specific 24-hole regression. CMake registers the
jig regression automatically when this local model exists.
