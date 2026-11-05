# Ordered roughing areas

Under a roughing operation's **Cutting > Roughing areas**, add an XY box for
each part to machine. Edit its name, centre, width and height; duplicate, remove,
or move boxes up/down to set their order. Setup draws the numbered boxes over
the models. Coordinates use the same work-origin display as the other operation
controls. Removing all areas restores whole-stock roughing.

Each area completes every coarse depth and its upward terrace cleanup before
moving to the next area. All boxes share the operation's tool, depth range,
stepover, allowances and other roughing settings. There is no depth-first mode.
The tool retracts to the existing safe plane between areas. The pass selector,
Result caption and exported program comments identify the area.

Boxes contain the **full cutter**: the planner insets each rectangle by the tool
radius to obtain its cutter-centre limit. Cutting paths, cleanup approaches and
links at depth remain inside that limit. A box must be wider and taller than
the cutter. Safe-plane rapids between boxes may cross outside their limits.

All part and fixture geometry remains protected. The box bounds the wavefront's
propagation domain, rather than cropping the models or replacing the physical
stock. Existing entry and engagement rules remain in effect. Normal stock-derived
entry fronts are restricted to the area; the area boundary itself creates none.

Areas share one evolving simulation heightmap. Later areas consume the previous
area's actual remaining stock, including where boxes overlap. This also works
with stock inherited from another setup; no models are reimported or remeshed
for individual areas.

Open [the four-part example](../data/examples/four-part-roughing.json) for a plate
with four parts and an ordered box around each. It uses the bundled
`3mm-2f-alu` tool and aluminium material profile.

The optional operation JSON field is an ordered array:

```json
"roughing_areas": [
  {"name": "First part", "center_x": 7, "center_y": 7, "width": 26, "height": 26},
  {"name": "Second part", "center_x": 33, "center_y": 7, "width": 26, "height": 26}
]
```

Missing or empty arrays preserve the original whole-stock behavior. Areas do
not apply to other operation modes.
