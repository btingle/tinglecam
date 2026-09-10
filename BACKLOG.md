# Deferred usability work

Checkpoint before active-stock work: `0f0c4a0`.

The current task is active-stock pocket/plane planning and the separate-stack
preview discussed with the user. Keep the following requests for a later pass:

- Replace the operation-specific Add buttons with an operation-type dropdown
  and one Add button.
- Show pocket and plane cutting volumes in the Plan preview (translucent red
  rectangles, cylinders, and rounded pocket volumes).
- Display/edit pocket and plane coordinates relative to the selected origin.
  When the origin changes, update the coordinate values while preserving each
  operation's physical location; dimensions/depth remain unchanged.
- Preserve computed plans when parameters change. Mark the edited operation
  dirty in the operation list and downstream operations as needing replanning.
  Keep the last preview available. This combines requests 5 and 9.
- Add a wireframe mode to the Plan preview.
- Extend XYZ toolpath overlays to roughing/finishing and provide explicit
  visibility controls for cutting paths and rapid travel.
- Change plane clearing into true facing: descend outside the selected area,
  overrun its edges, and clear the complete top surface. Do not check whether
  the entry air is accessible; the user explicitly accepts responsibility for
  selecting that area. Remove the pocket-style perimeter cleanup.

## Current feature acceptance

- Loaded stock and all earlier removals determine incoming pocket/plane stock.
- Each operation has a local stock stack inside its footprint and Z range.
- Earlier stacks remain visible outside that volume and above/below it.
- Starting/ending depths within an existing slab create height-adjusted aliases.
- New machining layers can coexist with earlier layers and span several of them.
- Local stacks start with all incoming stock, including future/uncut layers.
- Cache the earlier-stack clipping; playback only updates local material.
- Overlapping operations/areas compose without duplicate stock or lost material.
