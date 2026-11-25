// Marble track jigs. Deployed in the same Feature Studio as marble_track.fs (concatenated after it),
// so the track equations (trackSpec, pointPath, samplePath, curveBase) are shared.
//
// Layout frame ("glue frame"): world XY, looking down on the glue-up stack, fixture-hole (track top)
// side up. All 2D geometry is plain mm numbers; affine maps are [a, b, d, e, xo, yo] (x' = a x + b y + xo).

export enum JigSpecies
{
    annotation { "Name" : "lr (L, R)" }
    LR,
    annotation { "Name" : "slr (sL, sR)" }
    SL_SR,
    annotation { "Name" : "slr (Ls, Rs)" }
    LS_RS,
    annotation { "Name" : "slr (sL, sR, Ls, Rs)" }
    SLR,
    annotation { "Name" : "sSs" }
    SSS,
    annotation { "Name" : "RL (RL, LR)" }
    RL
}

// models: per ring corner (or row entry), the 3D track it holds and `pre`, taking that track's plan into the
// base track's plan. Ls/Rs share sR/sL outlines (Ls = sL mirrored in plan, climbing the other way), so the
// layout is computed for the base alone; labels follow from each slot's handedness.
function speciesInfo(species is JigSpecies) returns map
{
    const sl = [TrackType.SL, IDENTITY2];
    const ls = [TrackType.LS, MIRROR_X];
    if (species == JigSpecies.LR)
        return { "name" : "lr", "base" : TrackType.L, "copies" : 2, "models" : makeArray(4, [TrackType.L, IDENTITY2]) };
    if (species == JigSpecies.SL_SR)
        return { "name" : "sL-sR", "base" : TrackType.SL, "copies" : 1, "models" : [sl, sl, sl, sl] };
    if (species == JigSpecies.LS_RS)
        return { "name" : "Ls-Rs", "base" : TrackType.SL, "copies" : 1, "models" : [ls, ls, ls, ls] };
    if (species == JigSpecies.SLR)
        return { "name" : "slr", "base" : TrackType.SL, "copies" : 1, "models" : [sl, sl, ls, ls] };
    // grids: one piece per entry, row by row, `cols` per row, each flipped about its own centre.
    // RL: symmetric about both centrelines (mirrored entries are LR).
    if (species == JigSpecies.RL)
        return { "name" : "lrrl", "base" : TrackType.CURVE_RL, "copies" : 4, "cols" : 2, "models" : makeArray(4, [TrackType.CURVE_RL, IDENTITY2]),
                "flips" : [MIRROR_X, IDENTITY2, ROT180, MIRROR_Y] };
    return { "name" : "sSs", "base" : TrackType.SSS, "copies" : 4, "cols" : 4, "models" : makeArray(4, [TrackType.SSS, IDENTITY2]),
            "flips" : [IDENTITY2, IDENTITY2, IDENTITY2, IDENTITY2] };
}

// The track a slot holds: its model, or the model's mirror image when the slot's map mirrors it.
function slotTrack(model is array, m is array) returns string
{
    const t = compose(m, model[1]);
    return trackName(t[0] * t[3] - t[1] * t[2] < 0 ? mirrorTrack(model[0]) : model[0]);
}

function mirrorTrack(kind is TrackType) returns TrackType
{
    const pairs = [[TrackType.L, TrackType.R], [TrackType.SL, TrackType.SR], [TrackType.LS, TrackType.RS],
            [TrackType.SLR, TrackType.SRL], [TrackType.CURVE_RL, TrackType.CURVE_LR]];
    for (var pr in pairs)
    {
        if (kind == pr[0])
            return pr[1];
        if (kind == pr[1])
            return pr[0];
    }
    return kind;
}

// One bound per default value: the UI (and API adds without a value) take a parameter's default from its bound.
const MM_0 = { (millimeter) : [0.0, 0.0, 50.0] } as LengthBoundSpec;
const MM_0_1 = { (millimeter) : [0.0, 0.1, 50.0] } as LengthBoundSpec;
const MM_0_15 = { (millimeter) : [0.0, 0.15, 50.0] } as LengthBoundSpec;
const MM_0_3 = { (millimeter) : [0.0, 0.3, 50.0] } as LengthBoundSpec;
const MM_0_5 = { (millimeter) : [0.0, 0.5, 50.0] } as LengthBoundSpec;
const MM_0_75 = { (millimeter) : [0.0, 0.75, 50.0] } as LengthBoundSpec;
const MM_1 = { (millimeter) : [0.0, 1.0, 50.0] } as LengthBoundSpec;
const MM_1_5 = { (millimeter) : [0.0, 1.5, 50.0] } as LengthBoundSpec;
const MM_3_175 = { (millimeter) : [0.0, 3.175, 50.0] } as LengthBoundSpec;
const MM_3_4 = { (millimeter) : [0.0, 3.4, 50.0] } as LengthBoundSpec;
const MM_3_5 = { (millimeter) : [0.0, 3.5, 50.0] } as LengthBoundSpec;
const MM_4 = { (millimeter) : [0.0, 4.0, 100.0] } as LengthBoundSpec;
const MM_4_5 = { (millimeter) : [0.0, 4.5, 50.0] } as LengthBoundSpec;
const MM_5 = { (millimeter) : [0.0, 5.0, 100.0] } as LengthBoundSpec;
const MM_5_5 = { (millimeter) : [0.0, 5.5, 50.0] } as LengthBoundSpec;
const MM_6 = { (millimeter) : [0.0, 6.0, 100.0] } as LengthBoundSpec;
const MM_6_35 = { (millimeter) : [0.0, 6.35, 100.0] } as LengthBoundSpec;
const MM_6_5 = { (millimeter) : [0.0, 6.5, 50.0] } as LengthBoundSpec;
const MM_7 = { (millimeter) : [0.0, 7.0, 100.0] } as LengthBoundSpec;
const MM_8 = { (millimeter) : [0.0, 8.0, 100.0] } as LengthBoundSpec;
const MM_8_2 = { (millimeter) : [0.0, 8.2, 50.0] } as LengthBoundSpec;
const MM_12 = { (millimeter) : [0.0, 12.0, 200.0] } as LengthBoundSpec;
const MM_16 = { (millimeter) : [0.0, 16.0, 200.0] } as LengthBoundSpec;
const MM_20 = { (millimeter) : [0.0, 20.0, 200.0] } as LengthBoundSpec;
const MM_150 = { (millimeter) : [1.0, 150.0, 1000.0] } as LengthBoundSpec;
const MM_PLACE = { (millimeter) : [-5000.0, 0.0, 5000.0] } as LengthBoundSpec;

// ---------------------------------------------------------------------------------------------------
// Features

annotation { "Feature Type Name" : "Marble glue-up jig" }
export const marbleGlueUpJig = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Profile region", "Filter" : EntityType.FACE && SketchObject.YES && ConstructionObject.NO, "MaxNumberOfPicks" : 1 }
        definition.profile is Query;
        annotation { "Name" : "Species" }
        definition.species is JigSpecies;
        if (definition.species == JigSpecies.RL)
        {
            annotation { "Name" : "Arrange as an X (otherwise an O)" }
            definition.glueX is boolean;
        }
        jigCommonPrecondition(definition);
        annotation { "Name" : "Acrylic thickness" }
        isLength(definition.acrylic, MM_6);
        annotation { "Name" : "Web between blanks" }
        isLength(definition.web, MM_8);
        annotation { "Name" : "Guide clearance" }
        isLength(definition.guideClearance, MM_0_15);
        annotation { "Name" : "M5 bolt hole" }
        isLength(definition.boltD, MM_5_5);
        annotation { "Name" : "M5 nut across flats (+clearance)" }
        isLength(definition.nutAF, MM_8_2);
        annotation { "Name" : "Extra outer wall" }
        isLength(definition.glueWall, MM_4);
        annotation { "Name" : "Show wood blanks" }
        definition.showBlanks is boolean;
        placementPrecondition(definition);
    }
    {
        const layout = jigLayout(context, definition);
        buildGlueUp(context, id, definition, layout);
        placeResult(context, id, definition);
    }, jigDefaults());

annotation { "Feature Type Name" : "Marble op0 plate" }
export const marbleOp0Plate = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Profile region", "Filter" : EntityType.FACE && SketchObject.YES && ConstructionObject.NO, "MaxNumberOfPicks" : 1 }
        definition.profile is Query;
        annotation { "Name" : "Species" }
        definition.species is JigSpecies;
        if (definition.species == JigSpecies.RL)
        {
            annotation { "Name" : "Arrange as an X (otherwise an O)" }
            definition.glueX is boolean;
        }
        jigCommonPrecondition(definition);
        annotation { "Name" : "Web between blanks" }
        isLength(definition.web, MM_8);
        annotation { "Name" : "Guide clearance" }
        isLength(definition.guideClearance, MM_0_15);
        annotation { "Name" : "M5 bolt hole" }
        isLength(definition.boltD, MM_5_5);
        annotation { "Name" : "Plate margin" }
        isLength(definition.op0Margin, MM_12);
        annotation { "Name" : "M4 clearance hole" }
        isLength(definition.op0BoltD, MM_4_5);
        // the plate is as thick as the screw allows: its tip stops short of the far side of the hole sheet
        annotation { "Name" : "M4 screw length (under the head)" }
        isLength(definition.op0ScrewL, MM_16);
        annotation { "Name" : "Wood left past the screw tip" }
        isLength(definition.op0TipGap, MM_0_75);
        annotation { "Name" : "Counterbore diameter" }
        isLength(definition.op0CboreD, MM_8);
        annotation { "Name" : "Screw head height" }
        isLength(definition.op0HeadH, MM_4);
        annotation { "Name" : "Head recess" }
        isLength(definition.op0Recess, MM_0_5);
        annotation { "Name" : "Ball-nose trough width" }
        isLength(definition.op0TroughW, MM_7);
        annotation { "Name" : "Ball-nose trough depth" }
        isLength(definition.op0TroughD, MM_3_5);
        annotation { "Name" : "Stock above the track top" }
        isLength(definition.stockAbove, MM_0);
        modelPrecondition(definition);
        placementPrecondition(definition);
    }
    {
        const layout = jigLayout(context, definition);
        buildOp0(context, id, definition, layout);
        placeResult(context, id, definition);
    }, jigDefaults());

predicate trackPrecondition(definition is map)
{
    annotation { "Name" : "Grid size" }
    isLength(definition.gridSize, GRID_BOUNDS);
    annotation { "Name" : "Elevation gain" }
    isLength(definition.elevationGain, ELEVATION_BOUNDS);
}

predicate jigCommonPrecondition(definition is map)
{
    trackPrecondition(definition);
    stackPrecondition(definition);
}

// The glued blank stack (glue-up, op0, and op1's stock).
predicate stackPrecondition(definition is map)
{
    annotation { "Name" : "Wood sheet thickness" }
    isLength(definition.sheet, MM_6_35);
    annotation { "Name" : "Laser allowance" }
    isLength(definition.allowance, MM_1_5);
    annotation { "Name" : "Fixture hole diameter" }
    isLength(definition.fixtureD, MM_3_175);
}

function jigDefaults() returns map
{
    return {
            "species" : JigSpecies.LR,
            "glueX" : false,
            "gridSize" : 40 * millimeter,
            "elevationGain" : 6 * millimeter,
            "sheet" : 6.35 * millimeter,
            "allowance" : 1.5 * millimeter,
            "fixtureD" : 3.175 * millimeter,
            "acrylic" : 6 * millimeter,
            "web" : 8 * millimeter,
            "guideClearance" : 0.15 * millimeter,
            "boltD" : 5.5 * millimeter,
            "nutAF" : 8.2 * millimeter,
            "glueWall" : 4 * millimeter,
            "op0Margin" : 12 * millimeter,
            "op0TroughW" : 7 * millimeter,
            "op0TroughD" : 3.5 * millimeter,
            "op0BoltD" : 4.5 * millimeter,
            "op0ScrewL" : 16 * millimeter,
            "op0TipGap" : 0.75 * millimeter,
            "op0CboreD" : 8 * millimeter,
            "op0HeadH" : 4 * millimeter,
            "op0Recess" : 0.5 * millimeter,
            "showBlanks" : true,
            "stockModel" : false,
            "partModel" : false,
            "stockAbove" : 0 * millimeter,
            "placeX" : 0 * millimeter,
            "placeY" : 0 * millimeter
        };
}

// CAM models: the stock entering the op and the part leaving it, placed as they sit in the jig.
predicate modelPrecondition(definition is map)
{
    annotation { "Name" : "Stock model" }
    definition.stockModel is boolean;
    annotation { "Name" : "Part model" }
    definition.partModel is boolean;
}

// Lets several jigs share one Part Studio without overlapping.
predicate placementPrecondition(definition is map)
{
    annotation { "Name" : "Move result X" }
    isLength(definition.placeX, MM_PLACE);
    annotation { "Name" : "Move result Y" }
    isLength(definition.placeY, MM_PLACE);
}

function placeResult(context is Context, id is Id, definition is map)
{
    if (definition.placeX != 0 * millimeter || definition.placeY != 0 * millimeter)
        opTransform(context, id + "place", { "bodies" : qCreatedBy(id, EntityType.BODY),
                    "transform" : transform(vector(definition.placeX, definition.placeY, 0 * millimeter)) });
}

// ---------------------------------------------------------------------------------------------------
// Track shape in plan (template frame, mm)

// Plan/height point of profile point (px, pz) at parameter t, for every template mode.
function pathPoint(spec is map, px is number, pz is number, t is number) returns array
{
    if (spec.mode == "extrude")
        return [px, -spec.gs / 2 + t * spec.gs, pz];
    if (spec.mode == "sweep")
        return [px, -spec.gs / 2 + t * spec.gs, pz + spec.elev * (3 * t * t - 2 * t * t * t)];
    if (spec.mode == "revolve")
        return curveBase(spec.curveLeft, spec.gs / 2, px, pz, t);
    if (isSampled(spec))
        return samplePath(pointPath(spec, px, pz), t);
    throw regenError("Track type " ~ spec.mode ~ " has no jig support yet");
}

// Profile extents in the track frame: { x0, x1, z0, z1 } (mm).
function profileExtents(context is Context, profile is Query) returns map
{
    const frame = trackFrame(context, profile);
    var e = { "x0" : inf, "x1" : -inf, "z0" : inf, "z1" : -inf };
    for (var edge in evaluateQuery(context, qAdjacent(profile, AdjacencyType.EDGE, EntityType.EDGE)))
    {
        for (var line in evEdgeTangentLines(context, { "edge" : edge, "parameters" : uniformParameters(17) }))
        {
            const d = line.origin - frame.origin;
            const px = dot(d, frame.x) / millimeter;
            const pz = dot(d, frame.z) / millimeter;
            e.x0 = min(e.x0, px);
            e.x1 = max(e.x1, px);
            e.z0 = min(e.z0, pz);
            e.z1 = max(e.z1, pz);
        }
    }
    return e;
}

// Everything the layouts need about one track type.
function trackShape(spec is map, ext is map, allowance is number, n is number) returns map
{
    var centre = [];
    var zlo = inf;
    var zhi = -inf;
    var maxBottom = -inf;
    var minTop = inf;
    for (var i = 0; i < n; i += 1)
    {
        const t = i / (n - 1);
        const c = pathPoint(spec, 0, ext.z1, t);
        centre = append(centre, [c[0], c[1]]);
        var bottom = inf;
        var top = -inf;
        for (var px in [ext.x0, ext.x1])
        {
            bottom = min(bottom, pathPoint(spec, px, ext.z0, t)[2]);
            top = max(top, pathPoint(spec, px, ext.z1, t)[2]);
        }
        zlo = min(zlo, bottom);
        zhi = max(zhi, top);
        maxBottom = max(maxBottom, bottom);
        minTop = min(minTop, top);
    }
    return { "spec" : spec, "ext" : ext, "centre" : centre, "zlo" : zlo, "zhi" : zhi, "n" : n,
            "maxBottom" : maxBottom, "minTop" : minTop,
            "ends" : [[centre[0], unit2(sub2(centre[0], planAt(spec, ext.z1, 1e-5)))],
                      [centre[n - 1], unit2(sub2(centre[n - 1], planAt(spec, ext.z1, 1 - 1e-5)))]] };
}

function planAt(spec is map, z is number, t is number) returns array
{
    const p = pathPoint(spec, 0, z, t);
    return [p[0], p[1]];
}

// Closed outline offset `off` beyond the profile sides and ends: [side A (t 0..1), side B (t 1..0)].
function outlineSides(shape is map, off is number) returns array
{
    const spec = shape.spec;
    const n = shape.n;
    var a = [];
    var b = [];
    for (var i = 0; i < n; i += 1)
    {
        const t = i / (n - 1);
        const p = pathPoint(spec, shape.ext.x0 - off, shape.ext.z1, t);
        const q = pathPoint(spec, shape.ext.x1 + off, shape.ext.z1, t);
        a = append(a, [p[0], p[1]]);
        b = append(b, [q[0], q[1]]);
    }
    // extend the ends along the path tangent
    const d0 = shape.ends[0][1];
    const d1 = shape.ends[1][1];
    a[0] = add2(a[0], scale2(d0, off));
    b[0] = add2(b[0], scale2(d0, off));
    a[n - 1] = add2(a[n - 1], scale2(d1, off));
    b[n - 1] = add2(b[n - 1], scale2(d1, off));
    return [a, reverseArray(b)];
}

function outlinePolygon(shape is map, off is number) returns array
{
    const sides = outlineSides(shape, off);
    return concatenateArrays(sides);
}

// ---------------------------------------------------------------------------------------------------
// Layout (shared by glue-up and op0 so their hole patterns always agree)

function jigLayout(context is Context, definition is map) returns map
{
    const info = speciesInfo(definition.species);
    const gs = definition.gridSize / millimeter;
    const elev = definition.elevationGain / millimeter;
    const ext = profileExtents(context, definition.profile);
    const shape = trackShape(trackSpec(info.base, gs, elev), ext, definition.allowance / millimeter, 41);
    const allow = definition.allowance / millimeter;
    const clear = definition.guideClearance / millimeter;
    const web = definition.web / millimeter;
    const boltR = definition.boltD / millimeter / 2;
    const boltWall = 3;
    const blank = outlinePolygon(shape, allow);
    const holes = fixtureHoles(shape);

    var slots = [];
    var bolts = [];
    var kind = "";
    const corner = endCorner(shape.ends);
    if (corner != undefined)
    {
        kind = "ring";
        const q = quadrantMap(blank, corner, web / 2);
        const ring = [q, compose(MIRROR_X, q), compose(MIRROR_Y, q), compose(ROT180, q)];
        const cut = function(m) { return mapPolygon(m, outlinePolygon(shape, allow + clear)); };
        var ringCuts = [];
        for (var m in ring)
            ringCuts = append(ringCuts, cut(m));
        const local = ringBolts(ringCuts, boltR + boltWall);
        const width = polyBounds(concatenateArrays(ringCuts)).x1 - polyBounds(concatenateArrays(ringCuts)).x0;
        var pitch = width + web;
        var found = false;
        for (var tries = 0; tries < 200 && !found; tries += 1)
        {
            slots = [];
            bolts = [];
            var cuts = [];
            for (var r = 0; r < info.copies; r += 1)
            {
                const shift = translation2(r * pitch, 0);
                for (var i = 0; i < 4; i += 1)
                {
                    const sm = compose(shift, ring[i]);
                    slots = append(slots, { "m" : sm, "kind" : (i == 0 || i == 3) ? "A" : "B", "model" : info.models[i][0],
                                "pre" : info.models[i][1], "track" : slotTrack(info.models[i], sm), "ring" : r });
                    cuts = append(cuts, mapPolygon(shift, ringCuts[i]));
                }
                for (var b in local)
                    bolts = append(bolts, add2(b, [r * pitch, 0]));
            }
            found = boltsClear(bolts, cuts, boltR + boltWall, definition.boltD / millimeter + 2 * boltWall);
            pitch += 1;
        }
        if (!found)
            throw regenError("No ring spacing leaves the clamp bolts clear");
    }
    else
    {
        kind = "row";
        const bb = polyBounds(blank);
        // RL: the corner piece decides whether the four S shapes form an O or an X
        const pre = definition.glueX == true ? MIRROR_X : IDENTITY2;
        const cols = info.cols;
        const rows = ceil(info.copies / cols);
        const pitchX = bb.x1 - bb.x0 + web;
        const pitchY = bb.y1 - bb.y0 + web;
        var cuts = [];
        for (var i = 0; i < info.copies; i += 1)
        {
            const base = compose(info.flips[i], pre);
            const b = polyBounds(mapPolygon(base, blank));
            const cx = (i % cols - (cols - 1) / 2) * pitchX;
            const cy = ((rows - 1) / 2 - floor(i / cols)) * pitchY;
            const m = compose(translation2(cx - (b.x0 + b.x1) / 2, cy - (b.y0 + b.y1) / 2), base);
            slots = append(slots, { "m" : m, "kind" : "A", "model" : info.models[i][0], "pre" : info.models[i][1],
                        "track" : slotTrack(info.models[i], m), "ring" : 0 });
            cuts = append(cuts, mapPolygon(m, outlinePolygon(shape, allow + clear)));
        }
        bolts = rowBolts(cuts, boltR + boltWall);
    }

    // centre the layout on the origin
    var all = [];
    for (var s in slots)
        all = concatenateArrays([all, mapPolygon(s.m, blank)]);
    const bb = polyBounds(all);
    const shift = translation2(-(bb.x0 + bb.x1) / 2, -(bb.y0 + bb.y1) / 2);
    for (var i = 0; i < size(slots); i += 1)
        slots[i].m = compose(shift, slots[i].m);
    for (var i = 0; i < size(bolts); i += 1)
        bolts[i] = mapPoint(shift, bolts[i]);

    const sheets = stackSheets(shape, definition);
    return { "kind" : kind, "shape" : shape, "slots" : slots, "bolts" : bolts, "blank" : blank, "holes" : holes,
            "allow" : allow, "clear" : clear, "sheets" : sheets, "info" : info };
}

// Intersection of the two end-face lines, or undefined when they are parallel.
function endCorner(ends is array)
{
    const p0 = ends[0][0];
    const p1 = ends[1][0];
    const f0 = [-ends[0][1][1], ends[0][1][0]];
    const f1 = [-ends[1][1][1], ends[1][1][0]];
    const det = f0[0] * (-f1[1]) - f0[1] * (-f1[0]);
    if (abs(det) < 1e-6)
        return undefined;
    const r = sub2(p1, p0);
    const s = (r[0] * (-f1[1]) - r[1] * (-f1[0])) / det;
    return add2(p0, scale2(f0, s));
}

// Rotate (never mirror) the blank so it sits in quadrant II, half a web from both axes.
function quadrantMap(blank is array, corner is array, halfWeb is number) returns array
{
    var m = translation2(-corner[0], -corner[1]);
    for (var k = 0; k < 4; k += 1)
    {
        const c = polyCentroid(mapPolygon(m, blank));
        if (c[0] < 0 && c[1] > 0)
        {
            const b = polyBounds(mapPolygon(m, blank));
            return compose(translation2(-halfWeb - b.x1, halfWeb - b.y0), m);
        }
        m = compose(ROT90, m);
    }
    throw regenError("Track outline does not fit a quadrant");
}

// Centre bolt (if it fits) + one bolt per quadrant along the ring's bounding-box diagonals.
function ringBolts(cuts is array, clearR is number) returns array
{
    const all = concatenateArrays(cuts);
    const bb = polyBounds(all);
    const c = [(bb.x0 + bb.x1) / 2, (bb.y0 + bb.y1) / 2];
    const w = (bb.x1 - bb.x0) / 2;
    const h = (bb.y1 - bb.y0) / 2;
    var bolts = [];
    if (clearance(c, cuts) >= clearR)
        bolts = append(bolts, c);
    for (var d in [[w, h], [-w, h], [-w, -h], [w, -h]])
        bolts = append(bolts, firstClear(c, d, cuts, clearR));
    return bolts;
}

function rowBolts(cuts is array, clearR is number) returns array
{
    const bb = polyBounds(concatenateArrays(cuts));
    const c = [(bb.x0 + bb.x1) / 2, (bb.y0 + bb.y1) / 2];
    const w = (bb.x1 - bb.x0) / 2;
    const h = (bb.y1 - bb.y0) / 2;
    var bolts = [];
    for (var d in [[w, h], [-w, h], [-w, -h], [w, -h], [0, 1], [0, -1]])
    {
        // both side rays stop at the centre when it is already clear: one bolt there
        const b = firstClear(c, d, cuts, clearR);
        var dup = false;
        for (var o in bolts)
            dup = dup || norm2(sub2(o, b)) < 1;
        if (!dup)
            bolts = append(bolts, b);
    }
    return bolts;
}

// First point along the ray, beyond every cutout it crosses, that clears all cutouts by clearR.
function firstClear(origin is array, dir is array, cuts is array, clearR is number) returns array
{
    const d = unit2(dir);
    var s = rayExit(origin, d, cuts);
    for (var i = 0; i < 2000; i += 1)
    {
        const p = add2(origin, scale2(d, s));
        if (clearance(p, cuts) >= clearR)
            return p;
        s += 0.25;
    }
    throw regenError("No clear clamp bolt position");
}

function rayExit(origin is array, d is array, polys is array) returns number
{
    var best = 0;
    for (var poly in polys)
    {
        const n = size(poly);
        for (var i = 0; i < n; i += 1)
        {
            const a = poly[i];
            const b = poly[(i + 1) % n];
            const e = sub2(b, a);
            const den = d[0] * e[1] - d[1] * e[0];
            if (abs(den) < 1e-12)
                continue;
            const r = sub2(a, origin);
            const s = (r[0] * e[1] - r[1] * e[0]) / den;
            const u = (r[0] * d[1] - r[1] * d[0]) / den;
            if (s > best && u >= 0 && u <= 1)
                best = s;
        }
    }
    return best;
}

function boltsClear(bolts is array, cuts is array, clearR is number, spacing is number) returns boolean
{
    for (var i = 0; i < size(bolts); i += 1)
    {
        if (clearance(bolts[i], cuts) < clearR - 1e-6)
            return false;
        for (var j = i + 1; j < size(bolts); j += 1)
            if (norm2(sub2(bolts[i], bolts[j])) < spacing - 1e-6)
                return false;
    }
    return true;
}

// Signed distance from p to a set of polygons (negative inside any of them).
function clearance(p is array, polys is array) returns number
{
    var best = inf;
    for (var poly in polys)
    {
        var d = inf;
        const n = size(poly);
        var inside = false;
        for (var i = 0; i < n; i += 1)
        {
            const a = poly[i];
            const b = poly[(i + 1) % n];
            d = min(d, segDist(p, a, b));
            if ((a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
                inside = !inside;
        }
        best = min(best, inside ? -d : d);
    }
    return best;
}

function segDist(p is array, a is array, b is array) returns number
{
    const e = sub2(b, a);
    const l2 = e[0] * e[0] + e[1] * e[1];
    var t = l2 > 0 ? ((p[0] - a[0]) * e[0] + (p[1] - a[1]) * e[1]) / l2 : 0;
    t = max(0, min(1, t));
    return norm2(sub2(p, add2(a, scale2(e, t))));
}

// ---------------------------------------------------------------------------------------------------
// Builders

function buildGlueUp(context is Context, id is Id, definition is map, layout is map)
{
    const t = definition.acrylic / millimeter;
    const sheet = definition.sheet / millimeter;
    const n = layout.sheets;
    const plate = plateRect(layout, definition);
    const boltR = definition.boltD / millimeter / 2;
    const off = layout.allow + layout.clear;

    // bottom plate: hex nut sockets
    var sk = jigSketch(context, id + "bottomSk", 0);
    sketchRoundedRect(sk, "plate", plate, 10);
    for (var i = 0; i < size(layout.bolts); i += 1)
        sketchHex(sk, "nut" ~ i, layout.bolts[i], definition.nutAF / millimeter);
    skSolve(sk);
    extrudeSketch(context, id + "bottom", id + "bottomSk", t, layout.info.name ~ " glue bottom plate", color(0.75, 0.85, 0.95));

    for (var g = 0; g < n; g += 1)
    {
        sk = jigSketch(context, id + ("guideSk" ~ g), t + g * t);
        sketchRoundedRect(sk, "plate", plate, 10);
        for (var i = 0; i < size(layout.bolts); i += 1)
            skCircle(sk, "bolt" ~ i, { "center" : vec2mm(layout.bolts[i]), "radius" : boltR * millimeter });
        for (var i = 0; i < size(layout.slots); i += 1)
            sketchOutline(sk, "cut" ~ i, layout.slots[i].m, outlineSides(layout.shape, off));
        skSolve(sk);
        extrudeSketch(context, id + ("guide" ~ g), id + ("guideSk" ~ g), t, layout.info.name ~ " glue guide " ~ (g + 1), color(0.6, 0.8, 0.95));
    }

    const topZ = t + n * sheet;
    sk = jigSketch(context, id + "topSk", topZ);
    sketchRoundedRect(sk, "plate", plate, 10);
    for (var i = 0; i < size(layout.bolts); i += 1)
        skCircle(sk, "bolt" ~ i, { "center" : vec2mm(layout.bolts[i]), "radius" : boltR * millimeter });
    skSolve(sk);
    extrudeSketch(context, id + "top", id + "topSk", t, layout.info.name ~ " glue top plate (fixture-hole side)", color(0.75, 0.85, 0.95));

    if (definition.showBlanks)
        buildBlanks(context, id + "blanks", definition, layout, t, false);
}

// Wood sheets, one body per sheet per slot; the top sheet carries the fixture holes.
function buildBlanks(context is Context, id is Id, definition is map, layout is map, z0 is number, flip is boolean)
{
    const sheet = definition.sheet / millimeter;
    const sides = outlineSides(layout.shape, layout.allow);
    const fy = flip ? MIRROR_Y : IDENTITY2;
    for (var k = 0; k < layout.sheets; k += 1)
    {
        const top = k == layout.sheets - 1;
        const z = flip ? z0 + k * sheet : z0 + k * sheet;
        const sk = jigSketch(context, id + ("sk" ~ k), z);
        for (var i = 0; i < size(layout.slots); i += 1)
        {
            const m = compose(fy, layout.slots[i].m);
            sketchOutline(sk, "blank" ~ i, m, sides);
            // flipped (op0/mill frame): the hole sheet is the one against the plate
            if ((top && !flip) || (k == 0 && flip))
                for (var h = 0; h < size(layout.holes); h += 1)
                    skCircle(sk, "hole" ~ i ~ "_" ~ h, { "center" : vec2mm(mapPoint(m, layout.holes[h])),
                                "radius" : definition.fixtureD / 2 });
        }
        skSolve(sk);
        extrudeSketch(context, id + ("sheet" ~ k), id + ("sk" ~ k), sheet, layout.info.name ~ " glue blank sheet " ~ (k + 1), color(0.88, 0.75, 0.55));
    }
    // op0 (mill frame): each glued-up stack is one stock body for CAM
    if (flip)
    {
        const sheets = qCreatedBy(id, EntityType.BODY);
        opBoolean(context, id + "stack", { "tools" : sheets, "operationType" : BooleanOperationType.UNION });
        setProperty(context, { "entities" : sheets, "propertyType" : PropertyType.NAME, "value" : layout.info.name ~ " op0 stock" });
        setProperty(context, { "entities" : sheets, "propertyType" : PropertyType.APPEARANCE, "value" : STOCK_COLOUR });
    }
}

function buildOp0(context is Context, id is Id, definition is map, layout is map)
{
    // Mill frame: plate z in [-t, 0], blanks on top (track bottom up), counterbores underneath.
    // Seen from the other side than the glue frame: y -> -y.
    const cbore = (definition.op0HeadH + definition.op0Recess) / millimeter;
    const t = op0PlateThickness(definition);
    var all = [];
    for (var s in layout.slots)
        all = concatenateArrays([all, mapPolygon(compose(MIRROR_Y, s.m), layout.blank)]);
    const bb = polyBounds(all);
    const margin = definition.op0Margin / millimeter;
    const hx = max(-bb.x0, bb.x1) + margin;
    const hy = max(-bb.y0, bb.y1) + margin;
    const sk = jigSketch(context, id + "sk", -t);
    sketchRoundedRect(sk, "plate", { "x0" : -hx, "y0" : -hy, "x1" : hx, "y1" : hy }, 6);
    for (var i = 0; i < size(layout.slots); i += 1)
        for (var h = 0; h < size(layout.holes); h += 1)
            skCircle(sk, "hole" ~ i ~ "_" ~ h, { "center" : vec2mm(mapPoint(compose(MIRROR_Y, layout.slots[i].m), layout.holes[h])),
                        "radius" : definition.op0BoltD / 2 });
    skSolve(sk);
    extrudeSketch(context, id + "plate", id + "sk", t, layout.info.name ~ " op0 plate", color(0.62, 0.7, 0.78));
    const cb = jigSketch(context, id + "cboreSk", -t);
    for (var i = 0; i < size(layout.slots); i += 1)
        for (var h = 0; h < size(layout.holes); h += 1)
            skCircle(cb, "cbore" ~ i ~ "_" ~ h, { "center" : vec2mm(mapPoint(compose(MIRROR_Y, layout.slots[i].m), layout.holes[h])),
                        "radius" : definition.op0CboreD / 2 });
    skSolve(cb);
    opExtrude(context, id + "cbore", {
                "entities" : qSketchRegion(id + "cboreSk"),
                "direction" : vector(0, 0, 1),
                "endBound" : BoundingType.BLIND,
                "endDepth" : cbore * millimeter
            });
    opDeleteBodies(context, id + "cboreSkDelete", { "entities" : qCreatedBy(id + "cboreSk", EntityType.BODY) });
    opBoolean(context, id + "cboreCut", {
                "tools" : qCreatedBy(id + "cbore", EntityType.BODY),
                "targets" : qCreatedBy(id + "plate", EntityType.BODY),
                "operationType" : BooleanOperationType.SUBTRACTION
            });
    if (definition.op0TroughW > 0 && definition.op0TroughD > 0)
        cutTrough(context, id + "trough", definition, layout, qCreatedBy(id + "plate", EntityType.BODY));
    if (definition.stockModel)
        buildBlanks(context, id + "blanks", definition, layout, 0, true);
    if (definition.partModel)
    {
        // mill frame: track bottom up, the stack's fixture-hole face on the plate; one part body per 3D track
        const top = stackTop(layout.shape, definition);
        const spec = layout.shape.spec;
        var kinds = [];
        for (var sl in layout.slots)
            if (!isIn(sl.model, kinds))
                kinds = append(kinds, sl.model);
        for (var k = 0; k < size(kinds); k += 1)
        {
            const shape = trackShape(trackSpec(kinds[k], spec.gs, spec.elev), layout.shape.ext, 0, 41);
            const part = op0PartBody(context, id + ("part" ~ k), definition, shape, kinds[k]);
            var transforms = [];
            for (var sl in layout.slots)
                if (sl.model == kinds[k])
                    transforms = append(transforms, planTransformFlipZ(compose(MIRROR_Y, compose(sl.m, sl.pre)), top));
            placeModel(context, id + ("partModel" ~ k), part, transforms, layout.info.name ~ " op0 part", PART_COLOUR);
        }
    }
}

// Plate thickness (mm) from the screw: under the head it passes the plate, then threads into the hole sheet,
// stopping op0TipGap short of the sheet's far side.
function op0PlateThickness(definition is map) returns number
{
    const bite = (definition.sheet - definition.op0TipGap) / millimeter;
    const shank = definition.op0ScrewL / millimeter - bite;
    if (bite <= 0)
        throw regenError("Wood left past the screw tip must be less than the sheet thickness");
    if (shank < 3)
        throw regenError("Screw too short: under 3 mm of plate between its head and the blank");
    return shank + (definition.op0HeadH + definition.op0Recess) / millimeter;
}

// A band from each track's plan outline outward, sunk into the plate top, so a ball nose can finish the
// silhouette walls right down to the plate.
function cutTrough(context is Context, id is Id, definition is map, layout is map, plate is Query)
{
    const w = definition.op0TroughW / millimeter;
    const d = definition.op0TroughD / millimeter;
    // [name, outline offset, sketch z, extrude depth]: the inner tool overshoots so the band's walls are clean
    for (var pass in [["outer", w, -d, d + 1], ["inner", 0, -d - 1, d + 3]])
    {
        const sk = jigSketch(context, id + (pass[0] ~ "Sk"), pass[2]);
        const sides = outlineSides(layout.shape, pass[1]);
        for (var i = 0; i < size(layout.slots); i += 1)
            sketchOutline(sk, "o" ~ i, compose(MIRROR_Y, layout.slots[i].m), sides);
        skSolve(sk);
        opExtrude(context, id + pass[0], {
                    "entities" : qSketchRegion(id + (pass[0] ~ "Sk")),
                    "direction" : vector(0, 0, 1),
                    "endBound" : BoundingType.BLIND,
                    "endDepth" : pass[3] * millimeter
                });
        opDeleteBodies(context, id + (pass[0] ~ "Delete"), { "entities" : qCreatedBy(id + (pass[0] ~ "Sk"), EntityType.BODY) });
    }
    opBoolean(context, id + "band", {
                "tools" : qCreatedBy(id + "inner", EntityType.BODY),
                "targets" : qCreatedBy(id + "outer", EntityType.BODY),
                "operationType" : BooleanOperationType.SUBTRACTION
            });
    opBoolean(context, id + "cut", {
                "tools" : qCreatedBy(id + "outer", EntityType.BODY),
                "targets" : plate,
                "operationType" : BooleanOperationType.SUBTRACTION
            });
}

// Body names start with the track type (op1, op2) or species (glue-up, op0), so exports sort by name alone.
function trackName(kind is TrackType) returns string
{
    return { "S" : "s", "SSS" : "sSs", "LR" : "lr", "L" : "L", "R" : "R", "SL" : "sL", "SR" : "sR", "LS" : "Ls", "RS" : "Rs",
            "SLR" : "slr", "SRL" : "srl", "CURVE_RL" : "RL", "CURVE_LR" : "LR" }[toString(kind)];
}

// Fixture holes (plan points on the centreline), in the stack's track-top sheet.
function fixtureHoles(shape is map) returns array
{
    var holes = [];
    for (var t in [0.3, 0.7])
    {
        const c = pathPoint(shape.spec, 0, shape.ext.z1, t);
        holes = append(holes, [c[0], c[1]]);
    }
    return holes;
}

function stackSheets(shape is map, definition is map) returns number
{
    return ceil((shape.zhi - shape.zlo) / (definition.sheet / millimeter) - 1e-6);
}

// Template z of the stack's fixture-hole (track-top) face: the track top's highest point plus stockAbove.
function stackTop(shape is map, definition is map) returns number
{
    return shape.zhi + definition.stockAbove / millimeter;
}

// op0's result in the template frame: the glued stack (fixture holes included) cut 2.5D from the track-bottom
// side, down to the piece's bottom surface inside its plan silhouette.
function op0PartBody(context is Context, id is Id, definition is map, shape is map, kind is TrackType) returns Query
{
    const sheet = definition.sheet / millimeter;
    const top = stackTop(shape, definition);
    const bottom = top - stackSheets(shape, definition) * sheet;
    const sk = jigSketch(context, id + "stackSk", bottom);
    sketchOutline(sk, "o", IDENTITY2, outlineSides(shape, definition.allowance / millimeter));
    skSolve(sk);
    extrudeSketch(context, id + "stack", id + "stackSk", top - bottom, "op0 part", PART_COLOUR);
    var tools = [];
    for (var h in fixtureHoles(shape))
    {
        const hid = id + ("hole" ~ size(tools));
        fCylinder(context, hid, { "bottomCenter" : vector(h[0], h[1], top - sheet) * millimeter, "topCenter" : vector(h[0], h[1], top + 1) * millimeter,
                    "radius" : definition.fixtureD / 2 });
        tools = append(tools, qCreatedBy(hid, EntityType.BODY));
    }
    const stack = qCreatedBy(id + "stack", EntityType.BODY);
    opBoolean(context, id + "holes", { "targets" : stack, "tools" : qUnion(tools), "operationType" : BooleanOperationType.SUBTRACTION });

    // everything above the profile's underside, swept along the track
    const ext = shape.ext;
    const region = underSideRegion(context, id + "under", definition.profile, ext.z1 + (shape.zhi - shape.zlo) + definition.stockAbove / millimeter + 2);
    const env = trackBody(context, id + "env", kind, region, definition.gridSize / millimeter, definition.elevationGain / millimeter, 9);
    opDeleteBodies(context, id + "dropUnder", { "entities" : qCreatedBy(id + "under", EntityType.BODY) });
    opBoolean(context, id + "cut", { "tools" : qUnion([stack, env]), "operationType" : BooleanOperationType.INTERSECTION });
    return qCreatedBy(id, EntityType.BODY);
}

// Sketch region on the profile's plane: the profile's underside (the edges seen from below) closed straight
// up to profile height `zTop`. Swept, it is everything a 2.5D cut from below leaves above the piece's bottom.
function underSideRegion(context is Context, id is Id, profile is Query, zTop is number) returns Query
{
    const frame = trackFrame(context, profile);
    var edges = [];
    for (var edge in evaluateQuery(context, qAdjacent(profile, AdjacencyType.EDGE, EntityType.EDGE)))
    {
        var pts = [];
        for (var line in evEdgeTangentLines(context, { "edge" : edge, "parameters" : uniformParameters(33) }))
        {
            const d = line.origin - frame.origin;
            pts = append(pts, [dot(d, frame.x) / millimeter, dot(d, frame.z) / millimeter]);
        }
        if (pts[size(pts) - 1][0] < pts[0][0])
            pts = reverseArray(pts);
        edges = append(edges, { "pts" : pts, "line" : evCurveDefinition(context, { "edge" : edge }) is Line });
    }
    var under = [];
    for (var i = 0; i < size(edges); i += 1)
    {
        const pts = edges[i].pts;
        if (pts[size(pts) - 1][0] - pts[0][0] < 1e-6)
            continue;
        var seen = true;
        for (var k = 1; k < size(pts) - 1 && seen; k += 1)
            seen = !boundaryBelow(edges, i, pts[k]);
        if (seen)
            under = append(under, edges[i]);
    }
    under = sort(under, function(a, b) { return a.pts[0][0] - b.pts[0][0]; });
    if (size(under) == 0)
        throw regenError("Profile has no underside", ["profile"]);

    const sk = newSketchOnPlane(context, id, { "sketchPlane" : evOwnerSketchPlane(context, { "entity" : profile }) });
    var prev = under[0].pts[0];
    for (var i = 0; i < size(under); i += 1)
    {
        var pts = under[i].pts;
        if (norm2(sub2(pts[0], prev)) > 1e-3)
            throw regenError("The profile's underside is not one connected chain", ["profile"]);
        pts[0] = prev;
        if (under[i].line)
            skLineSegment(sk, "u" ~ i, { "start" : vec2mm(pts[0]), "end" : vec2mm(pts[size(pts) - 1]) });
        else
        {
            var mm = [];
            for (var q in pts)
                mm = append(mm, vec2mm(q));
            skFitSpline(sk, "u" ~ i, { "points" : mm });
        }
        prev = pts[size(pts) - 1];
    }
    const first = under[0].pts[0];
    skLineSegment(sk, "right", { "start" : vec2mm(prev), "end" : vec2mm([prev[0], zTop]) });
    skLineSegment(sk, "top", { "start" : vec2mm([prev[0], zTop]), "end" : vec2mm([first[0], zTop]) });
    skLineSegment(sk, "left", { "start" : vec2mm([first[0], zTop]), "end" : vec2mm(first) });
    skSolve(sk);
    return qSketchRegion(id);
}

// Does any other edge's polyline pass below q (same x, lower z)?
function boundaryBelow(edges is array, i is number, q is array) returns boolean
{
    for (var j = 0; j < size(edges); j += 1)
    {
        if (j == i)
            continue;
        const pts = edges[j].pts;
        for (var k = 0; k + 1 < size(pts); k += 1)
        {
            const a = pts[k];
            const b = pts[k + 1];
            if (abs(b[0] - a[0]) > 1e-9 && (a[0] - q[0]) * (b[0] - q[0]) <= 0 &&
                a[1] + (b[1] - a[1]) * (q[0] - a[0]) / (b[0] - a[0]) < q[1] - 1e-6)
                return true;
        }
    }
    return false;
}

// Copies of a template-frame body at each transform, named for CAM; the original is deleted.
function placeModel(context is Context, id is Id, body is Query, transforms is array, name is string, colour is Color)
{
    var names = [];
    for (var i = 0; i < size(transforms); i += 1)
        names = append(names, "" ~ i);
    opPattern(context, id + "copies", { "entities" : body, "transforms" : transforms, "instanceNames" : names });
    opDeleteBodies(context, id + "drop", { "entities" : body });
    const copies = qCreatedBy(id + "copies", EntityType.BODY);
    setProperty(context, { "entities" : copies, "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : copies, "propertyType" : PropertyType.APPEARANCE, "value" : colour });
}

// (x, y, z) -> (m(x, y), z0 - z): a plan affine with the body turned over.
function planTransformFlipZ(m is array, z0 is number) returns Transform
{
    return transform(matrix([[m[0], m[1], 0], [m[2], m[3], 0], [0, 0, -1]]), vector(m[4], m[5], z0) * millimeter);
}

function plateRect(layout is map, definition is map) returns map
{
    const off = layout.allow + layout.clear;
    var all = [];
    for (var s in layout.slots)
        all = concatenateArrays([all, mapPolygon(s.m, outlinePolygon(layout.shape, off))]);
    // outermost walls: at least glueWall around every cutout
    const cb = polyBounds(all);
    const w = definition.glueWall / millimeter;
    all = concatenateArrays([all, [[cb.x0 - w, cb.y0 - w], [cb.x1 + w, cb.y1 + w]]]);
    const reach = definition.boltD / millimeter / 2 + 7;
    for (var b in layout.bolts)
        all = concatenateArrays([all, [add2(b, [reach, reach]), add2(b, [-reach, -reach])]]);
    const bb = polyBounds(all);
    const hx = max(-bb.x0, bb.x1);
    const hy = max(-bb.y0, bb.y1);
    return { "x0" : -hx, "y0" : -hy, "x1" : hx, "y1" : hy };
}

// ---------------------------------------------------------------------------------------------------
// Sketch helpers

function jigSketch(context is Context, id is Id, z is number) returns Sketch
{
    return newSketchOnPlane(context, id, { "sketchPlane" : plane(vector(0, 0, z) * millimeter, vector(0, 0, 1), vector(1, 0, 0)) });
}

function extrudeSketch(context is Context, id is Id, sketchId is Id, depth is number, name is string, colour is Color)
{
    opExtrude(context, id, {
                "entities" : qSketchRegion(sketchId, true),
                "direction" : vector(0, 0, 1),
                "endBound" : BoundingType.BLIND,
                "endDepth" : depth * millimeter
            });
    setProperty(context, { "entities" : qCreatedBy(id, EntityType.BODY), "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : qCreatedBy(id, EntityType.BODY), "propertyType" : PropertyType.APPEARANCE, "value" : colour });
    opDeleteBodies(context, id + "deleteSketch", { "entities" : qCreatedBy(sketchId, EntityType.BODY) });
}

// Two fitted splines for the sides, straight lines across the ends.
function sketchOutline(sk is Sketch, name is string, m is array, sides is array)
{
    const a = mapPolygon(m, sides[0]);
    const b = mapPolygon(m, sides[1]);
    var pa = [];
    var pb = [];
    for (var p in a)
        pa = append(pa, vec2mm(p));
    for (var p in b)
        pb = append(pb, vec2mm(p));
    skFitSpline(sk, name ~ "a", { "points" : pa });
    skLineSegment(sk, name ~ "e1", { "start" : pa[size(pa) - 1], "end" : pb[0] });
    skFitSpline(sk, name ~ "b", { "points" : pb });
    skLineSegment(sk, name ~ "e0", { "start" : pb[size(pb) - 1], "end" : pa[0] });
}

function sketchRoundedRect(sk is Sketch, name is string, r is map, rad is number)
{
    const x0 = r.x0;
    const y0 = r.y0;
    const x1 = r.x1;
    const y1 = r.y1;
    const k = rad * (1 - sqrt(0.5));
    skLineSegment(sk, name ~ "b", { "start" : vec2mm([x0 + rad, y0]), "end" : vec2mm([x1 - rad, y0]) });
    skArc(sk, name ~ "br", { "start" : vec2mm([x1 - rad, y0]), "mid" : vec2mm([x1 - k, y0 + k]), "end" : vec2mm([x1, y0 + rad]) });
    skLineSegment(sk, name ~ "r", { "start" : vec2mm([x1, y0 + rad]), "end" : vec2mm([x1, y1 - rad]) });
    skArc(sk, name ~ "tr", { "start" : vec2mm([x1, y1 - rad]), "mid" : vec2mm([x1 - k, y1 - k]), "end" : vec2mm([x1 - rad, y1]) });
    skLineSegment(sk, name ~ "t", { "start" : vec2mm([x1 - rad, y1]), "end" : vec2mm([x0 + rad, y1]) });
    skArc(sk, name ~ "tl", { "start" : vec2mm([x0 + rad, y1]), "mid" : vec2mm([x0 + k, y1 - k]), "end" : vec2mm([x0, y1 - rad]) });
    skLineSegment(sk, name ~ "l", { "start" : vec2mm([x0, y1 - rad]), "end" : vec2mm([x0, y0 + rad]) });
    skArc(sk, name ~ "bl", { "start" : vec2mm([x0, y0 + rad]), "mid" : vec2mm([x0 + k, y0 + k]), "end" : vec2mm([x0 + rad, y0]) });
}

function sketchHex(sk is Sketch, name is string, c is array, acrossFlats is number)
{
    const r = acrossFlats / sqrt(3);
    var pts = [];
    for (var i = 0; i < 6; i += 1)
    {
        const a = (60 * i + 30) * degree;
        pts = append(pts, vec2mm([c[0] + r * cos(a), c[1] + r * sin(a)]));
    }
    for (var i = 0; i < 6; i += 1)
        skLineSegment(sk, name ~ "_" ~ i, { "start" : pts[i], "end" : pts[(i + 1) % 6] });
}

// ---------------------------------------------------------------------------------------------------
// 2D math (mm numbers)

const IDENTITY2 = [1, 0, 0, 1, 0, 0];
// stock is see-through so the part inside it stays visible when both models are on
const STOCK_COLOUR = color(0.88, 0.75, 0.55, 0.3);
const PART_COLOUR = color(0.7, 0.52, 0.32);
const MIRROR_X = [-1, 0, 0, 1, 0, 0];
const MIRROR_Y = [1, 0, 0, -1, 0, 0];
const ROT180 = [-1, 0, 0, -1, 0, 0];
const ROT90 = [0, -1, 1, 0, 0, 0];

function translation2(dx is number, dy is number) returns array
{
    return [1, 0, 0, 1, dx, dy];
}

// m2 after m1
function compose(m2 is array, m1 is array) returns array
{
    return [m2[0] * m1[0] + m2[1] * m1[2], m2[0] * m1[1] + m2[1] * m1[3],
            m2[2] * m1[0] + m2[3] * m1[2], m2[2] * m1[1] + m2[3] * m1[3],
            m2[0] * m1[4] + m2[1] * m1[5] + m2[4], m2[2] * m1[4] + m2[3] * m1[5] + m2[5]];
}

function mapPoint(m is array, p is array) returns array
{
    return [m[0] * p[0] + m[1] * p[1] + m[4], m[2] * p[0] + m[3] * p[1] + m[5]];
}

function mapPolygon(m is array, poly is array) returns array
{
    var out = [];
    for (var p in poly)
        out = append(out, mapPoint(m, p));
    return out;
}

function polyBounds(pts is array) returns map
{
    var b = { "x0" : inf, "y0" : inf, "x1" : -inf, "y1" : -inf };
    for (var p in pts)
    {
        b.x0 = min(b.x0, p[0]);
        b.y0 = min(b.y0, p[1]);
        b.x1 = max(b.x1, p[0]);
        b.y1 = max(b.y1, p[1]);
    }
    return b;
}

function polyCentroid(pts is array) returns array
{
    var c = [0, 0];
    for (var p in pts)
        c = add2(c, p);
    return scale2(c, 1 / size(pts));
}

function reverseArray(a is array) returns array
{
    var out = [];
    for (var i = size(a) - 1; i >= 0; i -= 1)
        out = append(out, a[i]);
    return out;
}

function add2(a is array, b is array) returns array
{
    return [a[0] + b[0], a[1] + b[1]];
}

function sub2(a is array, b is array) returns array
{
    return [a[0] - b[0], a[1] - b[1]];
}

function scale2(a is array, s is number) returns array
{
    return [a[0] * s, a[1] * s];
}

function norm2(a is array) returns number
{
    return sqrt(a[0] * a[0] + a[1] * a[1]);
}

function unit2(a is array) returns array
{
    return scale2(a, 1 / norm2(a));
}

function vec2mm(p is array) returns Vector
{
    return vector(p[0], p[1]) * millimeter;
}
