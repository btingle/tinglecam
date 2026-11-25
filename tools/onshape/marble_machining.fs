// op1 (top side soft jaws) and op2 (end face jaws). Concatenated after marble_track.fs and marble_jigs.fs.
// Jig frame = track template frame (mm): plan X/Y, Z up, track top (channel) facing +Z.
// Pockets are the profile's bounding box swept along the track (fillets and channel ignored).

const OP2_COUNT_BOUNDS = { (unitless) : [1, 4, 12] } as IntegerBoundSpec;
const OP1_COUNT_BOUNDS = { (unitless) : [0, 0, 12] } as IntegerBoundSpec;
// straight profile edges: 4 cubic samples are exact
const RECT_SAMPLES = 4;
const PATH_SAMPLES = 17;
// isocline angle for op2 reliefs: faces turned less than this toward the insertion direction are not swept
const RELIEF_ANGLE = 0.2;
// reliefs sweep a pocket this much smaller than the seat (mm), so they overlap the seat instead of touching it
const RELIEF_INSET = 0.05;

export enum JawSplit
{
    annotation { "Name" : "Through an end centreline" }
    END,
    annotation { "Name" : "Bounding box centre, along template X" }
    BOX_X,
    annotation { "Name" : "Bounding box centre, along template Y" }
    BOX_Y
}

annotation { "Feature Type Name" : "Marble op1 jaws" }
export const marbleOp1Jaws = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Profile region", "Filter" : EntityType.FACE && SketchObject.YES && ConstructionObject.NO, "MaxNumberOfPicks" : 1 }
        definition.profile is Query;
        annotation { "Name" : "Track type" }
        definition.trackType is TrackType;
        trackPrecondition(definition);
        machiningPrecondition(definition);
        annotation { "Name" : "Vise jaw width" }
        isLength(definition.viseWidth, MM_150);
        annotation { "Name" : "Split plane" }
        definition.op1Split is JawSplit;
        annotation { "Name" : "Max pieces (0 = fill the vise)" }
        isInteger(definition.op1MaxCount, OP1_COUNT_BOUNDS);
        annotation { "Name" : "Wall width beside the track" }
        isLength(definition.op1Wall, MM_7);
        annotation { "Name" : "Track side exposed above the walls" }
        isLength(definition.op1Exposed, MM_4);
        annotation { "Name" : "Floor under the pocket" }
        isLength(definition.op1Floor, MM_5);
        annotation { "Name" : "Block margin at the clamp faces" }
        isLength(definition.op1Margin, MM_6);
        stackPrecondition(definition);
        annotation { "Name" : "Stock above the track top" }
        isLength(definition.stockAbove, MM_0);
        annotation { "Name" : "End runout diameter" }
        isLength(definition.runoutD, MM_20);
        annotation { "Name" : "Runout axis below the track top" }
        isLength(definition.runoutDrop, MM_1);
        annotation { "Name" : "Runout length past each end" }
        isLength(definition.runoutLength, MM_8);
        placementPrecondition(definition);
    }
    {
        buildOp1(context, id, definition);
        placeResult(context, id, definition);
    }, machiningDefaults());

annotation { "Feature Type Name" : "Marble op2 jaws" }
export const marbleOp2Jaws = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Profile region", "Filter" : EntityType.FACE && SketchObject.YES && ConstructionObject.NO, "MaxNumberOfPicks" : 1 }
        definition.profile is Query;
        annotation { "Name" : "Track type" }
        definition.trackType is TrackType;
        trackPrecondition(definition);
        machiningPrecondition(definition);
        annotation { "Name" : "Split plane" }
        definition.op2Split is JawSplit;
        annotation { "Name" : "Pieces per jig" }
        isInteger(definition.op2Count, OP2_COUNT_BOUNDS);
        annotation { "Name" : "Material between stacked pieces" }
        isLength(definition.op2Gap, MM_7);
        annotation { "Name" : "Jaw wall around the piece" }
        isLength(definition.op2Wall, MM_8);
        annotation { "Name" : "End faces stand proud by" }
        isLength(definition.op2Proud, MM_0_5);
        annotation { "Name" : "M3 clearance hole" }
        isLength(definition.op2BoltD, MM_3_4);
        annotation { "Name" : "M3 head counterbore" }
        isLength(definition.op2HeadD, MM_6_5);
        annotation { "Name" : "Counterbore depth" }
        isLength(definition.op2HeadDepth, MM_3_5);
        annotation { "Name" : "Heat-set insert hole" }
        isLength(definition.op2InsertD, MM_4);
        annotation { "Name" : "Insert hole depth" }
        isLength(definition.op2InsertDepth, MM_6);
        placementPrecondition(definition);
    }
    {
        buildOp2(context, id, definition);
        placeResult(context, id, definition);
    }, machiningDefaults());

predicate machiningPrecondition(definition is map)
{
    annotation { "Name" : "Pocket clearance" }
    isLength(definition.pocketClearance, MM_0_1);
    annotation { "Name" : "Gap removed between jaws" }
    isLength(definition.splitGap, MM_0_3);
    modelPrecondition(definition);
}

function machiningDefaults() returns map
{
    return mergeMaps(jigDefaults(), {
                "trackType" : TrackType.L,
                "pocketClearance" : 0.1 * millimeter,
                "splitGap" : 0.3 * millimeter,
                "viseWidth" : 150 * millimeter,
                "op1Split" : JawSplit.END,
                "op1MaxCount" : 0,
                "op1Wall" : 7 * millimeter,
                "op1Exposed" : 4 * millimeter,
                "op1Floor" : 5 * millimeter,
                "op1Margin" : 6 * millimeter,
                "runoutD" : 20 * millimeter,
                "runoutDrop" : 1 * millimeter,
                "runoutLength" : 8 * millimeter,
                "op2Split" : JawSplit.END,
                "op2Count" : 4,
                "op2Gap" : 7 * millimeter,
                "op2Wall" : 8 * millimeter,
                "op2Proud" : 0.5 * millimeter,
                "op2BoltD" : 3.4 * millimeter,
                "op2HeadD" : 6.5 * millimeter,
                "op2HeadDepth" : 3.5 * millimeter,
                "op2InsertD" : 4 * millimeter,
                "op2InsertDepth" : 6 * millimeter
            });
}

// ---------------------------------------------------------------------------------------------------
// Track bodies in the template frame

function trackBody(context is Context, id is Id, kind is TrackType, region is Query, gs is number, elev is number, profileSamples is number) returns Query
{
    const frame = trackFrame(context, region);
    const spec = trackSpec(kind, gs, elev);
    if (isSampled(spec))
        buildSampled(context, id, { "profile" : region, "profileSamples" : profileSamples, "pathSamples" : PATH_SAMPLES,
                    "surfacesOnly" : false }, frame, spec);
    else
        buildNative(context, id, region, frame, spec);
    const origin = frame.origin - (frame.y * spec.y0 + frame.z * spec.z0) * millimeter;
    const bodies = qCreatedBy(id, EntityType.BODY);
    opTransform(context, id + "toTemplate", { "bodies" : bodies, "transform" : fromWorld(coordSystem(origin, frame.x, frame.z)) });
    return bodies;
}

// The profile rectangle [xa, xb] x [za, zb] (profile coordinates) swept along the track.
function rectBody(context is Context, id is Id, definition is map, m is map, xa is number, xb is number, za is number, zb is number) returns Query
{
    const sk = newSketchOnPlane(context, id + "profile", { "sketchPlane" : evOwnerSketchPlane(context, { "entity" : definition.profile }) });
    sketchPolygon(sk, "p", [[xa, za], [xb, za], [xb, zb], [xa, zb]]);
    skSolve(sk);
    const body = trackBody(context, id + "sweep", definition.trackType, qSketchRegion(id + "profile"), m.gs, m.elev, RECT_SAMPLES);
    opDeleteBodies(context, id + "dropProfile", { "entities" : qCreatedBy(id + "profile", EntityType.BODY) });
    return body;
}

function pieceBody(context is Context, id is Id, definition is map, m is map) returns Query
{
    return trackBody(context, id, definition.trackType, definition.profile, m.gs, m.elev, 9);
}

// Plan outline of the profile band [xa, xb] (profile x), ends pushed out by endExt.
function bandPolygon(shape is map, xa is number, xb is number, endExt is number) returns array
{
    const n = shape.n;
    var a = [];
    var b = [];
    for (var i = 0; i < n; i += 1)
    {
        const t = i / (n - 1);
        const p = pathPoint(shape.spec, xa, shape.ext.z1, t);
        const q = pathPoint(shape.spec, xb, shape.ext.z1, t);
        a = append(a, [p[0], p[1]]);
        b = append(b, [q[0], q[1]]);
    }
    for (var e = 0; e < 2; e += 1)
    {
        const i = e == 0 ? 0 : n - 1;
        const d = scale2(shape.ends[e][1], endExt);
        a[i] = add2(a[i], d);
        b[i] = add2(b[i], d);
    }
    return concatenateArrays([a, reverseArray(b)]);
}

// Profile x span grown by `off`, stopped short of the bend axis on the inside of curves.
function wallSpan(shape is map, off is number) returns array
{
    var xa = shape.ext.x0 - off;
    var xb = shape.ext.x1 + off;
    const spec = shape.spec;
    // curve + curve: each side is the inside of one turn
    if (spec.curveLeft == true || spec.mode == "curveCurve")
        xb = min(xb, spec.gs / 2 - 0.5);
    if (spec.curveLeft == false || spec.mode == "curveCurve")
        xa = max(xa, -spec.gs / 2 + 0.5);
    return [xa, xb];
}

function sketchPolygon(sk is Sketch, name is string, poly is array)
{
    const n = size(poly);
    for (var i = 0; i < n; i += 1)
        skLineSegment(sk, name ~ i, { "start" : vec2mm(poly[i]), "end" : vec2mm(poly[(i + 1) % n]) });
}

function prism(context is Context, id is Id, polys is array, z0 is number, z1 is number) returns Query
{
    const sk = jigSketch(context, id + "sk", z0);
    for (var i = 0; i < size(polys); i += 1)
        sketchPolygon(sk, "p" ~ i ~ "_", polys[i]);
    skSolve(sk);
    opExtrude(context, id, { "entities" : qSketchRegion(id + "sk"), "direction" : vector(0, 0, 1),
                "endBound" : BoundingType.BLIND, "endDepth" : (z1 - z0) * millimeter });
    opDeleteBodies(context, id + "dropSk", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    return qCreatedBy(id, EntityType.BODY);
}

function machiningSetup(context is Context, definition is map) returns map
{
    const gs = definition.gridSize / millimeter;
    const elev = definition.elevationGain / millimeter;
    const ext = profileExtents(context, definition.profile);
    const spec = trackSpec(definition.trackType, gs, elev);
    return { "gs" : gs, "elev" : elev, "ext" : ext, "spec" : spec, "shape" : trackShape(spec, ext, 0, 41),
            "clear" : definition.pocketClearance / millimeter, "gap" : definition.splitGap / millimeter };
}

function nameBodies(context is Context, q is Query, name is string, colour is Color)
{
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : colour });
}

// ---------------------------------------------------------------------------------------------------
// op1: pocket = bounding box swept (open on top), walls `op1Wall` wide rising to `op1Exposed` below the
// track top, block top at the lowest wall top, channel runouts past both ends. Pieces turn in 90 degree steps
// and sit in one row along the vise jaws; one plane y = c splits the block, so the jaws open along Y.

// Geometry and slots for op1, no bodies.
function op1Plan(context is Context, definition is map) returns map
{
    const m = machiningSetup(context, definition);
    const shape = m.shape;
    const ext = m.ext;
    const clear = m.clear;
    const wall = definition.op1Wall / millimeter;
    const W = definition.viseWidth / millimeter;
    const runL = definition.runoutLength / millimeter;
    const runR = definition.runoutD / millimeter / 2;
    const span = wallSpan(shape, clear + wall);
    const xm = (ext.x0 + ext.x1) / 2;

    // ends: runout troughs and the pocket's end clearance
    var ends = [];
    var runs = [];
    for (var e = 0; e < 2; e += 1)
    {
        const top = pathPoint(m.spec, xm, ext.z1, e);
        const bottom = pathPoint(m.spec, xm, ext.z0, e);
        const n = shape.ends[e][1];
        const f = [-n[1], n[0]];
        const p = [top[0], top[1]];
        ends = append(ends, { "p" : p, "n" : n, "f" : f, "top" : top[2], "bottom" : bottom[2] });
        runs = append(runs, alongRect(p, n, f, -0.5, runL, runR));
    }
    const geo = { "pocket" : bandPolygon(shape, ext.x0 - clear, ext.x1 + clear, clear),
            "keep" : bandPolygon(shape, span[0], span[1], clear + wall),
            "runs" : runs, "centre" : shape.centre, "ends" : ends };
    const layout = op1Layout(geo, W, definition.op1MaxCount, definition.op1Split);
    println(toString(definition.trackType) ~ " op1 (" ~ toString(definition.op1Split) ~ "): " ~ size(layout.slots) ~
            " pieces, " ~ roundTo(layout.width) ~ " x " ~ roundTo(layout.y1 - layout.y0) ~ " (vise " ~ W ~ ")");
    return { "m" : m, "span" : span, "ends" : ends, "layout" : layout, "W" : W, "runL" : runL, "runR" : runR,
            "halfW" : (ext.x1 - ext.x0) / 2 + clear };
}

function buildOp1(context is Context, id is Id, definition is map)
{
    const plan = op1Plan(context, definition);
    const m = plan.m;
    const shape = m.shape;
    const ext = m.ext;
    const clear = m.clear;
    const exposed = definition.op1Exposed / millimeter;
    const span = plan.span;
    const ends = plan.ends;
    const layout = plan.layout;
    const W = plan.W;
    const runL = plan.runL;
    const runR = plan.runR;
    const halfW = plan.halfW;

    // one set of bodies in the template frame, copied to every slot
    const pocket = rectBody(context, id + "pocket", definition, m, ext.x0 - clear, ext.x1 + clear, ext.z0 - clear, ext.z1 + 30);
    const walls = rectBody(context, id + "walls", definition, m, span[0], span[1],
                ext.z0 - clear - definition.op1Floor / millimeter - m.elev - 1, ext.z1 - exposed);
    var cutters = [pocket];
    for (var e = 0; e < 2; e += 1)
    {
        const d = ends[e];
        const z = d.top - definition.runoutDrop / millimeter;
        const at = function(s) { return vector(d.p[0] + d.n[0] * s, d.p[1] + d.n[1] * s, z) * millimeter; };
        fCylinder(context, id + ("runout" ~ e), { "bottomCenter" : at(-0.5), "topCenter" : at(runL), "radius" : runR * millimeter });
        cutters = append(cutters, qCreatedBy(id + ("runout" ~ e), EntityType.BODY));
        cutters = append(cutters, prism(context, id + ("endClear" ~ e), [alongRect(d.p, d.n, d.f, -0.5, clear, halfW)],
                        d.bottom - clear - 0.5, shape.zhi + 30));
    }
    var originals = concatenateArrays([cutters, [walls]]);
    var transforms = [];
    var names = [];
    for (var k = 0; k < size(layout.slots); k += 1)
    {
        transforms = append(transforms, planTransform(layout.slots[k]));
        names = append(names, "" ~ k);
    }
    opPattern(context, id + "cutterCopies", { "entities" : qUnion(cutters), "transforms" : transforms, "instanceNames" : names });
    opPattern(context, id + "wallCopies", { "entities" : walls, "transforms" : transforms, "instanceNames" : names });
    opDeleteBodies(context, id + "dropOriginals", { "entities" : qUnion(originals) });

    // block + walls, clipped flat underneath, minus pockets and runouts
    const zFloor = shape.zlo - clear - definition.op1Floor / millimeter;
    const top = shape.minTop - exposed;
    const margin = definition.op1Margin / millimeter;
    const half = max(W, layout.width) / 2;
    const y0 = layout.y0 - margin;
    const y1 = layout.y1 + margin;
    const cutterCopies = qCreatedBy(id + "cutterCopies", EntityType.BODY);
    const clipBox = qCreatedBy(id + "clip", EntityType.BODY);
    const gapBody = qCreatedBy(id + "gap", EntityType.BODY);
    const jaw = qSubtraction(qCreatedBy(id, EntityType.BODY), qUnion([clipBox, gapBody, cutterCopies]));
    fCuboid(context, id + "block", { "corner1" : vector(-half, y0, zFloor) * millimeter, "corner2" : vector(half, y1, top) * millimeter });
    opBoolean(context, id + "addWalls", { "tools" : jaw, "operationType" : BooleanOperationType.UNION });
    fCuboid(context, id + "clip", { "corner1" : vector(-half, y0, zFloor) * millimeter, "corner2" : vector(half, y1, shape.zhi + 50) * millimeter });
    opBoolean(context, id + "flatBottom", { "tools" : qUnion([jaw, clipBox]), "operationType" : BooleanOperationType.INTERSECTION });
    opBoolean(context, id + "pockets", { "targets" : jaw, "tools" : cutterCopies, "operationType" : BooleanOperationType.SUBTRACTION });

    // split: one plane y = c across the block, splitGap removed
    fCuboid(context, id + "gap", { "corner1" : vector(-half - 1, layout.c - m.gap / 2, zFloor - 1) * millimeter,
                "corner2" : vector(half + 1, layout.c + m.gap / 2, shape.zhi + 60) * millimeter });
    opBoolean(context, id + "split", { "targets" : jaw, "tools" : gapBody, "operationType" : BooleanOperationType.SUBTRACTION });

    const jaws = evaluateQuery(context, jaw);
    if (size(jaws) != 2)
        throw regenError("op1 split produced " ~ size(jaws) ~ " jaws instead of 2");
    const ya = evApproximateCentroid(context, { "entities" : jaws[0] })[1];
    const yb = evApproximateCentroid(context, { "entities" : jaws[1] })[1];
    nameBodies(context, ya < yb ? jaws[0] : jaws[1], trackName(definition.trackType) ~ " op1 jaw A (fixed)", color(0.55, 0.72, 0.85));
    nameBodies(context, ya < yb ? jaws[1] : jaws[0], trackName(definition.trackType) ~ " op1 jaw B (moving)", color(0.85, 0.66, 0.43));

    // CAM models (after the jaws, which are everything else this feature made): op0's part in, the piece out
    if (definition.stockModel)
        placeModel(context, id + "stockModel", op0PartBody(context, id + "stock", definition, shape, definition.trackType),
                transforms, trackName(definition.trackType) ~ " op1 stock", STOCK_COLOUR);
    if (definition.partModel)
        placeModel(context, id + "partModel", pieceBody(context, id + "part", definition, m), transforms, trackName(definition.trackType) ~ " op1 part", PART_COLOUR);
}

// Rectangle from p + n*s0 to p + n*s1, +-h along f.
function alongRect(p is array, n is array, f is array, s0 is number, s1 is number, h is number) returns array
{
    return [add2(add2(p, scale2(n, s0)), scale2(f, -h)), add2(add2(p, scale2(n, s1)), scale2(f, -h)),
            add2(add2(p, scale2(n, s1)), scale2(f, h)), add2(add2(p, scale2(n, s0)), scale2(f, h))];
}

function roundTo(x is number) returns number
{
    return round(x * 10) / 10;
}

// Slots (plan affines, template -> jig) for as many pieces as fit the vise width (at most maxCount when
// positive). Pieces turn in 90 degree steps and sit in one row along X, all crossed by the split plane y = 0:
// through an end's centreline (that end facing +-X), or through the plan bounding box centre, parallel to
// the template's X or Y axis.
function op1Layout(geo is map, W is number, maxCount is number, split is JawSplit) returns map
{
    var cands = [];
    for (var q = 0; q < 4; q += 1)
    {
        const R = quarterTurn(q);
        if (split == JawSplit.END)
        {
            // R has no translation, so it maps directions too
            for (var e in geo.ends)
                if (abs(mapPoint(R, e.n)[1]) < 1e-3)
                    cands = append(cands, op1Geometry(geo, compose(translation2(0, -mapPoint(R, e.p)[1]), R)));
        }
        else if ((q % 2 == 0) == (split == JawSplit.BOX_X))
        {
            const b = polyBounds(mapPolygon(R, geo.pocket));
            cands = append(cands, op1Geometry(geo, compose(translation2(0, -(b.y0 + b.y1) / 2), R)));
        }
    }
    if (size(cands) == 0)
        throw regenError("No op1 orientation puts the split plane through an end centreline of this track");
    var best = undefined;
    for (var first in cands)
    {
        const r = op1Pack(first, cands, W, maxCount);
        if (best == undefined || size(r.placed) > size(best.placed) || (size(r.placed) == size(best.placed) && r.area < best.area - 1e-6))
            best = r;
    }
    // centre on the origin
    const b = best.b;
    const sx = -(b.x0 + b.x1) / 2;
    const sy = -(b.y0 + b.y1) / 2;
    var slots = [];
    for (var p in best.placed)
        slots = append(slots, compose(translation2(p.dx + sx, sy), p.g.M));
    return { "slots" : slots, "width" : b.x1 - b.x0, "y0" : b.y0 + sy, "y1" : b.y1 + sy, "c" : sy };
}

function quarterTurn(q is number) returns array
{
    return [[1, 0, 0, 1, 0, 0], [0, -1, 1, 0, 0, 0], [-1, 0, 0, -1, 0, 0], [0, 1, -1, 0, 0, 0]][q];
}

function op1Geometry(geo is map, M is array) returns map
{
    var runs = [];
    for (var r in geo.runs)
        runs = append(runs, mapPolygon(M, r));
    const keep = mapPolygon(M, geo.keep);
    return { "M" : M, "centre" : mapPolygon(M, geo.centre), "kb" : polyBounds(keep),
            "pocketBands" : bandSpans([mapPolygon(M, geo.pocket)]), "keepBands" : bandSpans([keep]), "runBands" : bandSpans(runs) };
}

// Greedy packing left to right from `first`. Each new piece (any candidate) sits right of everything placed
// on every 1 mm band: its pocket clear of placed walls, its walls clear of placed pockets and runouts.
function op1Pack(first is map, cands is array, W is number, maxCount is number) returns map
{
    var placed = [{ "g" : first, "dx" : 0 }];
    var keepHi = first.keepBands.hi;
    var pocketHi = first.pocketBands.hi;
    var runHi = first.runBands.hi;
    var b = first.kb;
    var lastX0 = b.x0;
    while (maxCount <= 0 || size(placed) < maxCount)
    {
        var chosen = undefined;
        for (var g in cands)
        {
            var dx = lastX0 - g.kb.x0;
            dx = max(dx, bandGap(pocketHi, g.keepBands.lo));
            dx = max(dx, bandGap(runHi, g.keepBands.lo));
            dx = max(dx, bandGap(keepHi, g.pocketBands.lo));
            dx = max(dx, bandGap(keepHi, g.runBands.lo));
            const w = max(b.x1, g.kb.x1 + dx) - b.x0;
            const h = max(b.y1, g.kb.y1) - min(b.y0, g.kb.y0);
            const score = w + 0.25 * h;
            if (w <= W && (chosen == undefined || score < chosen.score - 1e-9))
                chosen = { "g" : g, "dx" : dx, "score" : score };
        }
        if (chosen == undefined)
            break;
        const g = chosen.g;
        const dx = chosen.dx;
        placed = append(placed, { "g" : g, "dx" : dx });
        keepHi = mergeHi(keepHi, g.keepBands.hi, dx);
        pocketHi = mergeHi(pocketHi, g.pocketBands.hi, dx);
        runHi = mergeHi(runHi, g.runBands.hi, dx);
        b = { "x0" : b.x0, "x1" : max(b.x1, g.kb.x1 + dx), "y0" : min(b.y0, g.kb.y0), "y1" : max(b.y1, g.kb.y1) };
        lastX0 = g.kb.x0 + dx;
    }
    return { "placed" : placed, "b" : b, "area" : (b.x1 - b.x0) * (b.y1 - b.y0) };
}

// Smallest shift along +X putting `lo` at or right of `hi` on every shared band.
function bandGap(hi is map, lo is map) returns number
{
    var gap = -inf;
    for (var entry in lo)
    {
        const h = hi[entry.key];
        if (h != undefined)
            gap = max(gap, h - entry.value);
    }
    return gap;
}

function mergeHi(hi is map, add is map, dx is number) returns map
{
    var out = hi;
    for (var entry in add)
    {
        const k = entry.key;
        out[k] = out[k] == undefined ? entry.value + dx : max(out[k], entry.value + dx);
    }
    return out;
}

// Per 1 mm band k (y in [k, k + 1]): lowest and highest x on the polygons' boundaries.
function bandSpans(polys is array) returns map
{
    var lo = {};
    var hi = {};
    var k0 = inf;
    var k1 = -inf;
    for (var poly in polys)
    {
        const n = size(poly);
        for (var i = 0; i < n; i += 1)
        {
            const a = poly[i];
            const b = poly[(i + 1) % n];
            const ya = min(a[1], b[1]);
            const yb = max(a[1], b[1]);
            for (var k = floor(ya); k <= floor(yb); k += 1)
            {
                var xs = [a[0], b[0]];
                if (yb - ya > 1e-9)
                {
                    const y0 = max(ya, k);
                    const y1 = min(yb, k + 1);
                    xs = [a[0] + (b[0] - a[0]) * (y0 - a[1]) / (b[1] - a[1]), a[0] + (b[0] - a[0]) * (y1 - a[1]) / (b[1] - a[1])];
                }
                for (var x in xs)
                {
                    lo[k] = lo[k] == undefined ? x : min(lo[k], x);
                    hi[k] = hi[k] == undefined ? x : max(hi[k], x);
                }
                k0 = min(k0, k);
                k1 = max(k1, k);
            }
        }
    }
    return { "lo" : lo, "hi" : hi, "k0" : k0, "k1" : k1 };
}

function planTransform(m is array) returns Transform
{
    return transform(matrix([[m[0], m[1], 0], [m[2], m[3], 0], [0, 0, 1]]), vector(m[4], m[5], 0) * millimeter);
}

// ---------------------------------------------------------------------------------------------------
// op2: N pieces stacked along Z (perpendicular to both end normals, so every end face stays exposed).
// One vertical split plane through an end's centreline, square to that end; of the two ends, the one whose
// split leaves the most track length pressed between the jaws. Jaws part along s, so relative to jaw A a piece
// arrives from +s and relative to jaw B from -s; each jaw loses the volume its pockets sweep on the way in.
// M3 bolts along s (heads in jaw B, inserts in jaw A).

function buildOp2(context is Context, id is Id, definition is map)
{
    const m = machiningSetup(context, definition);
    const shape = m.shape;
    const ext = m.ext;
    const clear = m.clear;
    const wall = definition.op2Wall / millimeter;
    const proud = definition.op2Proud / millimeter;
    const count = definition.op2Count;
    const band = [shape.zlo - clear, shape.zhi + clear];
    const pitch = band[1] - band[0] + definition.op2Gap / millimeter;
    const outline = bandPolygon(shape, ext.x0 - clear, ext.x1 + clear, clear);

    const best = op2Split(shape, outline, definition.op2Split);
    const s = best.s;
    const c = best.c;
    const g = m.gap / 2;
    const tdir = [-s[1], s[0]];

    // footprint: outline box in the split frame + wall, cut back so both end faces stand proud
    const F = frameOf(s);
    const ob = polyBounds(mapPolygon(F, outline));
    var foot = mapPolygon(invertRotation(F), [[ob.x0 - wall, ob.y0 - wall], [ob.x1 + wall, ob.y0 - wall],
                [ob.x1 + wall, ob.y1 + wall], [ob.x0 - wall, ob.y1 + wall]]);
    for (var en in shape.ends)
        foot = clipHalfPlane(foot, en[0], en[1], proud);
    var sMin = inf;
    var sMax = -inf;
    var uMin = inf;
    var uMax = -inf;
    for (var p in foot)
    {
        sMin = min(sMin, dot2(p, s));
        sMax = max(sMax, dot2(p, s));
        uMin = min(uMin, dot2(p, tdir));
        uMax = max(uMax, dot2(p, tdir));
    }

    const z0 = band[0] - wall;
    const z1 = band[1] + (count - 1) * pitch + wall;
    const jawA = prism(context, id + "jawA", [clipHalfPlane(foot, scale2(s, c), s, g)], z0, z1);
    const jawB = prism(context, id + "jawB", [clipHalfPlane(foot, scale2(s, c), scale2(s, -1), g)], z0, z1);

    // seat pocket; per jaw, a slightly smaller pocket swept from the piece's arrival side across the jaw
    const pocket = rectBody(context, id + "pocket", definition, m, ext.x0 - clear, ext.x1 + clear, ext.z0 - clear, ext.z1 + clear);
    const ci = clear - RELIEF_INSET;
    const inner = rectBody(context, id + "inner", definition, m, ext.x0 - ci, ext.x1 + ci, ext.z0 - ci, ext.z1 + ci);
    const reach = (sMax - sMin + 1) * millimeter;
    const reliefA = insertionRelief(context, id + "reliefA", inner, vector(s[0], s[1], 0), reach);
    const reliefB = insertionRelief(context, id + "reliefB", inner, vector(-s[0], -s[1], 0), reach);
    println(toString(definition.trackType) ~ " op2 (" ~ toString(definition.op2Split) ~ "): normal " ~ toString(s) ~
            ", clamped length " ~ roundTo(best.clamped) ~ ", relief bodies A " ~ size(evaluateQuery(context, reliefA)) ~
            " B " ~ size(evaluateQuery(context, reliefB)) ~ ", pitch " ~ roundTo(pitch));
    opDeleteBodies(context, id + "dropInner", { "entities" : inner });
    var transforms = [];
    var names = [];
    for (var k = 1; k < count; k += 1)
    {
        transforms = append(transforms, transform(vector(0, 0, k * pitch) * millimeter));
        names = append(names, "" ~ k);
    }
    var reliefsA = [reliefA];
    var reliefsB = [reliefB];
    var cutters = [pocket];
    if (count > 1)
    {
        for (var part in [["pocket", pocket], ["reliefA", reliefA], ["reliefB", reliefB]])
            opPattern(context, id + (part[0] ~ "Copies"), { "entities" : part[1], "transforms" : transforms, "instanceNames" : names });
        cutters = append(cutters, qCreatedBy(id + "pocketCopies", EntityType.BODY));
        reliefsA = append(reliefsA, qCreatedBy(id + "reliefACopies", EntityType.BODY));
        reliefsB = append(reliefsB, qCreatedBy(id + "reliefBCopies", EntityType.BODY));
    }

    // bolts along s: counterbored clearance in jaw B (the +s jaw), heat-set inserts in jaw A
    const headR = definition.op2HeadD / millimeter / 2;
    var us = [(uMin + uMax) / 2];
    if (uMax - uMin > 55)
        us = [uMin + (uMax - uMin) / 4, uMin + 3 * (uMax - uMin) / 4];
    var zs = [];
    for (var k = 0; k + 1 < count; k += 1)
        zs = append(zs, band[1] + k * pitch + definition.op2Gap / millimeter / 2);
    if (count == 1)
        zs = [z0 + wall / 2, z1 - wall / 2];
    var bolt = 0;
    for (var u in us)
        for (var z in zs)
        {
            const b = add2(scale2(s, c), scale2(tdir, u));
            const at = function(d) { return vector(b[0] + s[0] * d, b[1] + s[1] * d, z) * millimeter; };
            fCylinder(context, id + ("clear" ~ bolt), { "bottomCenter" : at(0), "topCenter" : at(sMax - c + 1), "radius" : definition.op2BoltD / 2 });
            fCylinder(context, id + ("head" ~ bolt), { "bottomCenter" : at(sMax - c - definition.op2HeadDepth / millimeter),
                        "topCenter" : at(sMax - c + 1), "radius" : headR * millimeter });
            fCylinder(context, id + ("insert" ~ bolt), { "bottomCenter" : at(-g - definition.op2InsertDepth / millimeter),
                        "topCenter" : at(0), "radius" : definition.op2InsertD / 2 });
            cutters = concatenateArrays([cutters, [qCreatedBy(id + ("clear" ~ bolt), EntityType.BODY),
                            qCreatedBy(id + ("head" ~ bolt), EntityType.BODY), qCreatedBy(id + ("insert" ~ bolt), EntityType.BODY)]]);
            bolt += 1;
        }
    opBoolean(context, id + "cutA", { "targets" : jawA, "tools" : qUnion(concatenateArrays([cutters, reliefsA])),
                "operationType" : BooleanOperationType.SUBTRACTION, "keepTools" : true });
    opBoolean(context, id + "cutB", { "targets" : jawB, "tools" : qUnion(concatenateArrays([cutters, reliefsB])),
                "operationType" : BooleanOperationType.SUBTRACTION });
    opDeleteBodies(context, id + "dropReliefs", { "entities" : qUnion(reliefsA) });
    nameBodies(context, jawA, trackName(definition.trackType) ~ " op2 jaw A (inserts)", color(0.55, 0.72, 0.85));
    nameBodies(context, jawB, trackName(definition.trackType) ~ " op2 jaw B (bolt heads)", color(0.85, 0.66, 0.43));

    // CAM models: the finished piece in, the piece with its connector holes out
    const stacked = concatenateArrays([[identityTransform()], transforms]);
    if (definition.stockModel)
        placeModel(context, id + "stockModel", pieceBody(context, id + "stock", definition, m), stacked, trackName(definition.trackType) ~ " op2 stock", STOCK_COLOUR);
    if (definition.partModel)
    {
        const part = pieceBody(context, id + "part", definition, m);
        cutConnectorHoles(context, id + "holes", part, m);
        placeModel(context, id + "partModel", part, stacked, trackName(definition.trackType) ~ " op2 part", PART_COLOUR);
    }
}

// Split plane { s (unit normal, jaw B on +s), c (offset along s), clamped (track length pressed) }. END: through
// an end's centreline, square to it, pointing at the other end; the end pressing more track wins (end 1 on ties).
// BOX_X / BOX_Y: through the plan bounding box centre, parallel to template X / Y.
function op2Split(shape is map, outline is array, mode is JawSplit) returns map
{
    if (mode != JawSplit.END)
    {
        const b = polyBounds(outline);
        const s = mode == JawSplit.BOX_X ? [0, 1] : [1, 0];
        return { "s" : s, "c" : dot2([(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2], s), "clamped" : pressedLength(shape, s) };
    }
    var best = undefined;
    for (var e in [1, 0])
    {
        const nE = shape.ends[e][1];
        var sn = unit2([abs(nE[1]) < 1e-4 ? 0 : -nE[1], abs(nE[0]) < 1e-4 ? 0 : nE[0]]);
        if (dot2(sub2(shape.ends[1 - e][0], shape.ends[e][0]), sn) < 0)
            sn = scale2(sn, -1);
        const clamped = pressedLength(shape, sn);
        if (best == undefined || clamped > best.clamped * 1.01)
            best = { "s" : sn, "c" : dot2(shape.ends[e][0], sn), "clamped" : clamped };
    }
    return best;
}

// Centreline length across s: how much track the jaws press when they close along s.
function pressedLength(shape is map, s is array) returns number
{
    var len = 0;
    for (var i = 1; i < size(shape.centre); i += 1)
        len += abs(cross2(sub2(shape.centre[i], shape.centre[i - 1]), s));
    return len;
}

// Connector holes (connector_holes.fs): a symmetric pair square to each end face, the same on every track.
function cutConnectorHoles(context is Context, id is Id, piece is Query, m is map)
{
    const cx = (m.ext.x0 + m.ext.x1) / 2;
    const cz = (m.ext.z0 + m.ext.z1) / 2;
    var holes = [];
    for (var e = 0; e < 2; e += 1)
    {
        const inward = scale2(m.shape.ends[e][1], -1);
        for (var sx in [1, -1])
        {
            const p = pathPoint(m.spec, cx + sx * CONNECTOR_X, cz + CONNECTOR_Z, e);
            const at = function(d) { return vector(p[0] + inward[0] * d, p[1] + inward[1] * d, p[2]) * millimeter; };
            const hid = id + ("e" ~ e ~ (sx > 0 ? "p" : "n"));
            fCylinder(context, hid, { "bottomCenter" : at(-0.5), "topCenter" : at(CONNECTOR_DEPTH), "radius" : CONNECTOR_D / 2 * millimeter });
            holes = append(holes, qCreatedBy(hid, EntityType.BODY));
        }
    }
    opBoolean(context, id + "cut", { "targets" : piece, "tools" : qUnion(holes), "operationType" : BooleanOperationType.SUBTRACTION });
}

// Keep the part of a convex polygon with (q - point) . normal <= -offset.
function clipHalfPlane(poly is array, point is array, normal is array, offset is number) returns array
{
    const nn = unit2(normal);
    const f = function(q) { return dot2(sub2(q, point), nn) + offset; };
    var out = [];
    const n = size(poly);
    for (var i = 0; i < n; i += 1)
    {
        const a = poly[i];
        const b = poly[(i + 1) % n];
        const fa = f(a);
        const fb = f(b);
        if (fa <= 0)
            out = append(out, a);
        if ((fa < 0 && fb > 0) || (fa > 0 && fb < 0))
            out = append(out, add2(a, scale2(sub2(b, a), fa / (fa - fb))));
    }
    return out;
}

function dot2(a is array, b is array) returns number
{
    return a[0] * b[0] + a[1] * b[1];
}

function cross2(a is array, b is array) returns number
{
    return a[0] * b[1] - a[1] * b[0];
}

// Rotation taking s to +X (x' = p.s, y' = p.t).
function frameOf(s is array) returns array
{
    const u = unit2(s);
    return [u[0], u[1], -u[1], u[0], 0, 0];
}

function invertRotation(m is array) returns array
{
    return [m[0], m[2], m[1], m[3], 0, 0];
}

// Volume swept by `pocket` arriving from direction `from` (unit, horizontal), less the pocket itself: its faces
// that look toward `from` (split at the isocline), each extruded that way by `reach`.
function insertionRelief(context is Context, id is Id, pocket is Query, from is Vector, reach is ValueWithUnits) returns Query
{
    const split = opSplitByIsocline(context, id + "split", { "faces" : qOwnedByBody(pocket, EntityType.FACE),
                "direction" : from, "angle" : RELIEF_ANGLE * degree });
    for (var i = 0; i < size(split.nonSteepFaces); i += 1)
        opExtrude(context, id + ("f" ~ i), { "entities" : split.nonSteepFaces[i], "direction" : from,
                    "endBound" : BoundingType.BLIND, "endDepth" : reach });
    return qCreatedBy(id, EntityType.BODY);
}
