// Marble track generator: Onshape port of tools/generate_tracks.py.
// Paste everything below into a Feature Studio, under the two header lines
// Onshape generates (FeatureScript ####; import(...std/...fs...)).
//
// Conventions:
// - The selected sketch region is the profile; sketch X = width, sketch Y = height.
// - The sketch origin is the profile origin, and the sketch is the track's start face.
// - The track runs away from the sketch normal (Front plane -> +Y world).

export enum TrackType
{
    annotation { "Name" : "s" }
    S,
    annotation { "Name" : "sSs" }
    SSS,
    annotation { "Name" : "lr" }
    LR,
    annotation { "Name" : "L" }
    L,
    annotation { "Name" : "R" }
    R,
    annotation { "Name" : "sL" }
    SL,
    annotation { "Name" : "sR" }
    SR,
    annotation { "Name" : "Ls" }
    LS,
    annotation { "Name" : "Rs" }
    RS,
    annotation { "Name" : "slr" }
    SLR,
    annotation { "Name" : "srl" }
    SRL,
    annotation { "Name" : "RL" }
    CURVE_RL,
    annotation { "Name" : "LR" }
    CURVE_LR
}

const GRID_BOUNDS = { (meter) : [0.001, 0.04, 10], (millimeter) : 40, (inch) : 1.5748 } as LengthBoundSpec;
const ELEVATION_BOUNDS = { (meter) : [0.0001, 0.006, 10], (millimeter) : 6, (inch) : 0.2362 } as LengthBoundSpec;
const PROFILE_SAMPLE_BOUNDS = { (unitless) : [4, 9, 65] } as IntegerBoundSpec;
const PATH_SAMPLE_BOUNDS = { (unitless) : [4, 33, 129] } as IntegerBoundSpec;

annotation { "Feature Type Name" : "Marble track" }
export const marbleTrack = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Profile region", "Filter" : EntityType.FACE && SketchObject.YES && ConstructionObject.NO, "MaxNumberOfPicks" : 1 }
        definition.profile is Query;

        annotation { "Name" : "Track type" }
        definition.trackType is TrackType;

        annotation { "Name" : "Grid size" }
        isLength(definition.gridSize, GRID_BOUNDS);

        annotation { "Name" : "Elevation gain" }
        isLength(definition.elevationGain, ELEVATION_BOUNDS);

        if (definition.trackType == TrackType.L || definition.trackType == TrackType.R ||
            definition.trackType == TrackType.SL || definition.trackType == TrackType.SR ||
            definition.trackType == TrackType.LS || definition.trackType == TrackType.RS ||
            definition.trackType == TrackType.CURVE_RL || definition.trackType == TrackType.CURVE_LR)
        {
            annotation { "Name" : "Profile samples per edge" }
            isInteger(definition.profileSamples, PROFILE_SAMPLE_BOUNDS);

            annotation { "Name" : "Path samples per section" }
            isInteger(definition.pathSamples, PATH_SAMPLE_BOUNDS);

            annotation { "Name" : "Surfaces only (debug)" }
            definition.surfacesOnly is boolean;
        }
    }
    {
        const frame = trackFrame(context, definition.profile);
        const spec = trackSpec(definition.trackType, definition.gridSize / millimeter, definition.elevationGain / millimeter);
        if (isSampled(spec))
            buildSampled(context, id, definition, frame, spec);
        else
            buildNative(context, id, definition.profile, frame, spec);
    });

// ---------------------------------------------------------------------------
// Track setup

function trackFrame(context is Context, profile is Query) returns map
{
    const sp = evOwnerSketchPlane(context, { "entity" : profile });
    return {
            "origin" : sp.origin,
            "originMm" : sp.origin / millimeter,
            "x" : sp.x,
            "y" : -sp.normal,
            "z" : cross(sp.normal, sp.x)
        };
}

// y0/z0: template coordinates of the start face, which is placed on the sketch.
function trackSpec(kind is TrackType, gs is number, elev is number) returns map
{
    var spec = { "gs" : gs, "elev" : elev, "y0" : -gs / 2, "z0" : 0 };
    if (kind == TrackType.S)
        spec.mode = "extrude";
    else if (kind == TrackType.SSS)
        spec.mode = "sweep";
    else if (kind == TrackType.LR)
    {
        spec.mode = "revolve";
        spec.curveLeft = false;
    }
    else if (kind == TrackType.L || kind == TrackType.R)
    {
        spec.mode = "curve";
        spec.curveLeft = kind == TrackType.L;
    }
    else if (kind == TrackType.SLR || kind == TrackType.SRL)
    {
        // Python: StraightCurveTransition(left, rising=False, elevation_gain=0)
        spec.mode = "straightRevolve";
        spec.curveLeft = kind == TrackType.SLR;
        spec.y0 = -gs;
    }
    else if (kind == TrackType.CURVE_RL || kind == TrackType.CURVE_LR)
    {
        // Python: CurveCurveTransitionTrackTemplate(left_first); curveLeft is the first turn, rise 2 * elev
        spec.mode = "curveCurve";
        spec.curveLeft = kind == TrackType.CURVE_LR;
    }
    else
    {
        // Python: sL(left, rising), sR(right, rising), Ls(left, falling), Rs(right, falling)
        spec.mode = "transition";
        spec.rising = kind == TrackType.SL || kind == TrackType.SR;
        spec.curveLeft = (kind == TrackType.SL || kind == TrackType.LS) == spec.rising;
        spec.y0 = -gs;
        spec.z0 = spec.rising ? 0 : elev;
    }
    return spec;
}

// Modes built from sampled B-spline patches (the rest use native extrude/sweep/revolve).
function isSampled(spec is map) returns boolean
{
    return spec.mode == "curve" || spec.mode == "transition" || spec.mode == "curveCurve";
}

// ---------------------------------------------------------------------------
// Rigid tracks: native operations

function buildNative(context is Context, id is Id, profile is Query, frame is map, spec is map)
{
    const gs = spec.gs * millimeter;
    if (spec.mode == "extrude")
    {
        opExtrude(context, id + "extrude", {
                    "entities" : profile,
                    "direction" : frame.y,
                    "endBound" : BoundingType.BLIND,
                    "endDepth" : gs
                });
    }
    else if (spec.mode == "sweep")
    {
        // The sSs bezier is a cubic Hermite with end derivatives (gs, 0): exact.
        const rise = (frame.y * spec.gs + frame.z * spec.elev) * millimeter;
        opFitSpline(context, id + "path", {
                    "points" : [frame.origin, frame.origin + rise],
                    "startDerivative" : frame.y * gs,
                    "endDerivative" : frame.y * gs
                });
        opSweep(context, id + "sweep", {
                    "profiles" : profile,
                    "path" : qCreatedBy(id + "path", EntityType.EDGE),
                    "profileControl" : ProfileControlMode.KEEP_ORIENTATION
                });
        opDeleteBodies(context, id + "deletePath", { "entities" : qCreatedBy(id + "path", EntityType.BODY) });
    }
    else if (spec.mode == "revolve")
    {
        revolveQuarter(context, id + "revolve", profile, frame, spec);
    }
    else if (spec.mode == "straightRevolve")
    {
        opExtrude(context, id + "straight", {
                    "entities" : profile,
                    "direction" : frame.y,
                    "endBound" : BoundingType.BLIND,
                    "endDepth" : gs
                });
        revolveQuarter(context, id + "curve", profile, frame, spec);
        opTransform(context, id + "moveCurve", {
                    "bodies" : qCreatedBy(id + "curve", EntityType.BODY),
                    "transform" : transform(frame.y * gs)
                });
        opBoolean(context, id + "union", {
                    "tools" : qUnion([qCreatedBy(id + "straight", EntityType.BODY), qCreatedBy(id + "curve", EntityType.BODY)]),
                    "operationType" : BooleanOperationType.UNION
                });
    }
}

// Quarter turn about a centre gs/2 to the side; right curves turn CCW about +Z.
function revolveQuarter(context is Context, id is Id, profile is Query, frame is map, spec is map)
{
    const side = spec.curveLeft ? 1 : -1;
    opRevolve(context, id, {
                "entities" : profile,
                "axis" : line(frame.origin + frame.x * (side * spec.gs / 2) * millimeter, frame.z * -side),
                "angleForward" : 90 * degree
            });
}

// ---------------------------------------------------------------------------
// Non-rigid tracks: sample the track equations, fit B-spline patches, enclose

function buildSampled(context is Context, id is Id, definition is map, frame is map, spec is map)
{
    const us = uniformParameters(definition.profileSamples);
    const vs = uniformParameters(definition.pathSamples);
    const uKnots = interpolationKnots(size(us));
    const vKnots = interpolationKnots(size(vs));
    const uRows = interpolationRows(us, uKnots);
    const vRows = interpolationRows(vs, vKnots);
    // Transitions get one patch per section (straight + curve, or curve + curve).
    const branchCount = spec.mode == "transition" || spec.mode == "curveCurve" ? 2 : 1;

    const edges = evaluateQuery(context, qAdjacent(definition.profile, AdjacencyType.EDGE, EntityType.EDGE));
    if (size(edges) == 0)
        throw regenError("Profile region has no edges", ["profile"]);

    var patch = 0;
    for (var edge in edges)
    {
        var paths = [];
        for (var tangent in evEdgeTangentLines(context, { "edge" : edge, "parameters" : us }))
        {
            const offset = tangent.origin - frame.origin;
            const px = dot(offset, frame.x) / millimeter;
            const pz = dot(offset, frame.z) / millimeter;
            if (curveRadius(spec.curveLeft, spec.gs / 2, px) <= 1e-6 ||
                (spec.mode == "curveCurve" && curveRadius(!spec.curveLeft, spec.gs / 2, px) <= 1e-6))
                throw regenError("Profile reaches or crosses the bend axis; increase grid size", ["profile"]);
            paths = append(paths, pointPath(spec, px, pz));
        }
        for (var branch = 0; branch < branchCount; branch += 1)
        {
            var grid = [];
            for (var path in paths)
            {
                const t0 = branch == 0 ? 0 : path.split;
                const t1 = branch == 0 ? path.split : 1;
                var row = [];
                for (var v in vs)
                    row = append(row, toWorldMm(frame, spec, samplePath(path, t0 + v * (t1 - t0))));
                grid = append(grid, row);
            }
            opCreateBSplineSurface(context, id + "patches" + ("p" ~ patch), {
                        "bSplineSurface" : bSplineSurface({
                                    "uDegree" : 3,
                                    "vDegree" : 3,
                                    "isUPeriodic" : false,
                                    "isVPeriodic" : false,
                                    "controlPoints" : controlPointMatrix(withMillimeters(interpolateGrid(grid, uRows, vRows))),
                                    "uKnots" : knotArray(uKnots),
                                    "vKnots" : knotArray(vKnots)
                                })
                    });
            patch += 1;
        }
    }

    if (definition.surfacesOnly)
        return;

    // Caps: the sketch plane, and the plane through the curve's rotation centre (curve + curve: one grid
    // size along the start direction, both ends face the same way).
    const endSide = spec.curveLeft ? 1 : -1;
    opPlane(context, id + "startPlane", { "plane" : plane(frame.origin, -frame.y) });
    const endPlane = spec.mode == "curveCurve" ? plane(frame.origin + frame.y * spec.gs * millimeter, frame.y) :
        plane(frame.origin + frame.x * (endSide * spec.gs / 2) * millimeter, frame.x * endSide);
    opPlane(context, id + "endPlane", { "plane" : endPlane });
    const bounds = qUnion([
                qCreatedBy(id + "patches", EntityType.BODY),
                qCreatedBy(id + "startPlane", EntityType.BODY),
                qCreatedBy(id + "endPlane", EntityType.BODY)
            ]);
    opEnclose(context, id + "enclose", { "entities" : bounds });
    opDeleteBodies(context, id + "cleanup", { "entities" : bounds });
}

function toWorldMm(frame is map, spec is map, p is array) returns Vector
{
    return frame.originMm + frame.x * p[0] + frame.y * (p[1] - spec.y0) + frame.z * (p[2] - spec.z0);
}

function withMillimeters(poles is array) returns array
{
    var result = poles;
    for (var i = 0; i < size(poles); i += 1)
        for (var j = 0; j < size(poles[i]); j += 1)
            result[i][j] = poles[i][j] * millimeter;
    return result;
}

// ---------------------------------------------------------------------------
// Track equations (mm, template coordinates), ported from generate_tracks.py.
// The per-point Z clamp is dropped, matching STEP mode.

function bezier(p0 is array, m1 is array, p3 is array, m2 is array, t is number) returns array
{
    const pl = p3[0] - p0[0];
    const p1 = [p0[0] + pl * m1[0] / 3, p0[1] + pl * m1[1] / 3];
    const p2 = [p3[0] - pl * m2[0] / 3, p3[1] - pl * m2[1] / 3];
    const s = 1 - t;
    const a = s * s * s;
    const b = 3 * s * s * t;
    const c = 3 * s * t * t;
    const d = t * t * t;
    return [a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0],
            a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]];
}

function curveRadius(left is boolean, gs2 is number, px is number) returns number
{
    return left ? gs2 - px : gs2 + px;
}

function curveBase(left is boolean, gs2 is number, px is number, pz is number, t is number) returns array
{
    const theta = t * PI / 2 * radian;
    const r = curveRadius(left, gs2, px);
    if (left)
        return [gs2 - cos(theta) * r, -gs2 + sin(theta) * r, pz];
    return [-gs2 + cos(theta) * r, -gs2 + sin(theta) * r, pz];
}

// Per profile point setup (Python's set_sample_pt).
function pointPath(spec is map, px is number, pz is number) returns map
{
    var path = { "spec" : spec, "px" : px, "pz" : pz, "split" : 1 };
    if (spec.mode == "curveCurve")
        return curveCurvePath(path);
    if (spec.mode != "transition")
        return path;

    const gs = spec.gs;
    const e = spec.elev;
    const curveLength = curveRadius(spec.curveLeft, gs / 2, px) * PI / 2;
    const mag = sqrt(curveLength * curveLength + e * e);
    var slope = [curveLength / mag, spec.rising ? e / mag : -e / mag];
    if (abs(slope[1]) > 0.5)
        slope = [sqrt(3) / 2, spec.rising ? 0.5 : -0.5];
    const pl = gs + curveLength;
    path.curveLength = curveLength;
    path.slope = slope;
    path.p0 = [0, spec.rising ? 0 : e];
    path.p3 = [pl, spec.rising ? e : 0];

    // Bezier parameter where the base path leaves the straight (x == gs).
    var lo = 0;
    var hi = 1;
    for (var i = 0; i < 60; i += 1)
    {
        const mid = (lo + hi) / 2;
        if (bezier(path.p0, [1, 0], path.p3, slope, mid)[0] < gs)
            lo = mid;
        else
            hi = mid;
    }
    path.split = (lo + hi) / 2;
    return path;
}

// Python's CurveCurveTransitionTrackTemplate.set_sample_pt: bezier height from the first turn's slope to the
// second's, rising 2 * elev over both turns.
function curveCurvePath(path is map) returns map
{
    const spec = path.spec;
    const e = spec.elev;
    var out = path;
    out.l1 = curveRadius(spec.curveLeft, spec.gs / 2, path.px) * PI / 2;
    const l2 = curveRadius(!spec.curveLeft, spec.gs / 2, path.px) * PI / 2;
    out.pl = out.l1 + l2;
    out.slope1 = turnSlope(out.l1, e);
    out.slope2 = turnSlope(l2, e);
    out.p0 = [0, 0];
    out.p3 = [out.pl, 2 * e];
    // Bezier parameter where the base path leaves the first turn.
    var lo = 0;
    var hi = 1;
    for (var i = 0; i < 60; i += 1)
    {
        const mid = (lo + hi) / 2;
        if (bezier(out.p0, out.slope1, out.p3, out.slope2, mid)[0] < out.l1)
            lo = mid;
        else
            hi = mid;
    }
    out.split = (lo + hi) / 2;
    return out;
}

// Unit slope of a rising quarter turn of length `len` (Python's get_slope), clamped to 30 degrees.
function turnSlope(len is number, e is number) returns array
{
    const mag = sqrt(len * len + e * e);
    return e / mag > 0.5 ? [sqrt(3) / 2, 0.5] : [len / mag, e / mag];
}

function samplePath(path is map, t is number) returns array
{
    const spec = path.spec;
    const gs = spec.gs;
    if (spec.mode == "curve")
    {
        var p = curveBase(spec.curveLeft, gs / 2, path.px, path.pz, t);
        p[2] += t * spec.elev;
        return p;
    }
    if (spec.mode == "curveCurve")
    {
        const xy = bezier(path.p0, path.slope1, path.p3, path.slope2, t);
        var p = curveBase(spec.curveLeft, gs / 2, path.px, path.pz, xy[0] / path.l1);
        if (xy[0] > path.l1)
        {
            // second turn: a quarter turn toward the first turn's side, one grid size over
            const q = curveBase(!spec.curveLeft, gs / 2, path.px, path.pz, (xy[0] - path.l1) / (path.pl - path.l1));
            p = spec.curveLeft ? [q[1] + gs, -q[0], q[2]] : [-q[1] - gs, q[0], q[2]];
        }
        p[2] += xy[1];
        return p;
    }
    const xy = bezier(path.p0, [1, 0], path.p3, path.slope, t);
    var p = [path.px, xy[0] - gs, path.pz];
    if (xy[0] > gs)
    {
        p = curveBase(spec.curveLeft, gs / 2, path.px, path.pz, (xy[0] - gs) / path.curveLength);
        p[1] += gs / 2;
    }
    p[2] += xy[1];
    return p;
}

// ---------------------------------------------------------------------------
// Cubic B-spline interpolation (uniform parameters, averaged knots)

function uniformParameters(count is number) returns array
{
    var params = [];
    for (var i = 0; i < count; i += 1)
        params = append(params, i / (count - 1));
    return params;
}

function interpolationKnots(count is number) returns array
{
    const m = count - 1;
    var knots = [0, 0, 0, 0];
    for (var j = 1; j <= m - 3; j += 1)
        knots = append(knots, (j + 1) / m);
    return concatenateArrays([knots, [1, 1, 1, 1]]);
}

// Row k holds the basis functions evaluated at params[k].
function interpolationRows(params is array, knots is array) returns array
{
    const count = size(params);
    const last = count - 1;
    var rows = [];
    for (var k = 0; k < count; k += 1)
    {
        var row = makeArray(count, 0);
        if (k == 0)
            row[0] = 1;
        else if (k == last)
            row[last] = 1;
        else
        {
            const u = params[k];
            var span = 3;
            while (span < last && knots[span + 1] <= u)
                span += 1;
            const basis = basisFunctions(span, u, knots);
            for (var r = 0; r <= 3; r += 1)
                row[span - 3 + r] = basis[r];
        }
        rows = append(rows, row);
    }
    return rows;
}

// Piegl & Tiller A2.2, degree 3.
function basisFunctions(span is number, u is number, knots is array) returns array
{
    var n = [1, 0, 0, 0];
    var left = [0, 0, 0, 0];
    var right = [0, 0, 0, 0];
    for (var j = 1; j <= 3; j += 1)
    {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        var saved = 0;
        for (var r = 0; r < j; r += 1)
        {
            const temp = n[r] / (right[r + 1] + left[j - r]);
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    return n;
}

// grid[u][v] of points -> control points [u][v].
function interpolateGrid(grid is array, uRows is array, vRows is array) returns array
{
    const nu = size(grid);
    const nv = size(grid[0]);
    var alongV = makeArray(nv);
    for (var j = 0; j < nv; j += 1)
    {
        var column = makeArray(nu);
        for (var i = 0; i < nu; i += 1)
            column[i] = grid[i][j];
        alongV[j] = column;
    }
    const q = solveLinear(vRows, alongV);
    var alongU = makeArray(nu);
    for (var i = 0; i < nu; i += 1)
    {
        var column = makeArray(nv);
        for (var j = 0; j < nv; j += 1)
            column[j] = q[j][i];
        alongU[i] = column;
    }
    return solveLinear(uRows, alongU);
}

// Solve A X = B; B rows are arrays of vectors. Gaussian elimination, partial pivoting.
function solveLinear(A is array, B is array) returns array
{
    const n = size(A);
    var a = A;
    var b = B;
    for (var col = 0; col < n; col += 1)
    {
        var pivot = col;
        for (var r = col + 1; r < n; r += 1)
        {
            if (abs(a[r][col]) > abs(a[pivot][col]))
                pivot = r;
        }
        const rowA = a[col];
        a[col] = a[pivot];
        a[pivot] = rowA;
        const rowB = b[col];
        b[col] = b[pivot];
        b[pivot] = rowB;
        for (var r = col + 1; r < n; r += 1)
        {
            const f = a[r][col] / a[col][col];
            if (f != 0)
            {
                for (var c = col; c < n; c += 1)
                    a[r][c] -= f * a[col][c];
                for (var c = 0; c < size(b[r]); c += 1)
                    b[r][c] -= f * b[col][c];
            }
        }
    }
    var x = makeArray(n);
    for (var r = n - 1; r >= 0; r -= 1)
    {
        var row = b[r];
        for (var c = r + 1; c < n; c += 1)
        {
            for (var k = 0; k < size(row); k += 1)
                row[k] -= a[r][c] * x[c][k];
        }
        for (var k = 0; k < size(row); k += 1)
            row[k] = row[k] / a[r][r];
        x[r] = row;
    }
    return x;
}
