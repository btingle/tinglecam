// Dev-only: the L_track_final profile (models/2D/trackprofile.dxf) as a sketch on the Front plane,
// so jig features in the review tab have a real profile region to pick.

annotation { "Feature Type Name" : "Jig dev: track profile" }
export const jigDevProfile = defineFeature(function(context is Context, id is Id, definition is map)
    precondition {}
    {
        const sk = newSketchOnPlane(context, id + "sketch", { "sketchPlane" : plane(vector(0, 0, 0) * millimeter, vector(0, -1, 0), vector(1, 0, 0)) });
        const v = function(x, y) { return vector(x, y) * millimeter; };
        const h = sqrt(0.5);
        const a = 8.6925;   // bottom fillet centres at (+-a, -2.6925), r 3.3075
        const r = 3.3075;
        const yc = -2.6925;
        skLineSegment(sk, "bottom", { "start" : v(a, -6), "end" : v(-a, -6) });
        skArc(sk, "filletL", { "start" : v(-a, -6), "mid" : v(-a - r * h, yc - r * h), "end" : v(-12, yc) });
        skLineSegment(sk, "sideL", { "start" : v(-12, yc), "end" : v(-12, 5) });
        skArc(sk, "lipLo", { "start" : v(-12, 5), "mid" : v(-11 - h, 5 + h), "end" : v(-11, 6) });
        skLineSegment(sk, "lipL", { "start" : v(-11, 6), "end" : v(-10, 6) });
        skArc(sk, "lipLi", { "start" : v(-10, 6), "mid" : v(-10 + h, 5 + h), "end" : v(-9, 5) });
        // half ellipse 18 wide, 8 deep, centred at (0, 5); parameters are fractions of a turn
        skEllipticalArc(sk, "channel", { "center" : v(0, 5), "majorAxis" : vector(1, 0), "majorRadius" : 9 * millimeter,
                    "minorRadius" : 8 * millimeter, "startParameter" : 0.5, "endParameter" : 1 });
        skArc(sk, "lipRi", { "start" : v(9, 5), "mid" : v(10 - h, 5 + h), "end" : v(10, 6) });
        skLineSegment(sk, "lipR", { "start" : v(10, 6), "end" : v(11, 6) });
        skArc(sk, "lipRo", { "start" : v(11, 6), "mid" : v(11 + h, 5 + h), "end" : v(12, 5) });
        skLineSegment(sk, "sideR", { "start" : v(12, 5), "end" : v(12, yc) });
        skArc(sk, "filletR", { "start" : v(12, yc), "mid" : v(a + r * h, yc - r * h), "end" : v(a, -6) });
        skSolve(sk);
    });
