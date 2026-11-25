"""Thin OpenCascade helpers: primitives, booleans, transforms, queries, export."""
from math import radians

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon,
                                BRepBuilderAPI_MakeWire, BRepBuilderAPI_Transform)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.Bnd import Bnd_Box
from OCP.GC import GC_MakeArcOfCircle
from OCP.GProp import GProp_GProps
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopTools import TopTools_ListOfShape
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec


def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def cylinder(x, y, z0, z1, d):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), d / 2, z1 - z0).Shape()


def cone(x, y, z0, z1, d0, d1):
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), d0 / 2, d1 / 2, z1 - z0).Shape()


def _ring_wire(coords, z):
    poly = BRepBuilderAPI_MakePolygon()
    for x, y in np.asarray(coords)[:-1]:
        poly.Add(gp_Pnt(float(x), float(y), z))
    poly.Close()
    return poly.Wire()


def prism(polygon, z0, z1):
    """Extrude a shapely Polygon (holes allowed) from z0 to z1."""
    face = BRepBuilderAPI_MakeFace(_ring_wire(polygon.exterior.coords, z0), True)
    for interior in polygon.interiors:
        face.Add(_ring_wire(interior.coords, z0).Reversed())
    return BRepPrimAPI_MakePrism(face.Face(), gp_Vec(0, 0, z1 - z0)).Shape()


def rounded_rect(x0, y0, x1, y1, r, z0, z1):
    """Box with true arcs on its vertical edges."""
    p = lambda x, y: gp_Pnt(x, y, z0)
    c = [(x1 - r, y1 - r), (x0 + r, y1 - r), (x0 + r, y0 + r), (x1 - r, y0 + r)]
    wire = BRepBuilderAPI_MakeWire()
    corners = [((x1, y1 - r), (x1 - r, y1)), ((x0 + r, y1), (x0, y1 - r)),
               ((x0, y0 + r), (x0 + r, y0)), ((x1 - r, y0), (x1, y0 + r))]
    for i, ((a, b), (cx, cy)) in enumerate(zip(corners, c)):
        mid_angle = radians(45 + 90 * i)
        m = (cx + r * np.cos(mid_angle), cy + r * np.sin(mid_angle))
        wire.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(p(*a), p(*m), p(*b)).Value()).Edge())
        nxt = corners[(i + 1) % 4][0]
        wire.Add(BRepBuilderAPI_MakeEdge(p(*b), p(*nxt)).Edge())
    face = BRepBuilderAPI_MakeFace(wire.Wire(), True).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, z1 - z0)).Shape()


def _list(shapes):
    out = TopTools_ListOfShape()
    for s in shapes:
        out.Append(s)
    return out


def _boolean(op, a, tools):
    tools = [tools] if not isinstance(tools, (list, tuple)) else list(tools)
    if not tools:
        return a
    b = op()
    b.SetArguments(_list([a]))
    b.SetTools(_list(tools))
    b.SetRunParallel(True)
    b.Build()
    if not b.IsDone():
        raise RuntimeError(f"{op.__name__} failed")
    unify = ShapeUpgrade_UnifySameDomain(b.Shape(), True, True, False)
    unify.Build()
    return unify.Shape()


def fuse(a, *others):
    return _boolean(BRepAlgoAPI_Fuse, a, list(others))


def cut(a, *others):
    return _boolean(BRepAlgoAPI_Cut, a, list(others))


def common(a, b):
    return _boolean(BRepAlgoAPI_Common, a, [b])


def trsf_matrix(m):
    """gp_Trsf from a 3x4 row-major affine (rotation/mirror + translation)."""
    t = gp_Trsf()
    m = np.asarray(m, float)
    t.SetValues(*m[0], *m[1], *m[2])
    return t


def transform(shape, m):
    return BRepBuilderAPI_Transform(shape, trsf_matrix(m), True).Shape()


def affine2d(m2, z=0.0):
    """Lift a shapely 2D affine [a, b, d, e, xoff, yoff] to 3x4."""
    a, b, d, e, xo, yo = m2
    return [[a, b, 0, xo], [d, e, 0, yo], [0, 0, 1, z]]


def solids(shape):
    out, ex = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        out.append(TopoDS.Solid_s(ex.Current()))
        ex.Next()
    return out


def compound(shapes):
    comp, builder = TopoDS_Compound(), BRep_Builder()
    builder.MakeCompound(comp)
    for s in shapes:
        builder.Add(comp, s)
    return comp


def volume(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    return g.Mass()


def centroid(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass()
    return np.array([c.X(), c.Y(), c.Z()])


def bounds(shape):
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, b, False, False)
    return np.array(b.Get())


def distance(a, b):
    d = BRepExtrema_DistShapeShape(a, b)
    d.Perform()
    return d.Value()


def valid(shape):
    return BRepCheck_Analyzer(shape).IsValid()


def triangles(shape, deflection=0.1):
    """(N, 3, 3) triangles for previews."""
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.3, True)
    tris, ex = [], TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            tr = loc.Transformation()
            pts = np.array([[p.X(), p.Y(), p.Z()] for p in
                            (tri.Node(i).Transformed(tr) for i in range(1, tri.NbNodes() + 1))])
            reverse = face.Orientation() == 1
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                tris.append(pts[[a - 1, c - 1, b - 1] if reverse else [a - 1, b - 1, c - 1]])
        ex.Next()
    return np.array(tris)


def write_stl(shape, path, deflection=0.02):
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.2, True)
    w = StlAPI_Writer()
    w.ASCIIMode = False
    if not w.Write(shape, str(path)):
        raise OSError(f"Could not write STL: {path}")
