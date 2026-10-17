"""DXF reconstruction benchmark, continuous surfaces, and watertight end caps."""

from collections import Counter
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BENCHMARK = ROOT / "data/track_profiles/trackprofile.dxf"
VERTICES = ROOT / "tests/fixtures/track_profiles/trackprofile.csv"

try:
    import ezdxf
    import scipy
except ImportError:
    HAVE_DXF = False
else:
    HAVE_DXF = True

try:
    import OCP
except ImportError:
    HAVE_STEP = False
else:
    HAVE_STEP = True


@unittest.skipUnless(HAVE_DXF, "Install tools/requirements-tracks.txt for DXF checks")
class DxfProfileTests(unittest.TestCase):
    def test_reconstructs_rotated_arcs_only_capsule(self):
        from tools.convert_track_profile import reconstruct_profile

        angles = np.linspace(-np.pi / 2, np.pi / 2, 41)
        right = [5, 0] + 2 * np.column_stack((np.cos(angles), np.sin(angles)))
        left = [-5, 0] + 2 * np.column_stack((-np.cos(angles), -np.sin(angles)))
        points = np.vstack(([-5, -2], right, left))
        angle = 0.37
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        points = points @ rotation.T + [7, -1]
        points[-1] = points[0]
        _, report = reconstruct_profile(points)
        self.assertEqual(report["entity_counts"], {"LINE": 2, "ARC": 2})
        self.assertLess(report["max_vertex_residual_mm"], 1e-10)

    def test_obj_reconstruction_benchmark(self):
        from scipy.optimize import minimize_scalar
        from scipy.spatial import cKDTree
        from tools.convert_track_profile import reconstruct_profile
        from tools.track_dxf import load_profile_dxf, write_profile_dxf

        points = np.loadtxt(VERTICES, delimiter=",")
        segments, report = reconstruct_profile(np.vstack((points, points[0])))
        self.assertEqual(report["input_vertices"], 389)
        self.assertEqual(report["entity_counts"], {"LINE": 5, "ARC": 6, "ELLIPSE": 1})
        self.assertLess(report["max_vertex_residual_mm"], 3e-6)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.dxf"
            write_profile_dxf(segments, path)
            profile = load_profile_dxf(path)
            self.assertTrue(profile.closed)
            self.assertEqual(len(profile.segments), 12)
            self.assertEqual(Counter(s.kind for s in profile.segments), report["entity_counts"])
            # Independent closest-point check against the reimported DXF, with
            # dense bracketing followed by one-dimensional local minimization.
            nodes = np.linspace(0, 1, 513)
            dense = np.concatenate([s.points(nodes) for s in profile.segments])
            _, indices = cKDTree(dense).query(points)
            errors = []
            for point, index in zip(points, indices):
                span, sample = divmod(index, len(nodes))
                segment = profile.segments[span]
                def distance(u):
                    return float(np.linalg.norm(segment.points([u])[0] - point))
                result = minimize_scalar(distance, bounds=(nodes[max(0, sample - 1)],
                                                          nodes[min(len(nodes) - 1, sample + 1)]),
                                         method="bounded", options={"xatol": 1e-14})
                errors.append(min(result.fun, distance(0), distance(1)))
            self.assertLess(max(errors), 4e-6)
            stored = load_profile_dxf(BENCHMARK)
            np.testing.assert_allclose(profile.bounds(), stored.bounds(), atol=1e-9)

    def test_unordered_reversed_entities_and_unit_conversion(self):
        from tools.track_dxf import load_profile_dxf

        document = ezdxf.new(units=ezdxf.units.MM)
        document.units = ezdxf.units.IN
        model = document.modelspace()
        model.add_line((1, 1), (0, 1))
        model.add_line((0, 0), (1, 0))
        model.add_line((0, 1), (0, 0))
        model.add_line((1, 1), (1, 0))
        model.add_line((10, 0), (20, 0), dxfattribs={"layer": "OTHER"})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "inches.dxf"
            document.saveas(path)
            with self.assertRaisesRegex(ValueError, "contour"):
                load_profile_dxf(path)
            profile = load_profile_dxf(path, layer="0")
            self.assertTrue(profile.closed)
            np.testing.assert_allclose(profile.bounds(), [[0, 0], [25.4, 25.4]], atol=1e-8)
            for a, b in zip(profile.segments, np.roll(profile.segments, -1)):
                np.testing.assert_array_equal(a.points([1.]), b.points([0.]))

    def test_bulges_circle_spline_and_rejects_nonplanar_geometry(self):
        from tools.track_dxf import load_profile_dxf

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "curves.dxf"
            document = ezdxf.new(units=ezdxf.units.MM)
            model = document.modelspace()
            model.add_lwpolyline([(-3, 0, 1), (3, 0, 0)], format="xyb", close=True)
            document.saveas(path)
            profile = load_profile_dxf(path)
            self.assertEqual(Counter(s.kind for s in profile.segments), {"ARC": 1, "LINE": 1})
            np.testing.assert_allclose(profile.bounds(), [[-3, -3], [3, 0]], atol=1e-8)
            document = ezdxf.new(units=ezdxf.units.MM)
            document.modelspace().add_circle((0, 0), 4)
            document.saveas(path)
            profile = load_profile_dxf(path)
            self.assertEqual(len(profile.segments), 1)
            np.testing.assert_allclose(profile.bounds(), [[-4, -4], [4, 4]], atol=1e-8)
            document = ezdxf.new(units=ezdxf.units.MM)
            document.modelspace().add_spline([(-4, 0), (-2, 3), (1, 1), (4, 0)])
            document.saveas(path)
            profile = load_profile_dxf(path, open_path=True)
            self.assertFalse(profile.closed)
            self.assertEqual(len(profile.segments), 1)
            sampled = profile.sample(0.2)
            bounds = profile.bounds()
            self.assertTrue(np.all(sampled >= bounds[0] - 1e-9))
            self.assertTrue(np.all(sampled <= bounds[1] + 1e-9))
            document.modelspace().add_line((0, 0, 0), (1, 1, 1))
            document.saveas(path)
            with self.assertRaisesRegex(ValueError, "XY plane"):
                load_profile_dxf(path)

    def test_small_endpoint_gaps_and_spline_corners(self):
        from tools.track_dxf import load_profile_dxf

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joined.dxf"
            document = ezdxf.new(units=ezdxf.units.MM)
            model = document.modelspace()
            model.add_line((0, 0), (4, 0))
            model.add_line((4 + 5e-6, 0), (4, 4))
            model.add_line((4, 4), (0, 4))
            model.add_line((0, 4), (0, 0))
            document.saveas(path)
            profile = load_profile_dxf(path)
            for a, b in zip(profile.segments, np.roll(profile.segments, -1)):
                np.testing.assert_array_equal(a.points([1.]), b.points([0.]))
            # A stricter tolerance leaves an open chain. Existing generator
            # semantics add a closing line unless --open-path is requested.
            strict = load_profile_dxf(path, join_tolerance=1e-7)
            self.assertEqual(len(strict.segments), 5)
            self.assertFalse(load_profile_dxf(path, open_path=True, join_tolerance=1e-7).closed)
            # A degree-one spline's interior knot is a deliberate corner.
            document = ezdxf.new(units=ezdxf.units.MM)
            document.modelspace().add_open_spline([(0, 0), (3, 3), (5, 0)], degree=1)
            document.saveas(path)
            profile = load_profile_dxf(path, open_path=True)
            self.assertEqual(len(profile.segments), 2)
            np.testing.assert_allclose(profile.segments[0].points([1.]), [[3, 3]])

    def test_capped_concave_obj_is_manifold(self):
        from tools.generate_tracks import create_all_tracks
        from tools.track_dxf import load_profile_dxf
        from tools.track_exports import save_obj

        profile = load_profile_dxf(BENCHMARK)
        _, contours = create_all_tracks(40, 6, profile.sample(), resolution=5)
        for contour in contours:
            text = save_obj(contour, cap_ends=True)
            edges = Counter()
            directed = Counter()
            for line in text.splitlines():
                if line.startswith("f "):
                    vertices = list(map(int, line.split()[1:]))
                    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
                        edges[tuple(sorted((a, b)))] += 1
                        directed[a, b] += 1
            self.assertTrue(all(count == 2 for count in edges.values()))
            self.assertTrue(all(directed[a, b] == directed[b, a] for a, b in directed))


@unittest.skipUnless(HAVE_DXF and HAVE_STEP, "Install tools/requirements-tracks.txt for STEP checks")
class CappedStepTests(unittest.TestCase):
    def test_benchmark_all_nine_capped_solids_and_formula_samples(self):
        from OCP.BRep import BRep_Tool
        from OCP.BRepCheck import BRepCheck_Analyzer
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPControl import STEPControl_Reader
        from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID
        from OCP.TopExp import TopExp, TopExp_Explorer
        from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
        from OCP.TopoDS import TopoDS
        from tools.generate_tracks import create_track_templates
        from tools.track_dxf import load_profile_dxf
        from tools.track_step import export_step, fit_track_surfaces

        profile = load_profile_dxf(BENCHMARK)
        lower, upper = profile.bounds()
        names, templates = create_track_templates(40, 6)
        rng = np.random.default_rng(529)
        with tempfile.TemporaryDirectory() as directory:
            for name, template in zip(names, templates):
                with self.subTest(track=name):
                    path = Path(directory) / f"{name}.step"
                    result = export_step(template, profile, path, cap_ends=True)
                    expected_patches = 24 if hasattr(template, "curve_segment") else 12
                    self.assertEqual(result.patch_count, expected_patches)
                    self.assertTrue(result.capped)
                    reader = STEPControl_Reader()
                    self.assertEqual(reader.ReadFile(str(path)), IFSelect_RetDone)
                    self.assertGreater(reader.TransferRoots(), 0)
                    shape = reader.OneShape()
                    self.assertTrue(BRepCheck_Analyzer(shape).IsValid())
                    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
                    self.assertTrue(explorer.More())
                    explorer.Next()
                    self.assertFalse(explorer.More())
                    adjacency = TopTools_IndexedDataMapOfShapeListOfShape()
                    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, adjacency)
                    self.assertTrue(all(adjacency.FindFromIndex(i).Extent() == 2
                                        for i in range(1, adjacency.Extent() + 1)))
                    properties = GProp_GProps()
                    BRepGProp.VolumeProperties_s(shape, properties)
                    self.assertGreater(properties.Mass(), 100)
                    surfaces = []
                    explorer = TopExp_Explorer(shape, TopAbs_FACE)
                    while explorer.More():
                        surfaces.append(BRep_Tool.Surface_s(TopoDS.Face_s(explorer.Current())))
                        explorer.Next()
                    self.assertEqual(len(surfaces), expected_patches + 2)
                    patches, _ = fit_track_surfaces(template, profile)
                    for patch in patches:
                        center = patch.surface.Value(0.5, 0.5)
                        surface = min(surfaces, key=lambda s: s.Value(0.5, 0.5).Distance(center))
                        for u, v in rng.random((17, 2)):
                            point = profile.segments[patch.profile_edge].points([u])[0]
                            template.set_sample_pt(point)
                            t = v
                            if patch.branch != -1:
                                join = 40 / template.get_base_path_length(point)
                                t = v * join if patch.branch == 0 else join + v * (1 - join)
                            expected = template.sample_path_t(t)
                            expected[2] = np.clip(expected[2], lower[1] - 6, upper[1] + 6)
                            actual = surface.Value(float(u), float(v)).Coord()
                            self.assertLess(np.linalg.norm(actual - expected), 0.01)

    def test_caps_on_legacy_profile_and_rejects_open_profile(self):
        from tools.generate_tracks import create_track_templates
        from tools.track_profiles import default_profile
        from tools.track_step import export_step

        _, templates = create_track_templates(40, 6)
        with tempfile.TemporaryDirectory() as directory:
            result = export_step(templates[0], default_profile(), Path(directory) / "legacy.step", cap_ends=True)
            self.assertTrue(result.capped)
            with self.assertRaisesRegex(ValueError, "closed profile"):
                export_step(templates[0], default_profile(True), Path(directory) / "open.step", cap_ends=True)


if __name__ == "__main__":
    unittest.main()
