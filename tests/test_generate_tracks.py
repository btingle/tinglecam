"""Run with: python3 -m unittest discover -s tests -p 'test_generate_tracks.py'."""

import contextlib
import io
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.generate_tracks import create_all_tracks, create_track_templates, parse_args
from tools.track_exports import save_gcode, save_obj
from tools.track_profiles import default_profile, load_profile_obj, prepare_profile, resample_profile


class ProfileAndExportTests(unittest.TestCase):
    def test_obj_profile_records_closure_and_resampling(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.obj"
            path.write_text("  v -3 1 0\nvn 0 0 1\nvt .2 .3\nv -3 1 0\n"
                            "v 0 2.13 0 # corner\nv 3 1 0\nl 1 2\n")
            opened = load_profile_obj(path, open_path=True)
            closed = load_profile_obj(path)
            self.assertEqual(opened.shape, (3, 2))
            self.assertEqual(closed.shape, (4, 2))
            np.testing.assert_array_equal(closed[0], closed[-1])
            sampled = resample_profile(opened)
            np.testing.assert_array_equal(sampled[0], opened[0])
            np.testing.assert_array_equal(sampled[-1], opened[-1])
            self.assertTrue(np.any(np.all(sampled == opened[1], axis=1)))
            np.testing.assert_array_equal(prepare_profile(closed), closed)

    def test_invalid_profiles(self):
        for points in ([], [[0, 0]], [[0, 0], [0, 0]], [[0, 0], [np.nan, 1]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                prepare_profile(points)

    def test_gcode_preview_order_translation_and_valid_polyline_indices(self):
        contours = np.arange(4 * 300 * 3, dtype=float).reshape(4, 300, 3)
        gcode, obj = save_gcode(contours, translate=(10, -4, 2))
        expected = np.concatenate([row if i % 2 == 0 else row[::-1]
                                   for i, row in enumerate(contours)]) + [10, -4, 2]
        vertices = np.array([[float(x) for x in line.split()[1:]]
                             for line in obj.splitlines() if line.startswith("v ")])
        np.testing.assert_array_equal(vertices, expected)
        moves = np.array([[float(x[1:]) for x in line.split()]
                         for line in gcode.splitlines() if line.startswith("X")])
        np.testing.assert_array_equal(moves, expected)
        edges = []
        for line in obj.splitlines():
            if line.startswith("l "):
                indices = list(map(int, line.split()[1:]))
                self.assertTrue(all(1 <= i <= len(vertices) for i in indices))
                edges.extend(zip(indices[:-1], indices[1:]))
        self.assertEqual(edges, list(zip(range(1, 1200), range(2, 1201))))

    def test_mesh_counts_and_formula_samples(self):
        profile = default_profile()
        with contextlib.redirect_stdout(io.StringIO()):
            names, results = create_all_tracks(40, 6, profile, resolution=7)
        _, templates = create_track_templates(40, 6)
        for template, contours in zip(templates, results):
            for i, pt in enumerate(profile):
                template.set_sample_pt(pt)
                for j in range(7):
                    expected = template.sample_path_t(j / 6)
                    expected[2] = np.clip(expected[2], -12, 12)
                    np.testing.assert_allclose(contours[i, j], expected, atol=1e-12)
            text = save_obj(contours)
            self.assertEqual(sum(line.startswith("v ") for line in text.splitlines()), 35)
            self.assertEqual(sum(line.startswith("f ") for line in text.splitlines()), 24)
        self.assertEqual(len(names), 9)

    def test_cli_formats_are_exclusive(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--step", "--gcode"])


try:
    import OCP
    import scipy
except ImportError:
    HAVE_STEP = False
else:
    HAVE_STEP = True


@unittest.skipUnless(HAVE_STEP, "Install tools/requirements-tracks.txt to test STEP")
class StepExportTests(unittest.TestCase):
    def test_all_tracks_roundtrip_and_independent_formula_checks(self):
        from OCP.BRep import BRep_Tool
        from OCP.BRepCheck import BRepCheck_Analyzer
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPControl import STEPControl_Reader
        from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID
        from OCP.TopExp import TopExp, TopExp_Explorer
        from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
        from OCP.TopoDS import TopoDS
        from tools.track_step import export_step, fit_track_surfaces

        def count_shapes(shape, kind):
            explorer = TopExp_Explorer(shape, kind)
            count = 0
            while explorer.More():
                count += 1
                explorer.Next()
            return count

        names, templates = create_track_templates(40, 6)
        # Check both closed and open profiles, including nonuniform custom corners.
        profiles = [default_profile(), prepare_profile([[-9, 1.3], [-2.4, -3.8], [7, 2.2]], True)]
        rng = np.random.default_rng(237)
        us, ts = np.sort(rng.random(19)), np.sort(rng.random(23))
        with tempfile.TemporaryDirectory() as directory:
            for profile in profiles:
                for name, template in zip(names, templates):
                    with self.subTest(track=name, closed=len(profile) == 5):
                        path = Path(directory) / f"{name}.step"
                        result = export_step(template, profile, path)
                        self.assertLessEqual(result.sampled_error, 0.01)
                        self.assertIn("B_SPLINE_SURFACE_WITH_KNOTS", path.read_text())
                        self.assertIn(".MILLI.,.METRE.", path.read_text())
                        reader = STEPControl_Reader()
                        self.assertEqual(reader.ReadFile(str(path)), IFSelect_RetDone)
                        self.assertGreater(reader.TransferRoots(), 0)
                        shape = reader.OneShape()
                        self.assertTrue(BRepCheck_Analyzer(shape).IsValid())
                        self.assertEqual(count_shapes(shape, TopAbs_FACE), result.patch_count)
                        self.assertEqual(count_shapes(shape, TopAbs_SHELL), 1)
                        self.assertEqual(count_shapes(shape, TopAbs_SOLID), 0)
                        adjacency = TopTools_IndexedDataMapOfShapeListOfShape()
                        TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, adjacency)
                        free_edges = sum(adjacency.FindFromIndex(i).Extent() == 1
                                         for i in range(1, adjacency.Extent() + 1))
                        closed = np.array_equal(profile[0], profile[-1])
                        branches = 2 if hasattr(template, "curve_segment") else 1
                        self.assertEqual(free_edges, 2 * (len(profile) - 1) + (0 if closed else 2 * branches))

                        patches, _ = fit_track_surfaces(template, profile)
                        explorer = TopExp_Explorer(shape, TopAbs_FACE)
                        surfaces = []
                        while explorer.More():
                            surfaces.append(BRep_Tool.Surface_s(TopoDS.Face_s(explorer.Current())))
                            explorer.Next()
                        # Match by center so the test doesn't depend on STEP face order.
                        surface_map = {}
                        for patch in patches:
                            center = patch.surface.Value(0.5, 0.5)
                            surface = min(surfaces, key=lambda s: s.Value(0.5, 0.5).Distance(center))
                            surfaces.remove(surface)
                            surface_map[patch.profile_edge, patch.branch] = surface
                        # Independently sample the original longitudinal t, choosing
                        # and parameterizing the imported face for that exact point.
                        for edge, (start, end) in enumerate(zip(profile[:-1], profile[1:])):
                            for u in us:
                                pt = start * (1 - u) + end * u
                                template.set_sample_pt(pt)
                                for t in ts:
                                    branch, v = -1, t
                                    if hasattr(template, "curve_segment"):
                                        join = 40 / template.get_base_path_length(pt)
                                        branch = 0 if t <= join else 1
                                        v = t / join if branch == 0 else (t - join) / (1 - join)
                                    actual = surface_map[edge, branch].Value(float(u), float(v)).Coord()
                                    expected = template.sample_path_t(t)
                                    expected[2] = np.clip(expected[2], profile[:, 1].min() - 6,
                                                          profile[:, 1].max() + 6)
                                    self.assertLessEqual(float(np.linalg.norm(actual - expected)), 0.01)

    def test_tight_tolerance_refinement_and_failure(self):
        from tools.track_step import fit_track_surfaces, sample_patch

        _, templates = create_track_templates(40, 6)
        profile = default_profile()
        coarse, coarse_samples = fit_track_surfaces(templates[4], profile)
        fine, fine_samples = fit_track_surfaces(templates[4], profile, tolerance=1e-5)
        self.assertLess(max(p.sampled_error for p in fine), 1e-5)
        self.assertGreater(max(fine_samples), max(coarse_samples))
        with self.assertRaisesRegex(ValueError, "exceeds"):
            fit_track_surfaces(templates[4], profile, tolerance=1e-6, max_samples=5)
        # Both transition branches must meet at their profile-dependent join.
        for template in templates[5:]:
            us = np.linspace(0, 1, 11)
            for start, end in zip(profile[:-1], profile[1:]):
                a = sample_patch(template, start, end, us, np.array([1.0]), 0, -12., 12.)
                b = sample_patch(template, start, end, us, np.array([0.0]), 1, -12., 12.)
                np.testing.assert_allclose(a, b, atol=1e-12)

    def test_rejects_bend_axis_crossing(self):
        from tools.track_step import fit_track_surfaces

        _, templates = create_track_templates(40, 6)
        for index in (2, 3, 4, 5, 6, 7, 8):
            with self.subTest(track=index), self.assertRaisesRegex(ValueError, "bend axis"):
                fit_track_surfaces(templates[index], np.array([[-25., 0.], [25., 1.]]))


if __name__ == "__main__":
    unittest.main()
