"""Run with: python3 -m unittest tests.test_jigs  (2D layout only; the full 3D run is python3 -m tools.jigs)."""
import unittest

from tools.jigs.layout import SPECIES, MIRROR_X, MIRROR_Y, build_layout
from tools.jigs.params import Params
from tools.jigs.track import Track
from tools.jigs import checks


class LayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = Params()
        cls.tracks = {}
        for sp in SPECIES.values():
            for m in sp.members:
                cls.tracks.setdefault(m, Track(m, cls.p))
        cls.layouts = {k: build_layout(sp, cls.tracks, cls.p) for k, sp in SPECIES.items()}

    def test_full_set_per_glue_up(self):
        lr = [s.track for s in self.layouts["lr"].slots]
        self.assertEqual(sorted(lr), ["L"] * 4 + ["R"] * 4)
        self.assertEqual(sorted(s.track for s in self.layouts["slr"].slots), ["Ls", "Rs", "sL", "sR"])

    def test_each_type_seats_exactly_one_way(self):
        for name, layout in self.layouts.items():
            for i, regs in layout.placements.items():
                self.assertEqual(len(regs), 1, f"{name} slot {i} ({layout.slots[i].track})")

    def test_layout_checks_pass(self):
        rep = checks.Report()
        for layout in self.layouts.values():
            checks.check_layout(rep, layout, self.tracks, self.p)
        self.assertEqual(rep.failures, [])

    def test_ring_bolts_symmetric_with_centre_bolts(self):
        for name in ("lr", "slr"):
            layout = self.layouts[name]
            self.assertTrue(checks._sym_set(layout.bolts, MIRROR_X) and checks._sym_set(layout.bolts, MIRROR_Y))
            self.assertEqual(len(layout.bolts), 5 * SPECIES[name].copies)


if __name__ == "__main__":
    unittest.main()
