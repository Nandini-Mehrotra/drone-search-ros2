import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from search_math import figure_eight, outside_geofence


class SearchMathTests(unittest.TestCase):
    def setUp(self):
        self.home = (1.0, -2.0, -0.5)

    def test_figure_eight_starts_at_home_xy(self):
        x, y, z = figure_eight(0, self.home)
        self.assertAlmostEqual(x, self.home[0])
        self.assertAlmostEqual(y, self.home[1])
        self.assertAlmostEqual(z, -5.5)

    def test_figure_eight_returns_to_start(self):
        start = figure_eight(0, self.home)
        end = figure_eight(20, self.home)
        for a, b in zip(start, end):
            self.assertAlmostEqual(a, b)

    def test_figure_eight_within_amplitudes(self):
        for step in range(201):
            x, y, _ = figure_eight(step / 10, self.home)
            self.assertLessEqual(abs(x - self.home[0]), 6 + 1e-9)
            self.assertLessEqual(abs(y - self.home[1]), 3 + 1e-9)

    def test_invalid_period(self):
        with self.assertRaises(ValueError):
            figure_eight(0, self.home, period_s=0)

    def test_inside_geofence(self):
        self.assertFalse(outside_geofence((6, -2, -5), self.home))

    def test_radius_breach(self):
        self.assertTrue(outside_geofence((14, -2, -5), self.home))

    def test_altitude_breach(self):
        self.assertTrue(outside_geofence((1, -2, -9), self.home))

    def test_exact_boundary_allowed(self):
        self.assertFalse(outside_geofence((13, -2, -8.5), self.home))

    def test_invalid_geofence(self):
        with self.assertRaises(ValueError):
            outside_geofence(self.home, self.home, radius_m=0)


if __name__ == '__main__':
    unittest.main()
