"""Regression tests: Crossing interpolation and no-crossing behavior for angle
references. Extend with revisit/reversal fixtures before claiming all crossing
occurrences.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import unittest

from build_angle_reference_csv import Sample, crossing_time


class AngleReferenceCsvTests(unittest.TestCase):
    def test_crossing_time_interpolates_between_encoder_samples(self):
        samples = [Sample(1.0, 100.0, 10.0, 1.0), Sample(1.1, 140.0, 10.0, 1.0)]
        self.assertAlmostEqual(crossing_time(samples, 120.0), 1.05)

    def test_crossing_outside_samples_is_absent(self):
        samples = [Sample(1.0, 100.0, 10.0, 1.0), Sample(1.1, 140.0, 10.0, 1.0)]
        self.assertIsNone(crossing_time(samples, 150.0))


if __name__ == "__main__":
    unittest.main()
