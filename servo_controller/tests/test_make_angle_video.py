"""Regression tests: Renderer helper behavior and label/time conventions. These
focused tests do not validate a full camera recording or codec installation.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import unittest
import csv
import tempfile
from pathlib import Path

from make_angle_video import AngleBin, bin_at_time, label_bin_at_time, load_bins, revolution_count, revolution_time_range, interpolated_angle_at_time


class AngleVideoTests(unittest.TestCase):
    def setUp(self):
        self.bins = [
            AngleBin(1.0, 1.1, 350.0, 350.0),
            AngleBin(1.1, 1.2, 370.0, 10.0),
        ]
        self.starts = [row.start_s for row in self.bins]

    def test_half_open_bin_lookup(self):
        self.assertIsNone(bin_at_time(self.bins, self.starts, 0.99))
        self.assertEqual(bin_at_time(self.bins, self.starts, 1.099), self.bins[0])
        self.assertEqual(bin_at_time(self.bins, self.starts, 1.1), self.bins[1])
        self.assertIsNone(bin_at_time(self.bins, self.starts, 1.2))

    def test_revolution_count_is_relative_to_run_origin(self):
        self.assertEqual(revolution_count(349.8, 350.0), 0)
        self.assertEqual(revolution_count(709.9, 350.0), 0)
        self.assertEqual(revolution_count(710.0, 350.0), 1)

    def test_video_labels_hold_first_angle_through_camera_preroll(self):
        self.assertEqual(label_bin_at_time(self.bins, self.starts, 0.0), self.bins[0])
        self.assertEqual(label_bin_at_time(self.bins, self.starts, 1.15), self.bins[1])
        self.assertEqual(label_bin_at_time(self.bins, self.starts, 2.0), self.bins[1])

    def test_inclusive_revolution_range_interpolates_boundaries(self):
        bins = [AngleBin(i, i + 1, 350 + i * 200, (350 + i * 200) % 360) for i in range(15)]
        start, end = revolution_time_range(bins, 3, 6)
        self.assertAlmostEqual(start, 5.4)
        self.assertAlmostEqual(end, 12.6)  # End of label 6 is the start of label 7.
        self.assertEqual(revolution_time_range(bins, None, None), (None, None))
        self.assertEqual(revolution_time_range(bins, 0, None), (0, None))
        self.assertAlmostEqual(revolution_time_range(bins, None, 0)[1], 1.8)
        label = interpolated_angle_at_time(bins, [row.start_s for row in bins], start + 0.001)
        self.assertEqual(revolution_count(label.unwrapped_deg, bins[0].unwrapped_deg), 3)
        self.assertAlmostEqual(label.wrapped_deg, 350.2)

    def test_revolution_range_rejects_invalid_or_incomplete_selection(self):
        bins = [AngleBin(i, i + 1, i * 360, 0) for i in range(5)]
        for start, end in [(-1, 2), (2, 1), (0, 4), (9, None), (None, -1)]:
            with self.subTest(start=start, end=end), self.assertRaises(ValueError):
                revolution_time_range(bins, start, end)
        reversing = [AngleBin(i, i + 1, angle, angle % 360) for i, angle in enumerate([0, 200, 100, 720])]
        with self.assertRaisesRegex(ValueError, 'without reversals'):
            revolution_time_range(reversing, 0, 1)

    def test_canonical_bins_load_speed_and_vertical_translation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bins.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=(
                    "camera_start_us", "camera_end_us", "platform_angle_deg",
                    "platform_wrapped_angle_deg", "recorded_platform_feedback_rpm",
                    "scene_vertical_displacement_mm",
                ))
                writer.writeheader()
                writer.writerow({
                    "camera_start_us": 0, "camera_end_us": 20_000,
                    "platform_angle_deg": 370, "platform_wrapped_angle_deg": 10,
                    "recorded_platform_feedback_rpm": 24.95,
                    "scene_vertical_displacement_mm": 0.351188,
                })
            loaded = load_bins(path)

        self.assertEqual(loaded[0].platform_speed_rpm, 24.95)
        self.assertEqual(loaded[0].vertical_translation_mm, 0.351188)


if __name__ == "__main__":
    unittest.main()
